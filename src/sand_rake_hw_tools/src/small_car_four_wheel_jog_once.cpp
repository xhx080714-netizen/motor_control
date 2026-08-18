#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "sand_rake_control/modbus_rtu.hpp"
#include "sand_rake_control/modbus_rtu_client.hpp"
#include "sand_rake_control/rs232_transport.hpp"

namespace
{
constexpr std::uint8_t kSlaveAddress = 0x01;
constexpr std::uint8_t kDualMotorFunction = 0x10;
constexpr std::uint16_t kStatusStartRegister = 0x0000;
constexpr std::uint16_t kStatusRegisterCount = 12;
constexpr std::uint16_t kModeReadRegister = 0x0008;
constexpr std::uint16_t kSingleRegister = 1;
constexpr std::uint16_t kCoastState = 0;
constexpr std::uint16_t kReferenceForwardState = 1;
constexpr std::uint16_t kReferenceReverseState = 2;
constexpr std::uint16_t kDefaultJogRpm = 30;
constexpr int kJogDurationMs = 300;
constexpr int kStateSampleDelayMs = 50;
constexpr int kReferenceWriteDelayMs = 2;
constexpr int kPostInitializationQuietMs = 20;
constexpr int kDefaultTimeoutMs = 100;
constexpr int kMaximumTimeoutMs = 1000;
constexpr int kMaximumProtocolRpm = 0x0FFF;

struct InitializationStep
{
  const char * stage;
  std::uint16_t register_address;
  std::uint16_t value;
};

constexpr std::array<InitializationStep, 5> kInitializationSteps{{
  {"init_clear_alarm", 0x0009, 0x0001},
  {"init_clear_hall", 0x0007, 0x0001},
  {"init_set_follow", 0x000F, 0x0001},
  {"init_m1_decel_zero", 0x0005, 0x0000},
  {"init_m2_decel_zero", 0x0006, 0x0000},
}};
#if defined(SAND_RAKE_HW_TOOLS_TEST_ALLOW_PTY)
constexpr bool kPtyTestBuild = true;
#else
constexpr bool kPtyTestBuild = false;
#endif

volatile std::sig_atomic_t g_stop_requested = 0;

enum class Board
{
  kFront,
  kRear,
};

enum class Motor
{
  kM1,
  kM2,
};

enum class Direction
{
  kReferenceForward,
  kReferenceReverse,
};

struct BoardProfile
{
  const char * connector;
  const char * device;
  const char * wheel_group;
};

struct Options
{
  std::optional<Board> board;
  std::optional<Motor> motor;
  std::optional<Direction> direction;
  std::string device;
  int timeout_ms{kDefaultTimeoutMs};
  std::uint16_t jog_rpm{kDefaultJogRpm};
  bool status_only{false};
  bool coast_only{false};
  bool initialize{false};
  bool execute{false};
  bool wheels_off_ground{false};
  bool hardware_stop_ready{false};
  bool exclusive_tty{false};
  bool show_help{false};
};

struct SendResult
{
  sand_rake_control::TransportResult transport;
  std::chrono::microseconds latency{0};

  explicit operator bool() const noexcept
  {
    return static_cast<bool>(transport);
  }
};

void handle_stop_signal(int)
{
  g_stop_requested = 1;
}

BoardProfile board_profile(Board board)
{
  if (board == Board::kFront) {
    return BoardProfile{"JP17", "/dev/ttyS6", "front_wheels"};
  }
  return BoardProfile{"JP18", "/dev/ttyS1", "rear_wheels"};
}

const char * board_string(Board board)
{
  return board == Board::kFront ? "front" : "rear";
}

const char * motor_string(Motor motor)
{
  return motor == Motor::kM1 ? "M1" : "M2";
}

const char * direction_string(Direction direction)
{
  return direction == Direction::kReferenceForward ?
         "table_forward_state_1" : "table_reverse_state_2";
}

std::uint16_t pack_motor_word(std::uint16_t rpm, std::uint16_t state)
{
  const std::uint16_t bounded_rpm = std::min<std::uint16_t>(rpm, 0x0FFF);
  return static_cast<std::uint16_t>((bounded_rpm << 4U) | (state & 0x000F));
}

sand_rake_control::modbus_rtu::Frame build_dual_motor_frame(
  std::uint16_t m1_word, std::uint16_t m2_word)
{
  sand_rake_control::modbus_rtu::Frame frame{
    kSlaveAddress,
    kDualMotorFunction,
    static_cast<std::uint8_t>((m1_word >> 8U) & 0xFFU),
    static_cast<std::uint8_t>(m1_word & 0xFFU),
    static_cast<std::uint8_t>((m2_word >> 8U) & 0xFFU),
    static_cast<std::uint8_t>(m2_word & 0xFFU),
  };
  const std::uint16_t crc =
    sand_rake_control::modbus_rtu::crc16_modbus(frame);
  frame.push_back(static_cast<std::uint8_t>(crc & 0xFFU));
  frame.push_back(static_cast<std::uint8_t>((crc >> 8U) & 0xFFU));
  return frame;
}

sand_rake_control::modbus_rtu::Frame build_coast_frame()
{
  return build_dual_motor_frame(
    pack_motor_word(0, kCoastState), pack_motor_word(0, kCoastState));
}

sand_rake_control::modbus_rtu::Frame build_jog_frame(
  Motor motor, Direction direction, std::uint16_t jog_rpm)
{
  const std::uint16_t state =
    direction == Direction::kReferenceForward ?
    kReferenceForwardState : kReferenceReverseState;
  const std::uint16_t jog_word = pack_motor_word(jog_rpm, state);
  const std::uint16_t coast_word = pack_motor_word(0, kCoastState);
  return motor == Motor::kM1 ?
         build_dual_motor_frame(jog_word, coast_word) :
         build_dual_motor_frame(coast_word, jog_word);
}

void print_usage(const char * program)
{
  std::cout <<
    "Usage: " << program <<
    " --board front|rear [--status-only | --coast-only | --initialize |\n"
    "       --motor M1|M2\n"
    "       --direction forward|reverse] [--rpm 30|50|60|100] [--timeout-ms N]\n"
    "       [--execute --device PATH --confirm-wheels-off-ground\n"
    "        --confirm-hardware-stop-ready --confirm-exclusive-tty]\n\n"
    "Small-car protocol from company register table V4: each board controls M1/M2 with\n"
    "an 8-byte 01 10 <M1 word> <M2 word> CRC frame. Default mode is\n"
    "dry-run. Jog duration is fixed at 300 ms; rpm is restricted to\n"
    "30, 50, 60, or 100.\n";
}

bool parse_positive_integer(
  const std::string & text, int & value, int maximum_value)
{
  if (text.empty()) {
    return false;
  }
  char * end = nullptr;
  errno = 0;
  const long parsed = std::strtol(text.c_str(), &end, 10);
  if (errno != 0 || end == text.c_str() || *end != '\0' ||
    parsed <= 0 || parsed > maximum_value ||
    parsed > std::numeric_limits<int>::max())
  {
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

bool parse_options(int argc, char ** argv, Options & options)
{
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      options.show_help = true;
      return true;
    }
    if (argument == "--board") {
      if (++index >= argc) {
        return false;
      }
      const std::string value = argv[index];
      if (value == "front") {
        options.board = Board::kFront;
      } else if (value == "rear") {
        options.board = Board::kRear;
      } else {
        return false;
      }
      continue;
    }
    if (argument == "--motor") {
      if (++index >= argc) {
        return false;
      }
      const std::string value = argv[index];
      if (value == "M1") {
        options.motor = Motor::kM1;
      } else if (value == "M2") {
        options.motor = Motor::kM2;
      } else {
        return false;
      }
      continue;
    }
    if (argument == "--direction") {
      if (++index >= argc) {
        return false;
      }
      const std::string value = argv[index];
      if (value == "forward") {
        options.direction = Direction::kReferenceForward;
      } else if (value == "reverse") {
        options.direction = Direction::kReferenceReverse;
      } else {
        return false;
      }
      continue;
    }
    if (argument == "--device") {
      if (++index >= argc || std::string(argv[index]).empty()) {
        return false;
      }
      options.device = argv[index];
      continue;
    }
    if (argument == "--timeout-ms") {
      if (++index >= argc ||
        !parse_positive_integer(
          argv[index], options.timeout_ms, kMaximumTimeoutMs))
      {
        return false;
      }
      continue;
    }
    if (argument == "--rpm") {
      int parsed_rpm = 0;
      if (++index >= argc ||
        !parse_positive_integer(
          argv[index], parsed_rpm, kMaximumProtocolRpm) ||
        (parsed_rpm != 30 && parsed_rpm != 50 &&
        parsed_rpm != 60 && parsed_rpm != 100))
      {
        return false;
      }
      options.jog_rpm = static_cast<std::uint16_t>(parsed_rpm);
      continue;
    }
    if (argument == "--coast-only") {
      options.coast_only = true;
      continue;
    }
    if (argument == "--status-only") {
      options.status_only = true;
      continue;
    }
    if (argument == "--initialize") {
      options.initialize = true;
      continue;
    }
    if (argument == "--execute") {
      options.execute = true;
      continue;
    }
    if (argument == "--confirm-wheels-off-ground") {
      options.wheels_off_ground = true;
      continue;
    }
    if (argument == "--confirm-hardware-stop-ready") {
      options.hardware_stop_ready = true;
      continue;
    }
    if (argument == "--confirm-exclusive-tty") {
      options.exclusive_tty = true;
      continue;
    }
    return false;
  }

  if (!options.board.has_value()) {
    return false;
  }
  const int standalone_operation_count =
    static_cast<int>(options.coast_only) +
    static_cast<int>(options.status_only) +
    static_cast<int>(options.initialize);
  if (standalone_operation_count > 0) {
    if (standalone_operation_count != 1) {
      return false;
    }
    return !options.motor.has_value() && !options.direction.has_value();
  }
  return options.motor.has_value() && options.direction.has_value();
}

void print_frame(
  const char * label, const sand_rake_control::modbus_rtu::Frame & frame)
{
  std::cout << label << '=';
  for (std::size_t index = 0; index < frame.size(); ++index) {
    if (index > 0) {
      std::cout << ' ';
    }
    std::cout << std::hex << std::uppercase << std::setfill('0') <<
      std::setw(2) << static_cast<unsigned int>(frame[index]);
  }
  std::cout << std::dec << '\n';
}

void print_hex_word(const char * label, std::uint16_t value)
{
  std::cout << label << "=0x" << std::hex << std::uppercase <<
    std::setfill('0') << std::setw(4) << value << std::dec << '\n';
}

void print_hex_dword(const char * label, std::uint32_t value)
{
  std::cout << label << "=0x" << std::hex << std::uppercase <<
    std::setfill('0') << std::setw(8) << value << std::dec << '\n';
}

const char * motor_state_string(std::uint16_t state)
{
  switch (state) {
    case 0:
      return "FREE_STOP";
    case 1:
      return "FORWARD";
    case 2:
      return "REVERSE";
    case 3:
      return "BRAKE_STOP";
    case 4:
      return "DECELERATE_TO_ZERO";
    default:
      return "UNKNOWN";
  }
}

void print_motor_word(const char * prefix, std::uint16_t value)
{
  const std::string label_prefix(prefix);
  print_hex_word((label_prefix + "_raw").c_str(), value);
  std::cout <<
    label_prefix << "_rpm=" << (value >> 4U) << '\n' <<
    label_prefix << "_code=" << (value & 0x000FU) << '\n' <<
    label_prefix << '=' << motor_state_string(value & 0x000FU) << '\n';
}

void print_full_status_values(
  const std::vector<std::uint16_t> & values)
{
  const std::uint32_t m1_hall_count =
    (static_cast<std::uint32_t>(values.at(2)) << 16U) | values.at(3);
  const std::uint32_t m2_hall_count =
    (static_cast<std::uint32_t>(values.at(4)) << 16U) | values.at(5);
  print_motor_word("m1_state", values.at(0));
  print_motor_word("m2_state", values.at(1));
  print_hex_dword("m1_hall_count_raw", m1_hall_count);
  std::cout << "m1_hall_count=" << m1_hall_count << '\n';
  print_hex_dword("m2_hall_count_raw", m2_hall_count);
  std::cout << "m2_hall_count=" << m2_hall_count << '\n';
  std::cout <<
    "m1_feedback_rpm=" << values.at(6) << '\n' <<
    "m2_feedback_rpm=" << values.at(7) << '\n';
  print_hex_word("status_mode_raw", values.at(8));
  print_hex_word("bus_voltage_raw", values.at(9));
  std::cout << "bus_voltage_mv=" <<
    static_cast<std::uint32_t>(values.at(9)) * 100U << '\n';
  print_hex_word("alarm_code_raw", values.at(10));
  print_hex_word("program_version_raw", values.at(11));
}

bool device_allowed(const BoardProfile & profile, const std::string & device)
{
  if (kPtyTestBuild) {
    return device.rfind("/dev/pts/", 0) == 0;
  }
  return device == profile.device;
}

bool read_expected(
  sand_rake_control::ModbusRtuClient & client,
  const char * stage,
  std::uint16_t register_address,
  std::uint16_t expected)
{
  const auto result = client.read_holding_registers(
    kSlaveAddress, register_address, kSingleRegister);
  if (!result) {
    std::cerr <<
      "stage=" << stage << '\n' <<
      "transaction_error=" <<
      sand_rake_control::transaction_error_string(result.error) << '\n' <<
      "transport_error=" <<
      sand_rake_control::transport_error_string(result.transport_error) << '\n' <<
      "protocol_error=" <<
      sand_rake_control::modbus_rtu::protocol_error_string(
      result.protocol_error) << '\n' <<
      "exception_code=" << static_cast<unsigned int>(result.exception_code) << '\n';
    return false;
  }
  const std::uint16_t actual = result.values.at(0);
  std::cout << stage << "_latency_us=" << result.latency.count() << '\n';
  print_hex_word((std::string(stage) + "_raw").c_str(), actual);
  if (actual != expected) {
    std::cerr << "stage=" << stage << "\nvalidation=UNEXPECTED_VALUE\n";
    print_hex_word("expected_raw", expected);
    return false;
  }
  return true;
}

SendResult send_reference_frame(
  sand_rake_control::Rs232Transport & transport,
  const sand_rake_control::modbus_rtu::Frame & frame)
{
  SendResult result;
  const auto discard_result = transport.discard_input();
  if (!discard_result) {
    result.transport = discard_result;
    return result;
  }
  const auto started_at = std::chrono::steady_clock::now();
  result.transport = transport.write_all(
    frame.data(), frame.size(),
    started_at + std::chrono::milliseconds(kDefaultTimeoutMs));
  result.latency = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - started_at);
  if (result) {
    std::this_thread::sleep_for(
      std::chrono::milliseconds(kReferenceWriteDelayMs));
  }
  return result;
}

bool send_and_report(
  sand_rake_control::Rs232Transport & transport,
  const char * stage,
  const sand_rake_control::modbus_rtu::Frame & frame)
{
  const auto result = send_reference_frame(transport, frame);
  std::cout << stage << "_latency_us=" << result.latency.count() << '\n';
  if (!result) {
    std::cerr <<
      "stage=" << stage << '\n' <<
      "transport_error=" <<
      sand_rake_control::transport_error_string(result.transport.error) << '\n' <<
      "system_error=" << result.transport.system_error << '\n';
    return false;
  }
  std::cout << stage << "_write=OK\n";
  return true;
}

bool interruptible_delay(int duration_ms)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(duration_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (g_stop_requested != 0) {
      return false;
    }
    const auto remaining = deadline - std::chrono::steady_clock::now();
    const auto slice = std::min(
      std::chrono::duration_cast<std::chrono::milliseconds>(remaining),
      std::chrono::milliseconds(10));
    if (slice.count() > 0) {
      std::this_thread::sleep_for(slice);
    }
  }
  return g_stop_requested == 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  if (!parse_options(argc, argv, options)) {
    std::cerr << "ERROR: invalid or incomplete arguments\n";
    print_usage(argv[0]);
    return 2;
  }
  if (options.show_help) {
    print_usage(argv[0]);
    return 0;
  }

  const BoardProfile profile = board_profile(*options.board);
  const auto coast_frame = build_coast_frame();
  const auto jog_frame =
    (options.coast_only || options.status_only || options.initialize) ?
    coast_frame :
    build_jog_frame(*options.motor, *options.direction, options.jog_rpm);

  std::cout <<
    "tool=SMALL_CAR_DUAL_BOARD_DUAL_MOTOR_JOG_ONCE\n" <<
    "protocol_source=MOTOR_REGISTER_TABLE_V4\n" <<
    "board=" << board_string(*options.board) << '\n' <<
    "connector=" << profile.connector << '\n' <<
    "wheel_group=" << profile.wheel_group << '\n' <<
    "expected_device=" << profile.device << '\n' <<
    "slave=0x01\n";
  if (options.status_only) {
    std::cout <<
      "operation=READ_SMALL_CAR_STATUS_ONLY\n" <<
      "function=0x03_READ_ONLY\n";
  } else if (options.coast_only) {
    std::cout <<
      "operation=COAST_BOTH_MOTORS\n" <<
      "function=0x10_CUSTOM_DUAL_MOTOR\n" <<
      "motor_word=RPM_SHIFT_LEFT_4_OR_STATE\n" <<
      "response_handling=NO_ACK_READ_MATCH_EXISTING_FIRMWARE\n";
  } else if (options.initialize) {
    std::cout <<
      "operation=REGISTER_TABLE_V4_INITIALIZE\n" <<
      "function=0x06_WRITE_SINGLE_NO_ACK_READ\n" <<
      "initialization_order=EXISTING_FIRMWARE_SEQUENCE\n" <<
      "response_handling=NO_ACK_READ_MATCH_EXISTING_FIRMWARE\n";
  } else {
    std::cout <<
      "operation=SINGLE_MOTOR_JOG\n" <<
      "function=0x10_CUSTOM_DUAL_MOTOR\n" <<
      "motor_word=RPM_SHIFT_LEFT_4_OR_STATE\n" <<
      "response_handling=NO_ACK_READ_MATCH_EXISTING_FIRMWARE\n" <<
      "motor=" << motor_string(*options.motor) << '\n' <<
      "direction=" << direction_string(*options.direction) << '\n' <<
      "direction_semantics=REGISTER_TABLE_STATE_NOT_PHYSICAL_MAPPING\n" <<
      "rpm=" << options.jog_rpm << '\n' <<
      "duration_ms=" << kJogDurationMs << '\n';
  }
  if (options.coast_only) {
    print_frame("planned_coast", coast_frame);
  } else if (options.initialize) {
    print_frame("planned_pre_coast", coast_frame);
    for (const auto & step : kInitializationSteps) {
      print_frame(
        (std::string("planned_") + step.stage).c_str(),
        sand_rake_control::modbus_rtu::build_write_single_register_request(
          kSlaveAddress, step.register_address, step.value));
    }
    print_frame("planned_final_coast", coast_frame);
  } else if (!options.status_only) {
    print_frame("planned_pre_coast", coast_frame);
    print_frame("planned_jog", jog_frame);
    print_frame("planned_final_coast", coast_frame);
  }

  if (!options.execute) {
    std::cout <<
      "mode=DRY_RUN\n"
      "serial_opened=NO\n"
      "write_operations=NOT_EXECUTED\n";
    return 0;
  }
  if (options.device.empty() || !device_allowed(profile, options.device)) {
    std::cerr <<
      "result=BLOCKED\n"
      "reason=DEVICE_DOES_NOT_MATCH_BOARD_PROFILE\n"
      "serial_opened=NO\n"
      "write_operations=NOT_EXECUTED\n";
    return 3;
  }
  if (!options.wheels_off_ground || !options.hardware_stop_ready ||
    !options.exclusive_tty)
  {
    std::cerr <<
      "result=BLOCKED\n"
      "reason=SAFETY_CONFIRMATIONS_INCOMPLETE\n"
      "serial_opened=NO\n"
      "write_operations=NOT_EXECUTED\n";
    return 3;
  }

  std::signal(SIGINT, handle_stop_signal);
  std::signal(SIGTERM, handle_stop_signal);

  sand_rake_control::Rs232Config serial_config;
  serial_config.device = options.device;
  sand_rake_control::Rs232Transport transport;
  const auto open_result = transport.open(serial_config);
  if (!open_result) {
    std::cerr <<
      "result=ERROR\n"
      "stage=open\n"
      "transport_error=" <<
      sand_rake_control::transport_error_string(open_result.error) << '\n' <<
      "system_error=" << open_result.system_error << '\n';
    return 4;
  }

  sand_rake_control::ModbusRtuClient client(
    transport, std::chrono::milliseconds(options.timeout_ms));
  if (!read_expected(client, "mode", kModeReadRegister, 0x0001)) {
    if (g_stop_requested != 0 && !options.status_only) {
      std::cerr << "stage=mode\nstop_requested=YES\n";
      const bool final_coast_ok =
        send_and_report(transport, "final_coast", coast_frame);
      std::cout << "final_coast_result=" <<
        (final_coast_ok ? "OK" : "FAILED") << '\n';
      std::cerr << "result=ERROR\n";
      return final_coast_ok ? 130 : 7;
    }
    std::cerr <<
      "result=ERROR\n"
      "write_operations=NOT_EXECUTED\n";
    return 5;
  }

  if (options.status_only) {
    const auto status_result = client.read_holding_registers(
      kSlaveAddress, kStatusStartRegister, kStatusRegisterCount);
    if (!status_result) {
      std::cerr <<
        "result=ERROR\n" <<
        "stage=full_status\n" <<
        "transaction_error=" <<
        sand_rake_control::transaction_error_string(status_result.error) << '\n';
      return 5;
    }
    std::cout <<
      "full_status_latency_us=" << status_result.latency.count() << '\n';
    print_full_status_values(status_result.values);
    std::cout <<
      "write_operations=NOT_EXECUTED\n" <<
      "result=OK\n";
    return 0;
  }

  if (!options.coast_only && !options.initialize) {
    const auto preflight_status = client.read_holding_registers(
      kSlaveAddress, kStatusStartRegister, kStatusRegisterCount);
    if (!preflight_status) {
      std::cerr <<
        "result=ERROR\n" <<
        "stage=preflight_status\n" <<
        "transaction_error=" <<
        sand_rake_control::transaction_error_string(preflight_status.error) << '\n' <<
        "write_operations=NOT_EXECUTED\n";
      return 5;
    }
    std::cout <<
      "preflight_status_latency_us=" << preflight_status.latency.count() << '\n';
    print_full_status_values(preflight_status.values);
    if (preflight_status.values.at(8) != 0x0001 ||
      preflight_status.values.at(10) != 0x0000)
    {
      std::cerr <<
        "result=BLOCKED\n" <<
        "reason=PREFLIGHT_STATUS_NOT_SAFE\n" <<
        "write_operations=NOT_EXECUTED\n";
      return 5;
    }
  }

  if (options.coast_only) {
    if (!send_and_report(transport, "coast", coast_frame)) {
      std::cerr << "result=ERROR\n";
      return 6;
    }
    std::cout << "result=OK\n";
    return 0;
  }

  if (options.initialize) {
    int result_code = 0;
    if (!send_and_report(transport, "pre_coast", coast_frame)) {
      result_code = 6;
    } else {
      for (const auto & step : kInitializationSteps) {
        if (g_stop_requested != 0) {
          std::cerr <<
            "stage=" << step.stage << '\n' <<
            "stop_requested=YES\n";
          result_code = 130;
          break;
        }
        const auto frame =
          sand_rake_control::modbus_rtu::build_write_single_register_request(
          kSlaveAddress, step.register_address, step.value);
        if (!send_and_report(transport, step.stage, frame)) {
          result_code = 6;
          break;
        }
      }
    }

    const bool final_coast_ok =
      send_and_report(transport, "final_coast", coast_frame);
    if (!final_coast_ok) {
      std::cerr << "final_coast_result=FAILED\n";
      result_code = 7;
    } else {
      std::cout << "final_coast_result=OK\n";
    }

    if (result_code == 0) {
      // Existing firmware does not consume 0x06/0x10 responses. Allow delayed
      // bytes to arrive before the Modbus read discards input and checks mode.
      std::this_thread::sleep_for(
        std::chrono::milliseconds(kPostInitializationQuietMs));
      if (!read_expected(
          client, "post_init_mode", kModeReadRegister, 0x0001))
      {
        result_code = 6;
      }
    }

    if (result_code == 0) {
      std::cout << "result=OK\n";
    } else {
      std::cerr << "result=ERROR\n";
    }
    return result_code;
  }

  int result_code = 0;
  if (!send_and_report(transport, "pre_coast", coast_frame)) {
    result_code = 6;
  } else if (g_stop_requested != 0) {
    std::cerr << "stage=before_jog\nstop_requested=YES\n";
    result_code = 130;
  } else {
    const auto jog_started_at = std::chrono::steady_clock::now();
    const auto jog_deadline = jog_started_at +
      std::chrono::milliseconds(kJogDurationMs);
    if (!send_and_report(transport, "jog", jog_frame)) {
      result_code = 6;
    } else if (!interruptible_delay(kStateSampleDelayMs)) {
      std::cerr << "stage=jog_delay\nstop_requested=YES\n";
      result_code = 130;
    } else {
      const auto now = std::chrono::steady_clock::now();
      const auto remaining = jog_deadline - now;
      const auto active_status_timeout =
        std::min(
        std::chrono::milliseconds(options.timeout_ms),
        std::chrono::duration_cast<std::chrono::milliseconds>(remaining));
      if (active_status_timeout.count() <= 0) {
        std::cerr << "stage=active_status\ntransaction_error=jog deadline reached\n";
        result_code = 6;
      } else {
        sand_rake_control::ModbusRtuClient active_status_client(
          transport, active_status_timeout);
        const auto active_status_result = active_status_client.read_holding_registers(
          kSlaveAddress, kStatusStartRegister, kStatusRegisterCount);
        if (!active_status_result) {
          std::cerr <<
            "stage=active_status\n" <<
            "transaction_error=" <<
            sand_rake_control::transaction_error_string(
            active_status_result.error) << '\n' <<
            "exception_code=" <<
            static_cast<unsigned int>(active_status_result.exception_code) << '\n';
          result_code = g_stop_requested != 0 ? 130 : 6;
        } else {
          std::cout <<
            "active_status_latency_us=" <<
            active_status_result.latency.count() << '\n';
          print_full_status_values(active_status_result.values);
          if (active_status_result.values.at(8) != 0x0001 ||
            active_status_result.values.at(10) != 0x0000)
          {
            std::cerr <<
              "stage=active_status\n" <<
              "validation=UNSAFE_STATUS\n";
            result_code = 6;
          }
        }
      }

      const auto remaining_after_status = std::chrono::duration_cast<
        std::chrono::milliseconds>(jog_deadline - std::chrono::steady_clock::now());
      if (remaining_after_status.count() > 0 &&
        !interruptible_delay(static_cast<int>(remaining_after_status.count())))
      {
        std::cerr << "stage=jog_delay\nstop_requested=YES\n";
        result_code = 130;
      }
    }
  }

  const bool final_coast_ok =
    send_and_report(transport, "final_coast", coast_frame);
  if (!final_coast_ok) {
    std::cerr << "final_coast_result=FAILED\n";
    result_code = 7;
  } else {
    std::cout << "final_coast_result=OK\n";
  }

  if (result_code == 0) {
    std::cout << "result=OK\n";
  } else {
    std::cerr << "result=ERROR\n";
  }
  return result_code;
}
