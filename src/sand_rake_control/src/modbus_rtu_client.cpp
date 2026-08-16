#include "sand_rake_control/modbus_rtu_client.hpp"

#include <stdexcept>
#include <utility>

namespace sand_rake_control
{

namespace
{
constexpr std::uint8_t kExceptionFunctionMask = 0x80;

class TransactionGuard
{
public:
  explicit TransactionGuard(std::atomic_flag & flag)
  : flag_(flag)
  {
  }

  ~TransactionGuard()
  {
    flag_.clear();
  }

  TransactionGuard(const TransactionGuard &) = delete;
  TransactionGuard & operator=(const TransactionGuard &) = delete;

private:
  std::atomic_flag & flag_;
};

std::chrono::microseconds elapsed_since(
  std::chrono::steady_clock::time_point started_at)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - started_at);
}
}  // namespace

ModbusRtuClient::ModbusRtuClient(
  Rs232Transport & transport,
  std::chrono::milliseconds response_timeout)
: transport_(transport),
  response_timeout_(response_timeout)
{
  if (response_timeout_.count() <= 0) {
    throw std::invalid_argument("response_timeout must be positive");
  }
}

TransactionResult ModbusRtuClient::read_holding_registers(
  std::uint8_t slave_address,
  std::uint16_t start_register,
  std::uint16_t register_count)
{
  if (transaction_in_progress_.test_and_set()) {
    return finish_busy();
  }
  TransactionGuard guard(transaction_in_progress_);
  const auto started_at = std::chrono::steady_clock::now();
  const auto deadline = started_at + response_timeout_;

  modbus_rtu::Frame request;
  try {
    request = modbus_rtu::build_read_holding_registers_request(
      slave_address, start_register, register_count);
  } catch (const std::invalid_argument &) {
    return finish_protocol_error(
      modbus_rtu::ProtocolError::kUnexpectedRegisterCount, 0, started_at);
  }

  const auto discard_result = transport_.discard_input();
  if (!discard_result) {
    return finish_transport_error(discard_result, started_at);
  }
  const auto write_result =
    transport_.write_all(request.data(), request.size(), deadline);
  if (!write_result) {
    return finish_transport_error(write_result, started_at);
  }

  auto frame_result = read_response_frame(
    modbus_rtu::kReadHoldingRegisters, deadline);
  if (!frame_result.transport_result) {
    return finish_transport_error(frame_result.transport_result, started_at);
  }
  if (frame_result.framing_error) {
    return finish_protocol_error(
      modbus_rtu::ProtocolError::kUnexpectedFunction, 0, started_at);
  }

  const auto parsed = modbus_rtu::parse_read_holding_registers_response(
    frame_result.frame, slave_address, register_count);
  if (!parsed) {
    return finish_protocol_error(
      parsed.error, parsed.exception_code, started_at);
  }
  return finish_success(parsed.values, started_at);
}

ModbusRtuClient::FrameReadResult ModbusRtuClient::read_response_frame(
  std::uint8_t expected_function,
  Rs232Transport::Deadline deadline)
{
  FrameReadResult result;
  result.frame.resize(2);
  result.transport_result =
    transport_.read_exact(result.frame.data(), result.frame.size(), deadline);
  if (!result.transport_result) {
    return result;
  }

  const std::uint8_t actual_function = result.frame[1];
  std::size_t remaining_length = 0;
  if (actual_function ==
    static_cast<std::uint8_t>(expected_function | kExceptionFunctionMask))
  {
    remaining_length = 3;
  } else if (actual_function == modbus_rtu::kReadHoldingRegisters) {
    result.frame.resize(3);
    result.transport_result =
      transport_.read_exact(result.frame.data() + 2, 1, deadline);
    if (!result.transport_result) {
      return result;
    }
    remaining_length = static_cast<std::size_t>(result.frame[2]) + 2;
  } else {
    result.framing_error = true;
    transport_.discard_input();
    return result;
  }

  const std::size_t previous_size = result.frame.size();
  result.frame.resize(previous_size + remaining_length);
  result.transport_result = transport_.read_exact(
    result.frame.data() + previous_size, remaining_length, deadline);
  return result;
}

TransactionResult ModbusRtuClient::finish_transport_error(
  const TransportResult & transport_result,
  std::chrono::steady_clock::time_point started_at)
{
  const auto latency = elapsed_since(started_at);
  const TransactionError error =
    transport_result.error == TransportError::kTimeout ?
    TransactionError::kTimeout : TransactionError::kTransport;
  TransactionResult result;
  result.error = error;
  result.transport_error = transport_result.error;
  result.latency = latency;
  return result;
}

TransactionResult ModbusRtuClient::finish_protocol_error(
  modbus_rtu::ProtocolError protocol_error,
  std::uint8_t exception_code,
  std::chrono::steady_clock::time_point started_at)
{
  const auto latency = elapsed_since(started_at);
  TransactionError error = TransactionError::kProtocol;
  if (protocol_error == modbus_rtu::ProtocolError::kInvalidCrc) {
    error = TransactionError::kInvalidCrc;
  } else if (protocol_error == modbus_rtu::ProtocolError::kExceptionResponse) {
    error = TransactionError::kExceptionResponse;
  }
  TransactionResult result;
  result.error = error;
  result.protocol_error = protocol_error;
  result.exception_code = exception_code;
  result.latency = latency;
  return result;
}

TransactionResult ModbusRtuClient::finish_success(
  std::vector<std::uint16_t> values,
  std::chrono::steady_clock::time_point started_at)
{
  const auto latency = elapsed_since(started_at);
  TransactionResult result;
  result.values = std::move(values);
  result.latency = latency;
  return result;
}

TransactionResult ModbusRtuClient::finish_busy()
{
  TransactionResult result;
  result.error = TransactionError::kBusy;
  return result;
}

const char * transaction_error_string(TransactionError error) noexcept
{
  switch (error) {
    case TransactionError::kNone:
      return "none";
    case TransactionError::kBusy:
      return "transaction already in progress";
    case TransactionError::kTimeout:
      return "transaction timeout";
    case TransactionError::kTransport:
      return "transport error";
    case TransactionError::kInvalidCrc:
      return "invalid CRC";
    case TransactionError::kProtocol:
      return "protocol error";
    case TransactionError::kExceptionResponse:
      return "Modbus exception response";
  }
  return "unknown transaction error";
}

}  // namespace sand_rake_control
