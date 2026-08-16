#include "sand_rake_control/modbus_rtu.hpp"

#include <stdexcept>

namespace sand_rake_control
{
namespace modbus_rtu
{

namespace
{
constexpr std::uint16_t kCrcInitialValue = 0xFFFF;
constexpr std::uint16_t kCrcPolynomial = 0xA001;
constexpr std::uint8_t kExceptionFunctionMask = 0x80;
constexpr std::size_t kRequestLength = 8;
constexpr std::size_t kExceptionResponseLength = 5;
constexpr std::size_t kReadResponseOverhead = 5;

void validate_slave_address(std::uint8_t slave_address)
{
  if (slave_address == 0 || slave_address > 247) {
    throw std::invalid_argument("Modbus slave address must be in [1, 247]");
  }
}

void append_u16_big_endian(Frame & frame, std::uint16_t value)
{
  frame.push_back(static_cast<std::uint8_t>(value >> 8));
  frame.push_back(static_cast<std::uint8_t>(value & 0xFF));
}

std::uint16_t read_u16_big_endian(
  const Frame & frame, std::size_t offset) noexcept
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(frame[offset]) << 8) |
    static_cast<std::uint16_t>(frame[offset + 1]));
}

void append_crc(Frame & frame)
{
  const std::uint16_t crc = crc16_modbus(frame);
  frame.push_back(static_cast<std::uint8_t>(crc & 0xFF));
  frame.push_back(static_cast<std::uint8_t>(crc >> 8));
}

bool has_valid_crc(const Frame & frame) noexcept
{
  if (frame.size() < 2) {
    return false;
  }

  const std::size_t payload_length = frame.size() - 2;
  const std::uint16_t received_crc = static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(frame[payload_length]) |
    (static_cast<std::uint16_t>(frame[payload_length + 1]) << 8));
  return crc16_modbus(frame.data(), payload_length) == received_crc;
}

ValidationResult validate_response_prefix(
  const Frame & response,
  std::uint8_t expected_slave_address,
  std::uint8_t expected_function)
{
  if (!has_valid_crc(response)) {
    return {ProtocolError::kInvalidCrc, 0};
  }
  if (response[0] != expected_slave_address) {
    return {ProtocolError::kUnexpectedSlaveAddress, 0};
  }
  if (response[1] ==
    static_cast<std::uint8_t>(expected_function | kExceptionFunctionMask))
  {
    if (response.size() != kExceptionResponseLength) {
      return {ProtocolError::kInvalidLength, 0};
    }
    return {ProtocolError::kExceptionResponse, response[2]};
  }
  if (response[1] != expected_function) {
    return {ProtocolError::kUnexpectedFunction, 0};
  }
  return {};
}

Frame build_request(
  std::uint8_t slave_address,
  std::uint8_t function,
  std::uint16_t first_value,
  std::uint16_t second_value)
{
  validate_slave_address(slave_address);

  Frame request;
  request.reserve(kRequestLength);
  request.push_back(slave_address);
  request.push_back(function);
  append_u16_big_endian(request, first_value);
  append_u16_big_endian(request, second_value);
  append_crc(request);
  return request;
}
}  // namespace

std::uint16_t crc16_modbus(
  const std::uint8_t * data, std::size_t length) noexcept
{
  std::uint16_t crc = kCrcInitialValue;
  if (data == nullptr) {
    return crc;
  }

  for (std::size_t byte_index = 0; byte_index < length; ++byte_index) {
    crc ^= data[byte_index];
    for (int bit = 0; bit < 8; ++bit) {
      const bool least_significant_bit_set = (crc & 0x0001) != 0;
      crc >>= 1;
      if (least_significant_bit_set) {
        crc ^= kCrcPolynomial;
      }
    }
  }
  return crc;
}

std::uint16_t crc16_modbus(const Frame & data) noexcept
{
  return crc16_modbus(data.data(), data.size());
}

Frame build_read_holding_registers_request(
  std::uint8_t slave_address,
  std::uint16_t start_register,
  std::uint16_t register_count)
{
  if (register_count == 0 || register_count > kMaxReadRegisterCount) {
    throw std::invalid_argument("Modbus 0x03 register count must be in [1, 125]");
  }
  return build_request(
    slave_address, kReadHoldingRegisters, start_register, register_count);
}

ReadHoldingRegistersResult parse_read_holding_registers_response(
  const Frame & response,
  std::uint8_t expected_slave_address,
  std::uint16_t expected_register_count)
{
  if (response.size() < kExceptionResponseLength) {
    return {{ProtocolError::kFrameTooShort, 0}, {}};
  }

  const auto prefix = validate_response_prefix(
    response, expected_slave_address, kReadHoldingRegisters);
  if (!prefix) {
    return {{prefix.error, prefix.exception_code}, {}};
  }

  const std::size_t byte_count = response[2];
  if (response.size() != byte_count + kReadResponseOverhead) {
    return {{ProtocolError::kInvalidLength, 0}, {}};
  }
  if (byte_count == 0 || (byte_count % 2) != 0) {
    return {{ProtocolError::kInvalidByteCount, 0}, {}};
  }
  if (expected_register_count == 0 ||
    expected_register_count > kMaxReadRegisterCount ||
    byte_count != static_cast<std::size_t>(expected_register_count) * 2)
  {
    return {{ProtocolError::kUnexpectedRegisterCount, 0}, {}};
  }

  ReadHoldingRegistersResult result;
  result.values.reserve(expected_register_count);
  for (std::size_t offset = 3; offset < 3 + byte_count; offset += 2) {
    result.values.push_back(read_u16_big_endian(response, offset));
  }
  return result;
}

Frame build_write_single_register_request(
  std::uint8_t slave_address,
  std::uint16_t register_address,
  std::uint16_t value)
{
  return build_request(
    slave_address, kWriteSingleRegister, register_address, value);
}

std::string_view protocol_error_string(ProtocolError error) noexcept
{
  switch (error) {
    case ProtocolError::kNone:
      return "none";
    case ProtocolError::kFrameTooShort:
      return "frame too short";
    case ProtocolError::kInvalidLength:
      return "invalid frame length";
    case ProtocolError::kInvalidCrc:
      return "CRC mismatch";
    case ProtocolError::kUnexpectedSlaveAddress:
      return "unexpected slave address";
    case ProtocolError::kUnexpectedFunction:
      return "unexpected function code";
    case ProtocolError::kInvalidByteCount:
      return "invalid byte count";
    case ProtocolError::kUnexpectedRegisterCount:
      return "unexpected register count";
    case ProtocolError::kExceptionResponse:
      return "Modbus exception response";
  }
  return "unknown protocol error";
}

}  // namespace modbus_rtu
}  // namespace sand_rake_control
