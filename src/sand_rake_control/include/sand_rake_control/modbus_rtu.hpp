#ifndef SAND_RAKE_CONTROL__MODBUS_RTU_HPP_
#define SAND_RAKE_CONTROL__MODBUS_RTU_HPP_

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

namespace sand_rake_control
{
namespace modbus_rtu
{

using Frame = std::vector<std::uint8_t>;

constexpr std::uint8_t kReadHoldingRegisters = 0x03;
constexpr std::uint8_t kWriteSingleRegister = 0x06;
constexpr std::uint16_t kMaxReadRegisterCount = 125;

enum class ProtocolError
{
  kNone,
  kFrameTooShort,
  kInvalidLength,
  kInvalidCrc,
  kUnexpectedSlaveAddress,
  kUnexpectedFunction,
  kInvalidByteCount,
  kUnexpectedRegisterCount,
  kExceptionResponse,
};

struct ValidationResult
{
  ProtocolError error{ProtocolError::kNone};
  std::uint8_t exception_code{0};

  explicit operator bool() const noexcept
  {
    return error == ProtocolError::kNone;
  }
};

struct ReadHoldingRegistersResult : ValidationResult
{
  std::vector<std::uint16_t> values;
};

std::uint16_t crc16_modbus(
  const std::uint8_t * data, std::size_t length) noexcept;

std::uint16_t crc16_modbus(const Frame & data) noexcept;

Frame build_read_holding_registers_request(
  std::uint8_t slave_address,
  std::uint16_t start_register,
  std::uint16_t register_count);

ReadHoldingRegistersResult parse_read_holding_registers_response(
  const Frame & response,
  std::uint8_t expected_slave_address,
  std::uint16_t expected_register_count);

Frame build_write_single_register_request(
  std::uint8_t slave_address,
  std::uint16_t register_address,
  std::uint16_t value);

std::string_view protocol_error_string(ProtocolError error) noexcept;

}  // namespace modbus_rtu
}  // namespace sand_rake_control

#endif  // SAND_RAKE_CONTROL__MODBUS_RTU_HPP_
