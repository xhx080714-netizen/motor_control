#ifndef SAND_RAKE_CONTROL__MODBUS_RTU_CLIENT_HPP_
#define SAND_RAKE_CONTROL__MODBUS_RTU_CLIENT_HPP_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "sand_rake_control/modbus_rtu.hpp"
#include "sand_rake_control/rs232_transport.hpp"

namespace sand_rake_control
{

enum class TransactionError
{
  kNone,
  kBusy,
  kTimeout,
  kTransport,
  kInvalidCrc,
  kProtocol,
  kExceptionResponse,
};

struct TransactionResult
{
  TransactionError error{TransactionError::kNone};
  TransportError transport_error{TransportError::kNone};
  modbus_rtu::ProtocolError protocol_error{modbus_rtu::ProtocolError::kNone};
  std::uint8_t exception_code{0};
  std::vector<std::uint16_t> values;
  std::chrono::microseconds latency{0};

  explicit operator bool() const noexcept
  {
    return error == TransactionError::kNone;
  }
};

class ModbusRtuClient
{
public:
  ModbusRtuClient(
    Rs232Transport & transport,
    std::chrono::milliseconds response_timeout);

  TransactionResult read_holding_registers(
    std::uint8_t slave_address,
    std::uint16_t start_register,
    std::uint16_t register_count);

private:
  struct FrameReadResult
  {
    TransportResult transport_result;
    modbus_rtu::Frame frame;
    bool framing_error{false};
  };

  FrameReadResult read_response_frame(
    std::uint8_t expected_function,
    Rs232Transport::Deadline deadline);

  TransactionResult finish_transport_error(
    const TransportResult & error,
    std::chrono::steady_clock::time_point started_at);
  TransactionResult finish_protocol_error(
    modbus_rtu::ProtocolError error,
    std::uint8_t exception_code,
    std::chrono::steady_clock::time_point started_at);
  TransactionResult finish_success(
    std::vector<std::uint16_t> values,
    std::chrono::steady_clock::time_point started_at);
  TransactionResult finish_busy();

  Rs232Transport & transport_;
  std::chrono::milliseconds response_timeout_;
  std::atomic_flag transaction_in_progress_ = ATOMIC_FLAG_INIT;
};

const char * transaction_error_string(TransactionError error) noexcept;

}  // namespace sand_rake_control

#endif  // SAND_RAKE_CONTROL__MODBUS_RTU_CLIENT_HPP_
