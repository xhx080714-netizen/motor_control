#ifndef SAND_RAKE_CONTROL__RS232_TRANSPORT_HPP_
#define SAND_RAKE_CONTROL__RS232_TRANSPORT_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace sand_rake_control
{

enum class SerialParity
{
  kNone,
  kEven,
  kOdd,
};

struct Rs232Config
{
  std::string device;
  int baudrate{115200};
  int data_bits{8};
  SerialParity parity{SerialParity::kNone};
  int stop_bits{1};
};

enum class TransportError
{
  kNone,
  kNotOpen,
  kInvalidConfig,
  kOpenFailed,
  kConfigureFailed,
  kTimeout,
  kPollFailed,
  kIoError,
  kDisconnected,
};

struct TransportResult
{
  TransportError error{TransportError::kNone};
  std::size_t bytes_transferred{0};
  int system_error{0};

  explicit operator bool() const noexcept
  {
    return error == TransportError::kNone;
  }
};

class Rs232Transport
{
public:
  // POSIX termios/poll transport for the confirmed JP17/JP18 RS232 links.
  // Electrical level conversion is provided by the RK3576 board hardware.
  using Clock = std::chrono::steady_clock;
  using Deadline = Clock::time_point;

  Rs232Transport() = default;
  ~Rs232Transport();

  Rs232Transport(const Rs232Transport &) = delete;
  Rs232Transport & operator=(const Rs232Transport &) = delete;
  Rs232Transport(Rs232Transport &&) = delete;
  Rs232Transport & operator=(Rs232Transport &&) = delete;

  // One instance represents the single owner of one tty. Callers must not
  // access it concurrently from multiple control/diagnostic callbacks.

  TransportResult open(const Rs232Config & config);
  void close() noexcept;
  bool is_open() const noexcept;
  const Rs232Config & config() const noexcept;

  TransportResult write_all(
    const std::uint8_t * data, std::size_t length, Deadline deadline);
  TransportResult read_exact(
    std::uint8_t * data, std::size_t length, Deadline deadline);
  TransportResult discard_input();

private:
  TransportResult wait_for(short events, Deadline deadline);

  int file_descriptor_{-1};
  Rs232Config config_{};
};

const char * transport_error_string(TransportError error) noexcept;

}  // namespace sand_rake_control

#endif  // SAND_RAKE_CONTROL__RS232_TRANSPORT_HPP_
