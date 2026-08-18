#include "sand_rake_control/rs232_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdint>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace sand_rake_control
{

namespace
{
bool baudrate_to_speed(int baudrate, speed_t & speed) noexcept
{
  switch (baudrate) {
    case 9600:
      speed = B9600;
      return true;
    case 19200:
      speed = B19200;
      return true;
    case 38400:
      speed = B38400;
      return true;
    case 57600:
      speed = B57600;
      return true;
    case 115200:
      speed = B115200;
      return true;
    default:
      return false;
  }
}

TransportResult configure_port(int file_descriptor, const Rs232Config & config)
{
  speed_t speed{};
  if (config.device.empty() || !baudrate_to_speed(config.baudrate, speed) ||
    config.data_bits != 8 || config.parity != SerialParity::kNone ||
    config.stop_bits != 1)
  {
    return {TransportError::kInvalidConfig, 0, EINVAL};
  }

  termios options{};
  if (tcgetattr(file_descriptor, &options) != 0) {
    return {TransportError::kConfigureFailed, 0, errno};
  }

  cfmakeraw(&options);
  options.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
  options.c_cflag |= CS8 | CLOCAL | CREAD;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (cfsetispeed(&options, speed) != 0 ||
    cfsetospeed(&options, speed) != 0 ||
    tcsetattr(file_descriptor, TCSANOW, &options) != 0)
  {
    return {TransportError::kConfigureFailed, 0, errno};
  }
  if (tcflush(file_descriptor, TCIOFLUSH) != 0) {
    return {TransportError::kConfigureFailed, 0, errno};
  }
  return {};
}
}  // namespace

Rs232Transport::~Rs232Transport()
{
  close();
}

TransportResult Rs232Transport::open(const Rs232Config & config)
{
  close();

  speed_t unused_speed{};
  if (config.device.empty() || !baudrate_to_speed(config.baudrate, unused_speed) ||
    config.data_bits != 8 || config.parity != SerialParity::kNone ||
    config.stop_bits != 1)
  {
    return {TransportError::kInvalidConfig, 0, EINVAL};
  }

  const int file_descriptor =
    ::open(config.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (file_descriptor < 0) {
    return {TransportError::kOpenFailed, 0, errno};
  }

  const auto configure_result = configure_port(file_descriptor, config);
  if (!configure_result) {
    ::close(file_descriptor);
    return configure_result;
  }

  file_descriptor_ = file_descriptor;
  config_ = config;
  return {};
}

void Rs232Transport::close() noexcept
{
  if (file_descriptor_ >= 0) {
    ::close(file_descriptor_);
    file_descriptor_ = -1;
  }
}

bool Rs232Transport::is_open() const noexcept
{
  return file_descriptor_ >= 0;
}

const Rs232Config & Rs232Transport::config() const noexcept
{
  return config_;
}

TransportResult Rs232Transport::wait_for(short events, Deadline deadline)
{
  if (!is_open()) {
    return {TransportError::kNotOpen, 0, EBADF};
  }

  while (true) {
    const auto now = Clock::now();
    if (now >= deadline) {
      return {TransportError::kTimeout, 0, ETIMEDOUT};
    }
    const auto remaining_us =
      std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count();
    const auto rounded_ms = (remaining_us + 999) / 1000;
    const int timeout_ms = static_cast<int>(
      std::min<std::int64_t>(rounded_ms, INT_MAX));

    pollfd descriptor{};
    descriptor.fd = file_descriptor_;
    descriptor.events = events;
    const int poll_result = ::poll(&descriptor, 1, timeout_ms);
    if (poll_result == 0) {
      return {TransportError::kTimeout, 0, ETIMEDOUT};
    }
    if (poll_result < 0) {
      if (errno == EINTR) {
        return {TransportError::kInterrupted, 0, EINTR};
      }
      return {TransportError::kPollFailed, 0, errno};
    }
    if ((descriptor.revents & events) != 0) {
      return {};
    }
    if ((descriptor.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
      return {TransportError::kDisconnected, 0, EIO};
    }
  }
}

TransportResult Rs232Transport::write_all(
  const std::uint8_t * data, std::size_t length, Deadline deadline)
{
  if (length > 0 && data == nullptr) {
    return {TransportError::kInvalidConfig, 0, EINVAL};
  }

  std::size_t total_written = 0;
  while (total_written < length) {
    const auto wait_result = wait_for(POLLOUT, deadline);
    if (!wait_result) {
      return {wait_result.error, total_written, wait_result.system_error};
    }

    const ssize_t written = ::write(
      file_descriptor_, data + total_written, length - total_written);
    if (written > 0) {
      total_written += static_cast<std::size_t>(written);
      continue;
    }
    if (written < 0 && errno == EINTR) {
      continue;
    }
    if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      continue;
    }
    return {
      written == 0 ? TransportError::kDisconnected : TransportError::kIoError,
      total_written,
      written == 0 ? EIO : errno};
  }
  return {TransportError::kNone, total_written, 0};
}

TransportResult Rs232Transport::read_exact(
  std::uint8_t * data, std::size_t length, Deadline deadline)
{
  if (length > 0 && data == nullptr) {
    return {TransportError::kInvalidConfig, 0, EINVAL};
  }

  std::size_t total_read = 0;
  while (total_read < length) {
    const auto wait_result = wait_for(POLLIN, deadline);
    if (!wait_result) {
      return {wait_result.error, total_read, wait_result.system_error};
    }

    const ssize_t bytes_read =
      ::read(file_descriptor_, data + total_read, length - total_read);
    if (bytes_read > 0) {
      total_read += static_cast<std::size_t>(bytes_read);
      continue;
    }
    if (bytes_read < 0 && errno == EINTR) {
      continue;
    }
    if (bytes_read < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      continue;
    }
    return {
      bytes_read == 0 ? TransportError::kDisconnected : TransportError::kIoError,
      total_read,
      bytes_read == 0 ? EIO : errno};
  }
  return {TransportError::kNone, total_read, 0};
}

TransportResult Rs232Transport::discard_input()
{
  if (!is_open()) {
    return {TransportError::kNotOpen, 0, EBADF};
  }
  if (tcflush(file_descriptor_, TCIFLUSH) != 0) {
    return {TransportError::kIoError, 0, errno};
  }
  return {};
}

const char * transport_error_string(TransportError error) noexcept
{
  switch (error) {
    case TransportError::kNone:
      return "none";
    case TransportError::kNotOpen:
      return "transport not open";
    case TransportError::kInvalidConfig:
      return "invalid serial configuration";
    case TransportError::kOpenFailed:
      return "open failed";
    case TransportError::kConfigureFailed:
      return "termios configuration failed";
    case TransportError::kTimeout:
      return "deadline timeout";
    case TransportError::kInterrupted:
      return "operation interrupted";
    case TransportError::kPollFailed:
      return "poll failed";
    case TransportError::kIoError:
      return "I/O error";
    case TransportError::kDisconnected:
      return "transport disconnected";
  }
  return "unknown transport error";
}

}  // namespace sand_rake_control
