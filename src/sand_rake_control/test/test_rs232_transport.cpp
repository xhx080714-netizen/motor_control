#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include "sand_rake_control/modbus_rtu_client.hpp"
#include "sand_rake_control/rs232_transport.hpp"

using sand_rake_control::Rs232Config;
using sand_rake_control::Rs232Transport;
using sand_rake_control::TransportError;

namespace
{
class PseudoTerminal
{
public:
  PseudoTerminal()
  : master_(::posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC))
  {
    if (master_ < 0 || ::grantpt(master_) != 0 || ::unlockpt(master_) != 0) {
      return;
    }
    const char * const name = ::ptsname(master_);
    if (name != nullptr) {
      slave_name_ = name;
    }
  }

  ~PseudoTerminal()
  {
    if (master_ >= 0) {
      ::close(master_);
    }
  }

  bool valid() const noexcept
  {
    return master_ >= 0 && !slave_name_.empty();
  }

  int master() const noexcept
  {
    return master_;
  }

  const std::string & slave_name() const noexcept
  {
    return slave_name_;
  }

private:
  int master_{-1};
  std::string slave_name_;
};

Rs232Config config_for(const PseudoTerminal & terminal)
{
  Rs232Config config;
  config.device = terminal.slave_name();
  return config;
}
}  // namespace

TEST(Rs232Transport, RejectsEmptyDevice)
{
  Rs232Transport transport;
  Rs232Config config;
  const auto result = transport.open(config);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, TransportError::kInvalidConfig);
}

TEST(Rs232Transport, ReportsNotOpen)
{
  Rs232Transport transport;
  std::uint8_t byte{0};
  const auto deadline = Rs232Transport::Clock::now() + std::chrono::milliseconds(10);
  const auto result = transport.read_exact(&byte, 1, deadline);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, TransportError::kNotOpen);
}

TEST(Rs232Transport, DiscardRequiresOpenTransport)
{
  Rs232Transport transport;
  const auto result = transport.discard_input();
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, TransportError::kNotOpen);
}

TEST(Rs232Transport, RejectsUnsupportedSerialSettings)
{
  const std::vector<Rs232Config> invalid_configs{
    Rs232Config{"unused", 12345, 8, sand_rake_control::SerialParity::kNone, 1},
    Rs232Config{"unused", 115200, 7, sand_rake_control::SerialParity::kNone, 1},
    Rs232Config{"unused", 115200, 8, sand_rake_control::SerialParity::kEven, 1},
    Rs232Config{"unused", 115200, 8, sand_rake_control::SerialParity::kNone, 2},
  };

  for (const auto & config : invalid_configs) {
    Rs232Transport transport;
    const auto result = transport.open(config);
    EXPECT_FALSE(result);
    EXPECT_EQ(result.error, TransportError::kInvalidConfig);
  }
}

TEST(Rs232Transport, ReportsMissingDevice)
{
  Rs232Transport transport;
  Rs232Config config;
  config.device = "/dev/sand_rake_test_device_that_does_not_exist";
  const auto result = transport.open(config);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, TransportError::kOpenFailed);
}

TEST(Rs232Transport, OpensPseudoTerminalAndRetainsConfig)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  const auto config = config_for(terminal);
  Rs232Transport transport;

  ASSERT_TRUE(transport.open(config));
  EXPECT_TRUE(transport.is_open());
  EXPECT_EQ(transport.config().device, config.device);
  EXPECT_EQ(transport.config().baudrate, 115200);

  transport.close();
  EXPECT_FALSE(transport.is_open());
}

TEST(Rs232Transport, ReadsExactBytesFromPseudoTerminal)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  Rs232Transport transport;
  ASSERT_TRUE(transport.open(config_for(terminal)));
  const std::uint8_t sent[]{0x01, 0x03, 0x00, 0x00};
  ASSERT_EQ(::write(terminal.master(), sent, sizeof(sent)), static_cast<ssize_t>(sizeof(sent)));

  std::uint8_t received[sizeof(sent)]{};
  const auto result = transport.read_exact(
    received, sizeof(received), Rs232Transport::Clock::now() + std::chrono::milliseconds(100));

  ASSERT_TRUE(result);
  EXPECT_EQ(result.bytes_transferred, sizeof(received));
  EXPECT_EQ(
    std::vector<std::uint8_t>(received, received + sizeof(received)),
    std::vector<std::uint8_t>(sent, sent + sizeof(sent)));
}

TEST(Rs232Transport, WritesAllBytesToPseudoTerminal)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  Rs232Transport transport;
  ASSERT_TRUE(transport.open(config_for(terminal)));
  const std::uint8_t sent[]{0x01, 0x10, 0x00, 0x02, 0x00, 0x02};

  const auto result = transport.write_all(
    sent, sizeof(sent), Rs232Transport::Clock::now() + std::chrono::milliseconds(100));
  ASSERT_TRUE(result);
  EXPECT_EQ(result.bytes_transferred, sizeof(sent));

  std::uint8_t received[sizeof(sent)]{};
  ASSERT_EQ(
    ::read(terminal.master(), received, sizeof(received)),
    static_cast<ssize_t>(sizeof(received)));
  EXPECT_EQ(
    std::vector<std::uint8_t>(received, received + sizeof(received)),
    std::vector<std::uint8_t>(sent, sent + sizeof(sent)));
}

TEST(Rs232Transport, ReadTimeoutReportsPartialProgress)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  Rs232Transport transport;
  ASSERT_TRUE(transport.open(config_for(terminal)));
  const std::uint8_t partial[]{0x01, 0x03};
  ASSERT_EQ(
    ::write(terminal.master(), partial, sizeof(partial)),
    static_cast<ssize_t>(sizeof(partial)));

  std::uint8_t received[4]{};
  const auto result = transport.read_exact(
    received, sizeof(received), Rs232Transport::Clock::now() + std::chrono::milliseconds(20));

  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, TransportError::kTimeout);
  EXPECT_EQ(result.bytes_transferred, sizeof(partial));
}

TEST(Rs232Transport, DiscardInputRemovesPendingBytes)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  Rs232Transport transport;
  ASSERT_TRUE(transport.open(config_for(terminal)));
  const std::uint8_t stale[]{0xAA, 0x55};
  ASSERT_EQ(::write(terminal.master(), stale, sizeof(stale)), static_cast<ssize_t>(sizeof(stale)));
  ASSERT_TRUE(transport.discard_input());

  std::uint8_t received{};
  const auto result = transport.read_exact(
    &received, 1, Rs232Transport::Clock::now() + std::chrono::milliseconds(20));
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, TransportError::kTimeout);
  EXPECT_EQ(result.bytes_transferred, 0U);
}
