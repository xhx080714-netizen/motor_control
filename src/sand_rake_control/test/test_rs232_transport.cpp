#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>

#include "sand_rake_control/modbus_rtu_client.hpp"
#include "sand_rake_control/rs232_transport.hpp"

using sand_rake_control::Rs232Config;
using sand_rake_control::Rs232Transport;
using sand_rake_control::TransportError;

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
