#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "sand_rake_control/modbus_rtu.hpp"

namespace modbus = sand_rake_control::modbus_rtu;

namespace
{
modbus::Frame with_crc(modbus::Frame payload)
{
  const auto crc = modbus::crc16_modbus(payload);
  payload.push_back(static_cast<std::uint8_t>(crc & 0xFFU));
  payload.push_back(static_cast<std::uint8_t>((crc >> 8U) & 0xFFU));
  return payload;
}
}  // namespace

TEST(ModbusRtu, BuildsSmallCarModeReadFrame)
{
  const modbus::Frame expected{0x01, 0x03, 0x00, 0x08, 0x00, 0x01, 0x05, 0xC8};
  EXPECT_EQ(
    modbus::build_read_holding_registers_request(0x01, 0x0008, 1),
    expected);
}

TEST(ModbusRtu, BuildsFullSmallCarStatusReadFrame)
{
  const modbus::Frame expected{0x01, 0x03, 0x00, 0x00, 0x00, 0x0C, 0x45, 0xCF};
  EXPECT_EQ(
    modbus::build_read_holding_registers_request(0x01, 0x0000, 12),
    expected);
}

TEST(ModbusRtu, MatchesKnownCrcVector)
{
  const modbus::Frame payload{0x01, 0x03, 0x00, 0x08, 0x00, 0x01};
  EXPECT_EQ(modbus::crc16_modbus(payload), 0xC805U);
}

TEST(ModbusRtu, RejectsInvalidRequestBounds)
{
  EXPECT_THROW(
    modbus::build_read_holding_registers_request(0x00, 0x0000, 1),
    std::invalid_argument);
  EXPECT_THROW(
    modbus::build_read_holding_registers_request(0xF8, 0x0000, 1),
    std::invalid_argument);
  EXPECT_THROW(
    modbus::build_read_holding_registers_request(0x01, 0x0000, 0),
    std::invalid_argument);
  EXPECT_THROW(
    modbus::build_read_holding_registers_request(0x01, 0x0000, 126),
    std::invalid_argument);
  EXPECT_NO_THROW(
    modbus::build_read_holding_registers_request(0x01, 0x0000, 125));
}

TEST(ModbusRtu, BuildsCompanyTableInitializationFrames)
{
  EXPECT_EQ(
    modbus::build_write_single_register_request(0x01, 0x0009, 0x0001),
    (modbus::Frame{0x01, 0x06, 0x00, 0x09, 0x00, 0x01, 0x98, 0x08}));
  EXPECT_EQ(
    modbus::build_write_single_register_request(0x01, 0x0007, 0x0001),
    (modbus::Frame{0x01, 0x06, 0x00, 0x07, 0x00, 0x01, 0xF9, 0xCB}));
  EXPECT_EQ(
    modbus::build_write_single_register_request(0x01, 0x000F, 0x0001),
    (modbus::Frame{0x01, 0x06, 0x00, 0x0F, 0x00, 0x01, 0x78, 0x09}));
  EXPECT_EQ(
    modbus::build_write_single_register_request(0x01, 0x0005, 0x0000),
    (modbus::Frame{0x01, 0x06, 0x00, 0x05, 0x00, 0x00, 0x99, 0xCB}));
  EXPECT_EQ(
    modbus::build_write_single_register_request(0x01, 0x0006, 0x0000),
    (modbus::Frame{0x01, 0x06, 0x00, 0x06, 0x00, 0x00, 0x69, 0xCB}));
}

TEST(ModbusRtu, ParsesReadResponse)
{
  const modbus::Frame response{0x01, 0x03, 0x02, 0x01, 0xF4, 0xB8, 0x53};
  const auto result =
    modbus::parse_read_holding_registers_response(response, 0x01, 1);
  ASSERT_TRUE(result);
  ASSERT_EQ(result.values.size(), 1U);
  EXPECT_EQ(result.values.front(), 500U);
}

TEST(ModbusRtu, ParsesFullSmallCarStatusResponse)
{
  const auto response = with_crc(
  {
    0x01, 0x03, 0x18,
    0x01, 0xE1, 0x00, 0x00,
    0x00, 0x00, 0xFC, 0x55,
    0x00, 0x00, 0x3A, 0x5B,
    0x01, 0xF4, 0x00, 0x64,
    0x00, 0x01, 0x00, 0xEE,
    0x00, 0x00, 0x01, 0x0F});
  const auto result =
    modbus::parse_read_holding_registers_response(response, 0x01, 12);
  ASSERT_TRUE(result);
  const std::vector<std::uint16_t> expected{
    0x01E1, 0x0000, 0x0000, 0xFC55, 0x0000, 0x3A5B,
    0x01F4, 0x0064, 0x0001, 0x00EE, 0x0000, 0x010F};
  EXPECT_EQ(result.values, expected);
}

TEST(ModbusRtu, RejectsBadCrc)
{
  const modbus::Frame response{0x01, 0x03, 0x02, 0x01, 0xF4, 0xB8, 0x54};
  const auto result =
    modbus::parse_read_holding_registers_response(response, 0x01, 1);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, modbus::ProtocolError::kInvalidCrc);
}

TEST(ModbusRtu, ClassifiesMalformedResponses)
{
  auto result = modbus::parse_read_holding_registers_response(
    {0x01, 0x03, 0x00, 0x00}, 0x01, 1);
  EXPECT_EQ(result.error, modbus::ProtocolError::kFrameTooShort);

  result = modbus::parse_read_holding_registers_response(
    {0x02, 0x03, 0x02, 0x00, 0x01, 0x3D, 0x84}, 0x01, 1);
  EXPECT_EQ(result.error, modbus::ProtocolError::kUnexpectedSlaveAddress);

  result = modbus::parse_read_holding_registers_response(
    {0x01, 0x04, 0x02, 0x00, 0x01, 0x78, 0xF0}, 0x01, 1);
  EXPECT_EQ(result.error, modbus::ProtocolError::kUnexpectedFunction);

  result = modbus::parse_read_holding_registers_response(
    with_crc({0x01, 0x03, 0x00}), 0x01, 1);
  EXPECT_EQ(result.error, modbus::ProtocolError::kInvalidByteCount);

  result = modbus::parse_read_holding_registers_response(
    with_crc({0x01, 0x03, 0x01, 0xAA}), 0x01, 1);
  EXPECT_EQ(result.error, modbus::ProtocolError::kInvalidByteCount);

  result = modbus::parse_read_holding_registers_response(
    with_crc({0x01, 0x03, 0x02, 0x00, 0x01, 0x00}), 0x01, 1);
  EXPECT_EQ(result.error, modbus::ProtocolError::kInvalidLength);

  result = modbus::parse_read_holding_registers_response(
    {0x01, 0x03, 0x04, 0x00, 0x01, 0x00, 0x02, 0x2A, 0x32}, 0x01, 1);
  EXPECT_EQ(result.error, modbus::ProtocolError::kUnexpectedRegisterCount);
}

TEST(ModbusRtu, ParsesExceptionResponse)
{
  const modbus::Frame response{0x01, 0x83, 0x02, 0xC0, 0xF1};
  const auto result =
    modbus::parse_read_holding_registers_response(response, 0x01, 1);
  EXPECT_EQ(result.error, modbus::ProtocolError::kExceptionResponse);
  EXPECT_EQ(result.exception_code, 0x02U);
}
