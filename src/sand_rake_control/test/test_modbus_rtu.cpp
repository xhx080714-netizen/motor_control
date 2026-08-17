#include <gtest/gtest.h>

#include "sand_rake_control/modbus_rtu.hpp"

namespace modbus = sand_rake_control::modbus_rtu;

TEST(ModbusRtu, BuildsSmallCarModeReadFrame)
{
  const modbus::Frame expected{0x01, 0x03, 0x00, 0x08, 0x00, 0x01, 0x05, 0xC8};
  EXPECT_EQ(
    modbus::build_read_holding_registers_request(0x01, 0x0008, 1),
    expected);
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

TEST(ModbusRtu, RejectsBadCrc)
{
  const modbus::Frame response{0x01, 0x03, 0x02, 0x01, 0xF4, 0xB8, 0x54};
  const auto result =
    modbus::parse_read_holding_registers_response(response, 0x01, 1);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, modbus::ProtocolError::kInvalidCrc);
}
