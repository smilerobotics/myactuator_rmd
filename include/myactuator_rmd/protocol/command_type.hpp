/**
 * \file command_type.hpp
 * \mainpage
 *    Contains enum for all supported command types
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__PROTOCOL__COMMAND_TYPE
#define MYACTUATOR_RMD__PROTOCOL__COMMAND_TYPE
#pragma once

#include <cstdint>


namespace myactuator_rmd {

  /**\enum CommandType
   * \brief
   *    Strongly typed enum for all supported command types
  */
  enum class CommandType: std::uint8_t {
    READ_PID_PARAMETERS = 0x30,
    WRITE_PID_PARAMETERS_TO_RAM = 0x31,
    WRITE_PID_PARAMETERS_TO_ROM = 0x32,
    READ_ACCELERATION = 0x33,
    WRITE_ACCELERATION_TO_RAM = 0x34,
    WRITE_ENCODER_MULTI_TURN_VALUE_TO_ROM_AS_ZERO = 0x91,
    WRITE_CURRENT_MULTI_TURN_POSITION_TO_ROM_AS_ZERO = 0x19,
    READ_SINGLE_TURN_ENCODER = 0x90,
    READ_MULTI_TURN_ANGLE = 0x92,
    READ_SINGLE_TURN_ANGLE = 0x94,
    READ_MOTOR_STATUS_1_AND_ERROR_FLAG = 0x9A,
    READ_MOTOR_STATUS_2 = 0x9C,
    READ_MOTOR_STATUS_3 = 0x9D,
    SHUTDOWN_MOTOR = 0x80,
    STOP_MOTOR = 0x81,
    TORQUE_CLOSED_LOOP_CONTROL = 0xA1,
    SPEED_CLOSED_LOOP_CONTROL = 0xA2,
    ABSOLUTE_POSITION_CLOSED_LOOP_CONTROL = 0xA4,
    // SINGLE_TURN_POSITION_CONTROL = 0xA6,
    // INCREMENTAL_POSITION_CLOSED_LOOP_CONTROL = 0xA8,
  };

  // Symmetric comparison operators
  constexpr bool operator == (CommandType const& c, std::uint8_t const i) noexcept {
    return i == static_cast<std::uint8_t>(c);
  }
  constexpr bool operator == (std::uint8_t const i, CommandType const& c) noexcept {
    return operator == (c, i);
  }

  constexpr bool operator != (CommandType const& c, std::uint8_t const i) noexcept {
    return !(operator == (c, i));
  }
  constexpr bool operator != (std::uint8_t const i, CommandType const& c) noexcept {
    return operator != (c, i);
  }

}

#endif // MYACTUATOR_RMD__PROTOCOL__COMMAND_TYPE
