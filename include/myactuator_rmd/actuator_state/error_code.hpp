/**
 * \file error_code.hpp
 * \mainpage
 *    Contains the error codes of the actuator
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__ACTUATOR_STATE__ERROR_CODE
#define MYACTUATOR_RMD__ACTUATOR_STATE__ERROR_CODE
#pragma once

#include <cstdint>


namespace myactuator_rmd {

  /**\enum ErrorCode
   * \brief
   *    Strongly typed enum for all known actuator error codes
  */
  enum class ErrorCode: std::uint8_t {
    NO_ERROR = 0x00,
    LOW_VOLTAGE = 0x01,
    OVERTEMPERATURE = 0x08
  };

}

#endif // MYACTUATOR_RMD__ACTUATOR_STATE__ERROR_CODE
