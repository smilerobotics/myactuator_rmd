/**
 * \file actuator_constants.hpp
 * \mainpage
 *    Contains technical specifications for the different actuators
 *    In 2024 the actuators were re-branded, see https://www.myactuator.com/downloads-x-series
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__ACTUATOR_CONSTANTS
#define MYACTUATOR_RMD__ACTUATOR_CONSTANTS
#pragma once

#include <cstdint>


namespace myactuator_rmd {

    ///<k-teck MF-series actuators

    /**\class MF7015V2T23
   * \brief
     *    Constants for the k-teck MF actuator
     *    See http://en.lkmotor.cn/ProDetail.aspx?ProId=249
  */
  class MF7015V2T23 {
    public:
      static constexpr float reducer_ratio {1};
      static constexpr float no_load_speed {805}; // in rpm
      static constexpr float rated_speed {580}; // in rpm
      static constexpr float rated_current {3.62}; // in A
      static constexpr float rated_power {75}; // in W
      static constexpr float rated_torque {1}; // in Nm
      static constexpr float speed_constant {24.2}; // in rpm/V
      static constexpr float torque_constant {0.28}; // (manufacturer: 0.625) in Nm/A
      static constexpr float rotor_inertia {933}; // in gcm2
      static constexpr std::int16_t number_of_pole_pairs {28};
      static constexpr float current_constant {0.00805}; //(|-16A|+|16A|) / (|-2048|+|2048|) = 0.00805A/LSB
  };

}

#endif // MYACTUATOR_RMD__ACTUATOR_CONSTANTS
