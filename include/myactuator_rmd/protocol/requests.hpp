/**
 * \file requests.hpp
 * \mainpage
 *    Contains all the requests sent from the driver to the actuator
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__PROTOCOL__REQUESTS
#define MYACTUATOR_RMD__PROTOCOL__REQUESTS
#pragma once

#include <chrono>
#include <cstdint>

#include "myactuator_rmd/actuator_state/acceleration_type.hpp"
#include "myactuator_rmd/actuator_state/can_baud_rate.hpp"
#include "myactuator_rmd/actuator_state/gains.hpp"
#include "myactuator_rmd/protocol/command_type.hpp"
#include "myactuator_rmd/protocol/single_motor_message.hpp"


namespace myactuator_rmd {
  
  using GetAccelerationRequest = SingleMotorRequest<CommandType::READ_ACCELERATION>;
  using GetControllerGainsRequest = SingleMotorRequest<CommandType::READ_PID_PARAMETERS>;
  using GetMotorStatus1Request = SingleMotorRequest<CommandType::READ_MOTOR_STATUS_1_AND_ERROR_FLAG>;
  using GetMotorStatus2Request =  SingleMotorRequest<CommandType::READ_MOTOR_STATUS_2>;
  using GetMotorStatus3Request = SingleMotorRequest<CommandType::READ_MOTOR_STATUS_3>;
  using GetMultiTurnAngleRequest = SingleMotorRequest<CommandType::READ_MULTI_TURN_ANGLE>;
  using GetSingleTurnAngleRequest = SingleMotorRequest<CommandType::READ_SINGLE_TURN_ANGLE>;
  using GetSingleTurnEncoderPositionRequest = SingleMotorRequest<CommandType::READ_SINGLE_TURN_ENCODER>;
  using SetCurrentPositionAsEncoderZeroRequest = SingleMotorRequest<CommandType::WRITE_CURRENT_MULTI_TURN_POSITION_TO_ROM_AS_ZERO>;



  /**\class SetAccelerationRequest
   * \brief
   *    Request for setting the maximum acceleration/deceleration of the actuator
  */
  class SetAccelerationRequest: public SingleMotorRequest<CommandType::WRITE_ACCELERATION_TO_RAM> {
    public:
      /**\fn SetAccelerationRequest
       * \brief
       *    Class constructor
       * 
       * \param[in] acceleration
       *    The desired acceleration/deceleration in dps with a resolution of 1 dps [100, 60000]
       * \param[in] mode
       *    The mode of the desired acceleration/deceleration to be set
      */
      SetAccelerationRequest(std::uint32_t const acceleration, AccelerationType const mode);
      SetAccelerationRequest() = delete;
      SetAccelerationRequest(SetAccelerationRequest const&) = default;
      SetAccelerationRequest& operator = (SetAccelerationRequest const&) = default;
      SetAccelerationRequest(SetAccelerationRequest&&) = default;
      SetAccelerationRequest& operator = (SetAccelerationRequest&&) = default;
      using SingleMotorRequest::SingleMotorRequest;

      /**\fn getAcceleration
       * \brief
       *    Get the acceleration
       * 
       * \return
       *    The acceleration in degree per second**2 [100, 60000]
      */
      [[nodiscard]]
      std::uint32_t getAcceleration() const noexcept;

      /**\fn getMode
       * \brief
       *    Get the acceleration mode
       * 
       * \return
       *    The acceleration mode
      */
      [[nodiscard]]
      AccelerationType getMode() const noexcept;
  };

  /**\class SetEncoderZeroRequest
   * \brief
   *    Request for setting the encoder zero to a given value
  */
  class SetEncoderZeroRequest: public SingleMotorRequest<CommandType::WRITE_ENCODER_MULTI_TURN_VALUE_TO_ROM_AS_ZERO> {
    public:
      SetEncoderZeroRequest(std::int16_t const encoder_offset);
      SetEncoderZeroRequest(SetEncoderZeroRequest const&) = default;
      SetEncoderZeroRequest& operator = (SetEncoderZeroRequest const&) = default;
      SetEncoderZeroRequest(SetEncoderZeroRequest&&) = default;
      SetEncoderZeroRequest& operator = (SetEncoderZeroRequest&&) = default;
      using SingleMotorRequest::SingleMotorRequest;

      /**\fn getEncoderZero
       * \brief
       *    Get the encoder zero value
       * 
       * \return
       *    The encoder zero value
      */
      [[nodiscard]]
      std::int16_t getEncoderZero() const noexcept;
  };

  /**\class SetGainsRequest
   * \brief
   *    Base class for all requests for setting controller gains
   *
   * \tparam C
   *    Type of the command to be requested
  */
  template <CommandType C>
  class SetGainsRequest: public SingleMotorRequest<C> {
    public:
      /**\fn GainsRequest
       * \brief
       *    Class constructor
       * 
       * \param[in] gains
       *    Gains that the actuator should be set to
      */
      constexpr SetGainsRequest(Gains const& gains) noexcept;
      SetGainsRequest() = delete;
      SetGainsRequest(SetGainsRequest const&) = default;
      SetGainsRequest& operator = (SetGainsRequest const&) = default;
      SetGainsRequest(SetGainsRequest&&) = default;
      SetGainsRequest& operator = (SetGainsRequest&&) = default;
      using SingleMotorRequest<C>::SingleMotorRequest;

      /**\fn getGains
       * \brief
       *    Get the controller gains
       * 
       * \return
       *    The controller gains
      */
      [[nodiscard]]
      constexpr Gains getGains() const noexcept;
  };

  template <CommandType C>
  constexpr SetGainsRequest<C>::SetGainsRequest(Gains const& gains) noexcept
  : SingleMotorRequest<C>{} {
    this->data_[2] = gains.position.kp;
    this->data_[3] = gains.position.ki;
    this->data_[4] = gains.speed.kp;
    this->data_[5] = gains.speed.ki;
    this->data_[6] = gains.current.kp;
    this->data_[7] = gains.current.ki;
    return;
  }

  template <CommandType C>
  constexpr Gains SetGainsRequest<C>::getGains() const noexcept {
    auto const position_kp {this->data_[2]};
    auto const position_ki {this->data_[3]};
    auto const speed_kp {this->data_[4]};
    auto const speed_ki {this->data_[5]};
    auto const current_kp {this->data_[6]};
    auto const current_ki {this->data_[7]};
    return Gains{current_kp, current_ki, speed_kp, speed_ki, position_kp, position_ki};
  }

  using SetControllerGainsPersistentlyRequest = SetGainsRequest<CommandType::WRITE_PID_PARAMETERS_TO_ROM>;
  using SetControllerGainsRequest = SetGainsRequest<CommandType::WRITE_PID_PARAMETERS_TO_RAM>;

  /**\class SetPositionAbsoluteRequest
   * \brief
   *    Request for setting the absolute position of the actuator with a given maximum speed
  */
  class SetPositionAbsoluteRequest: public SingleMotorRequest<CommandType::ABSOLUTE_POSITION_CLOSED_LOOP_CONTROL> {
    public:
      /**\fn SetPositionAbsoluteRequest
       * \brief
       *    Class constructor
       * 
       * \param[in] position
       *    The position set-point in degree
       * \param[in] max_speed
       *    The maximum speed for the motion in degree per second
      */
      SetPositionAbsoluteRequest(float const position, float const max_speed);
      SetPositionAbsoluteRequest() = delete;
      SetPositionAbsoluteRequest(SetPositionAbsoluteRequest const&) = default;
      SetPositionAbsoluteRequest& operator = (SetPositionAbsoluteRequest const&) = default;
      SetPositionAbsoluteRequest(SetPositionAbsoluteRequest&&) = default;
      SetPositionAbsoluteRequest& operator = (SetPositionAbsoluteRequest&&) = default;
      using SingleMotorRequest::SingleMotorRequest;

      /**\fn getMaxSpeed
       * \brief
       *    Get the maximum speed
       * 
       * \return
       *    The maximum speed for the motion in degree per second
      */
      [[nodiscard]]
      float getMaxSpeed() const noexcept;

      /**\fn getPosition
       * \brief
       *    Get the position
       * 
       * \return
       *    The position set-point in degree
      */
      [[nodiscard]]
      float getPosition() const noexcept;
  };


  /**\class SetTorqueRequest
   * \brief
   *    Request for setting the torque of the actuator by setting a target current
  */
  class SetTorqueRequest: public SingleMotorRequest<CommandType::TORQUE_CLOSED_LOOP_CONTROL> {
    public:
      /**\fn SetTorqueRequest
       * \brief
       *    Class constructor
       * 
       * \param[in] current
       *    The current set-point in Ampere
      */
      SetTorqueRequest(float const current, float const current_constant);
      SetTorqueRequest() = delete;
      SetTorqueRequest(SetTorqueRequest const&) = default;
      SetTorqueRequest& operator = (SetTorqueRequest const&) = default;
      SetTorqueRequest(SetTorqueRequest&&) = default;
      SetTorqueRequest& operator = (SetTorqueRequest&&) = default;
      using SingleMotorRequest::SingleMotorRequest;

      /**\fn getTorqueCurrent
       * \brief
       *    Get the torque current
       * 
       * \return
       *    The torque current in Ampere
      */
      [[nodiscard]]
      float getTorqueCurrent() const noexcept;
  };

  /**\class SetVelocityRequest
   * \brief
   *    Request for setting the velocity of the actuator
  */
  class SetVelocityRequest: public SingleMotorRequest<CommandType::SPEED_CLOSED_LOOP_CONTROL> {
    public:
      /**\fn SetVelocityRequest
       * \brief
       *    Class constructor
       * 
       * \param[in] speed
       *    The velocity set-point in degree per second
      */
      SetVelocityRequest(float const speed);
      SetVelocityRequest() = delete;
      SetVelocityRequest(SetVelocityRequest const&) = default;
      SetVelocityRequest& operator = (SetVelocityRequest const&) = default;
      SetVelocityRequest(SetVelocityRequest&&) = default;
      SetVelocityRequest& operator = (SetVelocityRequest&&) = default;
      using SingleMotorRequest::SingleMotorRequest;

      /**\fn getSpeed
       * \brief
       *    Get the velocity set-point
       * 
       * \return
       *    The speed for the motion in degree per second
      */
      [[nodiscard]]
      float getSpeed() const noexcept;
  };

  using ShutdownMotorRequest = SingleMotorRequest<CommandType::SHUTDOWN_MOTOR>;
  using StopMotorRequest = SingleMotorRequest<CommandType::STOP_MOTOR>;

}

#endif // MYACTUATOR_RMD__PROTOCOL__REQUESTS
