/**
 * \file responses.hpp
 * \mainpage
 *    Contains all the responses sent from the actuator to the driver
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#ifndef MYACTUATOR_RMD__PROTOCOL__RESPONSES
#define MYACTUATOR_RMD__PROTOCOL__RESPONSES
#pragma once

#include <chrono>
#include <cstdint>

#include "myactuator_rmd/actuator_state/control_mode.hpp"
#include "myactuator_rmd/actuator_state/feedback.hpp"
#include "myactuator_rmd/actuator_state/gains.hpp"
#include "myactuator_rmd/actuator_state/motor_status_1.hpp"
#include "myactuator_rmd/actuator_state/motor_status_2.hpp"
#include "myactuator_rmd/actuator_state/motor_status_3.hpp"
#include "myactuator_rmd/protocol/command_type.hpp"
#include "myactuator_rmd/protocol/single_motor_message.hpp"


namespace myactuator_rmd {


  /**\class GetAccelerationResponse
   * \brief
   *    Response to request for reading the motor model
  */
  class GetAccelerationResponse: public SingleMotorResponse<CommandType::READ_ACCELERATION> {
    public:
      GetAccelerationResponse() = delete;
      GetAccelerationResponse(GetAccelerationResponse const&) = default;
      GetAccelerationResponse& operator = (GetAccelerationResponse const&) = default;
      GetAccelerationResponse(GetAccelerationResponse&&) = default;
      GetAccelerationResponse& operator = (GetAccelerationResponse&&) = default;
      using SingleMotorResponse::SingleMotorResponse;

      /**\fn getAcceleration
       * \brief
       *    Get the current acceleration
       * 
       * \return
       *    The current acceleration with a resolution of 1 dps
      */
      [[nodiscard]]
      std::int32_t getAcceleration() const noexcept;
  };

  /**\class GetMultiTurnAngleResponse
   * \brief
   *    Response to request for reading a multi-turn angle
  */
  class GetMultiTurnAngleResponse: public SingleMotorResponse<CommandType::READ_MULTI_TURN_ANGLE> {
    public:
      GetMultiTurnAngleResponse() = delete;
      GetMultiTurnAngleResponse(GetMultiTurnAngleResponse const&) = default;
      GetMultiTurnAngleResponse& operator = (GetMultiTurnAngleResponse const&) = default;
      GetMultiTurnAngleResponse(GetMultiTurnAngleResponse&&) = default;
      GetMultiTurnAngleResponse& operator = (GetMultiTurnAngleResponse&&) = default;
      using SingleMotorResponse::SingleMotorResponse;

      /**\fn getPosition
       * \brief
       *    Get the multi-turn angle
       * 
       * \return
       *    The multi-turn angle with a resolution of 0.01 deg
      */
      [[nodiscard]]
      double getAngle() const noexcept;
  };

  /**\class GetSingleTurnAngleResponse
   * \brief
   *    Response to request for reading a single-turn angle
  */
  class GetSingleTurnAngleResponse: public SingleMotorResponse<CommandType::READ_SINGLE_TURN_ANGLE> {
    public:
      GetSingleTurnAngleResponse() = delete;
      GetSingleTurnAngleResponse(GetSingleTurnAngleResponse const&) = default;
      GetSingleTurnAngleResponse& operator = (GetSingleTurnAngleResponse const&) = default;
      GetSingleTurnAngleResponse(GetSingleTurnAngleResponse&&) = default;
      GetSingleTurnAngleResponse& operator = (GetSingleTurnAngleResponse&&) = default;
      using SingleMotorResponse::SingleMotorResponse;

      /**\fn getPosition
       * \brief
       *    Get the single-turn angle
       * 
       * \return
       *    The single-turn angle with a resolution of 0.01 deg
      */
      [[nodiscard]]
      float getAngle() const noexcept;
  };

  /**\class GetSingleTurnEncoderPositionResponse
   * \brief
   *    Response to request for reading a single-turn encoder position
  */
  class GetSingleTurnEncoderPositionResponse: public SingleMotorResponse<CommandType::READ_SINGLE_TURN_ENCODER> {
    public:
      GetSingleTurnEncoderPositionResponse() = delete;
      GetSingleTurnEncoderPositionResponse(GetSingleTurnEncoderPositionResponse const&) = default;
      GetSingleTurnEncoderPositionResponse& operator = (GetSingleTurnEncoderPositionResponse const&) = default;
      GetSingleTurnEncoderPositionResponse(GetSingleTurnEncoderPositionResponse&&) = default;
      GetSingleTurnEncoderPositionResponse& operator = (GetSingleTurnEncoderPositionResponse&&) = default;
      using SingleMotorResponse::SingleMotorResponse;

      /**\fn getPosition
       * \brief
       *    Get the encoder position
       * 
       * \return
       *    The current encoder position
      */
      [[nodiscard]]
      std::int16_t getPosition() const noexcept;

      /**\fn getRawPosition
       * \brief
       *    Get the encoder raw position
       * 
       * \return
       *    The current raw encoder position
      */
      [[nodiscard]]
      std::int16_t getRawPosition() const noexcept;

      /**\fn getOffset
       * \brief
       *    Get the encoder position offset
       * 
       * \return
       *    The current encoder position offset
      */
      [[nodiscard]]
      std::int16_t getOffset() const noexcept;
  };

  /**\class FeedbackResponse
   * \brief
   *    Base class for all responses containing a feedback
   *
   * \tparam C
   *    Type of the command to be requested
  */
  template <CommandType C>
  class FeedbackResponse: public SingleMotorResponse<C> {
    public:
      FeedbackResponse() = delete;
      FeedbackResponse(FeedbackResponse const&) = default;
      FeedbackResponse& operator = (FeedbackResponse const&) = default;
      FeedbackResponse(FeedbackResponse&&) = default;
      FeedbackResponse& operator = (FeedbackResponse&&) = default;
      using SingleMotorResponse<C>::SingleMotorResponse;

      /**\fn getStatus
       * \brief
       *    Get the feedback
       * 
       * \return
       *    Feedback from the actuator
      */
      [[nodiscard]]
      Feedback getStatus(float const current_constant) const noexcept;
  };

  template <CommandType C>
  Feedback FeedbackResponse<C>::getStatus(float const current_constant) const noexcept {
    auto const temperature {static_cast<int>(this->template getAs<std::int8_t>(1))};
    auto const current {static_cast<float>(this->template getAs<std::int16_t>(2))*current_constant};
    auto const shaft_speed {static_cast<float>(this->template getAs<std::int16_t>(4))};
    auto const shaft_encorder {static_cast<float>(this->template getAs<std::uint16_t>(6))};
    return Feedback{temperature, current, shaft_speed, shaft_encorder};
  }

  using GetMotorStatus2Response = FeedbackResponse<CommandType::READ_MOTOR_STATUS_2>;
  using SetPositionAbsoluteResponse = FeedbackResponse<CommandType::ABSOLUTE_POSITION_CLOSED_LOOP_CONTROL>;
  using SetTorqueResponse = FeedbackResponse<CommandType::TORQUE_CLOSED_LOOP_CONTROL>;
  using SetVelocityResponse = FeedbackResponse<CommandType::SPEED_CLOSED_LOOP_CONTROL>;

  /**\class GainsResponse
   * \brief
   *    Base class for all responses with controller gains
   *
   * \tparam C
   *    Type of the command to be requested
  */
  template <CommandType C>
  class GainsResponse: public SingleMotorResponse<C> {
    public:
      GainsResponse() = delete;
      GainsResponse(GainsResponse const&) = default;
      GainsResponse& operator = (GainsResponse const&) = default;
      GainsResponse(GainsResponse&&) = default;
      GainsResponse& operator = (GainsResponse&&) = default;
      using SingleMotorResponse<C>::SingleMotorResponse;

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
  constexpr Gains GainsResponse<C>::getGains() const noexcept {
    auto const position_kp {this->data_[2]};
    auto const position_ki {this->data_[3]};
    auto const speed_kp {this->data_[4]};
    auto const speed_ki {this->data_[5]};
    auto const current_kp {this->data_[6]};
    auto const current_ki {this->data_[7]};
    return Gains{current_kp, current_ki, speed_kp, speed_ki, position_kp, position_ki};
  }

  using GetControllerGainsResponse = GainsResponse<CommandType::READ_PID_PARAMETERS>;
  using SetControllerGainsPersistentlyResponse = GainsResponse<CommandType::WRITE_PID_PARAMETERS_TO_ROM>;
  using SetControllerGainsResponse = GainsResponse<CommandType::WRITE_PID_PARAMETERS_TO_RAM>;


  /**\class GetMotorStatus1Response
   * \brief
   *    Response to request for getting motor status
  */
  class GetMotorStatus1Response: public SingleMotorResponse<CommandType::READ_MOTOR_STATUS_1_AND_ERROR_FLAG> {
    public:
      GetMotorStatus1Response() = delete;
      GetMotorStatus1Response(GetMotorStatus1Response const&) = default;
      GetMotorStatus1Response& operator = (GetMotorStatus1Response const&) = default;
      GetMotorStatus1Response(GetMotorStatus1Response&&) = default;
      GetMotorStatus1Response& operator = (GetMotorStatus1Response&&) = default;
      using SingleMotorResponse::SingleMotorResponse;

      /**\fn getStatus
       * \brief
       *    Get the motor status
       * 
       * \return
       *    Motor status of the actuator
      */
      [[nodiscard]]
      MotorStatus1 getStatus() const noexcept;
  };

  /**\class GetMotorStatus3Response
   * \brief
   *    Response to request for getting motor status
  */
  class GetMotorStatus3Response: public SingleMotorResponse<CommandType::READ_MOTOR_STATUS_3> {
    public:
      GetMotorStatus3Response() = delete;
      GetMotorStatus3Response(GetMotorStatus3Response const&) = default;
      GetMotorStatus3Response& operator = (GetMotorStatus3Response const&) = default;
      GetMotorStatus3Response(GetMotorStatus3Response&&) = default;
      GetMotorStatus3Response& operator = (GetMotorStatus3Response&&) = default;
      using SingleMotorResponse::SingleMotorResponse;

      /**\fn getStatus
       * \brief
       *    Get the motor status
       * 
       * \return
       *    Motor status of the actuator
      */
      [[nodiscard]]
      MotorStatus3 getStatus() const noexcept;
  };


  using SetAccelerationResponse = SingleMotorResponse<CommandType::WRITE_ACCELERATION_TO_RAM>;

  /**\class SetCurrentPositionAsEncoderZeroResponse
   * \brief
   *    Response to request for setting the current position as the encoder zero position
  */
  class SetCurrentPositionAsEncoderZeroResponse: public SingleMotorResponse<CommandType::WRITE_CURRENT_MULTI_TURN_POSITION_TO_ROM_AS_ZERO> {
    public:
      SetCurrentPositionAsEncoderZeroResponse() = delete;
      SetCurrentPositionAsEncoderZeroResponse(SetCurrentPositionAsEncoderZeroResponse const&) = default;
      SetCurrentPositionAsEncoderZeroResponse& operator = (SetCurrentPositionAsEncoderZeroResponse const&) = default;
      SetCurrentPositionAsEncoderZeroResponse(SetCurrentPositionAsEncoderZeroResponse&&) = default;
      SetCurrentPositionAsEncoderZeroResponse& operator = (SetCurrentPositionAsEncoderZeroResponse&&) = default;
      using SingleMotorResponse::SingleMotorResponse;

      /**\fn getEncoderZero
       * \brief
       *    Get the encoder zero value
       * 
       * \return
       *    The encoder zero value
      */
      [[nodiscard]]
      std::int32_t getEncoderZero() const noexcept;
  };

  using SetEncoderZeroResponse = SingleMotorRequest<CommandType::WRITE_ENCODER_MULTI_TURN_VALUE_TO_ROM_AS_ZERO>;
  using ShutdownMotorResponse = SingleMotorResponse<CommandType::SHUTDOWN_MOTOR>;
  using StopMotorResponse = SingleMotorResponse<CommandType::STOP_MOTOR>;

}

#endif // MYACTUATOR_RMD__PROTOCOL__RESPONSES
