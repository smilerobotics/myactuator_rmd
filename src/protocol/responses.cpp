#include "myactuator_rmd/protocol/responses.hpp"

#include <chrono>
#include <cstdint>
#include <string>

#include "myactuator_rmd/actuator_state/error_code.hpp"
#include "myactuator_rmd/actuator_state/feedback.hpp"
#include "myactuator_rmd/actuator_state/motor_status_1.hpp"
#include "myactuator_rmd/actuator_state/motor_status_3.hpp"
#include "myactuator_rmd/exceptions.hpp"

#include <stdio.h>

namespace myactuator_rmd {


  std::int32_t GetAccelerationResponse::getAcceleration() const noexcept {
    auto const acceleration {getAs<std::int32_t>(4)};
    return acceleration;
  }

  MotorStatus1 GetMotorStatus1Response::getStatus() const noexcept {
    auto const temperature {static_cast<int>(getAs<std::int8_t>(1))};
    auto const voltage {static_cast<float>(getAs<std::uint16_t>(3))*0.1f};
    auto const error_code {static_cast<ErrorCode>(getAs<std::uint8_t>(7))};
    return MotorStatus1{temperature,voltage, error_code};
  }

  MotorStatus3 GetMotorStatus3Response::getStatus() const noexcept {
    auto const temperature {static_cast<int>(getAs<std::int8_t>(1))};
    auto const current_phase_a {static_cast<float>(getAs<std::int16_t>(2))*0.015625f};
    auto const current_phase_b {static_cast<float>(getAs<std::int16_t>(4))*0.015625f};
    auto const current_phase_c {static_cast<float>(getAs<std::int16_t>(6))*0.015625f};
    return MotorStatus3{temperature, current_phase_a, current_phase_b, current_phase_c};
  }

  double GetMultiTurnAngleResponse::getAngle() const noexcept {
    //TODO:
    int64_t angleShift {static_cast<int64_t>(getAs<std::int64_t>(0))};
    angleShift = angleShift >> 8;
    return static_cast<float>(angleShift *0.01f);
  }

  float GetSingleTurnAngleResponse::getAngle() const noexcept {
    // This does not seem to give the correct results at least with my motor
    //auto const angle {static_cast<float>(getAs<std::int16_t>(6))*0.01f};
    auto const angle {static_cast<float>(getAs<std::int32_t>(4))*0.01f};
    return angle;
  }

  std::int16_t GetSingleTurnEncoderPositionResponse::getPosition() const noexcept {
    auto const position {getAs<std::int16_t>(2)};
    return position;
  }

  std::int16_t GetSingleTurnEncoderPositionResponse::getRawPosition() const noexcept {
    auto const position {getAs<std::int16_t>(4)};
    return position;
  }

  std::int16_t GetSingleTurnEncoderPositionResponse::getOffset() const noexcept {
    auto const position {getAs<std::int16_t>(6)};
    return position;
  }

  std::int32_t SetCurrentPositionAsEncoderZeroResponse::getEncoderZero() const noexcept {
    return getAs<std::int16_t>(6);
  }

}
