#include "myactuator_rmd/protocol/requests.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>

#include "myactuator_rmd/actuator_state/acceleration_type.hpp"
#include "myactuator_rmd/actuator_state/can_baud_rate.hpp"
#include "myactuator_rmd/protocol/single_motor_message.hpp"
#include "myactuator_rmd/exceptions.hpp"


namespace myactuator_rmd {

  SetAccelerationRequest::SetAccelerationRequest(std::uint32_t const acceleration, AccelerationType const mode)
  : SingleMotorRequest{} {
    if ((acceleration != 0) && ((acceleration < 100) || (acceleration > 60000))) {
      throw ValueRangeException("Acceleration value '" + std::to_string(acceleration) + "' out of range [100, 60000]");
    }
    auto const acceleration_type {static_cast<std::uint8_t>(mode)};
    setAt(acceleration_type, 1);
    setAt(acceleration, 4);
    return;
  }

  std::uint32_t SetAccelerationRequest::getAcceleration() const noexcept {
    return getAs<std::uint32_t>(4);
  }

  SetEncoderZeroRequest::SetEncoderZeroRequest(std::int16_t const encoder_offset)
  : SingleMotorRequest{} {
    setAt(encoder_offset, 4);
    return;
  }

  std::int16_t SetEncoderZeroRequest::getEncoderZero() const noexcept {
    return getAs<std::int16_t>(6);
  }

  SetPositionAbsoluteRequest::SetPositionAbsoluteRequest(float const position, float const max_speed)
  : SingleMotorRequest{} {
    auto const v {static_cast<std::uint16_t>(max_speed)};
    auto const pos {static_cast<std::int32_t>(position*100.0f)};
    setAt(v, 2);
    setAt(pos, 4);
    return;
  }

  float SetPositionAbsoluteRequest::getMaxSpeed() const noexcept {
    return static_cast<float>(getAs<std::uint16_t>(2));
  }

  float SetPositionAbsoluteRequest::getPosition() const noexcept {
    return static_cast<float>(getAs<std::int32_t>(4)/100.0f);
  }

  SetTorqueRequest::SetTorqueRequest(float const current, float const current_constant)
  : SingleMotorRequest{} {
    auto const c {static_cast<std::int16_t>(current/current_constant)};
    setAt(c, 4);
    return;
  }

  float SetTorqueRequest::getTorqueCurrent() const noexcept {
    return static_cast<float>(getAs<std::int16_t>(4))*0.01f;
  }

  SetVelocityRequest::SetVelocityRequest(float const speed)
  : SingleMotorRequest{} {
    auto const s {static_cast<std::int32_t>(speed*100.0f)};
    setAt(s, 4);
    return;
  }

  float SetVelocityRequest::getSpeed() const noexcept {
    return static_cast<float>(getAs<std::int32_t>(4))/100.0f;
  }

}
