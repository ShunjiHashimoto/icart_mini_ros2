#ifndef ICART_MINI_LEG_TRACKER_FOLLOW_CONTROL_HPP
#define ICART_MINI_LEG_TRACKER_FOLLOW_CONTROL_HPP

#include <algorithm>
#include <cmath>

namespace icart_mini_leg_tracker
{

struct FollowControlConfig
{
  double desired_sensor_distance_m = 0.5;
  double near_enter_distance_m = 0.55;
  double near_exit_distance_m = 0.65;
  double align_start_angle_rad = 5.0 * M_PI / 180.0;
  double align_stop_angle_rad = 3.0 * M_PI / 180.0;
  double extreme_angle_rad = M_PI / 4.0;
  double max_linear_mps = 0.25;
  double min_linear_mps = 0.05;
  double max_angular_radps = M_PI / 3.0;
  double distance_kp = 0.5;
  double distance_ki = 0.01;
  double angle_kp = 1.0;
  double max_distance_integral = 10.0;
};

struct FollowControlState
{
  bool near_mode = false;
  bool aligning = false;
  double distance_integral = 0.0;
};

struct FollowControlCommand
{
  double linear_mps = 0.0;
  double angular_radps = 0.0;
};

struct ReferenceTarget
{
  double distance_m = 0.0;
  double angle_rad = 0.0;
};

inline ReferenceTarget targetRelativeToReference(
  double sensor_distance_m,
  double sensor_angle_rad,
  double sensor_offset_x_m)
{
  const double sensor_x = sensor_distance_m * std::cos(sensor_angle_rad);
  const double sensor_y = sensor_distance_m * std::sin(sensor_angle_rad);
  const double reference_x = sensor_x + sensor_offset_x_m;
  return {
    std::hypot(reference_x, sensor_y),
    std::atan2(sensor_y, reference_x)
  };
}

inline FollowControlCommand calculateDistanceAwareFollowCommand(
  double sensor_distance_m,
  double reference_distance_m,
  double reference_angle_rad,
  double dt_sec,
  FollowControlState & state,
  const FollowControlConfig & config)
{
  FollowControlCommand command;
  if (!std::isfinite(sensor_distance_m) || !std::isfinite(reference_distance_m) ||
    !std::isfinite(reference_angle_rad) || !std::isfinite(dt_sec) ||
    sensor_distance_m <= 0.0 || reference_distance_m <= 0.0)
  {
    state = FollowControlState{};
    return command;
  }

  if (state.near_mode) {
    if (sensor_distance_m >= config.near_exit_distance_m) {
      state.near_mode = false;
      state.aligning = false;
    }
  } else if (sensor_distance_m <= config.near_enter_distance_m) {
    state.near_mode = true;
  }

  const double abs_angle = std::fabs(reference_angle_rad);
  if (state.near_mode) {
    state.distance_integral = 0.0;
    if (state.aligning) {
      if (abs_angle <= config.align_stop_angle_rad) {
        state.aligning = false;
      }
    } else if (abs_angle >= config.align_start_angle_rad) {
      state.aligning = true;
    }

    if (state.aligning) {
      command.angular_radps = std::clamp(
        config.angle_kp * reference_angle_rad,
        -config.max_angular_radps,
        config.max_angular_radps);
    }
    return command;
  }

  const double distance_error = sensor_distance_m - config.desired_sensor_distance_m;
  if (dt_sec > 0.0) {
    state.distance_integral += distance_error * dt_sec;
    state.distance_integral = std::clamp(
      state.distance_integral,
      -config.max_distance_integral,
      config.max_distance_integral);
  }
  const double requested_linear =
    config.distance_kp * distance_error + config.distance_ki * state.distance_integral;
  command.linear_mps = std::clamp(
    requested_linear,
    config.min_linear_mps,
    config.max_linear_mps);

  if (abs_angle > config.extreme_angle_rad) {
    command.linear_mps = 0.0;
    command.angular_radps = std::copysign(config.max_angular_radps, reference_angle_rad);
    return command;
  }

  const double curvature = 2.0 * std::sin(reference_angle_rad) / reference_distance_m;
  command.angular_radps = std::clamp(
    command.linear_mps * curvature,
    -config.max_angular_radps,
    config.max_angular_radps);
  return command;
}

}  // namespace icart_mini_leg_tracker

#endif  // ICART_MINI_LEG_TRACKER_FOLLOW_CONTROL_HPP
