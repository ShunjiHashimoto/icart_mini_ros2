#include "icart_mini_leg_tracker/follow_control.hpp"

#include <cassert>
#include <cmath>

namespace
{

bool near(double lhs, double rhs, double tolerance = 1e-9)
{
  return std::fabs(lhs - rhs) <= tolerance;
}

}  // namespace

int main()
{
  using icart_mini_leg_tracker::FollowControlConfig;
  using icart_mini_leg_tracker::FollowControlState;
  using icart_mini_leg_tracker::calculateDistanceAwareFollowCommand;
  using icart_mini_leg_tracker::targetRelativeToReference;

  const double sensor_distance = std::hypot(0.5, 0.3);
  const double sensor_angle = std::atan2(0.3, 0.5);
  const auto reference_target = targetRelativeToReference(
    sensor_distance, sensor_angle, 0.32);
  assert(near(reference_target.distance_m, std::hypot(0.82, 0.3)));
  assert(near(reference_target.angle_rad, std::atan2(0.3, 0.82)));

  FollowControlConfig config;
  config.max_linear_mps = 0.15;
  config.max_angular_radps = 15.0 * M_PI / 180.0;
  config.extreme_angular_radps = 25.0 * M_PI / 180.0;

  FollowControlState far_state;
  const auto far_command = calculateDistanceAwareFollowCommand(
    2.0, 2.0, 30.0 * M_PI / 180.0, 0.05, far_state, config);
  assert(near(far_command.linear_mps, 0.15));
  assert(near(far_command.angular_radps, 0.075));

  FollowControlState near_state;
  const auto near_command = calculateDistanceAwareFollowCommand(
    0.55, 0.80, 10.0 * M_PI / 180.0, 0.05, near_state, config);
  assert(near_state.near_mode);
  assert(near_state.aligning);
  assert(near(near_command.linear_mps, 0.0));
  assert(near(near_command.angular_radps, 10.0 * M_PI / 180.0));

  const auto aligned_command = calculateDistanceAwareFollowCommand(
    0.55, 0.87, 2.0 * M_PI / 180.0, 0.05, near_state, config);
  assert(!near_state.aligning);
  assert(near(aligned_command.linear_mps, 0.0));
  assert(near(aligned_command.angular_radps, 0.0));

  const auto exit_command = calculateDistanceAwareFollowCommand(
    0.65, 0.97, 10.0 * M_PI / 180.0, 0.05, near_state, config);
  assert(!near_state.near_mode);
  assert(exit_command.linear_mps > 0.0);
  assert(exit_command.angular_radps > 0.0);

  FollowControlState extreme_state;
  const auto extreme_command = calculateDistanceAwareFollowCommand(
    2.0, 2.3, 60.0 * M_PI / 180.0, 0.05, extreme_state, config);
  assert(near(extreme_command.linear_mps, 0.0));
  assert(near(extreme_command.angular_radps, config.extreme_angular_radps));

  FollowControlState near_extreme_state;
  const auto near_extreme_command = calculateDistanceAwareFollowCommand(
    0.55, 0.80, -60.0 * M_PI / 180.0, 0.05, near_extreme_state, config);
  assert(near_extreme_state.near_mode);
  assert(near_extreme_state.aligning);
  assert(near(near_extreme_command.linear_mps, 0.0));
  assert(near(near_extreme_command.angular_radps, -config.extreme_angular_radps));

  extreme_state.near_mode = true;
  extreme_state.aligning = true;
  const auto lost_command = calculateDistanceAwareFollowCommand(
    0.0, 0.32, 0.0, 0.05, extreme_state, config);
  assert(!extreme_state.near_mode);
  assert(!extreme_state.aligning);
  assert(near(lost_command.linear_mps, 0.0));
  assert(near(lost_command.angular_radps, 0.0));

  return 0;
}
