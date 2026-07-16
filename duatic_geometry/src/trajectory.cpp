#include <duatic_geometry/trajectory.hpp>

namespace duatic::geometry
{

void ConstantTargetPose::update([[maybe_unused]] const TimedState3Dd& current_state,
                                const UpdateInformation& update_info)
{
  target_pose_ = update_info.target_pose;
}

void ConstantTargetPose::evaluate_at([[maybe_unused]] const rclcpp::Time& time, State3Dd& out_state) const
{
  out_state.pose = target_pose_;
  out_state.twist.setNeutral();
}

ConstantTargetPose::UpdateInformation ConstantTargetPose::NeutralUpdate(const TimedState3Dd& current_state)
{
  return UpdateInformation(current_state.pose);
}

void ConstantTargetState::update([[maybe_unused]] const TimedState3Dd& current_state,
                                 const UpdateInformation& update_info)
{
  target_state_ = update_info.target_state;
}

void ConstantTargetState::evaluate_at([[maybe_unused]] const rclcpp::Time& time, State3Dd& out_state) const
{
  out_state = target_state_;
}

ConstantTargetState::UpdateInformation ConstantTargetState::NeutralUpdate(const TimedState3Dd& current_state)
{
  return UpdateInformation(current_state);
}

}  // namespace duatic::geometry
