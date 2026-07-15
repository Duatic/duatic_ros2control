#include <duatic_geometry/trajectory.hpp>

namespace duatic::geometry
{

void ConstantTargetPoseTwist::update([[maybe_unused]] const StampedState3Dd& current_state,
                                     const TrajectoryUpdateInformation& update_info)
{
  target_state_ = update_info.target_state;
}

void ConstantTargetPoseTwist::evaluate_at([[maybe_unused]] const rclcpp::Time& time, State3Dd& out_state) const
{
  out_state = target_state_;
}

}  // namespace duatic::geometry
