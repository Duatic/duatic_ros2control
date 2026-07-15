#pragma once

#include <Eigen/Dense>
#include <duatic_geometry/state_3d.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <rclcpp/time.hpp>

namespace duatic::geometry
{

struct alignas(std::hardware_destructive_interference_size) TrajectoryUpdateInformation
{
  StampedState3Dd target_state;
};

class ConstantTargetPoseTwist
{
public:
  inline ConstantTargetPoseTwist() = default;
  inline ConstantTargetPoseTwist(const StampedState3Dd& current_state, const TrajectoryUpdateInformation& update_info)
    : ConstantTargetPoseTwist()
  {
    update(current_state, update_info);
  }

  void update(const StampedState3Dd& current_state, const TrajectoryUpdateInformation& update_info);

  void evaluate_at(const rclcpp::Time& time, State3Dd& out_state) const;

  inline State3Dd evaluate_at(const rclcpp::Time& time) const
  {
    State3Dd out_state;
    evaluate_at(time, out_state);
    return out_state;
  }

private:
  State3Dd target_state_;
};

}  // namespace duatic::geometry
