#pragma once

#include <duatic_geometry/pose_3d.hpp>
#include <duatic_geometry/state_3d.hpp>
#include <rclcpp/time.hpp>

namespace duatic::geometry
{

class ConstantTargetPose
{
public:
  struct UpdateInformation
  {
    using msg = StampedPose3Dd::msg;

    Pose3Dd target_pose;

    inline UpdateInformation() = default;
    inline explicit UpdateInformation(const Pose3Dd& pose) : target_pose(pose){};
    inline explicit UpdateInformation(const msg& msg) : target_pose(msg){};
  };

  inline ConstantTargetPose() = default;
  inline ConstantTargetPose(const TimedState3Dd& current_state, const UpdateInformation& update_info)
    : ConstantTargetPose()
  {
    update(current_state, update_info);
  }

  void update(const TimedState3Dd& current_state, const UpdateInformation& update_info);

  void evaluate_at(const rclcpp::Time& time, State3Dd& out_state) const;

  inline State3Dd evaluate_at(const rclcpp::Time& time) const
  {
    State3Dd out_state;
    evaluate_at(time, out_state);
    return out_state;
  }

  static UpdateInformation NeutralUpdate(const TimedState3Dd& current_state);

private:
  Pose3Dd target_pose_;
};

class ConstantTargetState
{
public:
  struct UpdateInformation
  {
    using msg = StampedState3Dd::msg;

    State3Dd target_state;

    inline UpdateInformation() = default;
    inline explicit UpdateInformation(const State3Dd& state) : target_state(state){};
    inline explicit UpdateInformation(const msg& msg) : target_state(msg){};
  };

  inline ConstantTargetState() = default;
  inline ConstantTargetState(const TimedState3Dd& current_state, const UpdateInformation& update_info)
    : ConstantTargetState()
  {
    update(current_state, update_info);
  }

  void update(const TimedState3Dd& current_state, const UpdateInformation& update_info);

  void evaluate_at(const rclcpp::Time& time, State3Dd& out_state) const;

  inline State3Dd evaluate_at(const rclcpp::Time& time) const
  {
    State3Dd out_state;
    evaluate_at(time, out_state);
    return out_state;
  }

  static UpdateInformation NeutralUpdate(const TimedState3Dd& current_state);

private:
  State3Dd target_state_;
};

}  // namespace duatic::geometry
