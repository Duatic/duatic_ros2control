/*
 * Copyright 2026 Duatic AG
 */

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <typeindex>
#include <utility>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <duatic_controllers/brake_release_controller.hpp>
#include <duatic_controllers/cartesian_pose_controller.hpp>
#include <duatic_controllers/duadrive_pid_tuner.hpp>
#include <duatic_controllers/duadrive_status_broadcaster.hpp>
#include <duatic_controllers/force_torque_broadcaster.hpp>
#include <duatic_controllers/freedrive_controller.hpp>
#include <duatic_controllers/freeze_controller.hpp>
#include <duatic_controllers/gravity_compensation_controller.hpp>
#include <duatic_controllers/ros2_control_compat.hpp>

namespace
{
const std::vector<std::string> kJoints = { "joint1", "joint2" };

// The Cartesian pose controller refuses to configure below this rate
constexpr unsigned int kUpdateRate = duatic::controllers::CartesianPoseController::min_update_rate_hz;

constexpr char kUrdf[] = R"(<?xml version="1.0"?>
<robot name="two_link">
  <link name="base_link"/>
  <link name="link1">
    <inertial><mass value="1.0"/><origin xyz="0 0 0.1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>
  <link name="flange">
    <inertial><mass value="1.0"/><origin xyz="0 0 0.1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="100" velocity="10"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="flange"/><origin xyz="0 0 0.2"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="100" velocity="10"/>
  </joint>
  <ros2_control name="TwoLinkSystem" type="system">
    <hardware><plugin>mock_components/GenericSystem</plugin></hardware>
    <joint name="joint1"><state_interface name="position"/></joint>
    <joint name="joint2"><state_interface name="position"/></joint>
  </ros2_control>
</robot>
)";

// The controller manager names a hardware node after its ros2_control tag in lower case
constexpr char kHardwareNodeName[] = "twolinksystem";

struct ControllerCase
{
  std::string name;
  std::function<std::unique_ptr<controller_interface::ControllerInterfaceBase>()> make;
  std::vector<rclcpp::Parameter> parameters;
};

/**
 * @brief Describe one controller under test.
 *
 * @tparam ControllerT Controller class, default constructible.
 * @param name Controller name, also used as the test name suffix.
 * @param parameters Node parameter overrides applied at init.
 * @return A case whose factory creates a fresh, uninitialized controller.
 */
template <typename ControllerT>
ControllerCase make_case(const std::string& name, std::vector<rclcpp::Parameter> parameters)
{
  return { name, [] { return std::make_unique<ControllerT>(); }, std::move(parameters) };
}

/**
 * @brief Build the interface descriptions for every name in a claimed interface configuration.
 *
 * Interfaces are doubles starting at 1.0, so controllers that reject all-zero joint readings on activation accept
 * them. `freeze_mode` and `target_brake_state` take the type and initial value the DuaDrive hardware interface
 * exports.
 *
 * @param names Full interface names, `<prefix>/<interface>`.
 * @return One description per name, in the same order.
 */
std::vector<hardware_interface::InterfaceDescription> describe(const std::vector<std::string>& names)
{
  std::vector<hardware_interface::InterfaceDescription> descriptions;
  for (const auto& full_name : names) {
    const auto slash = full_name.rfind('/');
    hardware_interface::InterfaceInfo info;
    info.name = full_name.substr(slash + 1);
    info.data_type = "double";
    info.initial_value = "1.0";
    if (info.name == "freeze_mode") {
      info.data_type = "bool";
      info.initial_value = "false";
    } else if (info.name == "target_brake_state") {
      info.data_type = "int32";
      info.initial_value = "0";
    }
    descriptions.emplace_back(full_name.substr(0, slash), info);
  }
  return descriptions;
}
}  // namespace

class ControllerActivationTest : public testing::TestWithParam<ControllerCase>
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
    // Stands in for the hardware node whose parameters the brake release controller reads on configure
    hardware_node_ = std::make_shared<rclcpp::Node>(kHardwareNodeName);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(hardware_node_);
    spin_thread_ = std::thread([] { executor_->spin(); });
  }

  static void TearDownTestSuite()
  {
    // Shutting down the context ends the spin even if the thread has not entered it yet
    rclcpp::shutdown();
    spin_thread_.join();
    executor_.reset();
    hardware_node_.reset();
  }

  static inline rclcpp::Node::SharedPtr hardware_node_;
  static inline std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  static inline std::thread spin_thread_;
};

TEST_P(ControllerActivationTest, ActivatesWithAllClaimedInterfaces)
{
  const auto& test_case = GetParam();
  auto controller = test_case.make();

  controller_interface::ControllerInterfaceParams params;
  params.controller_name = test_case.name;
  params.robot_description = kUrdf;
  params.update_rate = kUpdateRate;
  params.controller_manager_update_rate = kUpdateRate;
  params.node_options = controller->define_custom_node_options().parameter_overrides(test_case.parameters);
  ASSERT_EQ(controller->init(params), controller_interface::return_type::OK);

  ASSERT_EQ(controller->get_node()->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  // The controller manager exports the reference interfaces of chainable controllers after configuring them
  controller->export_reference_interfaces();

  // The interfaces must outlive the loans handed to the controller
  std::vector<hardware_interface::StateInterface::SharedPtr> state_storage;
  std::vector<hardware_interface::LoanedStateInterface> state_loans;
  for (const auto& description : describe(controller->state_interface_configuration().names)) {
    state_storage.push_back(std::make_shared<hardware_interface::StateInterface>(description));
    state_loans.emplace_back(state_storage.back(), nullptr);
  }
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_storage;
  std::vector<hardware_interface::LoanedCommandInterface> command_loans;
  for (const auto& description : describe(controller->command_interface_configuration().names)) {
    command_storage.push_back(std::make_shared<hardware_interface::CommandInterface>(description));
    command_loans.emplace_back(command_storage.back(), nullptr);
  }
  controller->assign_interfaces(std::move(command_loans), std::move(state_loans));

  ASSERT_EQ(controller->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // A controller switch deactivates and reactivates with the same interfaces
  ASSERT_EQ(controller->get_node()->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(controller->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  controller->release_interfaces();
}

namespace
{
/// Every controller of this package must have a case here, see ControllerRegistryTest.
std::vector<ControllerCase> controller_cases()
{
  return {
    make_case<duatic::controllers::BrakeReleaseController>("brake_release_controller", { { "joints", kJoints } }),
    make_case<duatic::controllers::CartesianPoseController>("cartesian_pose_controller", { { "target_frame", "flang"
                                                                                                             "e" } }),
    make_case<duatic::controllers::PIDTuner>("duadrive_pid_tuner", { { "joints", kJoints } }),
    make_case<duatic::controllers::StatusBroadcaster>("duadrive_status_broadcaster", { { "joints", kJoints } }),
    make_case<duatic::controllers::ForceTorqueBroadcaster>(
        "force_torque_broadcaster", { { "joints", kJoints }, { "endeffector_frame", "flange" } }),
    make_case<duatic::controllers::FreeDriveController>("freedrive_controller", { { "joints", kJoints } }),
    make_case<duatic::controllers::FreezeController>("freeze_controller",
                                                     { { "names", std::vector<std::string>{ "arm" } } }),
    make_case<duatic::controllers::GravityCompensationController>("gravity_compensation_controller",
                                                                  { { "joints", kJoints } })
  };
}
}  // namespace

INSTANTIATE_TEST_SUITE_P(Controllers, ControllerActivationTest, testing::ValuesIn(controller_cases()),
                         [](const testing::TestParamInfo<ControllerCase>& info) { return info.param.name; });

class GravityCompensationRestartTest : public testing::Test
{
protected:
  static void SetUpTestSuite()
  /// Start the ROS context shared by the tests of this suite.
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  /// Stop the ROS context shared by the tests of this suite.
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  /// Bring up an active controller that publishes its status, with every claimed interface loaned to it.
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "gravity_compensation_controller";
    params.robot_description = kUrdf;
    params.update_rate = kUpdateRate;
    params.controller_manager_update_rate = kUpdateRate;
    params.node_options = controller_.define_custom_node_options().parameter_overrides(
        std::vector<rclcpp::Parameter>{ { "joints", kJoints }, { "enable_state_topic", true } });
    ASSERT_EQ(controller_.init(params), controller_interface::return_type::OK);

    ASSERT_EQ(controller_.get_node()->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    references_ = controller_.export_reference_interfaces();

    std::vector<hardware_interface::LoanedStateInterface> state_loans;
    for (const auto& description : describe(controller_.state_interface_configuration().names)) {
      state_storage_.push_back(std::make_shared<hardware_interface::StateInterface>(description));
      state_loans.emplace_back(state_storage_.back(), nullptr);
    }
    std::vector<hardware_interface::LoanedCommandInterface> command_loans;
    for (const auto& description : describe(controller_.command_interface_configuration().names)) {
      command_storage_.push_back(std::make_shared<hardware_interface::CommandInterface>(description));
      command_loans.emplace_back(command_storage_.back(), nullptr);
    }
    controller_.assign_interfaces(std::move(command_loans), std::move(state_loans));

    ASSERT_EQ(controller_.get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  void TearDown() override
  /// Return the loaned interfaces before the storage backing them goes away.
  {
    controller_.release_interfaces();
  }

  controller_interface::return_type update()
  /// Run one control cycle.
  {
    return controller_.update(controller_.get_node()->now(), rclcpp::Duration::from_seconds(1.0 / kUpdateRate));
  }

  void restart()
  /// Deactivate and reactivate, as the controller manager does to switch the chained mode.
  {
    ASSERT_EQ(controller_.get_node()->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(controller_.get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  double commanded_effort() const
  /// Read the effort currently commanded to the first joint.
  {
    return duatic::controllers::compat::try_get_value(*command_storage_.at(0))
        .value_or(std::numeric_limits<double>::quiet_NaN());
  }

  duatic::controllers::GravityCompensationController controller_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> references_;
  std::vector<hardware_interface::StateInterface::SharedPtr> state_storage_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_storage_;
};

TEST_F(GravityCompensationRestartTest, DropsReferenceLeftBehindByPrecedingController)
{
  const double compensation = commanded_effort();

  ASSERT_TRUE(references_.at(0)->set_value(5.0));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  ASSERT_DOUBLE_EQ(commanded_effort(), compensation + 5.0);

  restart();

  EXPECT_DOUBLE_EQ(commanded_effort(), compensation);
}

TEST_F(GravityCompensationRestartTest, PublishesOneTorquePerJointAfterRestart)
{
  using StatusMsg = duatic_controller_msgs::msg::GravityCompensationControllerStatus;

  restart();

  auto listener = std::make_shared<rclcpp::Node>("status_listener");
  std::optional<StatusMsg> status;
  const auto subscription =
      listener->create_subscription<StatusMsg>(std::string(controller_.get_node()->get_fully_qualified_name()) + "/stat"
                                                                                                                 "e",
                                               10, [&status](const StatusMsg& msg) { status = msg; });

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!status.has_value() && std::chrono::steady_clock::now() < deadline) {
    ASSERT_EQ(update(), controller_interface::return_type::OK);
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(status.has_value()) << "no status message received";
  EXPECT_EQ(status->commanded_torque.size(), kJoints.size());
}

TEST(ControllerRegistryTest, EveryRegisteredControllerHasAnActivationCase)
{
  std::set<std::type_index> tested;
  for (const auto& test_case : controller_cases()) {
    const auto controller = test_case.make();
    tested.emplace(typeid(*controller));
  }

  std::size_t registered = 0;
  // Plugins are declared per base class, so chainable controllers need their own loader
  auto check_registered = [&](auto& loader) {
    for (const auto& lookup_name : loader.getDeclaredClasses()) {
      if (lookup_name.rfind("duatic_controllers/", 0) != 0) {
        continue;
      }
      ++registered;
      const auto controller = loader.createUniqueInstance(lookup_name);
      EXPECT_TRUE(tested.count(typeid(*controller)) == 1) << lookup_name << " has no case in controller_cases()";
    }
  };
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader("controller_interface", "controller_"
                                                                                                   "interface::"
                                                                                                   "ControllerInterfac"
                                                                                                   "e");
  check_registered(loader);
  pluginlib::ClassLoader<controller_interface::ChainableControllerInterface> chainable_loader("controller_interface",
                                                                                              "controller_interface::"
                                                                                              "ChainableControllerInter"
                                                                                              "face");
  check_registered(chainable_loader);
  EXPECT_GT(registered, 0u) << "no duatic_controllers plugins found, is the workspace sourced?";
}
