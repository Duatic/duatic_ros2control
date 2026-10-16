/*
 * Copyright 2026 Duatic AG
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

#include <duatic_controllers/brake_release_controller.hpp>
#include <duatic_controllers/ros2_control_compat.hpp>

namespace
{
const std::vector<std::string> kJoints = { "joint1", "joint2" };
constexpr unsigned int kUpdateRate = 100;
constexpr char kGravityCompensation[] = "gravity_compensation_controller";

/**
 * @brief Build a robot description whose single ros2_control hardware handles the given joints.
 *
 * @param hardware Name of the ros2_control tag, which the controller turns into the hardware node name.
 * @param joints Joints of the hardware.
 * @return The URDF as a string.
 */
std::string make_urdf(const std::string& hardware, const std::vector<std::string>& joints)
{
  std::string urdf = R"(<?xml version="1.0"?><robot name="coupled"><link name="base_link"/>)";
  // The ros2_control parser rejects joints that the URDF does not define
  for (const auto& joint : joints) {
    urdf += "<link name=\"" + joint + "_link\"/>";
    urdf += "<joint name=\"" + joint + "\" type=\"revolute\"><parent link=\"base_link\"/><child link=\"" + joint +
            "_link\"/><axis xyz=\"0 0 1\"/><limit lower=\"-3\" upper=\"3\" effort=\"100\" velocity=\"10\"/></joint>";
  }
  urdf += "<ros2_control name=\"" + hardware + "\" type=\"system\">";
  urdf += "<hardware><plugin>mock_components/GenericSystem</plugin></hardware>";
  for (const auto& joint : joints) {
    urdf += "<joint name=\"" + joint + "\"><state_interface name=\"position\"/></joint>";
  }
  urdf += "</ros2_control></robot>";
  return urdf;
}

/**
 * @brief Build the interface description for one claimed interface.
 *
 * Brakes start engaged and `target_brake_state` takes the type the DuaDrive hardware interface exports.
 *
 * @param full_name Full interface name, `<prefix>/<interface>`.
 * @return The description with its initial value.
 */
hardware_interface::InterfaceDescription describe(const std::string& full_name)
{
  const auto slash = full_name.rfind('/');
  hardware_interface::InterfaceInfo info;
  info.name = full_name.substr(slash + 1);
  info.data_type = "double";
  info.initial_value = info.name == "brake_released" ? "0.0" : "1.0";
  if (info.name == "target_brake_state") {
    info.data_type = "int32";
    info.initial_value = "0";
  }
  return { full_name.substr(0, slash), info };
}

std::mutex g_errors_mutex;
std::vector<std::string> g_errors;
// Read on the executor thread when the hardware node logs
std::atomic<rcutils_logging_output_handler_t> g_default_output_handler{ nullptr };

/**
 * @brief Log output handler that records every error message and passes all output on to the default handler.
 */
void record_errors(const rcutils_log_location_t* location, int severity, const char* name,
                   rcutils_time_point_value_t timestamp, const char* format, va_list* args)
{
  if (severity >= RCUTILS_LOG_SEVERITY_ERROR) {
    va_list copy;
    va_copy(copy, *args);
    std::array<char, 1024> message{};
    std::vsnprintf(message.data(), message.size(), format, copy);
    va_end(copy);
    const std::lock_guard<std::mutex> lock(g_errors_mutex);
    g_errors.emplace_back(message.data());
  }
  g_default_output_handler.load()(location, severity, name, timestamp, format, args);
}
}  // namespace

class BrakeReleaseCouplingTest : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    spin_thread_ = std::thread([] { executor_->spin(); });
  }

  static void TearDownTestSuite()
  {
    // Shutting down the context ends the spin even if the thread has not entered it yet
    rclcpp::shutdown();
    spin_thread_.join();
    executor_.reset();
  }

  void SetUp() override
  {
    {
      const std::lock_guard<std::mutex> lock(g_errors_mutex);
      g_errors.clear();
    }
    g_default_output_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(record_errors);
  }

  void TearDown() override
  {
    rcutils_logging_set_output_handler(g_default_output_handler);
    if (hardware_node_) {
      executor_->remove_node(hardware_node_);
    }
  }

  /**
   * @brief Whether an error message containing the given text was logged during this test.
   */
  static bool logged_error(const std::string& text)
  {
    const std::lock_guard<std::mutex> lock(g_errors_mutex);
    return std::any_of(g_errors.begin(), g_errors.end(),
                       [&text](const std::string& message) { return message.find(text) != std::string::npos; });
  }

  /**
   * @brief Serve a torque coupling the way the coupled DuaDrive hardware interface publishes it.
   *
   * @param hardware Name of the ros2_control tag of the hardware.
   * @param joints Value of `torque_coupling.joints`.
   * @param matrix Value of `torque_coupling.matrix`, row-major, joint torques by drive torques.
   */
  void serve_coupling(const std::string& hardware, const std::vector<std::string>& joints,
                      const std::vector<double>& matrix)
  {
    std::string node_name = hardware;
    std::transform(node_name.begin(), node_name.end(), node_name.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    hardware_node_ = std::make_shared<rclcpp::Node>(
        node_name,
        rclcpp::NodeOptions()
            .parameter_overrides({ { "torque_coupling.joints", joints }, { "torque_coupling.matrix", matrix } })
            .automatically_declare_parameters_from_overrides(true));
    executor_->add_node(hardware_node_);
  }

  /**
   * @brief Initialize and configure a controller for kJoints against the given robot description.
   *
   * @param urdf Robot description handed to the controller.
   * @return The lifecycle state id the controller ends up in after configure.
   */
  uint8_t configure(const std::string& urdf)
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "brake_release_controller";
    params.robot_description = urdf;
    params.update_rate = kUpdateRate;
    params.controller_manager_update_rate = kUpdateRate;
    params.node_options = controller_.define_custom_node_options().parameter_overrides({ { "joints", kJoints } });
    EXPECT_EQ(controller_.init(params), controller_interface::return_type::OK);
    return controller_.get_node()->configure().id();
  }

  /**
   * @brief Hand the controller every interface it claims and activate it.
   *
   * @return Whether the controller reached the active state.
   */
  bool activate()
  {
    std::vector<hardware_interface::LoanedStateInterface> state_loans;
    for (const auto& name : controller_.state_interface_configuration().names) {
      state_storage_.push_back(std::make_shared<hardware_interface::StateInterface>(describe(name)));
      state_loans.emplace_back(state_storage_.back(), nullptr);
    }
    std::vector<hardware_interface::LoanedCommandInterface> command_loans;
    for (const auto& name : controller_.command_interface_configuration().names) {
      command_storage_.push_back(std::make_shared<hardware_interface::CommandInterface>(describe(name)));
      command_loans.emplace_back(command_storage_.back(), nullptr);
    }
    controller_.assign_interfaces(std::move(command_loans), std::move(state_loans));
    return controller_.get_node()->activate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  /**
   * @brief Value the controller last wrote to a claimed command interface.
   *
   * @param full_name Full interface name, `<prefix>/<interface>`.
   * @return The value, or NaN if the controller does not claim the interface.
   */
  double command(const std::string& full_name) const
  {
    for (const auto& interface : command_storage_) {
      if (interface->get_name() == full_name) {
        return duatic::controllers::compat::try_get_value(*interface).value_or(std::nan(""));
      }
    }
    return std::nan("");
  }

  static inline std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  static inline std::thread spin_thread_;
  rclcpp::Node::SharedPtr hardware_node_;
  // The interfaces must outlive the loans handed to the controller
  std::vector<hardware_interface::StateInterface::SharedPtr> state_storage_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_storage_;
  duatic::controllers::BrakeReleaseController controller_;
};

TEST_F(BrakeReleaseCouplingTest, SpreadsDriveOffsetsOverCoupledJoints)
{
  // In hardware order joint2, joint1: the drive of joint1 also turns joint2 by half its torque
  serve_coupling("CoupledSystem", { "joint2", "joint1" }, { 1.0, 0.5, 0.0, 1.0 });
  ASSERT_EQ(configure(make_urdf("CoupledSystem", kJoints)), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(activate());

  // One update ramps both drive offsets by torque_rate * period = 20 Nm/s * 0.1 s
  ASSERT_EQ(controller_.update(controller_.get_node()->now(), rclcpp::Duration::from_seconds(0.1)),
            controller_interface::return_type::OK);

  const std::string prefix = std::string(kGravityCompensation) + "/";
  EXPECT_DOUBLE_EQ(command(prefix + "joint1/effort"), 2.0);
  EXPECT_DOUBLE_EQ(command(prefix + "joint2/effort"), 2.0 + 0.5 * 2.0);
}

TEST_F(BrakeReleaseCouplingTest, RejectsDriveTurningJointOutsideJoints)
{
  // The drive of joint2 also turns joint3, which the controller does not sweep
  serve_coupling("LeakingSystem", { "joint1", "joint2", "joint3" },
                 { 1.0, 0.0, 0.0,  //
                   0.0, 1.0, 0.0,  //
                   0.0, 0.5, 1.0 });
  EXPECT_EQ(configure(make_urdf("LeakingSystem", { "joint1", "joint2", "joint3" })),
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_TRUE(logged_error("Drive 'joint2' also turns joint 'joint3'"));
}

TEST_F(BrakeReleaseCouplingTest, RejectsMatrixNotMatchingJoints)
{
  serve_coupling("MalformedSystem", kJoints, { 1.0, 0.0, 1.0 });
  EXPECT_EQ(configure(make_urdf("MalformedSystem", kJoints)), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_TRUE(logged_error("has 3 entries for 2 joints"));
}
