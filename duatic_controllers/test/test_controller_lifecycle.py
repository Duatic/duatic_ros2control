# Copyright 2026 Duatic AG

import os
import unittest

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from rclpy.duration import Duration

TEST_DIR = os.path.dirname(__file__)

# Must match test_controllers.yaml.
CONTROLLERS = [
    "brake_release_controller",
    "cartesian_pose_controller",
    "duadrive_pid_tuner",
    "duadrive_status_broadcaster",
    "force_torque_broadcaster",
    "freedrive_controller",
    "freeze_controller",
    "gravity_compensation_controller",
]


def generate_test_description():
    with open(os.path.join(TEST_DIR, "test_robot.urdf")) as urdf:
        robot_description = urdf.read()

    return (
        LaunchDescription(
            [
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    parameters=[{"robot_description": robot_description}],
                ),
                Node(
                    package="controller_manager",
                    executable="ros2_control_node",
                    parameters=[os.path.join(TEST_DIR, "test_controllers.yaml")],
                    output="screen",
                ),
                ReadyToTest(),
            ]
        ),
        {},
    )


class TestControllerLifecycle(unittest.TestCase):
    """Load, configure, activate, deactivate and reactivate every controller on mock hardware."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("controller_lifecycle_test")
        cls.load_client = cls.node.create_client(
            LoadController, "controller_manager/load_controller"
        )
        cls.configure_client = cls.node.create_client(
            ConfigureController, "controller_manager/configure_controller"
        )
        cls.switch_client = cls.node.create_client(
            SwitchController, "controller_manager/switch_controller"
        )
        cls.list_client = cls.node.create_client(
            ListControllers, "controller_manager/list_controllers"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request, timeout=30.0):
        self.assertTrue(
            client.wait_for_service(timeout_sec=timeout), f"{client.srv_name} not available"
        )
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertIsNotNone(future.result(), f"{client.srv_name} did not answer")
        return future.result()

    def _state(self, name):
        response = self._call(self.list_client, ListControllers.Request())
        return {controller.name: controller.state for controller in response.controller}.get(name)

    def _switch(self, name, activate):
        request = SwitchController.Request()
        if activate:
            request.activate_controllers = [name]
        else:
            request.deactivate_controllers = [name]
        request.strictness = SwitchController.Request.STRICT
        request.timeout = Duration(seconds=10.0).to_msg()
        self.assertTrue(self._call(self.switch_client, request).ok)
        self.assertEqual(self._state(name), "active" if activate else "inactive")

    def test_controller_lifecycle(self):
        for name in CONTROLLERS:
            with self.subTest(controller=name):
                self.assertTrue(self._call(self.load_client, LoadController.Request(name=name)).ok)
                self.assertTrue(
                    self._call(self.configure_client, ConfigureController.Request(name=name)).ok
                )
                self.assertEqual(self._state(name), "inactive")
                for activate in (True, False, True, False):
                    self._switch(name, activate)
