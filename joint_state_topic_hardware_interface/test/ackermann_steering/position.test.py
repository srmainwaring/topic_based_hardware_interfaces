# Copyright 2026 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import unittest
from collections import OrderedDict
from pathlib import Path

import launch_testing
import launch_testing.markers
import math
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from controller_manager.test_utils import (
    check_controllers_running,
    check_if_js_published,
    check_node_running,
)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped


# This function specifies the processes to be run for our test
@pytest.mark.rostest
def generate_test_description():
    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env["PYTHONUNBUFFERED"] = "1"

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            str(Path(os.path.realpath(__file__)).parent),
                            "ackermann_steering.launch.py",
                        ],
                    ),
                ),
            ),
            Node(
                package="topic_tools",
                executable="relay",
                output="screen",
                arguments=["/robot_joint_commands", "/robot_joint_states"],
            ),
            KeepAliveProc(),
            ReadyToTest(),
        ],
    )


class TestFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_node")

        # Reported joint state from joint_state_broadcaster
        self.current_joint_state_subscriber = self.node.create_subscription(
            JointState,
            "joint_states",
            self.joint_states_callback,
            qos_profile_system_default,
        )

        self.joint_names = [
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
            "left_wheel_steering_joint",
            "right_wheel_steering_joint",
            "front_left_wheel_joint",
            "front_right_wheel_joint"
        ]

    def tearDown(self):
        self.node.destroy_node()

    def test_controller_running(self, proc_output):
        cnames = ["ackermann_steering_controller", "joint_state_broadcaster"]
        check_node_running(self.node, "relay")
        check_controllers_running(self.node, cnames)

    def test_check_if_msgs_published(self):
        check_if_js_published("/joint_states", self.joint_names)

    def test_main(self, launch_service, proc_info, proc_output):
        # By default the joint_states should have the initial_value from ackermann_steering.urdf.xacro
        self.node.get_logger().info("Checking initial joint states...")
        current_position_joint_state, current_joint_state_msg = self.get_current_joint_state()
        urdf_position_initial_values = [
            float("nan"),
            float("nan"),
            0.0,
            0.0,
            float("nan"),
            float("nan")
        ]
        for (current_position, urdf_initial) in zip(current_position_joint_state, urdf_position_initial_values):
            if math.isnan(urdf_initial):
                assert math.isnan(current_position), (
                    f"{math.isnan(current_position)=} != {math.isnan(urdf_initial)=}"
                )
            else:
                assert current_position == urdf_initial, (
                    f"{current_position=} != {urdf_initial=}"
                )

        # Check array sizes of joint_state message
        assert len(current_joint_state_msg.position) == len(current_joint_state_msg.name), (
            f"{len(current_joint_state_msg.position)=} != {len(current_joint_state_msg.name)=}"
        )
        assert len(current_joint_state_msg.velocity) == len(current_joint_state_msg.name), (
            f"{len(current_joint_state_msg.velocity)=} != {len(current_joint_state_msg.name)=}"
        )
        assert len(current_joint_state_msg.effort) == len(current_joint_state_msg.effort), (
            f"{len(current_joint_state_msg.effort)=} != {len(current_joint_state_msg.name)=}"
        )

        # Check effort arrays contain NaN
        for cmd in current_joint_state_msg.effort:
            assert type(cmd) == float, (f"{type(cmd)=} != {float=}")
            assert math.isnan(cmd), (f"{math.isnan(cmd)=}")

        # Test setting the robot joint states via controller
        # test_ackermann_steering was not installed, call it directly from build space
        pkg_name = "joint_state_topic_hardware_interface"
        proc_action = Node(
            executable=os.path.join(
                get_package_prefix(pkg_name).replace("install", "build"),
                "test_ackermann_steering",
            ),
            output="screen",
        )

        with launch_testing.tools.launch_process(
            launch_service, proc_action, proc_info, proc_output
        ):
            proc_info.assertWaitForShutdown(process=proc_action, timeout=300)
            launch_testing.asserts.assertExitCodes(
                proc_info, process=proc_action, allowable_exit_codes=[0]
            )

        self.node.get_logger().info("Checking final joint states...")
        current_position_joint_state, _ = self.get_current_joint_state()

        position_final_values = [
            float("nan"),
            float("nan"),
            0.5 * math.pi,
            0.5 * math.pi,
            float("nan"),
            float("nan")
        ]
        for (current_position, final_position) in zip(current_position_joint_state, position_final_values):
            if math.isnan(final_position):
                assert math.isnan(current_position), (
                    f"{math.isnan(current_position)=} != {math.isnan(final_position)=}"
                )
            else:
                assert np.allclose(
                    current_position,
                    final_position,
                    atol=1e-3
                ), (f"{current_position=} != {final_position=}")

    def joint_states_callback(self, msg: JointState):
        self.current_position_joint_state = self.filter_joint_state_msg(msg)
        self.current_joint_state_msg = msg

    def get_current_joint_state(self) -> OrderedDict[str, float]:
        """Get the current joint state (position) reported by ros2_control on joint_states topic."""
        self.current_joint_state_msg = None
        self.current_position_joint_state = []
        while len(self.current_position_joint_state) == 0:
            self.node.get_logger().warning(
                f"Waiting for current joint states from topic '{self.current_joint_state_subscriber.topic_name}'...",
                throttle_duration_sec=2.0,
                skip_first=True,
            )
            rclpy.spin_once(self.node, timeout_sec=1.0)
        return self.current_position_joint_state, self.current_joint_state_msg

    def filter_joint_state_msg(self, msg: JointState):
        joint_states = []
        for joint_name in self.joint_names:
            try:
                index = msg.name.index(joint_name)
            except ValueError:
                msg = f"Joint name '{joint_name}' not in input keys {msg.name}"
                raise ValueError(msg) from None
            joint_states.append(msg.position[index])
        return joint_states


@launch_testing.post_shutdown_test()
class TestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes (successful codes)
    def test_pass(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
