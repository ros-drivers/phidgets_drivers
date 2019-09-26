# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Launch a Phidgets IK in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            node_name='phidgets_ik',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='phidgets_digital_outputs',
                    node_plugin='phidgets::DigitalOutputsRosI',
                    node_name='phidgets_digital_outputs'),
                ComposableNode(
                    package='phidgets_digital_inputs',
                    node_plugin='phidgets::DigitalInputsRosI',
                    node_name='phidgets_digital_inputs'),
                ComposableNode(
                    package='phidgets_analog_inputs',
                    node_plugin='phidgets::AnalogInputsRosI',
                    node_name='phidgets_analog_inputs'),
            ],
            output='both',
    )

    return launch.LaunchDescription([container])
