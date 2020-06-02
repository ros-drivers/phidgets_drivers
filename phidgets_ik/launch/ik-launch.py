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

    do_params = {
        'serial': -1,
        'hub_port': 0,
        'is_hub_port_device': False,
    }

    di_params = {
        'serial': -1,
        'hub_port': 0,
        'is_hub_port_device': False,
        'publish_rate': 0.0,
    }

    ai_params = {
        'serial': -1,
        'hub_port': 0,
        'is_hub_port_device': False,
        'data_interval_ms': 250,
        'publish_rate': 0.0,
    }

    container = ComposableNodeContainer(
            name='phidgets_ik',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='phidgets_digital_outputs',
                    plugin='phidgets::DigitalOutputsRosI',
                    name='phidgets_digital_outputs',
                    parameters=[do_params]),
                ComposableNode(
                    package='phidgets_digital_inputs',
                    plugin='phidgets::DigitalInputsRosI',
                    name='phidgets_digital_inputs',
                    parameters=[di_params]),
                ComposableNode(
                    package='phidgets_analog_inputs',
                    plugin='phidgets::AnalogInputsRosI',
                    name='phidgets_analog_inputs',
                    parameters=[ai_params]),
            ],
            output='both',
    )

    return launch.LaunchDescription([container])
