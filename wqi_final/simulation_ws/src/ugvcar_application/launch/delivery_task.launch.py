# Copyright 2026 liyongqihhh
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    delivery_targets = LaunchConfiguration("delivery_targets")
    start_point = LaunchConfiguration("start_point")
    wait_duration = LaunchConfiguration("wait_duration")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "delivery_targets",
            default_value="['teaching_building']",
            description="List of building names to deliver to",
        ),
        DeclareLaunchArgument(
            "start_point",
            default_value="logistics_center",
            description="Starting point name",
        ),
        DeclareLaunchArgument(
            "wait_duration",
            default_value="10.0",
            description="Seconds to wait at each stop",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the Gazebo simulation clock",
        ),
        Node(
            package="ugvcar_application",
            executable="delivery_task",
            name="delivery_task_manager",
            output="screen",
            parameters=[{
                "delivery_targets": delivery_targets,
                "start_point": start_point,
                "wait_duration": wait_duration,
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
