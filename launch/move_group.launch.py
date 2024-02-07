# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
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

from typing import Dict
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_pal.arg_utils import read_launch_argument


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_args = declare_launch_arguments()

    for arg in launch_args.values():
        ld.add_action(arg)

    # Execute move_group node
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld


def declare_launch_arguments() -> Dict:

    arg_dict = {}

    use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation time"
    )

    arg_dict[use_sim_time.name] = use_sim_time

    use_sensor_manager = DeclareLaunchArgument(
        "use_sensor_manager",
        default_value="False",
        description="Use sensor manager for octomap",
    )

    arg_dict[use_sensor_manager.name] = use_sensor_manager

    end_effector_right = DeclareLaunchArgument(
        "end_effector_right",
        default_value="pal-pro-gripper",
        description="End effector model of the right arm.",
        choices=["pal-pro-gripper", "no-ee"],
    )

    arg_dict[end_effector_right.name] = end_effector_right

    end_effector_left = DeclareLaunchArgument(
        "end_effector_left",
        default_value="pal-pro-gripper",
        description="End effector model of the left arm.",
        choices=["pal-pro-gripper", "no-ee"],
    )

    arg_dict[end_effector_left.name] = end_effector_left

    ft_sensor_right = DeclareLaunchArgument(
        "ft_sensor_right",
        default_value="rokubi",
        description="FT sensor model. ",
        choices=["rokubi", "no-ft-sensor"],
    )

    arg_dict[ft_sensor_right.name] = ft_sensor_right

    ft_sensor_left = DeclareLaunchArgument(
        "ft_sensor_left",
        default_value="rokubi",
        description="FT sensor model. ",
        choices=["rokubi", "no-ft-sensor"],
    )

    arg_dict[ft_sensor_left.name] = ft_sensor_left

    return arg_dict


def launch_setup(context, *args, **kwargs):
    end_effector_left = read_launch_argument("end_effector_left", context)
    ft_sensor_left = read_launch_argument("ft_sensor_left", context)
    end_effector_right = read_launch_argument("end_effector_right", context)
    ft_sensor_right = read_launch_argument("ft_sensor_right", context)
    use_sensor_manager = read_launch_argument("use_sensor_manager", context)

    robot_description_path = os.path.join(
        get_package_share_directory("tiago_pro_description"),
        "robots",
        "tiago_pro.urdf.xacro",
    )
    mappings = {
        "end_effector_left": end_effector_left,
        "end_effector_right": end_effector_right,
        "ft_sensor_left": ft_sensor_left,
        "ft_sensor_right": ft_sensor_right,
    }

    robot_description_semantic = os.path.join(
        get_package_share_directory("tiago_pro_moveit_config"),
        "config/srdf/tiago_pro.srdf",
    )

    # Trajectory Execution Functionality
    moveit_simple_controllers_path = os.path.join(
        get_package_share_directory("tiago_pro_moveit_config"),
        "config/controllers/controllers_pal-pro-gripper.yaml",
    )

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    moveit_config = (
        MoveItConfigsBuilder("tiago_pro")
        .robot_description(file_path=robot_description_path, mappings=mappings)
        .robot_description_semantic(file_path=robot_description_semantic)
        .robot_description_kinematics(
            file_path=os.path.join("config", "kinematics_kdl.yaml")
        )
        .trajectory_execution(moveit_simple_controllers_path)
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(planning_scene_monitor_parameters)
        .pilz_cartesian_limits(
            file_path=os.path.join("config", "pilz_cartesian_limits.yaml")
        )
    )
    if use_sensor_manager:
        # moveit_sensors path
        moveit_sensors_path = "config/sensors_3d.yaml"
        moveit_config.sensors_3d(moveit_sensors_path)

    moveit_config.to_moveit_configs()

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            moveit_config.to_dict(),
            {"publish_robot_description_semantic": True},
        ],
    )

    return [run_move_group_node]
