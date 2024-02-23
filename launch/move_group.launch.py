# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_pal.arg_utils import read_launch_argument
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import TiagoProArgs
from dataclasses import dataclass


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    arm_type_right: DeclareLaunchArgument = TiagoProArgs.arm_type_right
    arm_type_left: DeclareLaunchArgument = TiagoProArgs.arm_type_left
    end_effector_right: DeclareLaunchArgument = TiagoProArgs.end_effector_right
    end_effector_left: DeclareLaunchArgument = TiagoProArgs.end_effector_left
    ft_sensor_right: DeclareLaunchArgument = TiagoProArgs.ft_sensor_right
    ft_sensor_left: DeclareLaunchArgument = TiagoProArgs.ft_sensor_left
    camera_model: DeclareLaunchArgument = TiagoProArgs.camera_model
    base_type: DeclareLaunchArgument = TiagoProArgs.base_type
    laser_model: DeclareLaunchArgument = TiagoProArgs.laser_model

    use_sim_time: DeclareLaunchArgument = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation time')
    use_sensor_manager_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name='use_sensor_manager',
        default_value='False',
        choices=['True', 'False'],
        description='Use moveit_sensor_manager for octomap')


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    launch_description.add_action(OpaqueFunction(function=start_move_group))
    return


def start_move_group(context, *args, **kwargs):

    arm_type_right = read_launch_argument('arm_type_right', context)
    arm_type_left = read_launch_argument('arm_type_left', context)
    end_effector_right = read_launch_argument('end_effector_right', context)
    end_effector_left = read_launch_argument('end_effector_left', context)
    ft_sensor_right = read_launch_argument('ft_sensor_right', context)
    ft_sensor_left = read_launch_argument('ft_sensor_left', context)
    camera_model = read_launch_argument('camera_model', context)
    laser_model = read_launch_argument('laser_model', context)
    base_type = read_launch_argument('base_type', context)
    use_sensor_manager = read_launch_argument('use_sensor_manager', context)

    robot_description_path = os.path.join(
        get_package_share_directory('tiago_pro_description'), 'robots', 'tiago_pro.urdf.xacro')

    mappings = {
        'end_effector_right': end_effector_right,
        'end_effector_left': end_effector_left,
        'ft_sensor_right': ft_sensor_right,
        'ft_sensor_left': ft_sensor_left,
        'camera_model': camera_model,
        'laser_model': laser_model,
        'base_type': base_type,
    }

    hw_suffix = get_hw_suffix(
        arm_right=arm_type_right,
        arm_left=arm_type_left,
        end_effector_right=end_effector_right,
        end_effector_left=end_effector_left,
        ft_sensor_right=ft_sensor_right,
        ft_sensor_left=ft_sensor_left)

    robot_description_semantic = (f'config/srdf/tiago_pro{hw_suffix}.srdf')

    # Trajectory Execution Functionality
    moveit_simple_controllers_path = (
        f'config/controllers/controllers{hw_suffix}.yaml')

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    moveit_config = (
        MoveItConfigsBuilder('tiago_pro')
        .robot_description(file_path=robot_description_path, mappings=mappings)
        .robot_description_semantic(file_path=robot_description_semantic)
        .robot_description_kinematics(file_path=os.path.join('config', 'kinematics_kdl.yaml'))
        .trajectory_execution(moveit_simple_controllers_path)
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(planning_scene_monitor_parameters)
        .pilz_cartesian_limits(file_path=os.path.join('config', 'pilz_cartesian_limits.yaml'))
    )

    if use_sensor_manager:
        # moveit_sensors path
        moveit_sensors_path = 'config/sensors_3d.yaml'
        moveit_config.sensors_3d(moveit_sensors_path)

    moveit_config.to_moveit_configs()

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            moveit_config.to_dict(),
        ],
    )

    return [run_move_group_node]


def get_hw_suffix(
        arm_right: str = 'no-arm',
        arm_left: str = 'no-arm',
        end_effector_right: str = 'no-end-effector',
        end_effector_left: str = 'no-end-effector',
        ft_sensor_right: str = 'no-ft-sensor',
        ft_sensor_left: str = 'no-ft-sensor'):

    if arm_left in ['no-arm']:
        suffix_left = arm_left
        return '_' + suffix_left

    components_left = []
    components_left.append(end_effector_left)

    if ft_sensor_left != 'no-ft-sensor':
        components_left.append(ft_sensor_left)

    suffix_left = '_' + '_'.join(components_left)

    if arm_right in ['no-arm']:
        suffix_right = arm_right
        return '_' + suffix_right

    components_right = []
    components_right.append(end_effector_right)

    if ft_sensor_right != 'no-ft-sensor':
        components_right.append(ft_sensor_right)

    suffix_right = '_' + '_'.join(components_right)

    suffix = suffix_left + suffix_right

    return suffix


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
