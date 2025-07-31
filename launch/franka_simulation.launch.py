#!/usr/bin/env python3
"""
FRANKA Gazebo Simulation Launch File

FRANKA Panda robotとGazeboシミュレーション環境を起動します。
MoveIt2統合とRL環境の初期化も含みます。
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Launch description generator"""
    
    # Launch arguments
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Start Gazebo with GUI'
    )
    
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='franka_world.world',
        description='Gazebo world file to use'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz for visualization'
    )
    
    # Package paths
    pkg_share = FindPackageShare('robo_manipulation_env')
    franka_description_share = FindPackageShare('franka_description')
    franka_gazebo_share = FindPackageShare('franka_gazebo')
    
    # World file path
    world_file_path = PathJoinSubstitution([
        pkg_share, 'worlds', LaunchConfiguration('world_file')
    ])
    
    # Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file_path],
        output='screen'
    )
    
    # Gazebo client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )
    
    # Robot state publisher
    robot_description_config = PathJoinSubstitution([
        franka_description_share, 'robots', 'fr3.urdf.xacro'
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': True
        }]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'franka_robot',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.0'
        ]
    )
    
    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'franka_controllers.yaml']),
            {'use_sim_time': True}
        ]
    )
    
    # Load controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'fr3_arm_controller'],
        output='screen'
    )
    
    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'fr3_gripper_controller'],
        output='screen'
    )
    
    # MoveIt2 integration
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('franka_fr3_moveit_config'),
                'launch', 'moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_rviz': LaunchConfiguration('use_rviz')
        }.items()
    )
    
    # Data management node
    data_manager_node = Node(
        package='robo_manipulation_env',
        executable='data_manager_node',
        name='data_manager_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Safety monitor node
    safety_monitor_node = Node(
        package='robo_manipulation_env',
        executable='safety_monitor_node',
        name='safety_monitor_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        # Arguments
        use_gui_arg,
        world_file_arg,
        use_rviz_arg,
        
        # Gazebo
        gazebo_server,
        gazebo_client,
        
        # Robot
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        
        # Controllers
        controller_manager,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller,
        
        # MoveIt2
        moveit_launch,
        
        # System nodes
        data_manager_node,
        safety_monitor_node,
    ])