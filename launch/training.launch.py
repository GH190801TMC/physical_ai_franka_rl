#!/usr/bin/env python3
"""
PPO Training Launch File

強化学習訓練環境を起動します。
シミュレーション、MoveIt2、PPOトレーナーを統合起動。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Launch description for PPO training"""
    
    # Arguments
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run simulation without GUI for faster training'
    )
    
    training_config_arg = DeclareLaunchArgument(
        'training_config',
        default_value='training_config.json',
        description='Training configuration file'
    )
    
    # Include base simulation
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robo_manipulation_env'),
                'launch', 'franka_simulation.launch.py'
            ])
        ]),
        launch_arguments={
            'use_gui': LaunchConfiguration('headless'),
            'use_rviz': 'false'  # Disable RViz for training
        }.items()
    )
    
    # PPO Trainer node
    ppo_trainer_node = Node(
        package='robo_manipulation_env',
        executable='ppo_trainer_node',
        name='ppo_trainer_node',
        output='screen',
        parameters=[{
            'training_config_file': LaunchConfiguration('training_config'),
            'use_sim_time': True
        }]
    )
    
    # Training coordinator
    training_coordinator_node = Node(
        package='robo_manipulation_env',
        executable='training_coordinator_node',
        name='training_coordinator_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Performance monitor
    performance_monitor_node = Node(
        package='robo_manipulation_env',
        executable='performance_monitor_node',
        name='performance_monitor_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        headless_arg,
        training_config_arg,
        simulation_launch,
        ppo_trainer_node,
        training_coordinator_node,
        performance_monitor_node,
    ])