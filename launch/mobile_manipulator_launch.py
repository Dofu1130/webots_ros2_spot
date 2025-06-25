#!/usr/bin/env python

import os
import launch
from launch import LaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)


package_dir = get_package_share_directory("webots_spot")


def get_ros2_nodes(*args):
    """Define all the ROS 2 nodes that need to be restart on simulation reset here"""
    
    # Mobile Manipulator Driver node
    mobile_manipulator_ros2_control_params = os.path.join(
        package_dir, "resource", "mobile_manipulator_controllers.yaml"
    )
    
    mobile_manipulator_driver = WebotsController(
        robot_name="MobileManipulator",
        parameters=[
            {
                "robot_description": os.path.join(
                    package_dir, "resource", "mobile_manipulator.urdf"
                )
            },
            {"use_sim_time": True},
            {"set_robot_state_publisher": True},
            mobile_manipulator_ros2_control_params,
        ],
    )

    # ROS2 control spawners for the manipulator
    controller_manager_timeout = ["--controller-manager-timeout", "50"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""
    
    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"]
        + controller_manager_timeout,
    )
    
    # Arm trajectory controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["arm_controller", "-c", "/controller_manager"]
        + controller_manager_timeout,
    )
    
    # Gripper controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["gripper_controller", "-c", "/controller_manager"]
        + controller_manager_timeout,
    )

    ros2_control_spawners = [
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ]

    # Wait for the simulation to be ready to start the controllers
    waiting_nodes = WaitForControllerConnection(
        target_driver=mobile_manipulator_driver, 
        nodes_to_start=ros2_control_spawners
    )

    # Robot state publisher
    with open(os.path.join(package_dir, "resource", "mobile_manipulator.urdf")) as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_desc,
                "use_sim_time": True,
            }
        ],
    )

    return [
        mobile_manipulator_driver, 
        waiting_nodes, 
        robot_state_publisher
    ]


def generate_launch_description():
    """Generate the launch description"""
    
    # Webots launcher
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", "mobile_manipulator.wbt"])
    )
    
    # ROS2 supervisor
    ros2_supervisor = Ros2SupervisorLauncher()

    # Mobile base driver (for differential drive control)
    mobile_base_driver = WebotsController(
        robot_name="MobileBase",
        parameters=[
            {
                "robot_description": os.path.join(
                    package_dir, "resource", "mobile_manipulator.urdf"
                ),
                "use_sim_time": True,
                "differential_drive": True,
                "wheel_separation": 0.6,
                "wheel_radius": 0.1,
            }
        ],
        respawn=True,
    )

    # Teleop keyboard node
    teleop_keyboard = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",
        parameters=[{"use_sim_time": True}],
    )

    # Static transform from base_link to base_footprint
    base_footprint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "-0.1", "0", "0", "0", "base_link", "base_footprint"],
        parameters=[{"use_sim_time": True}],
    )

    # RViz2 for visualization
    rviz_config_file = os.path.join(
        package_dir, "resource", "mobile_manipulator_visualization.rviz"
    )
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        condition=launch.conditions.IfCondition("true"),  # Always start RViz
    )

    # Event handler for simulation reset
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=ros2_supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    # Event handler for Webots shutdown
    webots_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    return LaunchDescription(
        [
            webots,
            ros2_supervisor,
            mobile_base_driver,
            teleop_keyboard,
            base_footprint_tf,
            rviz2,
            webots_event_handler,
            reset_handler,
        ]
        + get_ros2_nodes()
    )