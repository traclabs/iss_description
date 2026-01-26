from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch_ros.actions import Node, SetParameter
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os
import xacro


def generate_launch_description():

  launch_args = [
    DeclareLaunchArgument(name="move_demo", default_value="True"),
  ]

  # URDF
  mss_urdf = os.path.join(get_package_share_directory("iss_description"), "robots", "mobile_servicing_system.urdf.xacro")
  mss_doc = xacro.process_file(mss_urdf)
  mss_urdf_content = mss_doc.toxml()

  # Robot state publisher
  mss_robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    output="screen",
    parameters=[
      {"robot_description": mss_urdf_content},
      {"use_sim_time": True},
    ],
  )

  # Spawn in Gazebo
  mss_spawn = Node(
    package="ros_gz_sim",
    executable="create",
    name="spawn",
    output="screen",
    arguments=[
      "-string",
      mss_urdf_content,
      "-name", "mobile_servicing_system",
      "-allow_renaming", "true",
    ]
  )

  # Test motion with services
  mss_move = Node(
    package="iss_description",
    executable="move_mss",
    output="screen",
    condition=IfCondition(LaunchConfiguration('move_demo'))
  )

  # Control
  joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--switch-timeout", "100.0"],
    name="start_joint_state_broadcaster",
    output="screen",
  )

  canadarm2_joint_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["canadarm2_joint_trajectory_controller", "-c", "/controller_manager", "--switch-timeout", "100.0"],
    name="start_canadarm2_joint_trajectory_controller",
    output="screen",
  )

  dextre_arm_1_joint_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["dextre_arm_1_joint_trajectory_controller", "-c", "/controller_manager", "--switch-timeout", "100.0"],
    name="start_dextre_arm_1_joint_trajectory_controller",
    output="screen",
  )

  dextre_arm_2_joint_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["dextre_arm_2_joint_trajectory_controller", "-c", "/controller_manager", "--switch-timeout", "100.0"],
    name="start_dextre_arm_2_joint_trajectory_controller",
    output="screen",
  )

  mobile_base_system_joint_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["mobile_base_system_joint_trajectory_controller", "-c", "/controller_manager", "--switch-timeout", "100.0"],
    name="start_mobile_base_system_joint_trajectory_controller",
    output="screen",
  )

  dextre_body_joint_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["dextre_body_joint_trajectory_controller", "-c", "/controller_manager", "--switch-timeout", "100.0"],
    name="start_dextre_body_joint_trajectory_controller",
    output="screen",
  )


  image_bridge = Node(
    package="ros_gz_image",
    executable="image_bridge",
    arguments=["/image_raw", "/image_raw"],
    output="screen"
  )

  return LaunchDescription( launch_args + [
    mss_robot_state_publisher,
    mss_spawn,
    mss_move,
    image_bridge,
    RegisterEventHandler(
      OnProcessExit(
        target_action=mss_spawn,
        on_exit=[joint_state_broadcaster_spawner],
      )
    ),
    RegisterEventHandler(
      OnProcessExit(
        target_action=joint_state_broadcaster_spawner,
        on_exit=[canadarm2_joint_controller_spawner, 
                 dextre_arm_1_joint_controller_spawner,
                 dextre_arm_2_joint_controller_spawner,
                 mobile_base_system_joint_controller_spawner,
                 dextre_body_joint_controller_spawner],
      )
    )
  ])
