from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python.packages import get_package_share_directory

import os
import xacro


def generate_launch_description():

  gz_gui = DeclareLaunchArgument(
    'gz_gui',
    default_value='true',
    description='Enable/disable Gazebo GUI (true/false)'
  )

  gui = LaunchConfiguration('gz_gui')

  # Gazebo setup
  sim_resource_path = os.pathsep.join([
    os.environ.get("GZ_SIM_RESOURCE_PATH", default=""),
    os.path.join(get_package_share_directory("gateway_description"), "models" )
  ])
  env_gz_sim = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", sim_resource_path)

  # World
  leo_sdf = PathJoinSubstitution([
    FindPackageShare("iss_description"), "worlds", "leo.sdf"
  ])

  # Launch gazebo world
  gz_launch_gui = IncludeLaunchDescription(
    PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]),
    launch_arguments=[
      ("gz_args", [
          leo_sdf,
          " -r",
          " -v 4",
      ])
    ],
    condition=IfCondition(gui)
  )

    # Launch gazebo world
  gz_launch_headless = IncludeLaunchDescription(
    PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]),
    launch_arguments=[
      ("gz_args", [
          leo_sdf,
          " -r",
          " -v 4",
          " -s"
      ])
    ],
    condition=UnlessCondition(gui)
  )

  # Spawn Mobile Servicing System
  mss = IncludeLaunchDescription(
    PathJoinSubstitution([
      FindPackageShare("iss_description"), "launch", "spawn_mobile_servicing_system.launch.py"
    ])
  )


  # Make the /clock topic available in ROS
  gz_sim_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
      "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    ],
    output="screen",
  )

  return LaunchDescription([
    SetParameter(name="use_sim_time", value=True),
    env_gz_sim,
    gz_gui,
    gz_launch_gui,
    gz_launch_headless,
    mss,
    gz_sim_bridge
  ])
