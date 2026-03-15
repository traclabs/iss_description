from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from ros_gz_bridge.actions import RosGzBridge

import os

def generate_launch_description():

  launch_args = [
    DeclareLaunchArgument(name="top_left", default_value="True"),
    DeclareLaunchArgument(name="world_name", default_value="default"),
  ]

  iss_dir = os.path.join(get_package_share_directory("iss_description"))

  # Battery top right
  battery_top_right = os.path.join(iss_dir, "models", "s4_truss_23_battery_top_right", "model.sdf")
  spawn_top_right_battery = IncludeLaunchDescription(
    PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_spawn_model.launch.py"]),
    launch_arguments={"world": LaunchConfiguration("world_name"),
    "file": battery_top_right,
    "entity_name": "s4_truss_23_battery_top_right",
    "x": "-28.997",
    "y": "-6.090",
    "z": "4.408",
    "R": "1.047"}.items()
  )

  # Battery top left
  battery_top_left = os.path.join(iss_dir, "models", "s4_truss_23_battery_top_left", "model.sdf")
  spawn_top_left_battery = IncludeLaunchDescription(
    PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_spawn_model.launch.py"]),
    launch_arguments={"world": LaunchConfiguration("world_name"),
    "file": battery_top_left,
    "entity_name": "s4_truss_23_battery_top_left",
    "x": "-32.047",
    "y": "-6.073",
    "z": "4.412",
    "R": "1.047"}.items()
  )

  # Rest of batteries
  batteries_rest = os.path.join(iss_dir, "models", "s4_truss_23_batteries", "model.sdf")
  spawn_batteries_rest = IncludeLaunchDescription(
    PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_spawn_model.launch.py"]),
    launch_arguments={"world": LaunchConfiguration("world_name"),
    "file": batteries_rest,
    "entity_name": "s4_truss_23_batteries",
    "x": "-26.879",
    "y": "-4.789",
    "z": "4.986",
    "R": "1.047"}.items()
  )

  # Extra battery to be swapped, on pallet
  spawn_spare_battery = IncludeLaunchDescription(
    PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_spawn_model.launch.py"]),
    launch_arguments={"world": LaunchConfiguration("world_name"),
    "file": battery_top_right,
    "entity_name": "spare_battery",
    "x": "-11.9",
    "y": "-10.0",
    "z": "5.4",
    "R": "0.0",
    "P": "0.140",
    "Y": "-0.175"}.items() #Rotation: in RPY (radian) [1.606, 0.140, -0.175]
  )
  
  # Bridge to publish batteries TFs
  batteries_config_file=os.path.join(iss_dir, "config", "batteries_bridge_config.yaml")
  bridge_batteries = RosGzBridge(
    bridge_name="bridge_batteries",
    config_file=batteries_config_file
  )

  return LaunchDescription( launch_args + [
    spawn_top_right_battery,
    spawn_top_left_battery,
    spawn_batteries_rest,
    spawn_spare_battery,
    bridge_batteries
  ])
