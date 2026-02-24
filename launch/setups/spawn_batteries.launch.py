from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import os
import xacro


BRIDGE_FIXTURES = ['mobile_servicing_system', 'dextre_arm_1', 'dextre_arm_2']
BRIDGE_CMDS = ['attach', 'detach']

def generate_launch_description():

  launch_args = [
    DeclareLaunchArgument(name="top_left", default_value="True"),
    DeclareLaunchArgument(name="world_name", default_value="default"),
  ]

  # Battery top right
  battery_top_right = os.path.join(get_package_share_directory("iss_description"), "models", "s4_truss_23_battery_top_right", "model.sdf")
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

  bridge_top_right_battery_tf = Node(
    name="bridge_s4_truss_23_battery_top_right_pose",
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
      "/model/s4_truss_23_battery_top_right/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
    ],
    remappings=[
      ("/model/s4_truss_23_battery_top_right/pose", "/tf"),
    ],
    output="screen"
  )

  # create all of the relevant bridge topics for attaching/detaching
  bridge_top_right_cmds = [
    Node(
      name=f"bridge_s4_truss_23_battery_top_right_{fixture}_{cmd}",
      package="ros_gz_bridge",
      executable="parameter_bridge",
      arguments=[
        f"/model/s4_truss_23_battery_top_right/{fixture}/{cmd}@std_msgs/msg/Empty]gz.msgs.Empty"
      ],
      remappings=[
        (f"/model/s4_truss_23_battery_top_right/{fixture}/{cmd}", f"/s4_truss_23_battery_top_right/{fixture}/{cmd}"),
      ],
      output="screen"
    )
    for fixture in BRIDGE_FIXTURES for cmd in BRIDGE_CMDS
  ]

  # Battery top left
  battery_top_left = os.path.join(get_package_share_directory("iss_description"), "models", "s4_truss_23_battery_top_left", "model.sdf")
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

  bridge_top_left_battery_tf = Node(
    name="bridge_s4_truss_23_battery_top_left_pose",
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
      "/model/s4_truss_23_battery_top_left/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
    ],
    remappings=[
      ("/model/s4_truss_23_battery_top_left/pose", "/tf"),
    ],
    output="screen"
  )

  # create all of the relevant bridge topics for attaching/detaching
  bridge_top_left_cmds = [
    Node(
      name=f"bridge_s4_truss_23_battery_top_left_{fixture}_{cmd}",
      package="ros_gz_bridge",
      executable="parameter_bridge",
      arguments=[
        f"/model/s4_truss_23_battery_top_left/{fixture}/{cmd}@std_msgs/msg/Empty]gz.msgs.Empty"
      ],
      remappings=[
        (f"/model/s4_truss_23_battery_top_left/{fixture}/{cmd}", f"/s4_truss_23_battery_top_left/{fixture}/{cmd}"),
      ],
      output="screen"
    )
    for fixture in BRIDGE_FIXTURES for cmd in BRIDGE_CMDS
  ]

  # Rest of batteries
  batteries_rest = os.path.join(get_package_share_directory("iss_description"), "models", "s4_truss_23_batteries", "model.sdf")
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

  return LaunchDescription( launch_args + [
    spawn_top_right_battery,
    bridge_top_right_battery_tf,
    *bridge_top_right_cmds,
    spawn_top_left_battery,
    bridge_top_left_battery_tf,
    *bridge_top_left_cmds,
    spawn_batteries_rest,
  ])
