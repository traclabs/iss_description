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

  sim_time_params = [{"use_sim_time": True}]

  # Bridge nodes for pan-tilt cameras
  camera_names = [
    "boom_a_clpa",
    "boom_b_clpa",
    "outrigger_1_clpa",
    "outrigger_2_clpa",
    "etvcg_cp3",
    "etvcg_cp8",
    "etvcg_cp9",
    "etvcg_cp13"
  ]

  camera_bridges = []
  for camera_name in camera_names:
    # Bridge image_raw topic to temporary topic (frame_id will be wrong)
    camera_bridges.append(
      Node(
        package="ros_gz_image",
        executable="image_bridge",
        name=f"{camera_name}_image_bridge",
        arguments=[f"/{camera_name}/image_raw"],
        output="screen",
        parameters=[{
          "use_sim_time": True,
        }],
      )
    )
    # Bridge camera_info topic to temporary topic (frame_id will be wrong)
    camera_bridges.append(
      Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[f"/{camera_name}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        output="screen",
        name=f"{camera_name}_info_bridge",
        parameters=[{
          'use_sim_time': True,
        }]
      )
    )

  camera_joint_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["camera_joint_trajectory_controller", "-c", "/controller_manager", "--switch-timeout", "100.0"],
    name="start_camera_joint_trajectory_controller",
    output="screen",
    parameters=sim_time_params,
  )

  return LaunchDescription( 
    [camera_joint_controller_spawner] + 
    camera_bridges
  )  
  

