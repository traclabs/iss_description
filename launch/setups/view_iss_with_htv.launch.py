import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(name="rviz", default_value="True"),
        DeclareLaunchArgument(name="robot_publisher", default_value="True"),
        DeclareLaunchArgument(name="joint_publisher", default_value="True"),
    ]

    # Urdf for MSS
    robot_dir = get_package_share_directory("iss_description")
    urdf_string = xacro.process_file(
        os.path.join(robot_dir, "robots/mobile_servicing_system.urdf.xacro"),
    )
    #urdf_string = xacro.process_file(
    #    os.path.join(robot_dir, "robots/setups/mobile_servicing_system_with_htv_pallet.urdf.xacro"),
    #)    
    robot_description = {"robot_description": urdf_string.toxml()}

    # Robot state publisher for MSS
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        condition=IfCondition(LaunchConfiguration('robot_publisher'))
    )

    # Joint State publisher for MSS
    mss_zero_joints = {
      "zeros.joint_canadarm2_2": 0.628,
      "zeros.joint_canadarm2_3": -0.187,
      "zeros.joint_starboard_sarj": 1.0472,
      "zeros.joint_port_sarj": 1.0472
    }
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        parameters=[mss_zero_joints],
        output='screen',
        condition=IfCondition(LaunchConfiguration('joint_publisher'))
    )
    
    # Urdf for HTV
    htv_urdf_string = xacro.process_file(
        os.path.join(robot_dir, "models/htv1/urdf/htv1.urdf.xacro"),
        mappings ={'x': '-15.0', 'y': '-20.0', 'z': '0.0', 'roll': '0.0', 'pitch': '0.0', 'yaw': '-1.5708', 'parent': 'world'}
    )
    htv_robot_description = {"robot_description": htv_urdf_string.toxml()}

    # Robot state publisher for HTV
    htv_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[htv_robot_description],
        remappings=[('/robot_description', '/htv/robot_description')],
        condition=IfCondition(LaunchConfiguration('robot_publisher'))
    )


    # Rviz
    rviz_config = os.path.join(robot_dir, "rviz/iss_with_htv.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
        robot_description
        ]
        ,
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        launch_args +
        [rsp, jsp, htv_rsp, rviz]
    )
