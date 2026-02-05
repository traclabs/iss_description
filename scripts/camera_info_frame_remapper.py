#!/usr/bin/env python3
"""
Camera Frame ID Remapper

Subscribes to camera_info and image_raw topics from Gazebo (which use scoped frame names),
remaps the frame_id to match the TF tree frame names, and republishes.

This is necessary because Gazebo Harmonic uses scoped frame names like:
  'mobile_servicing_system/boom_a_clpa_tilt_link/boom_a_clpa_camera'
But the TF tree uses simple names like:
  'boom_a_clpa_optical_frame'
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CameraInfoFrameRemapper(Node):
    def __init__(self):
        super().__init__('camera_frame_remapper')

        # Camera names to remap
        self.camera_names = [
            'boom_a_clpa',
            'boom_b_clpa',
            'outrigger_1_clpa',
            'outrigger_2_clpa',
            'etvcg_cp3',
            'etvcg_cp8',
            'etvcg_cp9',
            'etvcg_cp13'
        ]

        # Create subscribers and publishers for each camera
        self.info_subs = {}
        self.info_pubs = {}
        self.image_subs = {}
        self.image_pubs = {}

        for camera_name in self.camera_names:
            # Subscribe to the bridged camera_info (with wrong frame_id)
            self.info_subs[camera_name] = self.create_subscription(
                CameraInfo,
                f'/{camera_name}/camera_info_gz',  # Renamed topic from bridge
                lambda msg, name=camera_name: self.camera_info_callback(msg, name),
                10
            )

            # Publish corrected camera_info
            self.info_pubs[camera_name] = self.create_publisher(
                CameraInfo,
                f'/{camera_name}/camera_info',
                10
            )

            # Subscribe to the bridged image_raw (with wrong frame_id)
            self.image_subs[camera_name] = self.create_subscription(
                Image,
                f'/{camera_name}/image_raw_gz',  # Renamed topic from bridge
                lambda msg, name=camera_name: self.image_callback(msg, name),
                10
            )

            # Publish corrected image_raw
            self.image_pubs[camera_name] = self.create_publisher(
                Image,
                f'/{camera_name}/image_raw',
                10
            )

        self.get_logger().info(f'Camera frame remapper started for {len(self.camera_names)} cameras')

    def camera_info_callback(self, msg: CameraInfo, camera_name: str):
        """Remap frame_id and republish"""
        # Change frame_id to the optical frame name
        msg.header.frame_id = f'{camera_name}_optical_frame'

        # Republish with corrected frame_id
        self.info_pubs[camera_name].publish(msg)

    def image_callback(self, msg: Image, camera_name: str):
        """Remap frame_id and republish"""
        # Change frame_id to the optical frame name
        msg.header.frame_id = f'{camera_name}_optical_frame'

        # Republish with corrected frame_id
        self.image_pubs[camera_name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoFrameRemapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
