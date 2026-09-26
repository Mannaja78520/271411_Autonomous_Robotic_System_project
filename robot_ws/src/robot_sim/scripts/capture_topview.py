#!/usr/bin/env python3
"""Save a to-scale top view of the running Gazebo world as a PNG.

Spawns a temporary downward-looking camera far above the world centre, grabs
one frame, writes it to disk and removes the camera again.  The camera sits
400 m up with a field of view that covers exactly `extent` x `extent` metres of
floor, so perspective distortion is negligible (a 2.6 m wall top at the edge of
a 32 m view shifts by under 1 %) and the image can be drawn next to the
occupancy grid with the same metric axes.

Image orientation: up = world +y, right = world +x.  The floor area covered is
[cx - extent/2, cx + extent/2] x [cy - extent/2, cy + extent/2].

    ros2 run robot_sim capture_topview.py --ros-args -p output:=topview.png
"""

import math
import os

import numpy as np
import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import Image

CAMERA_NAME = 'topview_camera'
HEIGHT = 400.0

CAMERA_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <sensor name="camera" type="camera">
        <always_on>true</always_on>
        <update_rate>2</update_rate>
        <camera>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image>
            <width>{pixels}</width>
            <height>{pixels}</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>{near}</near>
            <far>{far}</far>
          </clip>
        </camera>
        <plugin name="topview_plugin" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>/topview</namespace>
          </ros>
          <camera_name>camera</camera_name>
          <frame_name>topview_link</frame_name>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>
"""


class TopViewCapture(Node):
    def __init__(self):
        super().__init__('capture_topview')
        self.declare_parameter('output', os.path.join(os.getcwd(), 'topview.png'))
        self.declare_parameter('extent', 32.0)     # m of floor covered, square
        self.declare_parameter('pixels', 1600)
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)

        self.output = self.get_parameter('output').value
        self.extent = self.get_parameter('extent').value
        self.pixels = self.get_parameter('pixels').value
        self.cx = self.get_parameter('center_x').value
        self.cy = self.get_parameter('center_y').value

        self.image = None
        self.frames = 0
        self.create_subscription(Image, '/topview/camera/image_raw', self.on_image, 1)
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')

    def on_image(self, msg: Image):
        self.frames += 1
        # Skip the first frames: the renderer may still be loading materials.
        if self.frames < 3:
            return
        data = np.frombuffer(msg.data, dtype=np.uint8)
        self.image = data.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(
            msg.height, msg.width, 3)

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=20.0):
            raise RuntimeError(f'{client.srv_name} not available - is Gazebo running?')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        return future.result()

    def run(self):
        hfov = 2.0 * math.atan(self.extent / 2.0 / HEIGHT)
        request = SpawnEntity.Request()
        request.name = CAMERA_NAME
        request.xml = CAMERA_SDF.format(name=CAMERA_NAME, hfov=hfov, pixels=self.pixels,
                                        near=HEIGHT - 10.0, far=HEIGHT + 10.0)
        pose = Pose()
        pose.position.x = self.cx
        pose.position.y = self.cy
        pose.position.z = HEIGHT
        # roll 0, pitch +90 deg (look down), yaw +90 deg (image up = world +y)
        cy, sy = math.cos(math.pi / 4), math.sin(math.pi / 4)
        cp, sp = math.cos(math.pi / 4), math.sin(math.pi / 4)
        pose.orientation.w = cy * cp
        pose.orientation.x = -sy * sp
        pose.orientation.y = cy * sp
        pose.orientation.z = sy * cp
        request.initial_pose = pose

        result = self.call(self.spawn_cli, request)
        if result is None or not result.success:
            raise RuntimeError(f'spawning the camera failed: {result}')
        self.get_logger().info('Camera spawned, waiting for an image...')

        try:
            deadline = self.get_clock().now().nanoseconds + 60e9
            while self.image is None and self.get_clock().now().nanoseconds < deadline:
                rclpy.spin_once(self, timeout_sec=0.5)
            if self.image is None:
                raise RuntimeError('no image received from the top-view camera')

            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            plt.imsave(self.output, self.image)
            half = self.extent / 2.0
            self.get_logger().info(
                f'Saved {self.output}: x [{self.cx - half}, {self.cx + half}] m, '
                f'y [{self.cy - half}, {self.cy + half}] m')
        finally:
            request = DeleteEntity.Request()
            request.name = CAMERA_NAME
            self.call(self.delete_cli, request)


def main():
    rclpy.init()
    node = TopViewCapture()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
