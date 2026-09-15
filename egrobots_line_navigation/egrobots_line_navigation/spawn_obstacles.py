"""Spawn obstacles at fixed positions, so every test run sees the identical layout.

Single runs of the avoidance manoeuvre vary a lot, so comparisons are only
meaningful across repeated runs with the same obstacles in the same places.

    ros2 run egrobots_line_navigation spawn_obstacles --ros-args -p layout:=road
"""

import sys

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

# (x, y) of each obstacle. The road's centreline is y = 0 and its lane is 6 m
# wide, so these sit dead centre and 0.4 m either side of it.
LAYOUTS = {
    'road': [(8.0, 0.0), (18.0, 0.4), (27.0, -0.4)],
    'centre': [(10.0, 0.0), (20.0, 0.0)],
}


def box_sdf(size, height):
    return f'''<?xml version="1.0"?>
<sdf version="1.6"><model name="obstacle"><static>true</static><link name="link">
<collision name="c"><geometry><box><size>{size} {size} {height}</size></box></geometry></collision>
<visual name="v"><geometry><box><size>{size} {size} {height}</size></box></geometry>
<material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material></visual>
</link></model></sdf>'''


class SpawnObstacles(Node):

    def __init__(self):
        super().__init__('spawn_obstacles')
        self.declare_parameter('layout', 'road')
        self.declare_parameter('size', 1.0)
        self.declare_parameter('height', 1.2)
        self.client = self.create_client(SpawnEntity, '/spawn_entity')

    def run(self):
        layout = self.get_parameter('layout').value
        if layout not in LAYOUTS:
            self.get_logger().error(f'Unknown layout "{layout}"; choose from {list(LAYOUTS)}')
            return False
        if not self.client.wait_for_service(timeout_sec=60.0):
            self.get_logger().error('/spawn_entity not available — is Gazebo running?')
            return False

        size = self.get_parameter('size').value
        height = self.get_parameter('height').value
        ok = True
        for i, (x, y) in enumerate(LAYOUTS[layout]):
            request = SpawnEntity.Request()
            request.name = f'obstacle_{i}'
            request.xml = box_sdf(size, height)
            request.initial_pose = Pose()
            request.initial_pose.position.x = x
            request.initial_pose.position.y = y
            request.initial_pose.position.z = height / 2.0
            request.reference_frame = 'world'
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            response = future.result()
            if response is not None and response.success:
                self.get_logger().info(f'Spawned {request.name} at ({x:.1f}, {y:+.1f})')
            else:
                ok = False
                reason = response.status_message if response is not None else 'timed out'
                self.get_logger().error(f'Failed to spawn {request.name}: {reason}')
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = SpawnObstacles()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
