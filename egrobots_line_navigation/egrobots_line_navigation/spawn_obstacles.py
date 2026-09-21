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

# (x, y) of each obstacle. y is measured from the lane the rover drives in, not
# from the road's centreline, so obstacles stay in its way wherever the lane is.
#
# 'road' puts them towards the near (right-hand) row of parked cars, the way
# debris or an open door would sit at the kerb side. With the lane 1.5 m off
# the cars and 1 m boxes, the gaps left to the cars are 0.5, 0.4 and 0.3 m -
# all narrower than the rover (0.64 m across the wheels) - while each box still
# overlaps the rover's path by 32, 22 and 12 cm. The only way past is the wide
# side, so this is the layout that exercises avoidance's side rule.
#
# 'spread' is the previous layout (on the lane and 0.4 m either side of it),
# kept so earlier results can be reproduced.
LAYOUTS = {
    'road': [(8.0, -0.5), (18.0, -0.6), (27.0, -0.7)],
    'spread': [(8.0, 0.0), (18.0, 0.4), (27.0, -0.4)],
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
        # Sideways position of the lane, from the road's centreline at y = 0.
        # Keep this equal to lane_offset in config/line_params.yaml.
        self.declare_parameter('lane_offset', -1.5)
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
        lane = self.get_parameter('lane_offset').value
        ok = True
        for i, (x, y) in enumerate((x, y + lane) for x, y in LAYOUTS[layout]):
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
