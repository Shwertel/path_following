"""Sweep a greenhouse: every lane in turn, then park on the charging bay.

    ros2 run egrobots_line_navigation greenhouse_mission_node

The navigator already drives one lane. A sweep is that, repeated, with the
turns between lanes and the drive to the bay in between — so this node is a
client of the same FollowLine action rather than new navigation code, and the
whole mission is a list of legs:

    lane A  (0.5, 0.00) -> (24.0, 0.00)      down the first lane
    turn    (24.0, 0.00) -> (24.0, 2.05)     across the far headland
    lane B  (24.0, 2.05) -> (0.5, 2.05)      back down the second
    turn    (0.5, 2.05) -> (0.5, -2.05)      across the near headland
    lane C  (0.5, -2.05) -> (24.0, -2.05)    down the third
    align   (24.0, -2.05) -> (24.5, 0.0)     line up with the bay
    dock    (24.5, 0.0) -> (26.3, 0.0)       park

Each lane leg runs from headland to headland, so the rover enters and leaves
every lane in a straight line and the turns are pure sideways moves in the open.

Two things have to change between legs, and both are the same point: the lane a
leg is about. On a lane leg the rover steers by the lane measured either side of
it, and the localizer must forget the previous lane first or it will look for
rows where this one has none. On a turn there is no lane to steer by, so it
steers by the straight line between the two waypoints.

The waypoints above are nominal. They are not used as written, because the
rover's estimate of where it is drifts sideways as it drives a lane - by about
a centimetre per metre, from the error in the lane direction it measured on
entry. Driving the next leg to an absolute coordinate would then enter the next
lane off centre by that drift; at 0.6 m off centre in a 1.6 m lane the rover is
close enough to the plants that the lane cannot be measured at all, and it
drives into them. So each leg is planned from where the rover *believes* it
finished the last one: the drift is common to both ends of a sideways move, and
cancels.
"""

import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Point, PoseStamped
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

from egrobots_line_interfaces.action import FollowLine

NAVIGATOR = '/line_navigator_node'
LOCALIZER = '/row_localizer_node'


class Leg:

    def __init__(self, name, start, goal, lane, tolerance=0.0):
        self.name = name
        self.start = start          # (x, y)
        self.goal = goal
        self.lane = lane            # True: steer by the measured lane
        # A turn ends when the rover is within tolerance of its end, and the
        # turn's end IS the next lane's centre, so slack there is slack across
        # the lane. Turns and the dock get a tighter one than a lane leg, where
        # tolerance only decides where along the lane it stops.
        self.tolerance = tolerance

    def __str__(self):
        return (f'{self.name}: ({self.start[0]:.1f}, {self.start[1]:.2f}) -> '
                f'({self.goal[0]:.1f}, {self.goal[1]:.2f})'
                f'{" [lane]" if self.lane else " [open]"}')


class GreenhouseMission(Node):

    def __init__(self):
        super().__init__('greenhouse_mission_node')

        # The lanes, in the order they are driven. Defaults match
        # greenhouse_world.world: three lanes 2.05 m apart.
        self.declare_parameter('lane_centres', [0.0, 2.05, -2.05])
        self.declare_parameter('near_headland_x', 0.5)
        self.declare_parameter('far_headland_x', 24.0)
        self.declare_parameter('charging_bay', [26.3, 0.0])
        # How far in front of the bay to line up, so the rover parks straight.
        self.declare_parameter('bay_approach', 1.8)

        # Docking is done by looking at the bay, not by dead reckoning to its
        # coordinates: after a sweep of a greenhouse the rover's estimate of
        # where it is has drifted by the better part of a metre, which is more
        # than the pad is wide. The board behind the pad is the landmark.
        self.declare_parameter('dock_by_sight', True)
        self.declare_parameter('dock_standoff', 1.1)      # pad centre to board
        self.declare_parameter('dock_search_deg', 35.0)
        self.declare_parameter('dock_search_min', 0.5)
        self.declare_parameter('dock_search_max', 5.0)

        self.shift = (0.0, 0.0)     # plan -> where the rover believes it is
        self.scan = None
        self.pose = None
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, '/robot_pose', self.pose_callback, 10)
        self.navigator = ActionClient(self, FollowLine, '/follow_line')
        self.set_params = self.create_client(
            SetParameters, f'{NAVIGATOR}/set_parameters')
        self.reset_lane = self.create_client(Trigger, f'{LOCALIZER}/reset_lane')

    # ------------------------------------------------------------------

    def plan(self):
        """Build the leg list: lanes in order, with the turns between them."""
        lanes = list(self.get_parameter('lane_centres').value)
        near = self.get_parameter('near_headland_x').value
        far = self.get_parameter('far_headland_x').value
        bay = list(self.get_parameter('charging_bay').value)
        approach = self.get_parameter('bay_approach').value

        legs = []
        at = None
        for i, y in enumerate(lanes):
            # Alternate direction: down one lane, back up the next.
            start_x, goal_x = (near, far) if i % 2 == 0 else (far, near)
            if at is not None:
                legs.append(Leg(f'turn into lane {i + 1}', at, (start_x, y),
                                False, tolerance=0.10))
            legs.append(Leg(f'lane {i + 1} of {len(lanes)}',
                            (start_x, y), (goal_x, y), True))
            at = (goal_x, y)

        # Line up across the headland, then turn to face the bay and edge
        # forward: the bay can only be measured with it in front of the rover,
        # and the line-up leg leaves the rover facing across the greenhouse.
        legs.append(Leg('line up with the bay', at,
                        (bay[0] - approach, bay[1]), False, tolerance=0.10))
        legs.append(Leg('face the bay', (bay[0] - approach, bay[1]),
                        (bay[0] - approach + 0.5, bay[1]), False, tolerance=0.10))
        legs.append(Leg('dock', (bay[0] - approach + 0.5, bay[1]), tuple(bay),
                        False, tolerance=0.10))
        return legs

    def shifted(self, point):
        return (point[0] + self.shift[0], point[1] + self.shift[1])

    def scan_callback(self, msg):
        self.scan = msg

    def pose_callback(self, msg):
        q = msg.pose.orientation
        self.pose = (msg.pose.position.x, msg.pose.position.y,
                     math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def find_bay(self):
        """Locate the charging bay's board ahead, and return where to park.

        The headland is empty apart from the board, so the nearest run of
        returns in a cone ahead of the rover is it. Parking position is that
        board pulled back by the standoff, along the line from it to the rover:
        the rover drives at the board and stops a board's length short of it,
        on the pad. Returns None if nothing is in view.
        """
        for _ in range(20):                     # let a scan and a pose arrive
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.scan is not None and self.pose is not None:
                break
        if self.scan is None or self.pose is None:
            return None

        msg = self.scan
        cone = math.radians(self.get_parameter('dock_search_deg').value)
        near = self.get_parameter('dock_search_min').value
        far = self.get_parameter('dock_search_max').value

        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if abs(angle) < cone and near < r < far:
                points.append((r * math.cos(angle), r * math.sin(angle), r))
            angle += msg.angle_increment
        if len(points) < 3:
            return None

        # The nearest contiguous run of returns: contiguous in range, so the
        # board is not merged with the glasshouse wall behind it.
        best, run = [], [points[0]]
        for previous, point in zip(points, points[1:]):
            if abs(point[2] - previous[2]) < 0.15:
                run.append(point)
            else:
                if len(run) >= 3 and (not best or
                                      min(p[2] for p in run) < min(p[2] for p in best)):
                    best = run
                run = [point]
        if len(run) >= 3 and (not best or
                              min(p[2] for p in run) < min(p[2] for p in best)):
            best = run
        if len(best) < 3:
            return None

        bx = sum(p[0] for p in best) / len(best)
        by = sum(p[1] for p in best) / len(best)
        distance = math.hypot(bx, by)
        standoff = self.get_parameter('dock_standoff').value
        if distance <= standoff:
            return None                         # already past it

        # Board in odom, then pulled back towards the rover by the standoff.
        px, py, yaw = self.pose
        board = (px + bx * math.cos(yaw) - by * math.sin(yaw),
                 py + bx * math.sin(yaw) + by * math.cos(yaw))
        scale = (distance - standoff) / distance
        target = (px + (board[0] - px) * scale, py + (board[1] - py) * scale)
        self.get_logger().info(
            f'Charging bay seen {distance:.2f} m ahead ({len(best)} returns); '
            f'parking at ({target[0]:.2f}, {target[1]:.2f})')
        return target

    def wait_for_services(self, timeout=60.0):
        if not self.navigator.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('No /follow_line action server')
            return False
        for client, name in ((self.set_params, 'navigator parameters'),
                             (self.reset_lane, 'lane reset')):
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(f'No {name} service')
                return False
        return True

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        return future.result()

    def set_lane_centring(self, enabled):
        request = SetParameters.Request()
        request.parameters = [
            Parameter('use_lane_centring', Parameter.Type.BOOL, enabled)
            .to_parameter_msg()]
        if self.call(self.set_params, request) is None:
            self.get_logger().warn('Could not set use_lane_centring')

    def run_leg(self, leg):
        """Send one leg as a FollowLine goal and wait for it to finish."""
        self.set_lane_centring(leg.lane)
        if leg.lane:
            # Anchor this lane, not the last one.
            if self.call(self.reset_lane, Trigger.Request()) is None:
                self.get_logger().warn('Lane reset failed; continuing')

        start, target = self.shifted(leg.start), self.shifted(leg.goal)
        goal = FollowLine.Goal()
        goal.start = Point(x=float(start[0]), y=float(start[1]), z=0.0)
        goal.goal = Point(x=float(target[0]), y=float(target[1]), z=0.0)
        goal.tolerance = leg.tolerance

        self.get_logger().info(
            f'--> {leg}' + ('' if self.shift == (0.0, 0.0) else
                            f'  (shifted by {self.shift[0]:+.2f}, {self.shift[1]:+.2f})'))
        send = self.navigator.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{leg.name}: goal rejected')
            return False, leg.start

        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        outcome = result.result().result
        position = outcome.final_position
        # Carry on from where the rover believes it is, not from the plan.
        self.shift = (position.x - leg.goal[0], position.y - leg.goal[1])
        self.get_logger().info(
            f'    {leg.name}: {"done" if outcome.success else "FAILED"} — '
            f'{outcome.message} (worst offset from the path '
            f'{outcome.max_cross_track_error:.2f} m)')
        return outcome.success, (position.x, position.y)

    def run(self):
        if not self.wait_for_services():
            return False
        legs = self.plan()
        self.get_logger().info(
            f'Sweeping {len(legs)} legs: '
            f'{sum(1 for leg in legs if leg.lane)} lanes, then the charging bay')

        for leg in legs:
            if leg.name == 'dock' and self.get_parameter('dock_by_sight').value:
                seen = self.find_bay()
                if seen is None:
                    self.get_logger().warn(
                        'Charging bay not in view — docking on the planned '
                        'position, which carries whatever drift has built up')
                else:
                    leg.goal = (seen[0] - self.shift[0], seen[1] - self.shift[1])
            ok, position = self.run_leg(leg)
            if not ok:
                self.get_logger().error(
                    f'Sweep stopped during "{leg.name}" at '
                    f'({position[0]:.2f}, {position[1]:.2f})')
                return False

        self.get_logger().info(
            f'Sweep complete — parked at ({position[0]:.2f}, {position[1]:.2f}) '
            f'by the rover\'s own reckoning, which had drifted '
            f'({self.shift[0]:+.2f}, {self.shift[1]:+.2f}) from the plan by then. '
            f'How close that is to the real bay is for ground truth to say.')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = GreenhouseMission()
    try:
        ok = node.run()
    except KeyboardInterrupt:
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
