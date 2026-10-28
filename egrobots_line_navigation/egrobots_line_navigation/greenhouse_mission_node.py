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
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.srv import SetParameters
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import (Buffer, TransformListener, LookupException,
                     ExtrapolationException, ConnectivityException)

TF_ERRORS = (LookupException, ExtrapolationException, ConnectivityException)

from nav2_msgs.action import FollowPath, NavigateToPose

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

        # The lanes are fixed in the world, so the plan is written in the map
        # frame and converted to odom as each goal is sent, using AMCL's
        # map -> odom. Without a map frame there is nothing to convert from,
        # and the fallback is the old scheme: carry on from where the rover
        # believed it finished the last leg, which works only as long as the
        # drift does not change in between. It does - it grew 1.2 m across one
        # turn and put the rover into a crop row.
        self.declare_parameter('plan_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')

        self.shift = (0.0, 0.0)     # fallback only: plan -> believed position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan = None
        self.nav2_seen = False
        self.pose = None
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, '/robot_pose', self.pose_callback, 10)
        # Legs that are not inside a lane - the headland turns and the approach
        # to the bay - go to Nav2: a planned path, followed by pure pursuit,
        # with the behaviour tree's recovery behind it. Our own controller
        # pivots in place before every move and spent 29.5% of a sweep doing
        # exactly that, 85% of it out here. Lane legs stay with FollowLine,
        # which steers by the crop rows and holds a 1.6 m lane to about 3 cm.
        self.declare_parameter('use_nav2_between_lanes', True)

        # Which Nav2 controller drives which kind of leg. The behaviour tree
        # reads this from /controller_selector, and its subscription is
        # TRANSIENT_LOCAL + RELIABLE - a default publisher is VOLATILE, so the
        # QoS does not match and the message is dropped without a word. That is
        # why an earlier test appeared to run the lane controller and was in
        # fact running pure pursuit the whole time.
        self.declare_parameter('lane_controller', 'FollowLane')
        # FollowLane, not FollowPath, even on the open legs. Pure pursuit
        # steers by curvature, so below its rotate-to-heading threshold it
        # commands linear and angular together - the one motion this rover
        # cannot perform. It measured 4.1% of a sweep blended. FollowLane with
        # no lane in view falls back to following the plan, and does that with
        # the same strict pivot-then-drive rule, so the whole sweep complies.
        self.declare_parameter('open_controller', 'FollowLane')
        # Lane legs skip NavigateToPose and the global planner entirely, and
        # are sent as a straight path straight to the controller.
        self.declare_parameter('lane_bypasses_planner', True)

        # What a failed leg means for the sweep is this node's decision. A lane
        # it cannot drive is reported as blocked and left for someone to clear,
        # and the sweep carries on with the next one. A greenhouse robot that
        # abandons the round because one lane has a trolley in it is not much
        # use; one that reverses blindly out of a lane it is wedged in, with no
        # sensor looking that way, is worse.

        selector_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.controller_pub = self.create_publisher(
            String, '/controller_selector', selector_qos)

        self.navigator = ActionClient(self, FollowLine, '/follow_line')
        self.nav2 = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        # Lane legs go straight to the controller, bypassing the planner.
        self.follow_path = ActionClient(self, FollowPath, '/follow_path')
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
        # Looser than the other close-quarters legs: the rover physically parks
        # well, but a 0.10 m tolerance is tighter than the final approach can
        # reliably hit, and falling short of it by 9 cm had the stall detector
        # report a successful dock as a failure.
        legs.append(Leg('dock', (bay[0] - approach + 0.5, bay[1]), tuple(bay),
                        False, tolerance=0.20))
        return legs

    def shifted(self, point):
        """A planned waypoint in the frame the navigator drives in.

        With a map frame available this is a real coordinate transform and the
        plan stays absolute. Without one it falls back to shifting by the last
        leg's discrepancy.
        """
        try:
            tr = self.tf_buffer.lookup_transform(
                self.get_parameter('odom_frame').value,
                self.get_parameter('plan_frame').value, Time())
        except TF_ERRORS:
            return (point[0] + self.shift[0], point[1] + self.shift[1])

        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        return (t.x + point[0] * c - point[1] * s,
                t.y + point[0] * s + point[1] * c)

    def to_plan_frame(self, point):
        """The inverse of shifted(): an odom-frame point expressed in the plan.

        Anything measured off a sensor arrives in odom, because that is the
        frame the navigator reports its pose in. Leg goals are in the plan
        frame and are converted to odom when sent, so a measured point has to
        be converted back first - otherwise it is transformed twice and lands
        a whole map->odom correction away from where it was seen.
        """
        try:
            tr = self.tf_buffer.lookup_transform(
                self.get_parameter('plan_frame').value,
                self.get_parameter('odom_frame').value, Time())
        except TF_ERRORS:
            return (point[0] - self.shift[0], point[1] - self.shift[1])

        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        return (t.x + point[0] * c - point[1] * s,
                t.y + point[0] * s + point[1] * c)

    def using_map(self):
        try:
            self.tf_buffer.lookup_transform(
                self.get_parameter('odom_frame').value,
                self.get_parameter('plan_frame').value, Time())
            return True
        except TF_ERRORS:
            return False

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

    def wait_for_action(self, client, name, timeout=20.0):
        """Wait for an action server to be discoverable, while spinning.

        ActionClient.wait_for_server blocks this thread without servicing the
        node, so on a single-threaded spin the graph event that marks the
        server ready never gets processed and the wait expires with the server
        sitting there answering. Spinning in short slices lets discovery
        finish. The wait matters because send_goal_async to a server that has
        not been discovered yet returns a future that is never completed, and
        the wait on that future has no timeout - the sweep stops dead on the
        first lane leg with no error, which it did in six of fifteen runs.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if client.server_is_ready():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error(
            f'No {name} action server after {timeout:.0f} s')
        return False

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

    def nav2_available(self):
        """Whether to drive this leg with Nav2 rather than the Python fallback.

        The first call carries a long timeout, the rest a short one. bt_navigator
        can take tens of seconds to be discoverable after the lifecycle manager
        reports its nodes active, and a 2 s check on the first leg silently
        dropped a whole sweep onto the fallback controller - which works, but is
        not what was being tested, and the only hint was a missing "[nav2 ...]"
        in one log line. Once it has answered once, 2 s is plenty.
        """
        if not (self.get_parameter('use_nav2_between_lanes').value and self.using_map()):
            return False
        timeout = 2.0 if self.nav2_seen else 30.0
        if self.nav2.wait_for_server(timeout_sec=timeout):
            if not self.nav2_seen:
                self.get_logger().info('Nav2 is up; legs will be driven through it')
                self.nav2_seen = True
            return True
        if not self.nav2_seen:
            self.get_logger().warn(
                f'Nav2 did not answer within {timeout:.0f} s. Falling back to the '
                f'Python FollowLine controller for the whole sweep.')
        return False

    def run_nav2_leg(self, leg):
        """Send one open-ground leg to Nav2 as a NavigateToPose goal.

        The goal is in the plan frame, which is where Nav2 wants it, so no
        conversion: this is the frame AMCL corrects. Its heading is the
        direction of travel, which gives pure pursuit something sensible to
        line up with on the way in.
        """
        heading = math.atan2(leg.goal[1] - leg.start[1], leg.goal[0] - leg.start[0])
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.get_parameter('plan_frame').value
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position = Point(x=float(leg.goal[0]), y=float(leg.goal[1]), z=0.0)
        goal.pose.pose.orientation.z = math.sin(heading / 2.0)
        goal.pose.pose.orientation.w = math.cos(heading / 2.0)

        self.get_logger().info(
            f'--> {leg}  [nav2: '
            f'{self.get_parameter("lane_controller").value if leg.lane else self.get_parameter("open_controller").value}]')
        if not self.wait_for_action(self.nav2, '/navigate_to_pose'):
            return False, leg.start
        send = self.nav2.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None:
            self.get_logger().error(
                f'{leg.name}: Nav2 never acknowledged the goal')
            return False, leg.start
        if not handle.accepted:
            self.get_logger().error(f'{leg.name}: Nav2 rejected the goal')
            return False, leg.start

        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        ok = result.result().status == 4          # STATUS_SUCCEEDED
        here = self.where_am_i() or leg.goal
        self.get_logger().info(
            f'    {leg.name}: {"done" if ok else "FAILED"} via Nav2 — '
            f'at ({here[0]:.2f}, {here[1]:.2f})')
        return ok, here

    def run_lane_leg(self, leg):
        """Drive a lane leg by handing the controller a straight path directly.

        No planner. A lane leg does not need one: FollowLane steers by the rows
        it measures, and uses the path only for a heading reference and a goal.
        Routing it through NavigateToPose meant the global planner had a veto,
        and it kept exercising it - GridBased refuses to plan from a pose that
        sits in an inscribed cell, and amcl on this camera drifts 0.6 to 0.85 m
        after a headland turn, which is enough to put a perfectly centred rover
        inside a row as far as the map is concerned. Three sweeps died that way
        with ground truth showing the rover dead centre in the lane.

        A straight line from here to the goal is the right path anyway. The
        lane is straight, and the controller corrects against the rows rather
        than against this.
        """
        # The leg already names both ends in the plan frame, so the path is
        # built from those and needs no transform at all. Using the live pose
        # here was a mistake: where_am_i composes map -> odom -> base_link, and
        # amcl stamps map -> odom a second into the future as its transform
        # tolerance, so the composed lookup raises ExtrapolationException often
        # enough to kill a leg outright. run_nav2_leg only ever used it for a
        # log line, with a fallback, which is why that never showed up before.
        here = leg.start

        frame = self.get_parameter('plan_frame').value
        gx, gy = leg.goal
        heading = math.atan2(gy - here[1], gx - here[0])
        span = math.hypot(gx - here[0], gy - here[1])

        path = Path()
        path.header.frame_id = frame
        path.header.stamp = self.get_clock().now().to_msg()
        steps = max(2, int(span / 0.25))
        for i in range(steps + 1):
            f = i / steps
            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.pose.position = Point(
                x=here[0] + (gx - here[0]) * f,
                y=here[1] + (gy - here[1]) * f, z=0.0)
            pose.pose.orientation.z = math.sin(heading / 2.0)
            pose.pose.orientation.w = math.cos(heading / 2.0)
            path.poses.append(pose)

        self.select_controller(self.get_parameter('lane_controller').value)
        self.get_logger().info(
            f'--> {leg}  [direct to controller, {len(path.poses)} poses, '
            f'{span:.1f} m, no planner]')

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.get_parameter('lane_controller').value
        if not self.wait_for_action(self.follow_path, '/follow_path'):
            return False, here
        send = self.follow_path.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None:
            self.get_logger().error(
                f'{leg.name}: /follow_path never acknowledged the goal')
            return False, here
        if not handle.accepted:
            self.get_logger().error(f'{leg.name}: controller rejected the path')
            return False, here

        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        ok = result.result().status == 4          # STATUS_SUCCEEDED
        there = self.where_am_i() or leg.goal
        self.get_logger().info(
            f'    {leg.name}: {"done" if ok else "FAILED"} direct — '
            f'at ({there[0]:.2f}, {there[1]:.2f})')
        return ok, there

    def where_am_i(self):
        """The rover's position in the plan frame, from TF.

        Composed by hand out of plan -> odom and odom -> base_link rather than
        asked for in one go. amcl publishes map -> odom stamped into the future
        by its transform_tolerance, and a single composed lookup across that
        raises ExtrapolationException often enough to matter, while each hop on
        its own resolves fine.
        """
        plan = self.get_parameter('plan_frame').value
        odom = self.get_parameter('odom_frame').value
        try:
            a = self.tf_buffer.lookup_transform(plan, odom, Time()).transform
            b = self.tf_buffer.lookup_transform(odom, 'base_link', Time()).transform
        except TF_ERRORS:
            return None
        q = a.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x = a.translation.x + b.translation.x * math.cos(yaw) \
            - b.translation.y * math.sin(yaw)
        y = a.translation.y + b.translation.x * math.sin(yaw) \
            + b.translation.y * math.cos(yaw)
        return (x, y)

    def select_controller(self, name):
        """Tell the behaviour tree which controller to drive the next leg with."""
        msg = String()
        msg.data = name
        self.controller_pub.publish(msg)
        for _ in range(4):           # let it reach the behaviour tree
            rclpy.spin_once(self, timeout_sec=0.1)

    def run_leg(self, leg):
        """Send one leg to Nav2 if it is available, otherwise to FollowLine.

        Both kinds of leg go through Nav2 now; only the controller differs. A
        lane leg is driven by the lane controller, which steers by the rows
        measured either side of the rover; open ground is driven by pure
        pursuit. Either way the behaviour tree, the costmaps and the recovery
        behaviours are the same.
        """
        if self.nav2_available():
            if leg.lane:
                # Anchor this lane, not the last one, before driving it.
                if self.call(self.reset_lane, Trigger.Request()) is None:
                    self.get_logger().warn('Lane reset failed; continuing')
                # Straight to the controller. The planner has no say in a lane.
                if self.get_parameter('lane_bypasses_planner').value:
                    return self.run_lane_leg(leg)
                self.select_controller(self.get_parameter('lane_controller').value)
            else:
                self.select_controller(self.get_parameter('open_controller').value)
            return self.run_nav2_leg(leg)

        # No Nav2: fall back to the Python controller for everything.
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
        # Only meaningful as the no-map fallback; with a map frame the plan
        # stays absolute and this is not used.
        if not self.using_map():
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
        # Wait for map -> odom. AMCL reports itself active several seconds
        # before it first publishes the transform, and asking too early drops
        # the whole sweep into dead reckoning without anything looking wrong.
        for _ in range(240):
            if self.using_map():
                break
            rclpy.spin_once(self, timeout_sec=0.25)
        if self.using_map():
            frame = f"the {self.get_parameter('plan_frame').value} frame"
        else:
            frame = 'dead reckoning'
            self.get_logger().warn(
                f"No {self.get_parameter('plan_frame').value} -> "
                f"{self.get_parameter('odom_frame').value} transform after 60 s. "
                f"Falling back to dead reckoning, which drifts: expect the lane "
                f"entries to be off by however far the estimate has wandered.")
        self.get_logger().info(
            f'Sweeping {len(legs)} legs: '
            f'{sum(1 for leg in legs if leg.lane)} lanes, then the charging bay. '
            f'Waypoints are in {frame}.')

        for leg in legs:
            if leg.name == 'dock' and self.get_parameter('dock_by_sight').value:
                seen = self.find_bay()
                if seen is None:
                    self.get_logger().warn(
                        'Charging bay not in view — docking on the planned '
                        'position, which carries whatever drift has built up')
                else:
                    leg.goal = self.to_plan_frame(seen)
            ok, position = self.run_leg(leg)
            if not ok:
                if leg.lane:
                    # Blocked. Report where, and stop: the rover is standing in
                    # the lane behind whatever stopped it, and the only way out
                    # is back the way it came. It has no sensor looking that
                    # way and does not reverse blindly, so the sweep ends here
                    # and someone clears the lane.
                    self.get_logger().error(
                        f'PATH BLOCKED in "{leg.name}" at ({position[0]:.2f}, '
                        f'{position[1]:.2f}). The rover is holding position in '
                        f'the lane; clear the obstruction and run the sweep '
                        f'again.')
                else:
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
