"""Straight-line navigation from A to B using only a LiDAR and an IMU.

The line is treated as geometry, not as a list of waypoints. Given A and B:

    u = (B - A) / |B - A|          direction along the line
    n = (-u.y, u.x)                left-hand normal
    r = P - A                      robot relative to A

    along = r . u                  progress toward B
    cross = r . n                  signed perpendicular distance from the line

The controller nulls `cross` while advancing `along`. That single formulation
covers both R2 (travel in a straight line) and R7 (return to the same line after
avoiding), because recovering from a detour is not a special manoeuvre — it is
the same error being driven to zero from a larger starting value.

An important caveat, stated plainly because it governs how well any of this can
work: the line is expressed in the `odom` frame, and `odom` here is produced by
dead reckoning from the IMU and our own velocity commands, with no wheel
feedback and no absolute reference. It drifts. When it drifts, the line drifts
with it, and the rover will faithfully return to a line that has moved. Bounding
that requires matching LiDAR scans against features in the environment, which is
impossible in an empty world. See the README.
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf2_ros import (Buffer, TransformListener,
                     LookupException, ExtrapolationException, ConnectivityException)

from egrobots_line_interfaces.action import FollowLine

TF_ERRORS = (LookupException, ExtrapolationException, ConnectivityException)
CONTROL_PERIOD = 0.1


class LineNavigator(Node):

    def __init__(self):
        super().__init__('line_navigator_node')

        # --- walk / pause cycle (R3, R4) ---
        self.declare_parameter('walk_duration', 5.0)
        self.declare_parameter('pause_duration', 5.0)

        # --- line following (R2, R7) ---
        self.declare_parameter('linear_speed', 0.6)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('cross_track_kp', 1.2)
        self.declare_parameter('max_correction_deg', 60.0)
        # Gentler steering while cutting back to the line after a detour. The
        # return is where a skid-steer pivots while displaced from the line, and
        # the sideways slide during those pivots is invisible to the estimator.
        self.declare_parameter('return_angular_speed', 0.3)
        self.declare_parameter('return_max_correction_deg', 30.0)
        # Pivot until within this of the target bearing, then drive straight.
        self.declare_parameter('align_tolerance_deg', 4.0)
        # Never command a pivot slower than this. Below ~0.1 rad/s the skid-steer
        # cannot overcome wheel scrub and does not turn at all, so a proportional
        # command that shrinks with the error deadlocks just outside the tolerance.
        self.declare_parameter('min_pivot_speed', 0.3)
        # Ignore cross-track error smaller than this, so the rover does not stop
        # and pivot for noise.
        self.declare_parameter('cross_track_deadband', 0.05)
        self.declare_parameter('heading_kp', 1.5)
        self.declare_parameter('on_line_threshold', 0.20)
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('goal_approach_distance', 1.0)

        # --- obstacle avoidance (R5, R6) ---
        self.declare_parameter('safe_distance', 1.2)
        self.declare_parameter('clear_distance', 1.5)
        self.declare_parameter('cone_angle_deg', 30.0)
        self.declare_parameter('direction_scan_deg', 90.0)
        self.declare_parameter('clearing_distance', 1.5)

        # --- give up conditions ---
        self.declare_parameter('stall_timeout', 20.0)
        self.declare_parameter('stall_min_progress', 0.15)

        # When the car rows have been found, the path is set by the road: A and
        # B are projected onto its centreline, so they set where to start and
        # stop and the rows set where the path runs sideways.
        self.declare_parameter('use_road_centreline', True)
        # The rover drives in a lane rather than down the middle, so the path is
        # the centreline shifted sideways. Positive is left of the road centre,
        # negative right of it; 0.0 puts the path back on the centreline.
        self.declare_parameter('lane_offset', -1.5)
        # Room needed abeam before avoidance will swerve to that side: the
        # clearing distance plus this margin. Keeps the rover from turning into
        # the near row of cars when the lane it drives in is close to them.
        self.declare_parameter('side_clearance', 0.5)
        self.declare_parameter('side_window_deg', 20.0)

        # Steer by the lane measured at the rover rather than by the straight
        # line anchored at the start. In a greenhouse the crop rows wander and
        # the lane changes width, so "centred between the two sides" is a local
        # question; on a straight road the two answers coincide.
        self.declare_parameter('use_lane_centring', True)
        self.declare_parameter('lane_timeout', 1.0)
        # Keeping off the sides. Inside min_side_clearance the rover is pushed
        # away from the near side and slowed; with both sides inside
        # stop_side_clearance it stops rather than squeeze through.
        self.declare_parameter('min_side_clearance', 0.0)
        self.declare_parameter('stop_side_clearance', 0.2)
        self.declare_parameter('side_push_gain', 1.5)
        self.declare_parameter('side_min_speed_scale', 0.3)
        self.declare_parameter('side_look_ahead', 1.2)
        self.declare_parameter('side_look_behind', 0.4)
        # Clearances are measured from the side of the rover, not from the
        # LiDAR at its centre, so the numbers mean what they say: 0.25 m is
        # 0.25 m of air beside the bodywork.
        self.declare_parameter('rover_half_width', 0.32)

        self.declare_parameter('reference_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')

        self._lock = threading.Lock()
        self.latest_scan = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_pose = False

        self.avoid_state = 'CLEAR'
        self.turn_direction = 1.0
        self.clear_start_x = 0.0
        self.clear_start_y = 0.0
        self.goal_active = False
        self.line = None            # (ax, ay, ux, uy, nx, ny, length)
        self.road = None            # (cx, cy, phi) from row_localizer_node
        self.lane = None            # (cx, cy, phi, stamp) measured this scan

        scan_group = ReentrantCallbackGroup()
        action_group = ReentrantCallbackGroup()
        timer_group = MutuallyExclusiveCallbackGroup()

        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.line_publisher = self.create_publisher(Marker, '/line_marker', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10,
                                 callback_group=scan_group)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseStamped, '/road_centreline', self.road_callback,
                                 latched, callback_group=scan_group)
        self.create_subscription(PoseStamped, '/lane_centreline', self.lane_callback,
                                 10, callback_group=scan_group)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.update_pose_from_tf, callback_group=timer_group)
        self.create_timer(0.5, self.publish_line_marker, callback_group=timer_group)

        self.action_server = ActionServer(
            self, FollowLine, 'follow_line',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=action_group,
        )

        self.get_logger().info(
            'Line navigator ready. Send A and B with:\n'
            '  ros2 action send_goal -f /follow_line '
            'egrobots_line_interfaces/action/FollowLine '
            '"{start: {x: 0.0, y: 0.0}, goal: {x: 8.0, y: 0.0}, tolerance: 0.0}"')

    # ------------------------------------------------------------------
    # Perception
    # ------------------------------------------------------------------

    def scan_callback(self, msg):
        with self._lock:
            self.latest_scan = msg

    def road_callback(self, msg):
        q = msg.pose.orientation
        phi = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self.road = (msg.pose.position.x, msg.pose.position.y, phi)
        offset = self.get_parameter('lane_offset').value
        side = 'left of' if offset > 0 else 'right of' if offset < 0 else 'on'
        self.get_logger().info(
            f'Road centreline received: through ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}) heading {math.degrees(phi):+.2f} deg; '
            f'driving {abs(offset):.2f} m {side} it')

    def lane_callback(self, msg):
        """The lane as measured at the rover on the latest scan.

        Unlike the anchored centreline this arrives per scan and goes stale: if
        both sides disappear the localizer stops publishing, and after
        lane_timeout the controller falls back to the straight A->B line, which
        the EKF is still propagating. That is what carries the rover across a
        gap in a crop row.
        """
        q = msg.pose.orientation
        phi = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self.lane = (msg.pose.position.x, msg.pose.position.y, phi,
                         self.get_clock().now().nanoseconds * 1e-9)

    def steering_reference(self, x, y, line_cross):
        """What to steer by: (cross, heading, using_lane).

        The measured lane when it is fresh, otherwise the straight line through
        A and B.
        """
        if not self.get_parameter('use_lane_centring').value:
            return line_cross, self.line_heading(), False
        with self._lock:
            lane = self.lane
        if lane is None:
            return line_cross, self.line_heading(), False
        cx, cy, phi, stamp = lane
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if age > self.get_parameter('lane_timeout').value:
            return line_cross, self.line_heading(), False
        # Distance left of the measured lane centre, less where in the lane we
        # mean to be: lane_offset applies here exactly as it does to the line
        # through A and B, so a rover told to keep right still keeps right.
        cross = -(x - cx) * math.sin(phi) + (y - cy) * math.cos(phi)
        return cross - self.get_parameter('lane_offset').value, phi, True

    def side_clearances(self, msg):
        """Air beside the rover, left and right, in metres.

        Only returns beside the rover count - from side_look_behind behind it to
        side_look_ahead in front - so an obstacle straight ahead does not read
        as a side that is closing in. The rover's half width is taken off, so
        zero means its side is touching.
        """
        if msg is None:
            return float('inf'), float('inf')
        ahead = self.get_parameter('side_look_ahead').value
        behind = -self.get_parameter('side_look_behind').value
        left, right = float('inf'), float('inf')
        angle = msg.angle_min
        for r in msg.ranges:
            if 0.0 < r < msg.range_max:
                forward = r * math.cos(angle)
                if behind < forward < ahead:
                    lateral = r * math.sin(angle)
                    if lateral > 0.0:
                        left = min(left, lateral)
                    else:
                        right = min(right, -lateral)
            angle += msg.angle_increment
        half = self.get_parameter('rover_half_width').value
        return max(0.0, left - half), max(0.0, right - half)

    def side_safety(self, msg, cross):
        """R6. Returns (steering cross, speed scale, pinched).

        Inside min_side_clearance the near side adds to the cross-track error,
        which the controller already turns into a heading away from it, and the
        speed is scaled down with the remaining room. The two mechanisms are the
        ones already in the controller, so there is no separate avoidance state
        to get stuck in. If both sides are inside stop_side_clearance the lane is
        narrower than the rover can safely take and it stops instead.
        """
        margin = self.get_parameter('min_side_clearance').value
        if margin <= 0.0:
            return cross, 1.0, False
        left, right = self.side_clearances(msg)
        stop = self.get_parameter('stop_side_clearance').value
        if left < stop and right < stop:
            return cross, 0.0, True

        gain = self.get_parameter('side_push_gain').value
        push = 0.0
        if left < margin:
            push += gain * (margin - left)     # too close on the left: go right
        if right < margin:
            push -= gain * (margin - right)
        tightest = min(left, right)
        if tightest < margin:
            floor = self.get_parameter('side_min_speed_scale').value
            scale = max(floor, tightest / margin)
        else:
            scale = 1.0
        return cross + push, scale, False

    def update_pose_from_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('reference_frame').value,
                self.get_parameter('robot_frame').value, Time())
        except TF_ERRORS as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}', throttle_duration_sec=5.0)
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        with self._lock:
            self.current_x, self.current_y = t.x, t.y
            self.current_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.have_pose = True

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.get_parameter('reference_frame').value
        pose.pose.position.x, pose.pose.position.y = t.x, t.y
        pose.pose.orientation = q
        self.pose_publisher.publish(pose)

    def pose(self):
        with self._lock:
            return self.current_x, self.current_y, self.current_yaw, self.have_pose

    def read_cone(self):
        with self._lock:
            msg = self.latest_scan
        if msg is None or not msg.ranges:
            return None, float('inf')
        cone = int(math.radians(self.get_parameter('cone_angle_deg').value)
                   / msg.angle_increment)
        centre = int(round((0.0 - msg.angle_min) / msg.angle_increment))
        low, high = max(0, centre - cone), min(len(msg.ranges), centre + cone)
        valid = [msg.ranges[i] for i in range(low, high) if msg.ranges[i] > 0.0]
        return msg, (min(valid) if valid else float('inf'))

    # ------------------------------------------------------------------
    # Line geometry
    # ------------------------------------------------------------------

    def path_endpoints(self, a, b):
        """Project A and B onto the lane, if the road is known.

        The road localizer measures the centre between the two car rows. The
        lane the rover drives in is that centreline shifted sideways by
        lane_offset, the same way a vehicle keeps to one side of a road instead
        of straddling the middle of it. Only the path moves: position is still
        measured against both rows, so the accuracy is unchanged.
        """
        with self._lock:
            road = self.road
        if road is None or not self.get_parameter('use_road_centreline').value:
            return a, b
        cx, cy, phi = road
        ux, uy = math.cos(phi), math.sin(phi)
        nx, ny = -math.sin(phi), math.cos(phi)
        offset = self.get_parameter('lane_offset').value

        def onto(p):
            along = (p.x - cx) * ux + (p.y - cy) * uy
            return Point(x=cx + along * ux + offset * nx,
                         y=cy + along * uy + offset * ny, z=0.0)

        return onto(a), onto(b)

    def set_line(self, a, b):
        dx, dy = b.x - a.x, b.y - a.y
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return False
        ux, uy = dx / length, dy / length
        self.line = (a.x, a.y, ux, uy, -uy, ux, length)
        return True

    def project(self, x, y):
        """Return (along, cross) for a point relative to the line."""
        ax, ay, ux, uy, nx, ny, _ = self.line
        rx, ry = x - ax, y - ay
        return rx * ux + ry * uy, rx * nx + ry * ny

    def line_heading(self):
        _, _, ux, uy, _, _, _ = self.line
        return math.atan2(uy, ux)

    def publish_line_marker(self):
        if self.line is None:
            return
        ax, ay, ux, uy, _, _, length = self.line
        marker = Marker()
        marker.header.frame_id = self.get_parameter('reference_frame').value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'line'
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.g, marker.color.b, marker.color.a = 1.0, 0.4, 1.0
        marker.pose.orientation.w = 1.0
        marker.points = [Point(x=ax, y=ay, z=0.05),
                         Point(x=ax + ux * length, y=ay + uy * length, z=0.05)]
        self.line_publisher.publish(marker)

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def sector_min(self, msg, centre_deg, window_deg):
        """Closest return in a cone centred on centre_deg (0 = straight ahead)."""
        n = len(msg.ranges)
        lo = int(round((math.radians(centre_deg - window_deg) - msg.angle_min)
                       / msg.angle_increment))
        hi = int(round((math.radians(centre_deg + window_deg) - msg.angle_min)
                       / msg.angle_increment))
        values = [min(r, msg.range_max)
                  for r in msg.ranges[max(0, lo):min(n, hi + 1)] if r > 0.0]
        return min(values) if values else msg.range_max

    def choose_turn_direction(self, msg, scan_deg):
        """Pick the side to swerve towards: away from the obstacle, but never
        into a wall of parked cars.

        Driving in a lane puts one row close by, and the nearest-return rule
        alone would happily turn that way when the obstacle sits slightly to the
        other side. So a side is ruled out first if there is not enough room
        abeam to complete the manoeuvre. With room on both sides — an open
        world, or a path down the middle of the road — nothing is ruled out and
        the choice is the obstacle's position as before.
        """
        needed = (self.get_parameter('clearing_distance').value
                  + self.get_parameter('side_clearance').value)
        window = self.get_parameter('side_window_deg').value
        left_room = self.sector_min(msg, 90.0, window)
        right_room = self.sector_min(msg, -90.0, window)
        if left_room < needed and right_room >= needed:
            return -1.0
        if right_room < needed and left_room >= needed:
            return 1.0

        centre = int(round((0.0 - msg.angle_min) / msg.angle_increment))
        half = int(math.radians(scan_deg) / msg.angle_increment)
        n = len(msg.ranges)
        cap = lambda v: [min(r, msg.range_max) for r in v if r > 0.0]
        left = cap(msg.ranges[centre:min(n, centre + half)])
        right = cap(msg.ranges[max(0, centre - half):centre])
        left_min = min(left) if left else msg.range_max
        right_min = min(right) if right else msg.range_max
        if abs(left_min - right_min) > 0.05:
            return 1.0 if right_min < left_min else -1.0
        left_mean = sum(left) / len(left) if left else msg.range_max
        right_mean = sum(right) / len(right) if right else msg.range_max
        return 1.0 if right_mean < left_mean else -1.0

    def avoidance_step(self, cmd, msg, closest):
        """R5/R6. Returns True when avoidance owns the command."""
        safe = self.get_parameter('safe_distance').value
        clear = self.get_parameter('clear_distance').value
        speed = self.get_parameter('linear_speed').value
        turn = self.get_parameter('angular_speed').value
        clearing = self.get_parameter('clearing_distance').value
        x, y, _, _ = self.pose()

        if self.avoid_state == 'CLEAR' and closest < safe and msg is not None:
            self.avoid_state = 'TURNING'
            self.turn_direction = self.choose_turn_direction(
                msg, self.get_parameter('direction_scan_deg').value)
            self.get_logger().info(
                f'Obstacle at {closest:.2f} m — turning '
                f'{"left" if self.turn_direction > 0 else "right"}')

        if self.avoid_state == 'TURNING':
            if closest > clear:
                self.avoid_state = 'CLEARING'
                self.clear_start_x, self.clear_start_y = x, y
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_direction * turn
                return True

        if self.avoid_state == 'CLEARING':
            if closest < safe:
                self.avoid_state = 'TURNING'
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_direction * turn
                return True
            if math.hypot(x - self.clear_start_x, y - self.clear_start_y) < clearing:
                cmd.linear.x = speed
                cmd.angular.z = 0.0
                return True
            self.avoid_state = 'CLEAR'
            self.get_logger().info('Past the obstacle — returning to the line')

        return False

    def steer_towards(self, cmd, desired_heading, speed, angular_speed=None):
        """Either rotate or translate — never both at once.

        Measured against ground truth on this rover, driving while turning costs
        roughly 70x more position error per metre than driving straight:

            straight        0.00031 m/m   ->  0.03 m over 100 m
            arc (wz 0.35)   0.02174 m/m   ->  2.17 m over 100 m

        Every metre covered while the heading is still changing is integrated
        along a heading that is out of date, and the error is unrecoverable
        because nothing observes absolute position. Pivoting first costs time
        but keeps the whole driven path in the cheap regime.
        """
        if angular_speed is None:
            angular_speed = self.get_parameter('angular_speed').value
        heading_kp = self.get_parameter('heading_kp').value
        tolerance = math.radians(self.get_parameter('align_tolerance_deg').value)

        _, _, yaw, _ = self.pose()
        error = math.atan2(math.sin(desired_heading - yaw),
                           math.cos(desired_heading - yaw))

        if abs(error) > tolerance:
            min_pivot = min(self.get_parameter('min_pivot_speed').value, angular_speed)
            rate = min(angular_speed, max(min_pivot, heading_kp * abs(error)))
            cmd.angular.z = math.copysign(rate, error)
            cmd.linear.x = 0.0
        else:
            cmd.angular.z = 0.0
            cmd.linear.x = speed
        return error

    def follow_line_step(self, cmd, cross, returning=False, heading=None,
                         speed_scale=1.0):
        """Steer along the line, biased by how far off it we are.

        The correction angle is proportional to cross-track error, so a rover on
        the line drives parallel to it and one displaced from it cuts back at an
        angle that grows with the displacement.
        """
        kp = self.get_parameter('cross_track_kp').value
        if returning:
            max_correction = math.radians(
                self.get_parameter('return_max_correction_deg').value)
            angular_speed = self.get_parameter('return_angular_speed').value
        else:
            max_correction = math.radians(self.get_parameter('max_correction_deg').value)
            angular_speed = None
        deadband = self.get_parameter('cross_track_deadband').value

        # Inside the deadband, aim straight down the line rather than chasing a
        # correction. Without this the desired heading twitches with every
        # centimetre of cross-track noise, and since a heading change now means
        # a pivot, the rover would stop and turn constantly instead of driving.
        if heading is None:
            heading = self.line_heading()
        if abs(cross) < deadband:
            desired = heading
        else:
            correction = max(-max_correction, min(max_correction, -kp * cross))
            desired = heading + correction

        return self.steer_towards(
            cmd, desired,
            self.get_parameter('linear_speed').value * speed_scale,
            angular_speed)

    def stop(self):
        self.cmd_publisher.publish(Twist())

    # ------------------------------------------------------------------
    # Action
    # ------------------------------------------------------------------

    def goal_callback(self, goal_request):
        if self.goal_active:
            self.get_logger().warn('Rejecting goal: one is already running')
            return GoalResponse.REJECT
        _, _, _, have_pose = self.pose()
        if not have_pose:
            self.get_logger().warn('Rejecting goal: no pose from TF yet')
            return GoalResponse.REJECT
        if math.hypot(goal_request.goal.x - goal_request.start.x,
                      goal_request.goal.y - goal_request.start.y) < 1e-6:
            self.get_logger().warn('Rejecting goal: A and B are the same point')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def make_feedback(self, distance, along, cross, x, y, state, iteration):
        feedback = FollowLine.Feedback()
        feedback.distance_to_goal = distance
        feedback.distance_along_line = along
        feedback.cross_track_error = cross
        feedback.current_position = Point(x=x, y=y, z=0.0)
        feedback.state = state
        feedback.iteration = iteration
        return feedback

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        start, goal = self.path_endpoints(request.start, request.goal)
        if not self.set_line(start, goal):
            goal_handle.abort()
            result = FollowLine.Result()
            result.success = False
            result.message = 'Degenerate line: A and B coincide'
            return result

        tolerance = request.tolerance
        if tolerance <= 0.0:
            tolerance = self.get_parameter('goal_tolerance').value

        walk_duration = self.get_parameter('walk_duration').value
        pause_duration = self.get_parameter('pause_duration').value
        on_line = self.get_parameter('on_line_threshold').value
        approach = self.get_parameter('goal_approach_distance').value
        stall_timeout = self.get_parameter('stall_timeout').value
        stall_min_progress = self.get_parameter('stall_min_progress').value

        self.goal_active = True
        self.avoid_state = 'CLEAR'
        phase = 'WALKING'
        phase_started = time.time()
        iteration = 1
        max_cross = 0.0
        steering_by_lane = False
        best_distance = float('inf')
        best_time = time.time()
        result = FollowLine.Result()

        self.get_logger().info(
            f'Following the line ({start.x:.1f}, {start.y:.1f}) -> '
            f'({goal.x:.1f}, {goal.y:.1f})')

        try:
            while rclpy.ok():
                x, y, _, _ = self.pose()
                along, cross = self.project(x, y)
                distance = math.hypot(goal.x - x, goal.y - y)

                # --- R9: arrived ---
                # Arrival is progress ALONG the line, not distance to a point.
                # Judged by distance, a rover whose estimate is off sideways -
                # which it will be, by however much the lane's measured
                # direction is out - can pass the end of the lane without ever
                # coming within tolerance of B, and drive on out of the lane.
                if along >= self.line[6] - tolerance or distance < tolerance:
                    self.stop()
                    goal_handle.succeed()
                    result.success = True
                    result.message = f'Reached Point B at ({x:.2f}, {y:.2f})'
                    self.get_logger().info(result.message)
                    break

                if goal_handle.is_cancel_requested:
                    self.stop()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Cancelled by the client'
                    break

                if distance < best_distance - stall_min_progress:
                    best_distance, best_time = distance, time.time()
                if time.time() - best_time > stall_timeout:
                    self.stop()
                    goal_handle.abort()
                    result.success = False
                    result.message = (
                        f'Aborted: no progress for {stall_timeout:.0f} s, '
                        f'{distance:.2f} m short of Point B')
                    self.get_logger().warn(result.message)
                    break

                cmd = Twist()
                msg, closest = self.read_cone()
                # R3/R5/R7: steer by the lane measured here, when there is one.
                cross, heading, on_lane = self.steering_reference(x, y, cross)
                if on_lane != steering_by_lane:
                    steering_by_lane = on_lane
                    self.get_logger().info(
                        'Steering by the measured lane' if on_lane else
                        'Lane measurement lost - steering by the A->B line')
                max_cross = max(max_cross, abs(cross))
                # R6: keep off the sides.
                steer_cross, speed_scale, pinched = self.side_safety(msg, cross)
                if pinched:
                    self.stop()
                    self.get_logger().warn(
                        'Both sides within the stop clearance - holding',
                        throttle_duration_sec=2.0)
                    goal_handle.publish_feedback(self.make_feedback(
                        distance, along, cross, x, y, 'BLOCKED', iteration))
                    time.sleep(CONTROL_PERIOD)
                    continue

                # Obstacle avoidance outranks everything, including the pause:
                # it reads only the LiDAR and never waits on the cycle timer.
                if self.avoidance_step(cmd, msg, closest):
                    phase = 'AVOIDING'
                    phase_started = time.time()
                elif phase == 'PAUSED':
                    # --- R3: hold still for pause_duration ---
                    if time.time() - phase_started >= pause_duration:
                        phase = 'WALKING'
                        phase_started = time.time()
                        iteration += 1
                        self.get_logger().info(f'Walk iteration {iteration}')
                    else:
                        self.stop()
                elif abs(cross) > on_line:
                    # --- R7: off the line after a detour, cut back to it ---
                    phase = 'RETURNING'
                    self.follow_line_step(cmd, steer_cross, returning=True,
                                          heading=heading, speed_scale=speed_scale)
                else:
                    # --- R2/R4: walking along the line ---
                    if phase != 'WALKING':
                        phase = 'WALKING'
                        phase_started = time.time()
                    if time.time() - phase_started >= walk_duration:
                        phase = 'PAUSED'
                        phase_started = time.time()
                        self.stop()
                        self.get_logger().info(
                            f'Pausing {pause_duration:.0f} s '
                            f'(iteration {iteration}, {distance:.2f} m to go)')
                    else:
                        self.follow_line_step(cmd, steer_cross, heading=heading,
                                              speed_scale=speed_scale)

                if phase not in ('PAUSED',):
                    if cmd.linear.x > 0.0 and distance < approach:
                        cmd.linear.x *= max(0.2, distance / approach)
                    self.cmd_publisher.publish(cmd)

                goal_handle.publish_feedback(self.make_feedback(
                    distance, along, cross, x, y, phase, iteration))

                time.sleep(CONTROL_PERIOD)
        finally:
            self.goal_active = False
            self.stop()

        x, y, _, _ = self.pose()
        result.final_position = Point(x=x, y=y, z=0.0)
        result.final_distance_error = math.hypot(goal.x - x, goal.y - y)
        result.max_cross_track_error = max_cross
        return result


def main(args=None):
    rclpy.init(args=args)
    node = LineNavigator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
