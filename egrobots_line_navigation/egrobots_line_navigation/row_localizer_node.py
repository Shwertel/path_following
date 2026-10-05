"""Measure the rover's sideways position and heading from the parked-car rows.

The road is straight, lined with parked cars on both sides, and the path is its
centreline. Each LiDAR scan, the points belonging to the left and right rows are
isolated and a straight line is fitted to each (RANSAC, then a least-squares
refinement on the inliers). In the robot frame, with the road direction at angle
beta and its left-hand normal nL:

    s_L = nL . p_left  =  W/2 - e        offset of the left row
    s_R = nL . p_right = -W/2 - e        offset of the right row

    e   = -(s_L + s_R) / 2               rover's offset left of the centreline
    psi = -beta                          rover's heading relative to the road

Both are drift-free: they are measured afresh against the cars every scan, so
the sideways slide of a skid-steer during a turn is observed and corrected
rather than accumulated. Position ALONG the road is not observable this way —
parked cars look alike as you slide past — so that component is taken from the
EKF and published with a very large variance.

The road geometry (a point on the centreline, its direction, and the lane width)
is anchored in the odom frame from the first scans, while the rover is at its
start pose and odom is still exact. Nothing about the road is hard-coded.

Two things come out of each scan. `/row_pose` is an absolute pose for the EKF, a
PoseWithCovarianceStamped whose covariance is rotated to be tight across the
road and loose along it. `/lane_centreline` is the lane measured **here and
now** - the centre between the two rows at the rover's current position, and
their direction - which is what a controller should follow when the lane is not
perfectly straight or symmetrical, as a greenhouse crop lane is not. The stored
width tracks the measured one slowly, so a lane that pinches or widens is
followed rather than rejected.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import (Buffer, TransformListener,
                     LookupException, ExtrapolationException, ConnectivityException)

TF_ERRORS = (LookupException, ExtrapolationException, ConnectivityException)
UNUSED_VARIANCE = 1e6


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def fit_line(points, beta_hint, iterations, threshold, rng):
    """RANSAC line fit, refined by PCA on the inliers.

    Returns (beta, s, inliers, span, nearest) — the line direction oriented
    within 90 deg of beta_hint, its signed offset along that direction's left
    normal, the inlier count, how much road the inliers cover, and how far ahead
    the nearest of them is — or None.
    """
    n = len(points)
    if n < 2:
        return None

    best_mask, best_count = None, 0
    for _ in range(iterations):
        i, j = rng.choice(n, 2, replace=False)
        d = points[j] - points[i]
        length = math.hypot(d[0], d[1])
        if length < 0.3:
            continue
        normal = np.array([-d[1], d[0]]) / length
        mask = np.abs((points - points[i]) @ normal) < threshold
        count = int(mask.sum())
        if count > best_count:
            best_mask, best_count = mask, count
    if best_mask is None or best_count < 2:
        return None

    inliers = points[best_mask]
    mean = inliers.mean(axis=0)
    _, _, vt = np.linalg.svd(inliers - mean)
    beta = math.atan2(vt[0][1], vt[0][0])
    if abs(wrap(beta - beta_hint)) > math.pi / 2:
        beta = wrap(beta + math.pi)

    direction = np.array([math.cos(beta), math.sin(beta)])
    normal = np.array([-math.sin(beta), math.cos(beta)])
    along = inliers @ direction
    return (beta, float(mean @ normal), len(inliers),
            float(along.max() - along.min()), float(along.min()))


class RowLocalizerNode(Node):

    def __init__(self):
        super().__init__('row_localizer_node')

        self.declare_parameter('reference_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')
        # Off: still anchor and publish the road centreline, but send the EKF
        # nothing. Lets the same world be run with and without the correction.
        self.declare_parameter('publish_measurement', True)

        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_range', 9.0)
        self.declare_parameter('along_window_min', -4.0)
        self.declare_parameter('along_window_max', 8.0)
        # Before the road is known, row candidates are simply points well off to
        # each side. Once it is known, only points near the predicted row are
        # kept, which excludes an obstacle the rover is swerving around.
        self.declare_parameter('startup_side_min', 1.5)
        self.declare_parameter('startup_side_max', 6.0)
        self.declare_parameter('gate_width', 0.6)

        self.declare_parameter('ransac_iterations', 60)
        self.declare_parameter('ransac_threshold', 0.08)
        self.declare_parameter('min_inliers', 12)
        self.declare_parameter('min_span', 2.0)
        self.declare_parameter('max_heading_disagreement_deg', 25.0)
        self.declare_parameter('max_width_error', 0.5)

        # A road's lane width is fixed; a greenhouse lane's is not. The stored
        # width follows the measured one at this rate per scan, so the width
        # check stays a sanity check rather than a straitjacket.
        self.declare_parameter('width_tracking_gain', 0.05)

        self.declare_parameter('anchor_scans', 10)
        # Anchor the lane only once its sides are level with the rover. A row
        # seen entirely from in front is seen end-on: the scan grazes the ends
        # of the plants instead of their faces, which reads the lane as wider
        # than it is and its direction as a degree or so off. Anchored from
        # there, every later position is measured against a line that is not
        # quite the lane's, and the error grows with distance along it.
        self.declare_parameter('anchor_max_leading', 1.0)
        self.declare_parameter('lateral_std', 0.05)
        self.declare_parameter('heading_std_deg', 1.5)
        self.declare_parameter('along_std', 50.0)
        self.declare_parameter('single_row_inflation', 2.0)

        self.rng = np.random.default_rng(0)
        self.anchor_samples = []
        self.road = None            # (cx, cy, phi, width)
        self.scans_used = 0
        self.scans_skipped = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.measurement_pub = self.create_publisher(PoseWithCovarianceStamped, '/row_pose', 10)
        self.centreline_pub = self.create_publisher(PoseStamped, '/road_centreline', latched)
        self.lane_pub = self.create_publisher(PoseStamped, '/lane_centreline', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/row_markers', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_service(Trigger, '~/reset_lane', self.reset_lane)

        self.get_logger().info('Row localizer waiting for both car rows to anchor the road')

    # ------------------------------------------------------------------

    def reset_lane(self, request, response):
        """Forget the anchored lane and anchor the next one from scratch.

        The anchor is one lane's geometry: its direction, its width, and where
        its centre runs. Drive into a different lane and none of that applies,
        and the gating would look for rows where this lane has none. A sweep of
        a greenhouse calls this on entering each lane.
        """
        self.road = None
        self.anchor_samples.clear()
        self.scans_used = self.scans_skipped = 0
        self.get_logger().info('Lane reset — waiting to anchor the next one')
        response.success = True
        response.message = 'Lane forgotten; will anchor when rows are beside the rover'
        return response

    def lookup(self, target, source):
        try:
            tr = self.tf_buffer.lookup_transform(target, source, Time())
        except TF_ERRORS:
            return None
        t = tr.transform.translation
        return t.x, t.y, yaw_of(tr.transform.rotation)

    def scan_points(self, msg):
        """LiDAR returns as an (N, 2) array in the robot frame."""
        mount = self.lookup(self.get_parameter('robot_frame').value, msg.header.frame_id)
        if mount is None:
            return None
        mx, my, myaw = mount
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment + myaw
        keep = (np.isfinite(ranges)
                & (ranges > self.get_parameter('min_range').value)
                & (ranges < self.get_parameter('max_range').value))
        r, a = ranges[keep], angles[keep]
        return np.column_stack((mx + r * np.cos(a), my + r * np.sin(a)))

    def fit(self, points, beta_hint):
        result = fit_line(points, beta_hint,
                          self.get_parameter('ransac_iterations').value,
                          self.get_parameter('ransac_threshold').value, self.rng)
        if result is None:
            return None
        beta, s, count, span, nearest = result
        max_disagreement = math.radians(self.get_parameter('max_heading_disagreement_deg').value)
        if (count < self.get_parameter('min_inliers').value
                or span < self.get_parameter('min_span').value
                or abs(wrap(beta - beta_hint)) > max_disagreement):
            return None
        return beta, s, nearest

    # ------------------------------------------------------------------

    def scan_callback(self, msg):
        pose = self.lookup(self.get_parameter('reference_frame').value,
                           self.get_parameter('robot_frame').value)
        points = self.scan_points(msg)
        if pose is None or points is None or len(points) == 0:
            return
        if self.road is None:
            self.anchor(points, pose)
        else:
            self.measure(msg, points, pose)

    def anchor(self, points, pose):
        """Fix the road in odom from the first scans, while odom is still exact."""
        along_min = self.get_parameter('along_window_min').value
        along_max = self.get_parameter('along_window_max').value
        side_min = self.get_parameter('startup_side_min').value
        side_max = self.get_parameter('startup_side_max').value

        x, y = points[:, 0], points[:, 1]
        in_window = (x > along_min) & (x < along_max)
        left = self.fit(points[in_window & (y > side_min) & (y < side_max)], 0.0)
        right = self.fit(points[in_window & (y < -side_min) & (y > -side_max)], 0.0)
        if left is None or right is None:
            self.get_logger().warn('Anchoring: need both rows in view',
                                   throttle_duration_sec=5.0)
            return
        leading = max(left[2], right[2])
        if leading > self.get_parameter('anchor_max_leading').value:
            self.get_logger().warn(
                f'Anchoring: both rows still {leading:.1f} m ahead — waiting '
                f'until they are beside the rover', throttle_duration_sec=5.0)
            self.anchor_samples.clear()
            return

        beta = math.atan2(math.sin(left[0]) + math.sin(right[0]),
                          math.cos(left[0]) + math.cos(right[0]))
        self.anchor_samples.append((-(left[1] + right[1]) / 2.0,   # e
                                    left[1] - right[1],             # width
                                    -beta,                          # psi
                                    pose))
        if len(self.anchor_samples) < self.get_parameter('anchor_scans').value:
            return

        e = float(np.median([s[0] for s in self.anchor_samples]))
        width = float(np.median([s[1] for s in self.anchor_samples]))
        psi = float(np.median([s[2] for s in self.anchor_samples]))
        px, py, theta = self.anchor_samples[-1][3]
        phi = wrap(theta - psi)
        cx = px + e * math.sin(phi)
        cy = py - e * math.cos(phi)
        self.road = (cx, cy, phi, width)

        centreline = PoseStamped()
        centreline.header.stamp = self.get_clock().now().to_msg()
        centreline.header.frame_id = self.get_parameter('reference_frame').value
        centreline.pose.position.x, centreline.pose.position.y = cx, cy
        centreline.pose.orientation.z = math.sin(phi / 2.0)
        centreline.pose.orientation.w = math.cos(phi / 2.0)
        self.centreline_pub.publish(centreline)

        self.get_logger().info(
            f'Road anchored: lane width {width:.2f} m, rover {e:+.3f} m from centre, '
            f'heading {math.degrees(psi):+.2f} deg to the road')

    def measure(self, msg, points, pose):
        cx, cy, phi, width = self.road
        px, py, theta = pose

        # Where the rows should be, from the current estimate.
        u = (math.cos(phi), math.sin(phi))
        n = (-math.sin(phi), math.cos(phi))
        e_pred = n[0] * (px - cx) + n[1] * (py - cy)
        beta_pred = -wrap(theta - phi)
        v = np.array([math.cos(beta_pred), math.sin(beta_pred)])
        normal = np.array([-math.sin(beta_pred), math.cos(beta_pred)])

        along = points @ v
        offset = points @ normal
        gate = self.get_parameter('gate_width').value
        in_window = ((along > self.get_parameter('along_window_min').value)
                     & (along < self.get_parameter('along_window_max').value))
        left = self.fit(points[in_window & (np.abs(offset - (width / 2 - e_pred)) < gate)],
                        beta_pred)
        right = self.fit(points[in_window & (np.abs(offset - (-width / 2 - e_pred)) < gate)],
                         beta_pred)

        if left and right and abs((left[1] - right[1]) - width) > self.get_parameter('max_width_error').value:
            # The two fits disagree about the lane width, so one is wrong. Keep
            # the one that lands closer to where its row was predicted.
            left_miss = abs(left[1] - (width / 2 - e_pred))
            right_miss = abs(right[1] - (-width / 2 - e_pred))
            if left_miss < right_miss:
                right = None
            else:
                left = None

        if left and right:
            e = -(left[1] + right[1]) / 2.0
            beta = math.atan2(math.sin(left[0]) + math.sin(right[0]),
                              math.cos(left[0]) + math.cos(right[0]))
            rows, inflation = 'both', 1.0
            # Follow a lane that pinches or widens instead of fighting it. Only
            # a measurement that already passed the width check moves the
            # stored width, and it moves it slowly.
            gain = self.get_parameter('width_tracking_gain').value
            width += gain * ((left[1] - right[1]) - width)
            self.road = (cx, cy, phi, width)
        elif left:
            e, beta, rows = width / 2 - left[1], left[0], 'left'
            inflation = self.get_parameter('single_row_inflation').value
        elif right:
            e, beta, rows = -width / 2 - right[1], right[0], 'right'
            inflation = self.get_parameter('single_row_inflation').value
        else:
            self.scans_skipped += 1
            return
        self.scans_used += 1

        # Along the road: whatever the EKF believes. Across it: what the cars say.
        s_along = u[0] * (px - cx) + u[1] * (py - cy)
        mx = cx + s_along * u[0] + e * n[0]
        my = cy + s_along * u[1] + e * n[1]
        heading = wrap(phi - beta)

        self.publish_lane(msg, e, beta, pose)
        self.publish_markers(msg, left, right, pose)

        self.get_logger().info(
            f'rows={rows:5s}  offset={e:+.3f} m  heading={math.degrees(-beta):+.2f} deg  '
            f'(used {self.scans_used}, skipped {self.scans_skipped})',
            throttle_duration_sec=5.0)

        if not self.get_parameter('publish_measurement').value:
            return

        lat_var = (self.get_parameter('lateral_std').value * inflation) ** 2
        along_var = self.get_parameter('along_std').value ** 2
        c, s = math.cos(phi), math.sin(phi)
        # Rotate diag(along, lateral) from road axes into odom axes.
        cov_xx = c * c * along_var + s * s * lat_var
        cov_yy = s * s * along_var + c * c * lat_var
        cov_xy = c * s * (along_var - lat_var)

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.get_parameter('reference_frame').value
        out.pose.pose.position.x, out.pose.pose.position.y = mx, my
        out.pose.pose.orientation.z = math.sin(heading / 2.0)
        out.pose.pose.orientation.w = math.cos(heading / 2.0)
        cov = [0.0] * 36
        cov[0], cov[1], cov[6], cov[7] = cov_xx, cov_xy, cov_xy, cov_yy
        cov[14] = cov[21] = cov[28] = UNUSED_VARIANCE
        cov[35] = (math.radians(self.get_parameter('heading_std_deg').value) * inflation) ** 2
        out.pose.covariance = cov
        self.measurement_pub.publish(out)

    def publish_lane(self, msg, e, beta, pose):
        """The lane as measured at the rover right now, in odom.

        Where `/road_centreline` is the straight line anchored once at the
        start, this is the centre between the two rows *here*, with their local
        direction. On a road the two coincide; in a greenhouse, where the rows
        wander and the lane changes width, this is the one worth steering by -
        it is what "centred between the two sides" actually means at this point
        in the lane. It is published only when a measurement was made, so a
        consumer can tell a stale lane from a fresh one by its stamp.
        """
        px, py, theta = pose
        direction = wrap(theta + beta)
        lane = PoseStamped()
        lane.header.stamp = msg.header.stamp
        lane.header.frame_id = self.get_parameter('reference_frame').value
        lane.pose.position.x = px + e * math.sin(direction)
        lane.pose.position.y = py - e * math.cos(direction)
        lane.pose.orientation.z = math.sin(direction / 2.0)
        lane.pose.orientation.w = math.cos(direction / 2.0)
        self.lane_pub.publish(lane)

    def publish_markers(self, msg, left, right, pose):
        """Draw the fitted rows in odom so the fit can be checked in RViz."""
        px, py, theta = pose
        markers = MarkerArray()
        for idx, fit in enumerate((left, right)):
            marker = Marker()
            marker.header.frame_id = self.get_parameter('reference_frame').value
            marker.header.stamp = msg.header.stamp
            marker.ns = 'rows'
            marker.id = idx
            if fit is None:
                marker.action = Marker.DELETE
                markers.markers.append(marker)
                continue
            beta, s = fit[0], fit[1]
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.06
            marker.color.r, marker.color.g, marker.color.a = 1.0, 0.2, 1.0
            marker.pose.orientation.w = 1.0
            for a in (self.get_parameter('along_window_min').value,
                      self.get_parameter('along_window_max').value):
                rx = a * math.cos(beta) - s * math.sin(beta)
                ry = a * math.sin(beta) + s * math.cos(beta)
                marker.points.append(Point(
                    x=px + rx * math.cos(theta) - ry * math.sin(theta),
                    y=py + rx * math.sin(theta) + ry * math.cos(theta), z=0.1))
            markers.markers.append(marker)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = RowLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
