"""Print the rover's estimated pose, and its true pose, at a fixed period.

Prints the belief (the odom -> base_link transform, which is what the navigator
actually steers on) beside Gazebo ground truth (/ground_truth/odom), plus the
error between them broken into along-track and cross-track components.

Note on rate: the default period is 5 ms (200 Hz), but the sources are slower —
the EKF publishes at 30 Hz and ground truth at 20 Hz — so consecutive lines will
often repeat. Raise `period` to sample at a readable rate.

Ground truth is optional. Without it (real hardware, or p3d disabled) the truth
and error columns simply read `--`.

    ros2 run egrobots_line_navigation pose_logger_node --ros-args -p use_sim_time:=true
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from tf2_ros import (Buffer, TransformListener,
                     LookupException, ExtrapolationException, ConnectivityException)

TF_ERRORS = (LookupException, ExtrapolationException, ConnectivityException)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PoseLoggerNode(Node):

    def __init__(self):
        super().__init__('pose_logger_node')

        self.declare_parameter('period', 0.005)
        self.declare_parameter('reference_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('truth_topic', '/ground_truth/odom')
        self.declare_parameter('csv_path', '')
        self.declare_parameter('print_to_screen', True)

        self.truth = None
        self.start_time = None
        self.csv = None

        path = self.get_parameter('csv_path').value
        if path:
            self.csv = open(path, 'w')
            self.csv.write('t,bx,by,byaw,tx,ty,tyaw,along_err,cross_err,pos_err\n')
            self.get_logger().info(f'Also writing CSV to {path}')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            Odometry, self.get_parameter('truth_topic').value, self.truth_cb, 20)

        period = self.get_parameter('period').value
        self.create_timer(period, self.tick)

        if self.get_parameter('print_to_screen').value:
            print(f"{'t(s)':>8} {'bel_x':>9} {'bel_y':>9} {'bel_yaw':>9} "
                  f"{'true_x':>9} {'true_y':>9} {'true_yaw':>9} "
                  f"{'along':>8} {'cross':>8} {'err':>7}", flush=True)
            print('-' * 100, flush=True)

    def truth_cb(self, msg):
        p = msg.pose.pose
        self.truth = (p.position.x, p.position.y, yaw_of(p.orientation))

    def belief(self):
        try:
            tr = self.tf_buffer.lookup_transform(
                self.get_parameter('reference_frame').value,
                self.get_parameter('robot_frame').value, Time())
        except TF_ERRORS:
            return None
        t = tr.transform.translation
        return t.x, t.y, yaw_of(tr.transform.rotation)

    def tick(self):
        b = self.belief()
        if b is None:
            self.get_logger().warn(
                'no odom -> base_link transform yet', throttle_duration_sec=5.0)
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now
        t = now - self.start_time

        show = self.get_parameter('print_to_screen').value
        if self.truth is None:
            if show:
                print(f'{t:8.3f} {b[0]:9.3f} {b[1]:9.3f} {math.degrees(b[2]):9.2f} '
                      f"{'--':>9} {'--':>9} {'--':>9} {'--':>8} {'--':>8} {'--':>7}",
                      flush=True)
            return

        tr = self.truth
        # Split the error along the robot's own heading: "along" is how far
        # ahead or behind it thinks it is, "cross" is sideways.
        dx, dy = b[0] - tr[0], b[1] - tr[1]
        c, s = math.cos(tr[2]), math.sin(tr[2])
        along = dx * c + dy * s
        cross = -dx * s + dy * c
        err = math.hypot(dx, dy)

        if show:
            print(f'{t:8.3f} {b[0]:9.3f} {b[1]:9.3f} {math.degrees(b[2]):9.2f} '
                  f'{tr[0]:9.3f} {tr[1]:9.3f} {math.degrees(tr[2]):9.2f} '
                  f'{along:+8.3f} {cross:+8.3f} {err:7.3f}', flush=True)

        if self.csv:
            self.csv.write(
                f'{t:.4f},{b[0]:.4f},{b[1]:.4f},{b[2]:.4f},'
                f'{tr[0]:.4f},{tr[1]:.4f},{tr[2]:.4f},'
                f'{along:.4f},{cross:.4f},{err:.4f}\n')
            self.csv.flush()

    def destroy_node(self):
        if self.csv:
            self.csv.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
