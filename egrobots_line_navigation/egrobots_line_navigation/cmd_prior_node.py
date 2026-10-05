"""Republish the commanded velocity as a stamped, covariance-bearing twist.

The EKF needs a translation source, and wheel feedback is not allowed. What is
left is the command we issued ourselves — not a sensor reading at all, but our
own control output. This node adds the header and covariance that
robot_localization requires and nothing else.

Why this is legitimate under the task's constraint: no wheel is read. The cost
is that the prior is open loop, so it cannot notice the wheels slipping, being
stalled, or the rover being pushed. Correcting that needs an absolute reference,
which for this sensor suite means matching LiDAR scans against features in the
environment.

Linear x and y are both given usable covariances. Forward speed because the
command tracked reality at 98-102%; sideways speed because the rover has no way
to drive sideways, so zero is a real measurement rather than an absence of one.
It is not exactly zero - a skid-steer scrubs sideways while it turns - hence a
variance that admits a slide of a few centimetres a second rather than claiming
certainty.

That distinction matters. Left unmeasured, sideways velocity is a free state:
during a long pivot, with no position measurement to anchor it, the estimate
slides sideways at whatever the filter last believed. A 92 degree turn between
greenhouse lanes drifted the estimate 0.74 m sideways while the rover stood
still on the spot, and it then "corrected" towards a lane centre that was not
where it thought, and drove into the plants.

Angular velocity is deliberately left with a huge covariance so the filter
ignores it: on a skid-steer, commanded rotation bears little relation to actual
rotation (measured at roughly 40%). Heading comes from the IMU instead.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped

LARGE = 1e6


class CmdPriorNode(Node):

    def __init__(self):
        super().__init__('cmd_prior_node')

        self.declare_parameter('input_topic', '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('output_topic', '/cmd_vel_prior')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('linear_variance', 0.02)
        # Sideways: zero, give or take the scrub of a turning skid-steer.
        self.declare_parameter('lateral_variance', 0.01)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('command_timeout', 0.5)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        rate = self.get_parameter('publish_rate').value

        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        self.publisher = self.create_publisher(TwistWithCovarianceStamped, output_topic, 10)
        self.create_subscription(Twist, input_topic, self.cmd_callback, 10)

        # The prior must be a CONTINUOUS stream, not one message per command.
        # A Kalman filter with no velocity measurement does not sit still: it
        # propagates its last estimate through the process model with nothing to
        # correct it. Publishing only on command let the estimate free-run
        # between bursts and run away entirely once commands stopped — observed
        # as the pose climbing past 50 m while the rover stood still.
        self.create_timer(1.0 / rate, self.publish_prior)

        self.get_logger().info(
            f'Publishing the commanded velocity as a motion prior at {rate:.0f} Hz: '
            f'{input_topic} -> {output_topic}')

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def publish_prior(self):
        # A command older than the timeout means nothing is driving the robot,
        # which is itself information: the velocity is zero. Saying so keeps the
        # filter corrected instead of letting it coast.
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        msg = self.last_cmd if age < self.get_parameter('command_timeout').value else Twist()

        out = TwistWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.get_parameter('frame_id').value
        out.twist.twist = msg
        out.twist.twist.linear.y = 0.0      # the constraint, stated explicitly

        variance = self.get_parameter('linear_variance').value
        covariance = [0.0] * 36
        covariance[0] = variance     # vx — trusted
        covariance[7] = self.get_parameter('lateral_variance').value  # vy — ~0
        covariance[14] = LARGE       # vz
        covariance[21] = LARGE       # vroll
        covariance[28] = LARGE       # vpitch
        covariance[35] = LARGE       # vyaw — ignored; the IMU supplies heading
        out.twist.covariance = covariance

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdPriorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
