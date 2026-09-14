"""Republish the Gazebo IMU with realistic covariances.

Gazebo's IMU plugin publishes all-zero covariance matrices. A Kalman filter
reads a zero variance as "this measurement is perfect", which makes the update
ill-conditioned. In earlier tasks that went unnoticed because wheel odometry
also supplied position, which anchored the filter. With wheel feedback removed
there is no position observation at all, and the filter diverges outright —
observed here as the estimate running away to 1e6 metres within a few seconds.

SDF's <imu> block has noise settings for angular velocity and linear
acceleration but none for orientation, so the covariance cannot be fixed at the
source. This node supplies it instead.

Roll and pitch get a deliberately large variance: the rover is planar, the
filter runs in two_d_mode, and those axes carry no useful information. Yaw is
the measurement that matters and is trusted accordingly.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

UNUSED = 1e3


class ImuRelayNode(Node):

    def __init__(self):
        super().__init__('imu_relay_node')

        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/imu/data_cov')
        self.declare_parameter('yaw_variance', 0.0001)
        self.declare_parameter('yaw_rate_variance', 0.001)
        self.declare_parameter('accel_variance', 0.05)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.publisher = self.create_publisher(Imu, output_topic, 20)
        self.create_subscription(Imu, input_topic, self.imu_callback, 20)
        self.get_logger().info(
            f'Adding covariances to the IMU stream: {input_topic} -> {output_topic}')

    def imu_callback(self, msg):
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration

        yaw_var = self.get_parameter('yaw_variance').value
        rate_var = self.get_parameter('yaw_rate_variance').value
        accel_var = self.get_parameter('accel_variance').value

        out.orientation_covariance = [UNUSED, 0.0, 0.0,
                                      0.0, UNUSED, 0.0,
                                      0.0, 0.0, yaw_var]
        out.angular_velocity_covariance = [UNUSED, 0.0, 0.0,
                                           0.0, UNUSED, 0.0,
                                           0.0, 0.0, rate_var]
        out.linear_acceleration_covariance = [accel_var, 0.0, 0.0,
                                              0.0, accel_var, 0.0,
                                              0.0, 0.0, accel_var]
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
