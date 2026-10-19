"""Turn the front depth camera into the LaserScan the rest of the stack expects.

Six things in this stack consume /scan - amcl, slam_toolbox, both Nav2
costmaps, the collision monitor, and row_localizer_node - and not one of them
accepts a depth image. So rather than rewrite six consumers, this node
synthesises a scan from the camera.

The hard part is the tilt. Pointing the camera 18 degrees down is the whole
reason to carry one: a planar scanner at 0.77 m flies straight over a toolbox
lying in the lane, and a tilted camera does not. But it also means most of the
image is floor. Flattening the cloud naively turns that floor into a solid wall
of obstacles about 2.15 m ahead, exactly where the optical axis meets the
ground, and the robot refuses to move.

So the pipeline deprojects to 3D, rotates into base_link where "up" means
something, throws away everything outside a height band, and only then
flattens what is left.

      depth image + camera_info
               |
               v
      deproject with fx fy cx cy        -> points in camera_link_optical
               |
               v
      rotate by the TF                  -> points in base_link
               |
               v
      keep min_z < z < max_z            -> floor and ceiling gone
               |
               v
      bearing/range, min per bin        -> LaserScan in camera_scan_frame

camera_scan_frame is the camera's position but LEVEL. The scan is a horizontal
slice, so publishing it in the pitched camera frame would have amcl read every
beam as tilted into the ground.

Bins with nothing in them are published as inf rather than as a max-range
return. A finite reading would mark a phantom obstacle at whatever range the
floor happened to be; inf marks nothing, and the costmaps are configured with
inf_is_valid so it still clears the ray.
"""
import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan


class DepthToScan(Node):

    def __init__(self):
        super().__init__('depth_to_scan_node')

        # Frame the scan is published in. Must be level, not the pitched
        # camera frame.
        self.declare_parameter('scan_frame', 'camera_scan_frame')
        self.declare_parameter('base_frame', 'base_link')

        # Height band, in base_link. base_link rides 0.25 m above the ground,
        # so z = -0.25 IS the floor.
        #   min_z -0.15 keeps anything standing more than 10 cm proud of the
        #               floor, which is the point of tilting the camera down.
        #   max_z  1.25 is 1.5 m above the ground, above the plants.
        self.declare_parameter('min_z', -0.15)
        self.declare_parameter('max_z', 1.25)

        self.declare_parameter('range_min', 0.3)
        self.declare_parameter('range_max', 6.0)

        # One bin per half degree across the camera's 87 degree field.
        self.declare_parameter('angle_increment_deg', 0.5)

        # Subsampling. 424x240 is 102k points; striding 2x2 leaves 25k, which
        # numpy handles comfortably at 15 Hz and costs nothing in practice -
        # a plant is hundreds of pixels wide, not one.
        self.declare_parameter('row_stride', 2)
        self.declare_parameter('col_stride', 2)

        self.declare_parameter('publish_cloud', True)
        # The debug cloud is about 25k points a frame and only rviz reads it.
        # Publishing every frame is what made rviz stop responding, so send
        # one in N. The scan itself is never throttled.
        self.declare_parameter('cloud_every_n', 5)

        # Normally /scan. Pointed somewhere else when the LiDAR is also
        # fitted, so the two can be compared against each other instead of
        # fighting over one topic.
        self.declare_parameter('scan_topic', '/scan')

        self.info = None
        self.tf_base_optical = None     # 4x4, cached. The joints are fixed.
        self.scan_offset = None         # base_link -> camera_scan_frame

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.scan_pub = self.create_publisher(
            LaserScan, self.get_parameter('scan_topic').value, 10)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info',
                                 self.info_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self.image_callback, sensor_qos)

        self.warned_tf = False
        self.frames = 0
        self.create_timer(10.0, self.report)

    # ------------------------------------------------------------------ setup

    def info_callback(self, msg):
        if self.info is None:
            self.info = msg
            k = msg.k
            self.get_logger().info(
                f'camera_info received, {msg.width}x{msg.height}, '
                f'fx={k[0]:.1f} fy={k[4]:.1f} cx={k[2]:.1f} cy={k[5]:.1f}')

    def lookup_static(self, optical_frame):
        """Cache the two fixed transforms. Both are URDF joints, so once is enough."""
        base = self.get_parameter('base_frame').value
        scan = self.get_parameter('scan_frame').value
        try:
            t1 = self.tf_buffer.lookup_transform(
                base, optical_frame, rclpy.time.Time())
            t2 = self.tf_buffer.lookup_transform(
                base, scan, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            if not self.warned_tf:
                self.get_logger().warn(f'waiting for TF - {exc}')
                self.warned_tf = True
            return False

        self.tf_base_optical = matrix_of(t1.transform)
        tr = t2.transform.translation
        self.scan_offset = np.array([tr.x, tr.y, tr.z])
        self.get_logger().info(
            f'TF cached. {optical_frame} -> {base}, and {scan} sits at '
            f'({tr.x:.2f}, {tr.y:.2f}, {tr.z:.2f}) in {base}.')
        return True

    # ------------------------------------------------------------- conversion

    def depth_metres(self, msg):
        """Depth as float metres, whatever the encoding says."""
        if msg.encoding == '32FC1':
            d = np.frombuffer(msg.data, dtype=np.float32)
        elif msg.encoding == '16UC1':
            # A real RealSense publishes millimetres as uint16.
            d = np.frombuffer(msg.data, dtype=np.uint16).astype(np.float32) / 1000.0
        else:
            self.get_logger().error(
                f'unsupported depth encoding {msg.encoding}', once=True)
            return None
        return d.reshape(msg.height, msg.width)

    def image_callback(self, msg):
        if self.info is None:
            return
        if self.tf_base_optical is None and not self.lookup_static(msg.header.frame_id):
            return

        depth = self.depth_metres(msg)
        if depth is None:
            return

        rs = max(1, self.get_parameter('row_stride').value)
        cs = max(1, self.get_parameter('col_stride').value)
        depth = depth[::rs, ::cs]

        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]

        rows = np.arange(0, msg.height, rs, dtype=np.float32)
        cols = np.arange(0, msg.width, cs, dtype=np.float32)
        uu, vv = np.meshgrid(cols, rows)

        z = depth
        valid = np.isfinite(z) & (z > 0.0)
        if not np.any(valid):
            return

        z = z[valid]
        uu = uu[valid]
        vv = vv[valid]

        # Optical convention. x right, y down, z forward.
        x = (uu - cx) * z / fx
        y = (vv - cy) * z / fy
        pts_optical = np.stack([x, y, z, np.ones_like(z)], axis=0)

        pts_base = self.tf_base_optical @ pts_optical
        bx, by, bz = pts_base[0], pts_base[1], pts_base[2]

        # The floor and the ceiling go here, and this single line is what
        # makes a down-tilted camera usable at all.
        band = ((bz > self.get_parameter('min_z').value)
                & (bz < self.get_parameter('max_z').value))
        bx, by, bz = bx[band], by[band], bz[band]

        # Into the level scan frame.
        sx = bx - self.scan_offset[0]
        sy = by - self.scan_offset[1]

        rng = np.hypot(sx, sy)
        rmin = self.get_parameter('range_min').value
        rmax = self.get_parameter('range_max').value
        near = (rng > rmin) & (rng < rmax)
        sx, sy, rng = sx[near], sy[near], rng[near]

        self.publish_scan(msg, sx, sy, rng)
        every = max(1, self.get_parameter('cloud_every_n').value)
        if self.get_parameter('publish_cloud').value and self.frames % every == 0:
            self.publish_cloud(msg, bx[near], by[near], bz[near])
        self.frames += 1

    def publish_scan(self, msg, sx, sy, rng):
        half_fov = math.atan(self.info.width / (2.0 * self.info.k[0]))
        inc = math.radians(self.get_parameter('angle_increment_deg').value)
        nbins = int(2.0 * half_fov / inc) + 1

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.get_parameter('scan_frame').value
        scan.angle_min = -half_fov
        scan.angle_max = -half_fov + inc * (nbins - 1)
        scan.angle_increment = inc
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / 10.0
        scan.range_min = float(self.get_parameter('range_min').value)
        scan.range_max = float(self.get_parameter('range_max').value)

        ranges = np.full(nbins, np.inf, dtype=np.float32)
        if rng.size:
            idx = np.round((np.arctan2(sy, sx) - scan.angle_min) / inc).astype(int)
            keep = (idx >= 0) & (idx < nbins)
            idx, r = idx[keep], rng[keep].astype(np.float32)
            # Nearest return wins the bin, the same rule a real scanner uses.
            np.minimum.at(ranges, idx, r)

        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)

    def publish_cloud(self, msg, bx, by, bz):
        """The kept points, for looking at in rviz. Not consumed by anything."""
        if not hasattr(self, 'cloud_pub'):
            from sensor_msgs.msg import PointCloud2
            self.cloud_pub = self.create_publisher(
                PointCloud2, '/camera/obstacle_points', 1)
        from sensor_msgs_py import point_cloud2
        header = msg.header
        header.frame_id = self.get_parameter('base_frame').value
        pts = np.stack([bx, by, bz], axis=1).astype(np.float32)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))

    def report(self):
        self.get_logger().info(f'{self.frames} scans synthesised')
        self.frames = 0


def matrix_of(transform):
    """4x4 homogeneous matrix from a geometry_msgs Transform."""
    q = transform.rotation
    t = transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),     t.x],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),     t.y],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y), t.z],
        [0.0,                     0.0,                     0.0,                     1.0],
    ])
    return m


def main(args=None):
    rclpy.init(args=args)
    node = DepthToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
