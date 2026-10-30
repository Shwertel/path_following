#include "egrobots_lane_controller/lane_controller.hpp"

#include <algorithm>
#include <limits>
#include <cmath>
#include <memory>
#include <string>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace egrobots_lane_controller
{

static double wrap(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

void LaneController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  using nav2_util::declare_parameter_if_not_declared;
  auto p = [&](const std::string & key) { return plugin_name_ + "." + key; };

  declare_parameter_if_not_declared(node, p("desired_linear_vel"), rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(node, p("max_angular_vel"), rclcpp::ParameterValue(1.2));
  declare_parameter_if_not_declared(node, p("min_pivot_speed"), rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(node, p("align_tolerance_deg"), rclcpp::ParameterValue(4.0));
  declare_parameter_if_not_declared(node, p("heading_kp"), rclcpp::ParameterValue(2.5));
  declare_parameter_if_not_declared(node, p("cross_track_kp"), rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, p("max_correction_deg"), rclcpp::ParameterValue(50.0));
  declare_parameter_if_not_declared(node, p("cross_track_deadband"), rclcpp::ParameterValue(0.03));
  declare_parameter_if_not_declared(node, p("rotate_in_place_deg"), rclcpp::ParameterValue(45.0));
  // The real rover cannot rotate and translate at once. With this true the
  // controller only ever issues a pure pivot or a pure straight run, and
  // rotate_in_place_deg is unused.
  declare_parameter_if_not_declared(node, p("turn_in_place_only"), rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, p("min_speed_fraction"), rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(node, p("lane_offset"), rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, p("lane_timeout"), rclcpp::ParameterValue(1.0));
  // Whether to steer by the measured lane at all. Re-read every cycle, so
  // the mission node can turn it off for the legs that are not lanes.
  declare_parameter_if_not_declared(node, p("use_lane"), rclcpp::ParameterValue(true));
  // Correcting gently keeps the pivot count down, and at 0.25 m of deadband
  // with kp 0.8 the rover drifted to 46.8 cm of a 48 cm margin while the lane
  // fit was perfectly good - it noticed, and then asked for 21 degrees, which
  // was not enough. Past urgent_offset it stops being gentle: a lane that is
  // nearly touching a plant is worth a pivot.
  declare_parameter_if_not_declared(node, p("urgent_offset"), rclcpp::ParameterValue(0.30));
  declare_parameter_if_not_declared(node, p("urgent_cross_track_kp"), rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, p("urgent_max_correction_deg"), rclcpp::ParameterValue(50.0));
  declare_parameter_if_not_declared(node, p("lookahead_dist"), rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, p("lane_topic"), rclcpp::ParameterValue(std::string("/lane_centreline")));

  double align_deg, correction_deg;
  node->get_parameter(p("desired_linear_vel"), desired_linear_vel_);
  node->get_parameter(p("max_angular_vel"), max_angular_vel_);
  node->get_parameter(p("min_pivot_speed"), min_pivot_speed_);
  node->get_parameter(p("align_tolerance_deg"), align_deg);
  node->get_parameter(p("heading_kp"), heading_kp_);
  node->get_parameter(p("cross_track_kp"), cross_track_kp_);
  node->get_parameter(p("max_correction_deg"), correction_deg);
  node->get_parameter(p("cross_track_deadband"), cross_track_deadband_);
  double rotate_deg;
  node->get_parameter(p("rotate_in_place_deg"), rotate_deg);
  node->get_parameter(p("min_speed_fraction"), min_speed_fraction_);
  node->get_parameter(p("turn_in_place_only"), turn_in_place_only_);
  rotate_in_place_ = rotate_deg * M_PI / 180.0;
  node->get_parameter(p("lane_offset"), lane_offset_);
  node->get_parameter(p("lane_timeout"), lane_timeout_);
  node->get_parameter(p("use_lane"), use_lane_);
  node->get_parameter(p("lookahead_dist"), lookahead_);
  node->get_parameter(p("lane_topic"), lane_topic_);
  align_tolerance_ = align_deg * M_PI / 180.0;
  max_correction_ = correction_deg * M_PI / 180.0;

  lane_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    lane_topic_, rclcpp::QoS(10),
    std::bind(&LaneController::laneCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_, "%s: steering by %s, falling back to the plan after %.1f s without it",
    plugin_name_.c_str(), lane_topic_.c_str(), lane_timeout_);
}

void LaneController::cleanup() { lane_sub_.reset(); }
void LaneController::activate() {}
void LaneController::deactivate() {}

void LaneController::setPlan(const nav_msgs::msg::Path & path) { global_plan_ = path; }

void LaneController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    speed_limit_ = 1.0;
  } else {
    speed_limit_ = percentage ? speed_limit / 100.0 : speed_limit / desired_linear_vel_;
  }
  speed_limit_ = std::clamp(speed_limit_, 0.0, 1.0);
}

void LaneController::laneCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  lane_ = *msg;
  have_lane_ = true;
}

bool LaneController::measuredLane(double & x, double & y, double & yaw)
{
  // Re-read rather than cache: a headland turn has no lane to steer by, and
  // the last lane's measurement is anchored in odom, so it stays live and
  // plausible for the whole of lane_timeout after the rover has stopped
  // looking at the rows it came from - long enough to drag a turn sideways.
  if (auto node = node_.lock()) {
    node->get_parameter(plugin_name_ + ".use_lane", use_lane_);
  }
  if (!use_lane_) {
    return false;
  }
  if (!have_lane_) {
    return false;
  }
  // A small NEGATIVE age is normal and must not be treated as invalid. The
  // measurement carries the scan's timestamp, and under simulated time this
  // node's /clock can lag the scan by a few tens of milliseconds - measured,
  // -0.047 s, well inside one 10 Hz scan period. Rejecting those threw away
  // every other good measurement and left the controller following the planned
  // path for about half of every lane leg, which is exactly what it exists not
  // to do. Only a large negative age means something is actually wrong, such
  // as the clock jumping or the wrong clock source.
  const double age =
    (clock_->now() - rclcpp::Time(lane_.header.stamp, clock_->get_clock_type())).seconds();
  if (age > lane_timeout_ || age < -lane_timeout_) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
                         "%s: no usable lane measurement, age %.3f s (limit %.1f)",
                         plugin_name_.c_str(), age, lane_timeout_);
    return false;
  }

  geometry_msgs::msg::PoseStamped in_costmap_frame;
  try {
    tf_->transform(lane_, in_costmap_frame, costmap_ros_->getGlobalFrameID(),
                   tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "Lane transform failed: %s", e.what());
    return false;
  }

  x = in_costmap_frame.pose.position.x;
  y = in_costmap_frame.pose.position.y;
  yaw = tf2::getYaw(in_costmap_frame.pose.orientation);
  return true;
}

bool LaneController::headingFromPlan(
  const geometry_msgs::msg::PoseStamped & pose, double & heading)
{
  if (global_plan_.poses.empty()) {
    return false;
  }

  // The plan is planned in the global costmap's frame (map); the pose handed
  // to a controller is in the LOCAL costmap's frame (odom). Comparing them
  // directly silently offsets the carrot by the whole map->odom correction -
  // measured, that walked the rover from 4 cm off the lane centre to 32 cm,
  // far enough for the collision monitor to freeze it against a crop row.
  geometry_msgs::msg::TransformStamped plan_to_costmap;
  try {
    plan_to_costmap = tf_->lookupTransform(
      costmap_ros_->getGlobalFrameID(), global_plan_.header.frame_id,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "Plan transform failed: %s", e.what());
    return false;
  }

  auto to_costmap_frame = [&](const geometry_msgs::msg::PoseStamped & in) {
    geometry_msgs::msg::PoseStamped out;
    tf2::doTransform(in, out, plan_to_costmap);
    return out;
  };

  // Find where the rover is ALONG the plan first, then take the carrot from
  // there. Searching the whole plan from its start, as this used to, returns
  // the first pose merely far enough away - which, once the rover is well down
  // a long path, is the pose BEHIND it at the very beginning. The rover then
  // turns round and drives back to the start. It drove 19.9 m up a lane and
  // 18.7 m back out again before this was spotted.
  //
  // It never showed while legs came through NavigateToPose, because the
  // behaviour tree replans at 1 Hz and every plan therefore began at the
  // rover. A path sent once and left alone has no such property.
  size_t closest = 0;
  double closest_d = std::numeric_limits<double>::max();
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const auto q = to_costmap_frame(global_plan_.poses[i]);
    const double d = std::hypot(
      q.pose.position.x - pose.pose.position.x,
      q.pose.position.y - pose.pose.position.y);
    if (d < closest_d) {
      closest_d = d;
      closest = i;
    }
  }

  // The first pose at or beyond lookahead_, searching FORWARD from there.
  for (size_t i = closest; i < global_plan_.poses.size(); ++i) {
    const auto q = to_costmap_frame(global_plan_.poses[i]);
    const double dx = q.pose.position.x - pose.pose.position.x;
    const double dy = q.pose.position.y - pose.pose.position.y;
    if (std::hypot(dx, dy) >= lookahead_) {
      heading = std::atan2(dy, dx);
      return true;
    }
  }
  const auto last = to_costmap_frame(global_plan_.poses.back());
  const double dx = last.pose.position.x - pose.pose.position.x;
  const double dy = last.pose.position.y - pose.pose.position.y;
  if (std::hypot(dx, dy) < 1e-3) {
    return false;
  }
  heading = std::atan2(dy, dx);
  return true;
}

geometry_msgs::msg::TwistStamped LaneController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = pose.header.frame_id;
  cmd.header.stamp = clock_->now();

  // Re-read the two values an experiment wants to vary, so a config can be
  // tested with a parameter set instead of a rebuild. Both are cheap lookups.
  if (auto node = node_.lock()) {
    node->get_parameter(plugin_name_ + ".cross_track_deadband", cross_track_deadband_);
    node->get_parameter(plugin_name_ + ".lane_offset", lane_offset_);
    // How hard it corrects, re-read so it can be tuned without a rebuild.
    // The lane is 1.60 m of free corridor and the rover is 0.64 m wide, so
    // there is 0.48 m of clearance a side: chasing the centreline to within a
    // few centimetres buys nothing and costs a pivot every time.
    double correction_deg = max_correction_ * 180.0 / M_PI;
    double align_deg = align_tolerance_ * 180.0 / M_PI;
    node->get_parameter(plugin_name_ + ".cross_track_kp", cross_track_kp_);
    node->get_parameter(plugin_name_ + ".max_correction_deg", correction_deg);
    node->get_parameter(plugin_name_ + ".align_tolerance_deg", align_deg);
    double urgent_deg = urgent_max_correction_ * 180.0 / M_PI;
    node->get_parameter(plugin_name_ + ".urgent_offset", urgent_offset_);
    node->get_parameter(plugin_name_ + ".urgent_cross_track_kp", urgent_cross_track_kp_);
    node->get_parameter(plugin_name_ + ".urgent_max_correction_deg", urgent_deg);
    urgent_max_correction_ = urgent_deg * M_PI / 180.0;
    max_correction_ = correction_deg * M_PI / 180.0;
    align_tolerance_ = align_deg * M_PI / 180.0;
  }

  const double rover_yaw = tf2::getYaw(pose.pose.orientation);
  double desired_heading = rover_yaw;

  double lx, ly, lyaw;
  if (measuredLane(lx, ly, lyaw)) {
    // Steer by the lane itself. Cross-track is measured from the line through
    // the measured centre, less where in the lane we mean to sit.
    const double nx = -std::sin(lyaw), ny = std::cos(lyaw);
    const double cross =
      (pose.pose.position.x - lx) * nx + (pose.pose.position.y - ly) * ny - lane_offset_;

    if (std::abs(cross) < cross_track_deadband_) {
      desired_heading = lyaw;
    } else {
      const bool urgent = std::abs(cross) > urgent_offset_;
      const double kp = urgent ? urgent_cross_track_kp_ : cross_track_kp_;
      const double cap = urgent ? urgent_max_correction_ : max_correction_;
      const double correction = std::clamp(-kp * cross, -cap, cap);
      desired_heading = lyaw + correction;
      if (urgent) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "%s: %.2f m off the lane centre - correcting hard (%.0f deg)",
          plugin_name_.c_str(), cross, correction * 180.0 / M_PI);
      }
    }
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 5000, "%s: steering by the measured lane",
                         plugin_name_.c_str());
  } else if (headingFromPlan(pose, desired_heading)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
                         "%s: no lane measurement - following the plan instead",
                         plugin_name_.c_str());
  } else {
    return cmd;   // nothing to steer by and nowhere to go
  }

  const double error = wrap(desired_heading - rover_yaw);

  // Turn while driving, on an arc, and only stop to turn when the correction
  // is too sharp to drive through.
  //
  // The old rule was rotate OR translate, never both - inherited from a brief
  // where nothing corrected position, so every metre driven while the heading
  // was still changing was integrated along a stale heading and cost roughly
  // 70x more error per metre. AMCL corrects position now, so that reasoning no
  // longer holds, and the cost of the old rule is visible: the rover spent
  // 14.2% of a sweep stopped and turning, with 49 reversals of turn direction,
  // against pure pursuit's 0% and 0.
  //
  // min_pivot_speed is deliberately NOT applied here. Its job is to clear the
  // ~0.1 rad/s at which this skid-steer breaks away from rest; once the wheels
  // are rolling, an arbitrarily small angular command executes fine. Applying
  // that floor to a rolling correction is what made the steering twitch.
  // turn_in_place_only is the real machine's constraint, not a tuning choice.
  // It never rotates while translating, so the command is either a pure pivot
  // or a pure straight run and never a blend of the two. Arc steering is
  // smoother and tracks a lane better, but this platform cannot do it, so a
  // simulation that arcs is not simulating this robot.
  const double threshold = turn_in_place_only_ ? align_tolerance_ : rotate_in_place_;

  if (std::abs(error) > threshold) {
    // Pure rotation. min_pivot_speed clears the roughly 0.1 rad/s at which this
    // skid-steer breaks away from rest, which matters here in a way it did not
    // for arc steering: every correction now starts from a standstill.
    const double floor_rate = std::min(min_pivot_speed_, max_angular_vel_);
    const double rate = std::min(
      max_angular_vel_, std::max(floor_rate, heading_kp_ * std::abs(error)));
    cmd.twist.angular.z = std::copysign(rate, error);
    cmd.twist.linear.x = 0.0;
  } else if (turn_in_place_only_) {
    // Pure translation. No angular term at all, however small the residual
    // heading error: the alternative is a blend, which is the thing being
    // excluded.
    cmd.twist.angular.z = 0.0;
    cmd.twist.linear.x = desired_linear_vel_ * speed_limit_;
  } else {
    cmd.twist.angular.z = std::clamp(
      heading_kp_ * error, -max_angular_vel_, max_angular_vel_);
    // Ease off through a correction: full speed when aligned, tapering with the
    // cosine of the heading error so a sharp turn is not taken at full tilt.
    const double taper = std::max(min_speed_fraction_, std::cos(error));
    cmd.twist.linear.x = desired_linear_vel_ * speed_limit_ * taper;
  }
  return cmd;
}

}  // namespace egrobots_lane_controller

PLUGINLIB_EXPORT_CLASS(
  egrobots_lane_controller::LaneController, nav2_core::Controller)
