// A Nav2 controller that steers by the crop lane, not by the planned path.
//
// Nav2's own controllers follow a path computed from a map. In a 1.6 m lane
// that is the wrong reference: the path is only as good as the map and the
// pose estimate, while the rows either side of the rover are measured afresh
// every scan and put it within about 3 cm of the lane's true centre.
//
// So this splits the two jobs. The global plan decides how far down the lane
// to go and where to stop. The measured lane - published by row_localizer_node
// on /lane_centreline - decides where the rover sits across it. When the lane
// cannot be measured (a gap in a row, a headland) it falls back to following
// the plan, so it is always safe to use.

#ifndef EGROBOTS_LANE_CONTROLLER__LANE_CONTROLLER_HPP_
#define EGROBOTS_LANE_CONTROLLER__LANE_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace egrobots_lane_controller
{

class LaneController : public nav2_core::Controller
{
public:
  LaneController() = default;
  ~LaneController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  void laneCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // The lane as measured right now, in the costmap's global frame. Returns
  // false when there is no fresh measurement to steer by.
  bool measuredLane(double & x, double & y, double & yaw);

  // Heading to aim for when no lane is in view: towards a point lookahead_
  // metres along the plan.
  bool headingFromPlan(const geometry_msgs::msg::PoseStamped & pose, double & heading);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("LaneController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr lane_sub_;
  geometry_msgs::msg::PoseStamped lane_;
  bool have_lane_{false};

  nav_msgs::msg::Path global_plan_;

  // Tuning, mirroring config/greenhouse_params.yaml so the two controllers
  // behave the same way.
  double desired_linear_vel_{0.6};
  double max_angular_vel_{1.2};
  double min_pivot_speed_{0.6};
  double align_tolerance_{0.0698};     // 4 degrees
  double rotate_in_place_{0.785};      // 45 degrees: beyond this, turn on the spot
  double min_speed_fraction_{0.4};     // floor under the cornering speed taper
  bool turn_in_place_only_{true};      // never rotate and translate at once
  double heading_kp_{2.5};
  double cross_track_kp_{1.5};
  double max_correction_{0.873};       // 50 degrees
  double cross_track_deadband_{0.03};
  double lane_offset_{0.0};
  double lane_timeout_{1.0};
  bool use_lane_{true};                // false: steer by the plan only
  double lookahead_{0.6};
  std::string lane_topic_{"/lane_centreline"};

  double speed_limit_{1.0};            // multiplier applied by setSpeedLimit
};

}  // namespace egrobots_lane_controller

#endif  // EGROBOTS_LANE_CONTROLLER__LANE_CONTROLLER_HPP_
