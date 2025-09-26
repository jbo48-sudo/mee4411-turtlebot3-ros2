/*
  Copyright (C) 2013 Nathan Michael

  This file is part of laser_simulator a small laser_simulator for ROS.

  mesh80211s is free software: you can redistribute it and/or modify it under
  the terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
  details.

  You should have received a copy of the GNU General Public License along with
  this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <boost/thread/mutex.hpp>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "laser_simulator/msg/pose_stamped_named_array.hpp"
#include "laser_simulator/LaserSimulator.h"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using std::placeholders::_1;

class LatchedSubscriptionQoS: public rclcpp::QoS
{
public:
  explicit
  LatchedSubscriptionQoS(const int depth = 10)  // NOLINT
  : rclcpp::QoS(rclcpp::KeepLast(depth))
  {
    this->reliable();
    this->transient_local();
  }
};

class LatchedPublisherQoS: public rclcpp::QoS
{
public:
  explicit
  LatchedPublisherQoS(const int depth = 1)  // NOLINT
  : rclcpp::QoS(rclcpp::KeepLast(depth))
  {
    this->reliable();
    this->transient_local();
  }
};

class LaserSimulatorWrapper: public rclcpp::Node {
private:
  LaserSimulator sim;
  std::shared_ptr < rclcpp::Publisher < sensor_msgs::msg::LaserScan >> pub {nullptr};
  std::shared_ptr < rclcpp::Publisher < visualization_msgs::msg::Marker >> pub_marker {nullptr};
  std::shared_ptr < rclcpp::Subscription < nav_msgs::msg::OccupancyGrid >> grid_sub {nullptr};
  std::shared_ptr < rclcpp::Subscription <
  laser_simulator::msg::PoseStampedNamedArray >> agg_sub {nullptr};
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group;
  sensor_msgs::msg::LaserScan msg;
  boost::mutex sim_mutex;
  std::string map_frame_id;
  std::shared_ptr < tf2_ros::TransformListener > tf_listener_ {nullptr};
  std::unique_ptr < tf2_ros::Buffer > tf_buffer_;
  double depth = 0;
  int map_count = 0;
  bool map_set = false;

public:
  LaserSimulatorWrapper() : Node("laser_sim_node")
  {
    // Declare all parameters
    this->declare_parameter("depth", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("offset/x", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("offset/y", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("offset/z", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("offset/roll", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("offset/pitch", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("offset/yaw", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("noise_sd", rclcpp::ParameterType::PARAMETER_DOUBLE);
    this->declare_parameter("frame_id", rclcpp::ParameterType::PARAMETER_STRING);

    // Publishers and subscribers
    //ros::Subscriber odom_sub = n.subscribe("odom", 1, handle_odometry);
    this->grid_sub = this->create_subscription < nav_msgs::msg::OccupancyGrid > (
      "map",
      LatchedSubscriptionQoS(),
      std::bind(&LaserSimulatorWrapper::handle_occupancy_grid, this, _1)
    );
    this->agg_sub = this->create_subscription < laser_simulator::msg::PoseStampedNamedArray > (
      "pose_array", 1, std::bind(&LaserSimulatorWrapper::handle_pose_array, this, _1));

    this->pub = this->create_publisher < sensor_msgs::msg::LaserScan > ("scan", 1);
    this->pub_marker = this->create_publisher < visualization_msgs::msg::Marker >
      ("map_triangles", LatchedPublisherQoS());

    this->tf_buffer_ = std::make_unique < tf2_ros::Buffer > (this->get_clock());
    this->tf_listener_ = std::make_shared < tf2_ros::TransformListener > (*this->tf_buffer_);

    // Get parameters
    this->get_parameter_or("depth", this->depth, 0.5);

    geometry_msgs::msg::Pose offset;
    this->get_parameter_or("offset/x", offset.position.x, 0.0);
    this->get_parameter_or("offset/y", offset.position.y, 0.0);
    this->get_parameter_or("offset/z", offset.position.z, 0.0);
    double roll, pitch, yaw;
    this->get_parameter_or("offset/roll", roll, 0.0);
    this->get_parameter_or("offset/pitch", pitch, 0.0);
    this->get_parameter_or("offset/yaw", yaw, 0.0);

    tf2::Quaternion quat;
    quat.setEuler(yaw, pitch, roll);
    quat.normalize();

    offset.orientation.x = quat.x();
    offset.orientation.y = quat.y();
    offset.orientation.z = quat.z();
    offset.orientation.w = quat.w();

    sim.SetLaserOffset(offset);

    double noise_sd;
    this->get_parameter_or("noise_sd", noise_sd, 0.0);
    if (noise_sd < 0.0) {
      RCLCPP_ERROR(
        this->get_logger(), "%s: noise cannot be negative",
        this->get_name());
    }
    sim.SetLaserNoiseStdDev(noise_sd);
  }

  void initialize(void)
  {
    if (sim.LoadLaserModel(this->shared_from_this()) != 0) {
      RCLCPP_ERROR(
        this->get_logger(), "%s: failed to load laser model",
        this->get_name());
    }

    this->get_parameter_or("frame_id", msg.header.frame_id, std::string("laser"));
    msg.angle_min = sim.GetMinimumAngle();
    msg.angle_max = sim.GetMaximumAngle();
    msg.angle_increment = sim.GetAngleIncrement();
    msg.range_min = sim.GetMinimumRange();
    msg.range_max = sim.GetMaximumRange();

    msg.ranges.resize(sim.GetScanCount());
    msg.intensities.resize(0);
    msg.scan_time = sim.GetScanTime();
    msg.time_increment = msg.scan_time / sim.GetScanCount();

    this->timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    std::chrono::duration < double > timer_duration(sim.GetScanTime());
    this->timer = this->create_wall_timer(
      timer_duration,
      std::bind(&LaserSimulatorWrapper::publish, this),
      timer_cb_group);
  }

private:
  void handle_odometry(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    boost::mutex::scoped_lock sim_mutex;
    msg.header.stamp = odom->header.stamp;
    msg.header.stamp.nanosec += 1000;
    sim.SetPose(odom->pose.pose);
    sim.SetFrameID(odom->child_frame_id);
  }

  void handle_occupancy_grid(const nav_msgs::msg::OccupancyGrid grid)
  {
    boost::mutex::scoped_lock sim_mutex;
    this->map_frame_id = grid.header.frame_id;
    this->sim.SetFrameID(grid.header.frame_id);
    sim.LoadOccupancyGrid(grid, this->depth);
    this->map_set = true;
    RCLCPP_INFO(this->get_logger(), "Got map %d", this->map_count++);
    if (this->pub_marker->get_subscription_count() > 0) {
      visualization_msgs::msg::Marker m = this->sim.GetMarker();
      m.header.frame_id = grid.header.frame_id;
      m.header.stamp = grid.header.stamp;
      this->pub_marker->publish(m);
    }
  }

  void handle_pose_array(const laser_simulator::msg::PoseStampedNamedArray::SharedPtr pose)
  {
    boost::mutex::scoped_lock sim_mutex;
    sim.UpdatePoseArray(*pose);
  }

public:
  void publish()
  {
    boost::mutex::scoped_lock sim_mutex;
    if (!this->map_set) {
      RCLCPP_WARN_ONCE(this->get_logger(), "No map received yet");
      return;
    }
    // msg.header.stamp = this->get_clock()->now() - rclcpp::Duration::from_seconds(sim.GetScanTime());
    msg.header.stamp = this->get_clock()->now();
    geometry_msgs::msg::PoseStamped p, pm;
    // Create pose at origin of laser frame
    p.header.stamp = msg.header.stamp;
    p.header.frame_id = msg.header.frame_id;
    p.pose.orientation.w = 1.0;
    // Transform to map frame to get laser pose
    try {
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        map_frame_id,
        p.header.frame_id,
        rclcpp::Time(),
        rclcpp::Duration::from_seconds(sim.GetScanTime())
      );

      tf2::doTransform(p, pm, transform); // robot_pose is the PoseStamped I want to transform
      sim.SetPose(pm.pose);
      // Get scan and publish
      sim.GetScan(msg.ranges);

      pub->publish(msg);
    } catch(const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "No scan published, could not find transform between %s and %s: %s",
        map_frame_id.c_str(), p.header.frame_id.c_str(), ex.what());
      return;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr < LaserSimulatorWrapper > ptr = std::make_shared < LaserSimulatorWrapper > ();
  ptr->initialize();
  rclcpp::spin(ptr);
  rclcpp::shutdown();
  return 0;
}
