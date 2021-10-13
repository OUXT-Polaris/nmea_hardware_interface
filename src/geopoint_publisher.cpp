// Copyright (c) 2021 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <nmea_hardware_interface/geopoint_publisher.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace nmea_hardware_interface
{
controller_interface::return_type GeoPointPublisher::init(const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }
  auto node = get_node();
  clock_ptr_ = node->get_clock();
 
  node->declare_parameter("geopoint_topic", "");
  geopoint_topic_ = node->get_parameter("geopoint_topic").as_string();

  node->declare_parameter("frame_id", "");
  frame_id_ = node->get_parameter("frame_id").as_string();

  node->declare_parameter("publish_rate", 30.0);
  publish_rate_ = node->get_parameter("publish_rate").as_double();
  update_duration_ = 1.0 / publish_rate_;
  
  node->declare_parameter("qos", "sensor");
  qos_ = node->get_parameter("qos").as_string();
  
  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GeoPointPublisher::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  configure_time_ = node->get_clock()->now().seconds();
  next_update_time_ = configure_time_ + update_duration_;
  size_ = 10;

  if (qos_ == "sensor") {
    geopoint_pub_ =
      node->create_publisher<geographic_msgs::msg::GeoPoint>(
      geopoint_topic_,
      rclcpp::SensorDataQoS());
  } else if (qos_ == "system_default") {
    geopoint_pub_ =
      node->create_publisher<geographic_msgs::msg::GeoPoint>(
      geopoint_topic_,
      rclcpp::SystemDefaultsQoS());
  } else {
    throw std::runtime_error("invalid qos setting : " + qos_);
  }
  geopoint_pub_realtime_ =
    std::make_shared<realtime_tools::RealtimePublisher<
        geographic_msgs::msg::GeoPoint>>(geopoint_pub_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


void GeoPointPublisher::publishGeopoint()
{
  auto node = get_node();
  const auto now = node->get_clock()->now();

  std_msgs::msg::Header header;
  header.frame_id = frame_id_;

  header.stamp = now;


  geographic_msgs::msg::GeoPoint::SharedPtr geopoint_msg = std::make_shared<geographic_msgs::msg::GeoPoint>(geopoint_);
  if (geopoint_pub_realtime_->trylock()) {
    geopoint_pub_realtime_->msg_ = *geopoint_msg;
    geopoint_pub_realtime_->unlockAndPublish();
  }
  next_update_time_ = next_update_time_ + update_duration_;
}

#if GALACTIC
controller_interface::return_type GeoPointPublisher::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
#else
controller_interface::return_type GeoPointPublisher::update()
#endif
{
  auto node = get_node();
#if GALACTIC
  const auto now = time;
#else
  const auto now = node->get_clock()->now();
#endif

  if (std::fabs(now.seconds() - next_update_time_) < update_duration_ * 0.5) {
    publishGeopoint();
    return controller_interface::return_type::OK;
  }
  return controller_interface::return_type::OK;
}
}  // namespace nmea_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  nmea_hardware_interface::GeoPointPublisher, controller_interface::ControllerInterface)
