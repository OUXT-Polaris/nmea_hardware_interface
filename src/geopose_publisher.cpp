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

#include <nmea_hardware_interface/geopose_publisher.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace nmea_hardware_interface
{
controller_interface::return_type GeoPosePublisher::init(const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }
  auto node = get_node();
  clock_ptr_ = node->get_clock();
 
  std::cout << __FILE__ << "," << __LINE__ << std::endl;
  //node->declare_parameter("geopose_topic", "");
  geopose_topic_ = node->get_parameter("geopose_topic").as_string();

  //node->declare_parameter("frame_id", "");
  frame_id_ = node->get_parameter("frame_id").as_string();

  //node->declare_parameter("publish_rate", 30.0);
  publish_rate_ = node->get_parameter("publish_rate").as_double();
  update_duration_ = 1.0 / publish_rate_;
  
  //node->declare_parameter("qos", "sensor");
  qos_ = node->get_parameter("qos").as_string();
  
  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GeoPosePublisher::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  configure_time_ = node->get_clock()->now().seconds();
  next_update_time_ = configure_time_ + update_duration_;
  size_ = 10;

  if (qos_ == "sensor") {
    geopose_pub_ =
      node->create_publisher<geographic_msgs::msg::GeoPose>(
      geopose_topic_,
      rclcpp::SensorDataQoS());
  } else if (qos_ == "system_default") {
    geopose_pub_ =
      node->create_publisher<geographic_msgs::msg::GeoPose>(
      geopose_topic_,
      rclcpp::SystemDefaultsQoS());
  } else {
    throw std::runtime_error("invalid qos setting : " + qos_);
  }
  geopose_pub_realtime_ =
    std::make_shared<realtime_tools::RealtimePublisher<
        geographic_msgs::msg::GeoPose>>(geopose_pub_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

#if GALACTIC
controller_interface::return_type GeoPosePublisher::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
#else
controller_interface::return_type GeoPosePublisher::update()
#endif
{
  auto node = get_node();
#if GALACTIC
  const auto now = time;
#else
  const auto now = node->get_clock()->now();
#endif

  if (std::fabs(now.seconds() - next_update_time_) < update_duration_ * 0.5) {

    std_msgs::msg::Header header;
    header.frame_id = frame_id_;

    header.stamp = now;

    geographic_msgs::msg::GeoPose::SharedPtr geopose_msg = std::make_shared<geographic_msgs::msg::GeoPose>(geopose_);
    if (geopose_pub_realtime_->trylock()) {
      geopose_pub_realtime_->msg_ = *geopose_msg;
      geopose_pub_realtime_->unlockAndPublish();
    }
    next_update_time_ = next_update_time_ + update_duration_;
  }
  return controller_interface::return_type::OK;
}
}  // namespace nmea_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  nmea_hardware_interface::GeoPosePublisher, controller_interface::ControllerInterface)
