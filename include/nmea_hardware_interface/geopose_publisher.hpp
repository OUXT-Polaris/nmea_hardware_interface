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

#ifndef NMEA_HARDWARE_INTERFACE__GEOPOINT_PUBLISHER_HPP_
#define NMEA_HARDWARE_INTERFACE__GEOPOINT_PUBLISHER_HPP_

#include <Poco/SharedMemory.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <controller_interface/controller_interface.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <geographic_msg/msg/geopoint.hpp>
#include <rclcpp/rclcpp.hpp>


#include <string>
#include <unordered_map>
#include <vector>

namespace nmea_hardware_interface
{
class GeoPosePublisher : public controller_interface::ControllerInterface
{
public:

  NMEA_HARDWARE_INTERFACE_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;
  
  NMEA_HARDWARE_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  NMEA_HARDWARE_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    std::vector<std::string> interface_names = {};
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

#if GALACTIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init()
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
#endif

  NMEA_HARDWARE_INTERFACE_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override;

  NMEA_HARDWARE_INTERFACE_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  NMEA_HARDWARE_INTERFACE_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  NMEA_HARDWARE_INTERFACE_PUBLIC
  controller_interface::return_type update() override;

private:
  void publishGeopoint();
  double publish_rate_;
  double update_duration_;
  std::string joint_;
  std::string shared_memory_key_;
  std::shared_ptr<rclcpp::Clock> clock_ptr_;
  std::shared_ptr<Poco::SharedMemory> geopose_memory_ptr_;
  double configure_time_;
  double next_update_time_;
  rclcpp::Publisher<geographic_msg::msg::geopoint>::SharedPtr geopoint_pub_;
  

};
}  // namespace nmea_hardware_interface

#endif  // NMEA_HARDWARE_INTERFACE__GEOPOINT_PUBLISHER_HPP_