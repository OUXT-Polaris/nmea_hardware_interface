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

#ifndef NMEA_HARDWARE_INTERFACE__GPS_HARDWARE_INTERFACE_HPP_
#define NMEA_HARDWARE_INTERFACE__GPS_HARDWARE_INTERFACE_HPP_

#include <Poco/SharedMemory.h>

#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <memory>
#include <nmea_msgs/msg/sentence.hpp>
#include <geographic_msg/msg/geopoint.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace nmea_hardware_interface
{
class GPSHardwareInterface
: public hardware_interface::BaseInterface<hardware_interface::SensorInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GPSHardwareInterface)

  NMEA_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  NMEA_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  NMEA_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type start() override;

  NMEA_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type stop() override;

  NMEA_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

private:
  std::string device_file_;
  std::string joint_;
  int baud_rate_;
  std::string geopoint_key_;
  
  std::string frame_id_;
  boost::asio::io_service io_;
  std::shared_ptr<boost::asio::serial_port> port_ptr_;
  boost::thread io_thread_;
  boost::thread togeopoint_thread_;
  std::shared_ptr<Poco::SharedMemory> geopoint_memory_ptr_;
  void readSentence();
  void nmeaSentenceCallback(const nmea_msgs::msg::Sentence::SharedPtr msg);
  std::string calculateChecksum(std::string sentence);
  std::string getHexString(uint8_t value);
  
  boost::optional<geographic_msgs::msg::GeoPoint> geopoint_;
  boost::optional<geometry_msgs::msg::Quaternion> quat_;
  bool isGprmcSentence(nmea_msgs::msg::Sentence sentence);
  bool isGphdtSentence(nmea_msgs::msg::Sentence sentence);
  std::vector<std::string> split(const std::string & s, char delim);
  std::vector<std::string> splitChecksum(std::string str);
  boost::optional<std::vector<std::string>> splitSentence(nmea_msgs::msg::Sentence sentence);
  boost::optional<rclcpp::Time> last_timestamp_;

  template <typename T>
  T getParameter(const std::string key, const hardware_interface::ComponentInfo & info) const
  {
    T param;
    getParameter(key, info, param);
    return param;
  }
  void getParameter(
    const std::string & key, const hardware_interface::ComponentInfo & info,
    std::string & parameter) const
  {
    try {
      parameter = info.parameters.at(key);
    } catch (std::out_of_range & e) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("REALSENSE_hardware_interface"),
        "parameter : " << key << " does not exist.");
    }
  }
  void getParameter(
    const std::string & key, const hardware_interface::ComponentInfo & info, int & parameter) const
  {
    std::string param_string;
    getParameter(key, info, param_string);
    parameter = std::stoi(param_string);
  }
  void getParameter(
    const std::string & key, const hardware_interface::ComponentInfo & info, bool & parameter) const
  {
    parameter = false;
    std::string param_string;
    getParameter(key, info, param_string);
    if (param_string == "true" || param_string == "True") {
      parameter = true;
    }
  }
  template <typename T>
  T getHardwareParameter(const std::string key) const
  {
    T param;
    getHardwareParameter(key, param);
    return param;
  }
  void getHardwareParameter(const std::string & key, std::string & parameter) const
  {
    try {
      parameter = info_.hardware_parameters.at(key);
    } catch (std::out_of_range & e) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("REALSENSE_hardware_interface"),
        "hardware parameter : " << key << " does not exist.");
    }
  }
  void getHardwareParameter(const std::string & key, int & parameter) const
  {
    std::string param_string;
    getHardwareParameter(key, param_string);
    parameter = std::stoi(param_string);
  }
  void getHardwareParameter(const std::string & key, bool & parameter) const
  {
    parameter = false;
    std::string param_string;
    getHardwareParameter(key, param_string);
    if (param_string == "true" || param_string == "True") {
      parameter = true;
    }
  }
};
}  // namespace nmea_hardware_interface

#endif  // NMEA_HARDWARE_INTERFACE__GPS_HARDWARE_INTERFACE_HPP_