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

#include <memory>
#include <nmea_hardware_interface/gps_hardware_interface.hpp>
#include <vector>

namespace nmea_hardware_interface
{
hardware_interface::return_type GPSHardwareInterface::configure(
  const hardware_interface::HardwareInfo & info)
{
  declare_parameter("device_file", "/dev/ttyACM0");
  get_parameter("device_file", device_file_);
  declare_parameter("baud_rate", 9600);
  get_parameter("baud_rate", baud_rate_);
  declare_parameter("frame_id", "gps");
  get_parameter("frame_id", frame_id_);

  

  connectSerialPort();
 
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }
  if (info.joints.size() != 1) {
    throw std::runtime_error("joint size should be 1");
  }
  joint_ = info.joints[0].name;
  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}



std::vector<hardware_interface::StateInterface> GPSHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  return state_interfaces;
}

hardware_interface::return_type GPSHardwareInterface::start()
{
  status_ = hardware_interface::status::STARTED;
  geopoint_memory_ptr_ = std::make_shared<Poco::SharedMemory>(
      geopoint_key_, sizeof("gps_geopoint"), Poco::SharedMemory::AccessMode::AM_WRITE);

  togeopoint_thread_ = boost::thread(boost::bind(&GPSHardwareInterface::nmeaSentenceCallback, this));
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GPSHardwareInterface::stop()
{
  status_ = hardware_interface::status::STOPPED;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GPSHardwareInterface::read()
{
  memcpy(geopoint_memory_ptr_->begin(), geopoint_, sizeof("gps_geopoint"));
  return hardware_interface::return_type::OK;
}

void GPSHardwareInterface::connectSerialPort()
{
  try {
    port_ptr_ = std::make_shared<boost::asio::serial_port>(io_, device_file_);
    port_ptr_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    port_ptr_->set_option(boost::asio::serial_port_base::character_size(8));
    port_ptr_->set_option(boost::asio::serial_port_base::flow_control(
      boost::asio::serial_port_base ::flow_control::none));
    port_ptr_->set_option(
      boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    port_ptr_->set_option(
      boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    io_thread_ = boost::thread(boost::bind(&GPSHardwareInterface::readSentence, this));
    connected_ = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), e.what());
    connected_ = false;
  }
}

void GPSHardwareInterface::nmeaSentenceCallback(const nmea_msgs::msg::Sentence::SharedPtr msg)
{
  nmea_msgs::msg::Sentence sentence = *msg;
  if (isGprmcSentence(sentence)) {
    geographic_msgs::msg::GeoPoint geopoint;
    boost::optional<std::vector<std::string>> data = splitSentence(sentence);
    if (data) {
      std::string lat_str = data.get()[3];
      std::string north_or_south_str = data.get()[4];
      double latitude = std::stod(lat_str.substr(0, 2)) + std::stod(lat_str.substr(2)) / 60.0;
      assert(north_or_south_str == "N" || north_or_south_str == "S");
      if (north_or_south_str == "S") {
        latitude = latitude * -1;
      }
      std::string lon_str = data.get()[5];
      std::string east_or_west_str = data.get()[6];
      double longitude = std::stod(lon_str.substr(0, 3)) + std::stod(lon_str.substr(3)) / 60.0;
      assert(east_or_west_str == "E" || east_or_west_str == "W");
      if (east_or_west_str == "W") {
        longitude = longitude * -1;
      }
      geopoint.latitude = latitude;
      geopoint.longitude = longitude;
      geopoint.altitude = 0.0;
      geopoint_ = geopoint;
    }
  }
}

std::vector<std::string> GPSHardwareInterface::split(const std::string & s, char delim)
{
  std::vector<std::string> elems;
  std::stringstream ss(s);
  std::string item;
  while (getline(ss, item, delim)) {
    if (!item.empty()) {
      elems.push_back(item);
    }
  }
  return elems;
}

std::vector<std::string> GPSHardwareInterface::splitChecksum(std::string str)
{
  return split(str, '*');
}

boost::optional<std::vector<std::string>> GPSHardwareInterface::splitSentence(
  nmea_msgs::msg::Sentence sentence)
{
  std::vector<std::string> data = splitChecksum(sentence.sentence);
  if (data.size() != 2) {
    return boost::none;
  }
  if (calculateChecksum(data[0]) == data[1]) {
    return split(data[0], ',');
  }
  return boost::none;
}

bool GPSHardwareInterface::isGprmcSentence(nmea_msgs::msg::Sentence sentence)
{
  std::string type = sentence.sentence.substr(0, 6);
  if (type == "$GPRMC") {
    return true;
  }
  return false;
}

bool GPSHardwareInterface::isGphdtSentence(nmea_msgs::msg::Sentence sentence)
{
  std::string type = sentence.sentence.substr(0, 6);
  if (type == "$GPHDT") {
    return true;
  }
  return false;
}

std::string GPSHardwareInterface::calculateChecksum(std::string sentence)
{
  uint8_t checksum;
  for (unsigned int i = 1; i < sentence.size(); i++) {
    int16_t c = sentence[i];
    checksum ^= c;
  }
  uint8_t rest = checksum % 16;
  uint8_t quotient = (checksum - rest) / 16;
  std::string ret = getHexString(quotient) + getHexString(rest);
  return ret;
}

std::string GPSHardwareInterface::getHexString(uint8_t value)
{
  assert(value <= 16);
  std::string ret;
  if (value == 10) {
    ret = "A";
  } else if (value == 11) {
    ret = "B";
  } else if (value == 12) {
    ret = "C";
  } else if (value == 13) {
    ret = "D";
  } else if (value == 14) {
    ret = "E";
  } else if (value == 15) {
    ret = "F";
  } else {
    ret = std::to_string(value);
  }
  return ret;
}

}  // namespace nmea_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  nmea_hardware_interface::GPSHardwareInterface, hardware_interface::SensorInterface)