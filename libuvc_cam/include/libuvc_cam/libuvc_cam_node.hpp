// Copyright 2022 Whitley Software Services
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef LIBUVC_CAM__LIBUVC_CAM_NODE_HPP_
#define LIBUVC_CAM__LIBUVC_CAM_NODE_HPP_

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "sensor_msgs/msg/compressed_image.hpp"
#include <libuvc_cam/libuvc_cam.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include <string>
#include <memory>

namespace libuvc_cam
{

class UvcCameraNode : public rclcpp::Node
{
public:
  explicit UvcCameraNode(const rclcpp::NodeOptions & options);

private:
  void frame_callback(UvcFrame * frame);
  void split_and_publish(const UvcFrame * frame);
  void publish_compressed_image(const UvcFrame * frame);
  void imu_loop();

  std::string m_frame{};
  std::unique_ptr<UvcCamera> m_camera{};
  std::vector<image_transport::Publisher> m_publishers {};
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_compressed_image_publisher;
  rclcpp::Duration guest_host_time_offset;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_publisher;
  std::unique_ptr<serial::Serial> m_imu_serial;
  std::thread m_imu_thread;
};

}  // namespace libuvc_cam

#endif  // LIBUVC_CAM__LIBUVC_CAM_NODE_HPP_
