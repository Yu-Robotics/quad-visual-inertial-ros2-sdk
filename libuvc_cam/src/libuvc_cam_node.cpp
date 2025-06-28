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

#include <libuvc_cam/libuvc_cam_node.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>
#include "libuvc_calibration.h"

using libuvc_cam::StreamFormat;

namespace libuvc_cam
{

#define START_FLAG "\x34\x12"
#define END_FLAG "\x78\x56"

#define HEADER_SIZE  2
#define CRC_SIZE     4
#define SUFFIX_SIZE  2
#define PAYLOAD_SIZE 56
#define PACKET_SIZE  (HEADER_SIZE + PAYLOAD_SIZE + CRC_SIZE + SUFFIX_SIZE)
#define BUFFER_BATCH_SIZE 32

struct __attribute__((packed)) Packet {
  uint16_t header;
  double ax;
  double ay;
  double az;
  double rx;
  double ry;
  double rz;
  uint64_t timestamp;
  uint32_t hash;
  uint16_t suffix;
};
static_assert(sizeof(Packet) == PACKET_SIZE);

UvcCameraNode::UvcCameraNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("uvc_camera_node", options), guest_host_time_offset(0, 0)
{
  declare_parameter("calibration", "");
  declare_parameter("vendor_id", "");
  auto vendor_id_param = get_parameter("vendor_id");
  declare_parameter("product_id", "");
  declare_parameter("camera_topic_suffix", "");
  declare_parameter("camera_topic_prefix", "");
  declare_parameter("imu_topic_name", "");
  auto product_id_param = get_parameter("product_id");
  std::string serial_num = declare_parameter<std::string>("serial_num", "");
  std::string frame_fmt_string = declare_parameter<std::string>("frame_fmt", "");
  int requested_width = declare_parameter("image_width", 0);
  int requested_height = declare_parameter("image_height", 0);
  int requested_frame_rate = declare_parameter("frames_per_second", 0);
  m_frame = declare_parameter<std::string>("frame_id", "camera");
  std::string imu_device = declare_parameter<std::string>("imu_device", "");

  if (vendor_id_param.get_type() == rclcpp::PARAMETER_NOT_SET) {
    throw rclcpp::exceptions::InvalidParameterValueException{"vendor_id is missing."};
  } else if (product_id_param.get_type() == rclcpp::PARAMETER_NOT_SET) {
    throw rclcpp::exceptions::InvalidParameterValueException{"product_id is missing."};
  }

  auto calibration = uvc_read_calibration(std::stoul(vendor_id_param.as_string(), nullptr, 16), std::stoul(product_id_param.as_string(), nullptr, 16), "Calibration");
  set_parameter({"calibration", calibration});

  StreamFormat requested_fmt = StreamFormat::ANY;

  if (frame_fmt_string.empty() ||
    frame_fmt_string == "ANY")
  {
    // do nothing
  } else if (frame_fmt_string == "UNCOMPRESSED") {
    requested_fmt = StreamFormat::UNCOMPRESSED;
  } else if (frame_fmt_string == "MJPEG") {
    requested_fmt = StreamFormat::MJPEG;
  } else {
    throw rclcpp::exceptions::InvalidParameterValueException{
            "Invalid frame_fmt provided. Valid values are ANY, UNCOMPRESSED, or MJPEG"};
  }

  m_camera = std::make_unique<UvcCamera>(
    vendor_id_param.as_string(),
    product_id_param.as_string(),
    serial_num);


  // Get parameter values
  std::string camera_topic_suffix = get_parameter("camera_topic_suffix").as_string();
  std::string camera_topic_prefix = get_parameter("camera_topic_prefix").as_string();

  // Create publishers
  m_publishers = {
    image_transport::create_publisher(this, camera_topic_prefix + "0" + camera_topic_suffix),
    image_transport::create_publisher(this, camera_topic_prefix + "1" + camera_topic_suffix),
    image_transport::create_publisher(this, camera_topic_prefix + "2" + camera_topic_suffix),
    image_transport::create_publisher(this, camera_topic_prefix + "3" + camera_topic_suffix)
  };

  m_compressed_image_publisher =
    this->create_publisher<sensor_msgs::msg::CompressedImage>(
    camera_topic_prefix + "/compressed", rclcpp::QoS(100));

  if (imu_device.empty()) {
    std::string serial_id = vendor_id_param.as_string() + ":" + product_id_param.as_string();
    for (const auto& info : serial::list_ports()) {
      if (info.hardware_id.find(serial_id) != std::string::npos) {
        imu_device = info.port;
        RCLCPP_INFO(get_logger(), "Found matched imu serial port: %s", imu_device.c_str());
      }
    }
  }
  if (imu_device.empty()) {
    RCLCPP_ERROR(get_logger(), "No imu serial port found!");
  } else {
    m_imu_serial = std::make_unique<serial::Serial>(imu_device, 115200);
    auto timeout = serial::Timeout::simpleTimeout(BUFFER_BATCH_SIZE + 2); // 32+2ms timeout for readout
    m_imu_serial->setTimeout(timeout);
    if (!m_imu_serial->isOpen()) {
      RCLCPP_INFO(get_logger(), "Open imu serial port %s failed", imu_device.c_str());
      m_imu_serial = nullptr;
    } else {
      std::string imu_topic_name = get_parameter("imu_topic_name").as_string();
      rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(1000));
      qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);
      m_imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name, qos_profile);
      RCLCPP_INFO(get_logger(), "Open imu serial port %s success", imu_device.c_str());
    }
  }

  // Register callback
  m_camera->register_frame_callback(
    std::bind(&UvcCameraNode::frame_callback, this, std::placeholders::_1));

  if (requested_fmt == StreamFormat::ANY &&
    requested_width == 0 &&
    requested_height == 0 &&
    requested_frame_rate == 0)
  {
    // Use first-available stream
    RCLCPP_INFO(
      get_logger(), "No frame parameters specified. Using first available stream type.");

    m_camera->start_streaming();
    if (m_imu_serial) {
      m_imu_thread = std::thread(std::bind(&UvcCameraNode::imu_loop, this));
    }
  } else {
    // Try to find supported stream matching requested parameters
    RCLCPP_INFO(
      get_logger(), "Attempting to acquire stream with specified parameters.");

    if (m_camera->format_is_supported(
        requested_fmt,
        requested_width,
        requested_height,
        requested_frame_rate))
    {
      RCLCPP_INFO(get_logger(), "Requested stream parameters available! Connecting...");
      m_camera->start_streaming_with_format(
        requested_fmt,
        requested_width,
        requested_height,
        requested_frame_rate);
      if (m_imu_serial) {
        m_imu_thread = std::thread(std::bind(&UvcCameraNode::imu_loop, this));
      }
      RCLCPP_INFO(get_logger(), "Connected to camera.");
    } else {
      RCLCPP_FATAL(
        get_logger(), "Requested stream is not supported. "
        "See output below for formats supported by this camera.\n");
      m_camera->print_supported_formats();
      rclcpp::shutdown();
    }
  }
}

void UvcCameraNode::imu_loop()
{
  auto read_chars = [this](unsigned char* buffer, int size) {
    int offset = 0;
    while (offset < size) {
      offset += m_imu_serial->read(buffer + offset, size - offset);
    }
  };
  auto is_packet_correct = [](unsigned char* buffer) {
    // TODO: add crc check
    if (buffer[0] == START_FLAG[0] && buffer[1] == START_FLAG[1] && buffer[PACKET_SIZE-1] == END_FLAG[1] && buffer[PACKET_SIZE-2] == END_FLAG[0]) {
      return true;
    }
    return false;
  };
  auto find_packet = [is_packet_correct, read_chars, this](unsigned char* buffer) {
    read_chars(buffer, PACKET_SIZE * 2);
    for (int i = 0; i <= PACKET_SIZE; i++) {
      unsigned char* p = buffer + i;
      if (is_packet_correct(p)) {
        RCLCPP_INFO(get_logger(), "Find packet! Skip %d bytes\n", i);
        read_chars(buffer, i);
        break;
      }
    }
  };

  unsigned char buffer[PACKET_SIZE * 2];
  find_packet(buffer);
  while (m_imu_serial) {
    std::vector<std::chrono::time_point<std::chrono::steady_clock>> stable_queue;
    stable_queue.reserve(1024);
    for (int i = 0; i < 1000; i++) {
      read_chars(buffer, PACKET_SIZE);
      if (is_packet_correct(buffer)) {
        stable_queue.push_back(std::chrono::steady_clock::now());
      } else {
        RCLCPP_ERROR(get_logger(), "Read packet error found!\n");
	      find_packet(buffer);
        break;
      }
    }
    if (stable_queue.size() >= 1000) {
      auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(stable_queue[999] - stable_queue[0]).count();
      if (diff >= 1000 - BUFFER_BATCH_SIZE && diff <= 1000 + BUFFER_BATCH_SIZE) {
        RCLCPP_INFO(get_logger(), "Packets stabilized, 1000 packet duration = %ldms\n", diff);
        break;
      } else {
        RCLCPP_INFO(get_logger(), "Packets unstable, 1000 packet duration = %ldms\n", diff);
      }
    }
  }

  uint64_t last_timestamp = 0;
  uint64_t time_shift = 0;
  uint64_t realtime_shift = 0;
  std::vector<std::chrono::time_point<std::chrono::steady_clock>> stable_queue;
  stable_queue.reserve(1024);
  while (m_imu_serial) {
    read_chars(buffer, PACKET_SIZE);
    Packet* packet = (Packet*)buffer;
    if (is_packet_correct(buffer)) {
      // TODO: add gpio sync
      if (guest_host_time_offset.seconds() != 0) {
        rclcpp::Time timestamp = rclcpp::Time(packet->timestamp / 1000000000, packet->timestamp % 1000000000) + guest_host_time_offset;
        auto message = sensor_msgs::msg::Imu();
        message.header.frame_id = m_frame;
        message.header.stamp = timestamp;
        message.linear_acceleration.x = packet->ax;
        message.linear_acceleration.y = packet->ay;
        message.linear_acceleration.z = packet->az;
        message.angular_velocity.x = packet->rx;
        message.angular_velocity.y = packet->ry;
        message.angular_velocity.z = packet->rz;
        m_imu_publisher->publish(message);
      }
#if 0
      stable_queue.push_back(std::chrono::steady_clock::now());
      if (stable_queue.size() == 1000) {
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(stable_queue[999] - stable_queue[0]).count();
	      stable_queue.clear();
	      printf("Packets running, 1000 packet duration = %ldms\n", diff);
      }
#endif
    } else {
      RCLCPP_ERROR(get_logger(), "Read packet error found!\n");
      find_packet(buffer);
      continue;
    }
    last_timestamp = packet->timestamp;
  }
}

void UvcCameraNode::frame_callback(UvcFrame * frame)
{
  uvc_frame* converted_frame = nullptr;
  if (frame->frame_format == UvcFrameFormat::MJPEG)
  {
      converted_frame = uvc_allocate_frame(frame->width * frame->height);
      uvc_mjpeg2gray(frame->frame, converted_frame);
      publish_compressed_image(frame);
      UvcFrame new_frame{converted_frame};
      frame = &new_frame;
  }

  switch (frame->frame_format) {
    case UvcFrameFormat::GRAY8:
      split_and_publish(frame);
      break;
    default:
      break;
  }

  if (converted_frame)
  {
    uvc_free_frame(converted_frame);
  }
}

void UvcCameraNode::split_and_publish(const UvcFrame * frame)
{
  if (guest_host_time_offset.seconds() == 0) {
    rclcpp::Time now = rclcpp::Clock().now();
    guest_host_time_offset = now - rclcpp::Time(frame->frame->capture_time.tv_sec, frame->frame->capture_time.tv_usec * 1000);
    RCLCPP_INFO(get_logger(), "guest_host_time_offset: %f", guest_host_time_offset.seconds());
  }

  // re-calculate the capture time based on the offset
  rclcpp::Time ros_clock_capture_time = rclcpp::Time(frame->frame->capture_time.tv_sec, frame->frame->capture_time.tv_usec * 1000) + guest_host_time_offset;

  // Split the frame into 4 fisheye images
  // Publish each image
  auto width = frame->width / 4;
  for (int i = 0; i < 4; i++) {
    sensor_msgs::msg::Image img;
    img.header.frame_id = m_frame;
    img.header.stamp = ros_clock_capture_time;
    img.height = frame->height;
    img.width = width;
    img.step = width;
    img.encoding = sensor_msgs::image_encodings::MONO8;
    img.data.resize(img.height * img.width);

    for (unsigned int y = 0; y < img.height; y++) {
      for (unsigned int x = 0; x < img.width; x++) {
        img.data[y * img.width + x] = frame->data[y * frame->step + x + i * width];
      }
    }

    m_publishers[i].publish(img);
  }
}

void UvcCameraNode::publish_compressed_image(const UvcFrame * frame)
{
  if (guest_host_time_offset.seconds() == 0) {
    rclcpp::Time now = rclcpp::Clock().now();
    guest_host_time_offset = now - rclcpp::Time(frame->frame->capture_time.tv_sec, frame->frame->capture_time.tv_usec * 1000);
    RCLCPP_INFO(get_logger(), "guest_host_time_offset: %f", guest_host_time_offset.seconds());
  }

  sensor_msgs::msg::CompressedImage img;
  img.header.frame_id = m_frame;
  img.header.stamp = rclcpp::Time(frame->frame->capture_time.tv_sec, frame->frame->capture_time.tv_usec * 1000) + guest_host_time_offset;
  img.format = "jpeg";
  img.data.resize(frame->frame->data_bytes);
  uint8_t* raw_data = reinterpret_cast<uint8_t*>(frame->frame->data);
  std::copy(raw_data, raw_data + frame->frame->data_bytes, img.data.begin());

  m_compressed_image_publisher->publish(img);
}

}  // namespace libuvc_cam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(libuvc_cam::UvcCameraNode)
