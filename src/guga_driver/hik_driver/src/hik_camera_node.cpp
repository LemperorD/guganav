/**
 * @file hik_camera_node.cpp
 * @brief HikCameraNode implementation — ROS2 ↔ CameraInterface bridge.
 */

#include "hik_driver/hik_camera_node.hpp"
#include "hik_driver/hik_camera_main.hpp"

#include <chrono>
#include <iostream>

namespace hik_driver {

// ======================== Constructor ========================

HikCameraNode::HikCameraNode(const rclcpp::NodeOptions& options)
  : Node("hik_camera_node", options)
{
  std::cout << "\033[32m[HikCameraNode] Starting...\033[0m" << std::endl;

  // 1. Load connection parameters
  onConfigure();

  // 2. Create the concrete camera implementation
  auto hik_cam = std::make_shared<HikCameraMain>();
  hik_cam->setCameraType(camera_type_);
  hik_cam->setGigEParams(cam_ip_, pc_ip_);
  hik_cam->setUSBIndex(device_index_);
  camera_ = hik_cam;

  // 3. Initialise SDK, open device
  if (!camera_->initialize()) {
    RCLCPP_FATAL(this->get_logger(),
                 "\033[31mSDK initialisation failed\033[0m");
    rclcpp::shutdown();
    return;
  }
  if (!camera_->open()) {
    RCLCPP_FATAL(this->get_logger(),
                 "\033[31mFailed to open camera device\033[0m");
    rclcpp::shutdown();
    return;
  }

  // 4. Declare image parameters (needs open handle for range queries)
  declareImageParameters();

  // 5. Start grabbing
  if (!camera_->startGrabbing()) {
    RCLCPP_FATAL(this->get_logger(),
                 "\033[31mFailed to start grabbing\033[0m");
    rclcpp::shutdown();
    return;
  }

  // 6. Set up ROS publishers
  auto qos = use_sensor_data_qos_
                 ? rmw_qos_profile_sensor_data
                 : rmw_qos_profile_default;
  camera_pub_ = image_transport::create_camera_publisher(this, camera_topic_, qos);

  // 7. Camera info manager
  camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
  if (camera_info_manager_->validateURL(camera_info_url_)) {
    camera_info_manager_->loadCameraInfo(camera_info_url_);
    camera_info_msg_ = camera_info_manager_->getCameraInfo();
  } else {
    RCLCPP_WARN(this->get_logger(),
                "\033[33mInvalid camera info URL: %s\033[0m",
                camera_info_url_.c_str());
  }

  // 8. Register dynamic parameter callback
  params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::dynamicParametersCallback, this,
                std::placeholders::_1));

  // 9. Launch capture thread
  running_.store(true);
  capture_thread_ = std::thread(&HikCameraNode::captureLoop, this);

  RCLCPP_INFO(this->get_logger(),
              "\033[32m[HikCameraNode] Ready — publishing to %s\033[0m",
              camera_topic_.c_str());
}

HikCameraNode::~HikCameraNode() {
  RCLCPP_INFO(this->get_logger(), "[HikCameraNode] Shutting down...");

  running_.store(false);
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }

  // Shut down camera cleanly
  if (camera_) {
    camera_->stopGrabbing();
    camera_->close();
  }
  camera_.reset();

  std::cout << "\033[32m[HikCameraNode] Shutdown complete\033[0m" << std::endl;
}

// ======================== Configuration ========================

void HikCameraNode::onConfigure() {
  camera_type_ = this->declare_parameter("camera_type", 0);
  cam_ip_      = this->declare_parameter("cam_ip", "192.168.1.100");
  pc_ip_       = this->declare_parameter("pc_ip", "192.168.10.30");
  device_index_ = this->declare_parameter("device_index", 0);

  camera_name_      = this->declare_parameter("camera_name", "camera");
  frame_id_         = this->declare_parameter("frame_id",
                                              camera_name_ + "_optical_frame");
  camera_topic_     = this->declare_parameter("camera_topic",
                                              camera_name_ + "/image");
  camera_info_url_  = this->declare_parameter(
      "camera_info_url", "package://hik_driver/config/camera_info.yaml");
  use_sensor_data_qos_ = this->declare_parameter("use_sensor_data_qos", true);

  RCLCPP_INFO(this->get_logger(),
              "Configured: camera_type=%d cam_ip=%s pc_ip=%s device_index=%d",
              camera_type_, cam_ip_.c_str(), pc_ip_.c_str(), device_index_);
}

void HikCameraNode::declareImageParameters() {
  rcl_interfaces::msg::ParameterDescriptor desc;

  // ---- Acquisition Frame Rate ----
  camera_interface::FloatRange fr;
  desc.description = "Acquisition frame rate in Hz";
  if (camera_->getFloatParamRange("AcquisitionFrameRate", fr)) {
    desc.floating_point_range.resize(1);
    desc.floating_point_range[0].from_value = static_cast<double>(fr.min);
    desc.floating_point_range[0].to_value   = static_cast<double>(fr.max);
    desc.floating_point_range[0].step = 1.0;
  }
  double acq_fps = this->declare_parameter("acquisition_frame_rate", 165.0, desc);
  camera_->setBoolParam("AcquisitionFrameRateEnable", true);
  camera_->setFloatParam("AcquisitionFrameRate", static_cast<float>(acq_fps));
  RCLCPP_INFO(this->get_logger(), "Acquisition frame rate: %.1f Hz", acq_fps);

  // ---- Exposure Time ----
  desc = rcl_interfaces::msg::ParameterDescriptor{};
  desc.description = "Exposure time in microseconds";
  if (camera_->getFloatParamRange("ExposureTime", fr)) {
    desc.floating_point_range.resize(1);
    desc.floating_point_range[0].from_value = static_cast<double>(fr.min);
    desc.floating_point_range[0].to_value   = static_cast<double>(fr.max);
    desc.floating_point_range[0].step = 1.0;
  }
  double expo = this->declare_parameter("exposure_time", 5000.0, desc);
  camera_->setFloatParam("ExposureTime", static_cast<float>(expo));
  RCLCPP_INFO(this->get_logger(), "Exposure time: %.0f us", expo);

  // ---- Gain ----
  desc = rcl_interfaces::msg::ParameterDescriptor{};
  desc.description = "Sensor gain";
  if (camera_->getFloatParamRange("Gain", fr)) {
    desc.floating_point_range.resize(1);
    desc.floating_point_range[0].from_value = static_cast<double>(fr.min);
    desc.floating_point_range[0].to_value   = static_cast<double>(fr.max);
    desc.floating_point_range[0].step = 0.1;
    double gain_val = this->declare_parameter(
        "gain", static_cast<double>(fr.current), desc);
    camera_->setFloatParam("Gain", static_cast<float>(gain_val));
    RCLCPP_INFO(this->get_logger(), "Gain: %.1f", gain_val);
  } else {
    double gain_val = this->declare_parameter("gain", 12.0, desc);
    camera_->setFloatParam("Gain", static_cast<float>(gain_val));
    RCLCPP_INFO(this->get_logger(), "Gain (no range): %.1f", gain_val);
  }

  // ---- Pixel Format ----
  desc = rcl_interfaces::msg::ParameterDescriptor{};
  desc.description = "Pixel format (e.g. RGB8Packed, BayerRG8)";
  std::string pixel_fmt = this->declare_parameter("pixel_format",
                                                  "RGB8Packed", desc);
  if (camera_->setEnumParam("PixelFormat", pixel_fmt)) {
    RCLCPP_INFO(this->get_logger(), "Pixel format set to: %s",
                pixel_fmt.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(),
                "\033[33mFailed to set pixel format to %s\033[0m",
                pixel_fmt.c_str());
  }
}

// ======================== Capture Loop ========================

void HikCameraNode::captureLoop() {
  sensor_msgs::msg::Image image_msg;
  image_msg.header.frame_id = frame_id_;
  image_msg.encoding = "rgb8";

  RCLCPP_INFO(this->get_logger(), "Capture loop started");

  auto last_log_time = std::chrono::steady_clock::now();

  while (running_.load() && rclcpp::ok()) {
    camera_interface::ImageData img;
    if (camera_->grabImage(1000, img)) {
      // Reset failure counter on success
      fail_count_ = 0;

      // Fill ROS image message
      image_msg.header.stamp = this->now();
      image_msg.width  = img.width;
      image_msg.height = img.height;
      image_msg.step   = img.step;
      image_msg.data   = std::move(img.data);

      // Sync camera_info header
      camera_info_msg_.header = image_msg.header;

      camera_pub_.publish(image_msg, camera_info_msg_);

      // Periodic frame-rate logging
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(
              now - last_log_time).count() >= 3) {
        float fps = 0.f;
        if (camera_->getFloatParam("ResultingFrameRate", fps)) {
          RCLCPP_DEBUG(this->get_logger(),
                       "ResultingFrameRate: %.1f Hz", fps);
        }
        last_log_time = now;
      }
    } else {
      fail_count_++;
      RCLCPP_WARN(this->get_logger(),
                  "\033[33mGrab failed! fail count: %d/%d\033[0m",
                  fail_count_, MAX_FAIL_COUNT);

      // Try recovery: stop+start grabbing
      camera_->stopGrabbing();
      camera_->startGrabbing();
    }

    if (fail_count_ > MAX_FAIL_COUNT) {
      RCLCPP_FATAL(this->get_logger(),
                   "\033[31mCamera failed after %d attempts! Shutting down\033[0m",
                   MAX_FAIL_COUNT);
      rclcpp::shutdown();
      return;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Capture loop exited");
}

// ======================== Dynamic Parameters ========================

rcl_interfaces::msg::SetParametersResult
HikCameraNode::dynamicParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& param : parameters) {
    const auto& name = param.get_name();
    const auto& type = param.get_type();

    if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (name == "exposure_time") {
        if (!camera_->setFloatParam("ExposureTime",
                                    static_cast<float>(param.as_double()))) {
          result.successful = false;
          result.reason = "Failed to set ExposureTime";
          continue;
        }
      } else if (name == "gain") {
        if (!camera_->setFloatParam("Gain",
                                    static_cast<float>(param.as_double()))) {
          result.successful = false;
          result.reason = "Failed to set Gain";
          continue;
        }
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + name;
        continue;
      }
    } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
      if (name == "exposure_time") {
        if (!camera_->setFloatParam("ExposureTime",
                                    static_cast<float>(param.as_int()))) {
          result.successful = false;
          result.reason = "Failed to set ExposureTime";
          continue;
        }
      } else if (name == "gain") {
        if (!camera_->setFloatParam("Gain",
                                    static_cast<float>(param.as_int()))) {
          result.successful = false;
          result.reason = "Failed to set Gain";
          continue;
        }
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + name;
        continue;
      }
    } else {
      result.successful = false;
      result.reason = "Unsupported parameter type for: " + name;
      continue;
    }

    RCLCPP_INFO(this->get_logger(),
                "Parameter updated: %s", name.c_str());
  }

  return result;
}

}  // namespace hik_driver

// Register as a ROS2 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_driver::HikCameraNode)
