/**
 * @file hik_camera_node.hpp
 * @brief ROS2 node layer for the Hikrobot camera driver.
 *
 * Holds a shared_ptr<CameraInterface> and bridges it to ROS topics:
 *   - image_raw + camera_info via image_transport::CameraPublisher
 *   - Dynamic ROS parameters (exposure_time, gain, ...) → camera set*Param
 *
 * Follows the usbjs_driver / serial_driver Main/Node split pattern:
 * all SDK-specific logic is in HikCameraMain; this class only speaks ROS.
 */

#ifndef HIK_CAMERA_NODE_HPP_
#define HIK_CAMERA_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "camera_interface/camera_interface.hpp"

namespace hik_driver {

class HikCameraNode : public rclcpp::Node {
public:
  explicit HikCameraNode(const rclcpp::NodeOptions& options =
                         rclcpp::NodeOptions());
  ~HikCameraNode() override;

private:
  // ---- Setup ----

  /** Load connection parameters from the ROS parameter server. */
  void onConfigure();

  /** Declare camera-image parameters (needs an open camera handle). */
  void declareImageParameters();

  // ---- Capture thread ----

  /** Main capture loop: grab frame → convert → publish. */
  void captureLoop();

  // ---- Dynamic parameters ----

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);

  // ---- Camera backend ----
  std::shared_ptr<camera_interface::CameraInterface> camera_;

  // ---- Parameters (connection) ----
  int camera_type_{0};   // 0 = GIGE, 1 = USB
  std::string cam_ip_{"192.168.1.100"};
  std::string pc_ip_{"192.168.10.30"};
  int device_index_{0};

  // ---- Parameters (ROS) ----
  std::string camera_name_{"camera"};
  std::string frame_id_{"camera_optical_frame"};
  std::string camera_topic_{"camera/image"};
  std::string camera_info_url_{"package://hik_driver/config/camera_info.yaml"};
  bool use_sensor_data_qos_{true};

  // ---- Publishing ----
  image_transport::CameraPublisher camera_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  // ---- Dynamic parameter handle ----
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      params_callback_handle_;

  // ---- Capture thread ----
  std::thread capture_thread_;
  std::atomic<bool> running_{false};
  int fail_count_{0};
  static constexpr int MAX_FAIL_COUNT = 10;
};

}  // namespace hik_driver

#endif  // HIK_CAMERA_NODE_HPP_
