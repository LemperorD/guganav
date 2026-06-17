/**
 * @file hik_camera_main.hpp
 * @brief Hikrobot MVS SDK hardware abstraction — implements CameraInterface.
 *
 * Encapsulates all MVS SDK calls (MV_CC_*) behind the camera_interface API.
 * No ROS dependencies — pure C++17 with std::cerr/cout logging and ANSI
 * colour escapes (matching the serial_driver convention).
 */

#ifndef HIK_CAMERA_MAIN_HPP_
#define HIK_CAMERA_MAIN_HPP_

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include "MvCameraControl.h"
#include "camera_interface/camera_interface.hpp"

namespace hik_driver {

/// Hikrobot MVS SDK adapter implementing the vendor-agnostic CameraInterface.
///
/// Thread safety: a std::mutex serialises all access through the camera
/// handle, so grabImage() in the capture thread and setFloatParam() from
/// the ROS parameter callback thread do not race.
class HikCameraMain : public camera_interface::CameraInterface {
public:
  HikCameraMain();
  ~HikCameraMain() override;

  // ---- Configuration (call BEFORE open()) ----

  /** Set GigE connection parameters (camera & PC IP addresses). */
  void setGigEParams(const std::string& cam_ip, const std::string& pc_ip);

  /** Set USB device index (0-based). */
  void setUSBIndex(int device_index);

  /** Set camera type (GIGE or USB). */
  void setCameraType(int camera_type);

  /** Set the device info for direct connection (bypasses enumeration). */
  bool setDeviceInfo(const MV_CC_DEVICE_INFO& device_info);

  // ---- CameraInterface overrides ----

  bool initialize() override;
  bool open() override;
  void close() override;
  bool isOpen() const override;

  bool startGrabbing() override;
  bool stopGrabbing() override;
  bool isGrabbing() const override;

  bool grabImage(unsigned int timeout_ms,
                 camera_interface::ImageData& out_image) override;

  // Float params
  bool getFloatParam(const std::string& name, float& value) override;
  bool setFloatParam(const std::string& name, float value) override;
  bool getFloatParamRange(const std::string& name,
                          camera_interface::FloatRange& range) override;

  // Int params
  bool getIntParam(const std::string& name, int64_t& value) override;
  bool setIntParam(const std::string& name, int64_t value) override;
  bool getIntParamRange(const std::string& name,
                        camera_interface::IntRange& range) override;

  // Enum params
  bool getEnumParam(const std::string& name, std::string& value) override;
  bool setEnumParam(const std::string& name,
                    const std::string& value) override;
  std::vector<std::string> getEnumParamOptions(
      const std::string& name) override;

  // Bool params
  bool getBoolParam(const std::string& name, bool& value) override;
  bool setBoolParam(const std::string& name, bool value) override;

  // Pixel conversion
  bool convertPixelFormat(const camera_interface::ImageData& src,
                          const std::string& target_format,
                          uint8_t* dst,
                          unsigned int dst_size) override;

  // Device info
  std::string getDeviceModelName() const override;
  std::string getDeviceSerialNumber() const override;
  std::string getSdkVendorName() const override;

  // Image info
  unsigned int getMaxWidth() const override;
  unsigned int getMaxHeight() const override;

  // ---- Static enumeration ----

  /** Enumerate all connected Hikrobot devices. */
  static std::vector<camera_interface::DeviceInfo> enumerateDevices();

private:
  // ---- Internal helpers ----

  /** Parse dotted-quad IPv4 string to unsigned int. */
  static void parseIp(const std::string& ip, unsigned int& parsed_ip);

  /** Connect to a GigE camera using the stored IP settings. */
  bool connectGigE();

  /** Connect to a USB camera using the stored device index. */
  bool connectUSB();

  /** Convert an MVS pixel type enum to a string key. */
  static unsigned int pixelTypeFromString(const std::string& fmt);

  // ---- Camera handle ----
  void* camera_handle_ = nullptr;
  mutable std::mutex handle_mutex_;

  // ---- Configuration ----
  int camera_type_ = 0;  // 0 = GIGE, 1 = USB
  std::string cam_ip_{"192.168.1.100"};
  std::string pc_ip_{"192.168.10.30"};
  int device_index_ = 0;

  // ---- State ----
  std::atomic<bool> is_open_{false};
  std::atomic<bool> is_grabbing_{false};

  // ---- Image info (cached from camera on open) ----
  MV_IMAGE_BASIC_INFO img_info_{};

  // ---- Pixel conversion workspace ----
  MV_CC_PIXEL_CONVERT_PARAM convert_param_{};
};

}  // namespace hik_driver

#endif  // HIK_CAMERA_MAIN_HPP_
