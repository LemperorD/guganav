/**
 * @file camera_interface.hpp
 * @brief Vendor-agnostic abstract interface for industrial camera SDKs.
 *
 * Defines common data structures and a pure virtual interface that all
 * industrial camera SDK adapters (Hikrobot MVS, Daheng, Basler Pylon, ...)
 * must implement. The ROS2 node layer depends only on this interface,
 * making it SDK-agnostic.
 *
 * Design:
 *  - String-keyed parameters match how industrial camera SDKs work internally.
 *  - No ROS dependencies — pure C++17 standard library types.
 *  - grabImage() returns raw image data; pixel conversion is a separate step.
 */

#ifndef CAMERA_INTERFACE_HPP_
#define CAMERA_INTERFACE_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace camera_interface {

// =============================================================================
// Enums
// =============================================================================

/// Transport layer type used to connect to the camera.
enum class TransportType {
  GIGE = 0,       ///< GigE Vision (Ethernet)
  USB = 1,        ///< USB3 Vision
  CAMERA_LINK = 2,///< Camera Link
  UNKNOWN = 99    ///< Unknown / not set
};

// =============================================================================
// Data Structures
// =============================================================================

/// Device info returned by enumeration (SDK-agnostic).
struct DeviceInfo {
  std::string model_name;        ///< e.g. "MV-CA050-20UC"
  std::string serial_number;     ///< e.g. "DA1234567"
  std::string user_defined_name; ///< User-assigned name (may be empty)
  std::string manufacturer;      ///< e.g. "Hikrobot"
  TransportType transport;       ///< Connection type
};

/// Raw image data captured from the camera.
struct ImageData {
  std::vector<uint8_t> data;   ///< Raw pixel data
  unsigned int width{0};       ///< Image width in pixels
  unsigned int height{0};      ///< Image height in pixels
  unsigned int step{0};        ///< Bytes per row
  unsigned int pixel_format{0};///< SDK-specific pixel format enum value
  uint64_t timestamp_us{0};    ///< Monotonic capture timestamp (μs)
};

/// Range constraints for a float-type camera parameter.
struct FloatRange {
  float min;
  float max;
  float current;
};

/// Range constraints for an integer-type camera parameter.
struct IntRange {
  int64_t min;
  int64_t max;
  int64_t current;
};

// =============================================================================
// Abstract Interface
// =============================================================================

/**
 * @brief Abstract base class for industrial camera SDK adapters.
 *
 * Each concrete implementation wraps a specific vendor SDK behind this
 * uniform API. The ROS2 node layer (HikCameraNode, etc.) works exclusively
 * through this interface, enabling multi-vendor support with minimal
 * node-side code changes.
 *
 * Thread safety: implementations must ensure that grabImage() and
 * set*Param() can be called concurrently from different threads.
 */
class CameraInterface {
public:
  virtual ~CameraInterface() = default;

  // ---- Lifecycle ----

  /** Initialize the SDK and enumerate devices. Blocks until a device
   *  is found or the calling thread is interrupted. */
  virtual bool initialize() = 0;

  /** Open the camera device (must call initialize() first). */
  virtual bool open() = 0;

  /** Close the device handle and release SDK resources. */
  virtual void close() = 0;

  /** Check if the camera device is currently open. */
  virtual bool isOpen() const = 0;

  // ---- Streaming ----

  /** Start image acquisition. */
  virtual bool startGrabbing() = 0;

  /** Stop image acquisition. */
  virtual bool stopGrabbing() = 0;

  /** Check if grabbing is active. */
  virtual bool isGrabbing() const = 0;

  // ---- Image Acquisition ----

  /**
   * @brief Grab one frame from the camera (blocking with timeout).
   * @param timeout_ms  Timeout in milliseconds.
   * @param out_image   Filled with captured image data on success.
   * @return true if a frame was captured, false on timeout or error.
   */
  virtual bool grabImage(unsigned int timeout_ms, ImageData& out_image) = 0;

  // ---- Float Parameters ----

  /** Read a float parameter (e.g. "ExposureTime", "Gain"). */
  virtual bool getFloatParam(const std::string& name, float& value) = 0;

  /** Write a float parameter. */
  virtual bool setFloatParam(const std::string& name, float value) = 0;

  /** Get min / max / current values for a float parameter. */
  virtual bool getFloatParamRange(const std::string& name,
                                  FloatRange& range) = 0;

  // ---- Integer Parameters ----

  /** Read an integer parameter (e.g. "Width", "Height"). */
  virtual bool getIntParam(const std::string& name, int64_t& value) = 0;

  /** Write an integer parameter. */
  virtual bool setIntParam(const std::string& name, int64_t value) = 0;

  /** Get min / max / current values for an integer parameter. */
  virtual bool getIntParamRange(const std::string& name,
                                IntRange& range) = 0;

  // ---- Enum Parameters ----

  /** Read an enum parameter (e.g. "PixelFormat", "TriggerMode"). */
  virtual bool getEnumParam(const std::string& name, std::string& value) = 0;

  /** Write an enum parameter by string value. */
  virtual bool setEnumParam(const std::string& name,
                            const std::string& value) = 0;

  /** List all valid string values for an enum parameter. */
  virtual std::vector<std::string> getEnumParamOptions(
      const std::string& name) = 0;

  // ---- Boolean Parameters ----

  /** Read a boolean parameter. */
  virtual bool getBoolParam(const std::string& name, bool& value) = 0;

  /** Write a boolean parameter. */
  virtual bool setBoolParam(const std::string& name, bool value) = 0;

  // ---- Pixel Conversion ----

  /**
   * @brief Convert raw image data to a target pixel format.
   * @param src            Source image (from grabImage).
   * @param target_format  Target format string (e.g. "RGB8Packed", "BGR8").
   * @param dst            Pre-allocated destination buffer.
   * @param dst_size       Size of dst buffer in bytes.
   * @return true on success.
   */
  virtual bool convertPixelFormat(const ImageData& src,
                                  const std::string& target_format,
                                  uint8_t* dst,
                                  unsigned int dst_size) = 0;

  // ---- Device Info ----

  /** Return the camera model name (e.g. "MV-CA050-20UC"). */
  virtual std::string getDeviceModelName() const = 0;

  /** Return the camera serial number. */
  virtual std::string getDeviceSerialNumber() const = 0;

  /** Return the SDK / vendor name for diagnostics. */
  virtual std::string getSdkVendorName() const = 0;

  // ---- Image Info ----

  /** Get max supported image width. */
  virtual unsigned int getMaxWidth() const = 0;

  /** Get max supported image height. */
  virtual unsigned int getMaxHeight() const = 0;
};

}  // namespace camera_interface

#endif  // CAMERA_INTERFACE_HPP_
