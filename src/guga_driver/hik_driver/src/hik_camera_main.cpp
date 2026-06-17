/**
 * @file hik_camera_main.cpp
 * @brief HikCameraMain implementation — all MVS SDK calls live here.
 */

#include "hik_driver/hik_camera_main.hpp"

#include <cstring>
#include <iostream>
#include <thread>

// (MvCameraControl.h defines PixelType_Gvsp_RGB8_Packed etc. — no manual defs needed)

namespace hik_driver {

// ======================== ANSI terminal colours ========================

namespace termcolor {
static constexpr const char* reset  = "\033[0m";
static constexpr const char* red    = "\033[31m";
static constexpr const char* yellow = "\033[33m";
static constexpr const char* green  = "\033[32m";
}

// ======================== IP parsing ========================

void HikCameraMain::parseIp(const std::string& ip, unsigned int& parsed_ip) {
  int parts[4] = {0, 0, 0, 0};
  std::sscanf(ip.c_str(), "%d.%d.%d.%d",
              &parts[0], &parts[1], &parts[2], &parts[3]);
  parsed_ip = (static_cast<unsigned int>(parts[0]) << 24)
            | (static_cast<unsigned int>(parts[1]) << 16)
            | (static_cast<unsigned int>(parts[2]) << 8)
            | (static_cast<unsigned int>(parts[3]));
}

// ======================== Pixel format helper ========================

unsigned int HikCameraMain::pixelTypeFromString(const std::string& fmt) {
  if (fmt == "RGB8Packed" || fmt == "RGB8") {
    return PixelType_Gvsp_RGB8_Packed;
  }
  // For other formats we rely on the SDK's string->enum conversion
  // inside setEnumParam; here we just provide a fallback.
  return PixelType_Gvsp_RGB8_Packed;
}

// ======================== Constructor / Destructor ========================

HikCameraMain::HikCameraMain() {
  std::cout << termcolor::green
            << "[HikCameraMain] Constructed (SDK not yet initialised)"
            << termcolor::reset << std::endl;
}

HikCameraMain::~HikCameraMain() {
  close();
  std::cout << termcolor::green
            << "[HikCameraMain] Destroyed"
            << termcolor::reset << std::endl;
}

// ======================== Configuration ========================

void HikCameraMain::setGigEParams(const std::string& cam_ip,
                                  const std::string& pc_ip) {
  cam_ip_ = cam_ip;
  pc_ip_  = pc_ip;
  std::cout << termcolor::green
            << "[HikCameraMain] GigE params — cam: " << cam_ip_
            << "  pc: " << pc_ip_
            << termcolor::reset << std::endl;
}

void HikCameraMain::setUSBIndex(int device_index) {
  device_index_ = device_index;
  std::cout << termcolor::green
            << "[HikCameraMain] USB device index: " << device_index_
            << termcolor::reset << std::endl;
}

void HikCameraMain::setCameraType(int camera_type) {
  camera_type_ = camera_type;
}

bool HikCameraMain::setDeviceInfo(const MV_CC_DEVICE_INFO& device_info) {
  std::lock_guard<std::mutex> lock(handle_mutex_);

  int n_ret = MV_CC_CreateHandle(&camera_handle_, &device_info);
  if (n_ret != MV_OK) {
    std::cerr << termcolor::red
              << "[HikCameraMain] CreateHandle from DeviceInfo fail! nRet[0x"
              << std::hex << n_ret << "]"
              << termcolor::reset << std::endl;
    return false;
  }
  return true;
}

// ======================== Lifecycle ========================

bool HikCameraMain::initialize() {
  std::lock_guard<std::mutex> lock(handle_mutex_);

  int n_ret = MV_CC_Initialize();
  if (n_ret != MV_OK) {
    std::cerr << termcolor::red
              << "[HikCameraMain] MV_CC_Initialize fail! nRet[0x"
              << std::hex << n_ret << "]"
              << termcolor::reset << std::endl;
    return false;
  }

  // Enumerate devices — block until at least one is found
  MV_CC_DEVICE_INFO_LIST device_list;
  std::memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  while (true) {
    n_ret = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &device_list);
    if (n_ret != MV_OK) {
      std::cout << termcolor::yellow
                << "[HikCameraMain] EnumDevices fail, retrying..."
                << termcolor::reset << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    if (device_list.nDeviceNum == 0) {
      std::cout << termcolor::yellow
                << "[HikCameraMain] No camera found, retrying..."
                << termcolor::reset << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    std::cout << termcolor::green
              << "[HikCameraMain] Found " << device_list.nDeviceNum
              << " camera(s)"
              << termcolor::reset << std::endl;
    break;
  }

  return true;
}

bool HikCameraMain::open() {
  std::lock_guard<std::mutex> lock(handle_mutex_);

  bool ok = false;

  if (camera_type_ == 0) {   // GIGE_CAMERA
    ok = connectGigE();
  } else {                   // USB_CAMERA
    ok = connectUSB();
  }

  if (!ok) {
    return false;
  }

  // Cache image info
  int n_ret = MV_CC_GetImageInfo(camera_handle_, &img_info_);
  if (n_ret != MV_OK) {
    std::cerr << termcolor::red
              << "[HikCameraMain] GetImageInfo fail! nRet[0x"
              << std::hex << n_ret << "]"
              << termcolor::reset << std::endl;
    return false;
  }

  // Pre-configure pixel conversion workspace
  convert_param_.nWidth      = img_info_.nWidthValue;
  convert_param_.nHeight     = img_info_.nHeightValue;
  convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

  is_open_.store(true);
  std::cout << termcolor::green
            << "[HikCameraMain] Camera opened — "
            << img_info_.nWidthValue << "x" << img_info_.nHeightValue
            << " max"
            << termcolor::reset << std::endl;
  return true;
}

void HikCameraMain::close() {
  std::lock_guard<std::mutex> lock(handle_mutex_);

  if (is_grabbing_.load()) {
    MV_CC_StopGrabbing(camera_handle_);
    is_grabbing_.store(false);
  }

  if (camera_handle_) {
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(&camera_handle_);
    camera_handle_ = nullptr;
  }

  MV_CC_Finalize();
  is_open_.store(false);

  std::cout << termcolor::green
            << "[HikCameraMain] Camera closed & SDK finalised"
            << termcolor::reset << std::endl;
}

bool HikCameraMain::isOpen() const {
  return is_open_.load();
}

// ======================== Streaming ========================

bool HikCameraMain::startGrabbing() {
  std::lock_guard<std::mutex> lock(handle_mutex_);

  int n_ret = MV_CC_StartGrabbing(camera_handle_);
  if (n_ret != MV_OK) {
    std::cerr << termcolor::red
              << "[HikCameraMain] StartGrabbing fail! nRet[0x"
              << std::hex << n_ret << "]"
              << termcolor::reset << std::endl;
    return false;
  }
  is_grabbing_.store(true);
  return true;
}

bool HikCameraMain::stopGrabbing() {
  std::lock_guard<std::mutex> lock(handle_mutex_);

  int n_ret = MV_CC_StopGrabbing(camera_handle_);
  if (n_ret != MV_OK) {
    std::cerr << termcolor::red
              << "[HikCameraMain] StopGrabbing fail! nRet[0x"
              << std::hex << n_ret << "]"
              << termcolor::reset << std::endl;
    return false;
  }
  is_grabbing_.store(false);
  return true;
}

bool HikCameraMain::isGrabbing() const {
  return is_grabbing_.load();
}

// ======================== Image Acquisition ========================

bool HikCameraMain::grabImage(unsigned int timeout_ms,
                              camera_interface::ImageData& out_image) {
  std::lock_guard<std::mutex> lock(handle_mutex_);

  MV_FRAME_OUT out_frame;
  std::memset(&out_frame, 0, sizeof(MV_FRAME_OUT));

  int n_ret = MV_CC_GetImageBuffer(camera_handle_, &out_frame,
                                   static_cast<int>(timeout_ms));
  if (n_ret != MV_OK) {
    return false;
  }

  // Allocate / resize destination
  out_image.width  = out_frame.stFrameInfo.nWidth;
  out_image.height = out_frame.stFrameInfo.nHeight;
  out_image.step   = out_frame.stFrameInfo.nWidth * 3;  // RGB8 = 3 bytes/px

  const unsigned int required_size = out_image.width * out_image.height * 3;
  out_image.data.resize(required_size);

  // Update convert param with this frame's metadata
  convert_param_.pSrcData     = out_frame.pBufAddr;
  convert_param_.nSrcDataLen  = out_frame.stFrameInfo.nFrameLen;
  convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
  convert_param_.pDstBuffer   = out_image.data.data();
  convert_param_.nDstBufferSize = required_size;

  n_ret = MV_CC_ConvertPixelType(camera_handle_, &convert_param_);
  MV_CC_FreeImageBuffer(camera_handle_, &out_frame);

  if (n_ret != MV_OK) {
    std::cerr << termcolor::yellow
              << "[HikCameraMain] ConvertPixelType fail! nRet[0x"
              << std::hex << n_ret << "]"
              << termcolor::reset << std::endl;
    return false;
  }

  out_image.pixel_format = PixelType_Gvsp_RGB8_Packed;

  return true;
}

// ======================== Float Parameters ========================

bool HikCameraMain::getFloatParam(const std::string& name, float& value) {
  std::lock_guard<std::mutex> lock(handle_mutex_);
  MVCC_FLOATVALUE fv;
  std::memset(&fv, 0, sizeof(fv));
  int n_ret = MV_CC_GetFloatValue(camera_handle_, name.c_str(), &fv);
  if (n_ret != MV_OK) return false;
  value = fv.fCurValue;
  return true;
}

bool HikCameraMain::setFloatParam(const std::string& name, float value) {
  std::lock_guard<std::mutex> lock(handle_mutex_);
  int n_ret = MV_CC_SetFloatValue(camera_handle_, name.c_str(), value);
  return (n_ret == MV_OK);
}

bool HikCameraMain::getFloatParamRange(const std::string& name,
                                       camera_interface::FloatRange& range) {
  std::lock_guard<std::mutex> lock(handle_mutex_);
  MVCC_FLOATVALUE fv;
  std::memset(&fv, 0, sizeof(fv));
  int n_ret = MV_CC_GetFloatValue(camera_handle_, name.c_str(), &fv);
  if (n_ret != MV_OK) return false;
  range.min     = fv.fMin;
  range.max     = fv.fMax;
  range.current = fv.fCurValue;
  return true;
}

// ======================== Integer Parameters ========================

bool HikCameraMain::getIntParam(const std::string& name, int64_t& value) {
  std::lock_guard<std::mutex> lock(handle_mutex_);
  MVCC_INTVALUE iv;
  std::memset(&iv, 0, sizeof(iv));
  int n_ret = MV_CC_GetIntValue(camera_handle_, name.c_str(), &iv);
  if (n_ret != MV_OK) return false;
  value = static_cast<int64_t>(iv.nCurValue);
  return true;
}

bool HikCameraMain::setIntParam(const std::string& name, int64_t value) {
  std::lock_guard<std::mutex> lock(handle_mutex_);
  int n_ret = MV_CC_SetIntValue(camera_handle_, name.c_str(),
                                static_cast<unsigned int>(value));
  return (n_ret == MV_OK);
}

bool HikCameraMain::getIntParamRange(const std::string& name,
                                     camera_interface::IntRange& range) {
  std::lock_guard<std::mutex> lock(handle_mutex_);
  MVCC_INTVALUE iv;
  std::memset(&iv, 0, sizeof(iv));
  int n_ret = MV_CC_GetIntValue(camera_handle_, name.c_str(), &iv);
  if (n_ret != MV_OK) return false;
  range.min     = static_cast<int64_t>(iv.nMin);
  range.max     = static_cast<int64_t>(iv.nMax);
  range.current = static_cast<int64_t>(iv.nCurValue);
  return true;
}

// ======================== Enum Parameters ========================

bool HikCameraMain::getEnumParam(const std::string& name, std::string& value) {
  // MVS SDK provides GetEnumValue but it returns an int; the string version
  // is less common for reads.  We store the last-set enum name in a simple way
  // by reading the int and doing a best-effort mapping.
  //
  // For most use-cases the caller sets the enum via setEnumParam and does not
  // need to read it back, so this is a minimal best-effort implementation.
  std::lock_guard<std::mutex> lock(handle_mutex_);
  MVCC_ENUMVALUE ev;
  std::memset(&ev, 0, sizeof(ev));
  int n_ret = MV_CC_GetEnumValue(camera_handle_, name.c_str(), &ev);
  if (n_ret != MV_OK) return false;
  value = std::to_string(ev.nCurValue);
  return true;
}

bool HikCameraMain::setEnumParam(const std::string& name,
                                 const std::string& value) {
  std::lock_guard<std::mutex> lock(handle_mutex_);
  int n_ret = MV_CC_SetEnumValueByString(camera_handle_, name.c_str(),
                                         value.c_str());
  return (n_ret == MV_OK);
}

std::vector<std::string> HikCameraMain::getEnumParamOptions(
    const std::string& /*name*/) {
  // MVS SDK does not expose a simple "list enum entries" API in the C layer;
  // the entries are documented in the parameter table Excel file instead.
  return {};
}

// ======================== Boolean Parameters ========================

bool HikCameraMain::getBoolParam(const std::string& name, bool& value) {
  std::lock_guard<std::mutex> lock(handle_mutex_);
  // MVS does not have a dedicated GetBoolValue — bools are exposed as ints
  int64_t iv = 0;
  if (!getIntParam(name, iv)) return false;
  value = (iv != 0);
  return true;
}

bool HikCameraMain::setBoolParam(const std::string& name, bool value) {
  std::lock_guard<std::mutex> lock(handle_mutex_);
  int n_ret = MV_CC_SetBoolValue(camera_handle_, name.c_str(), value);
  return (n_ret == MV_OK);
}

// ======================== Pixel Conversion ========================

bool HikCameraMain::convertPixelFormat(const camera_interface::ImageData& src,
                                       const std::string& target_format,
                                       uint8_t* dst,
                                       unsigned int dst_size) {
  std::lock_guard<std::mutex> lock(handle_mutex_);

  MV_CC_PIXEL_CONVERT_PARAM param;
  std::memset(&param, 0, sizeof(param));
  param.nWidth         = src.width;
  param.nHeight        = src.height;
  param.enSrcPixelType = static_cast<MvGvspPixelType>(src.pixel_format);
  param.pSrcData       = const_cast<uint8_t*>(src.data.data());
  param.nSrcDataLen    = static_cast<unsigned int>(src.data.size());
  param.enDstPixelType = static_cast<MvGvspPixelType>(
      pixelTypeFromString(target_format));
  param.pDstBuffer     = dst;
  param.nDstBufferSize = dst_size;

  int n_ret = MV_CC_ConvertPixelType(camera_handle_, &param);
  return (n_ret == MV_OK);
}

// ======================== Device Info ========================

std::string HikCameraMain::getDeviceModelName() const {
  if (!camera_handle_) return "N/A";
  // Read "DeviceModelName" string parameter (if available)
  // The MVS SDK exposes device info via MV_CC_GetDeviceInfo but that
  // requires an open handle and returns a struct.  For simplicity we
  // return a fixed indicator.
  return "Hikrobot";
}

std::string HikCameraMain::getDeviceSerialNumber() const {
  return "N/A";
}

std::string HikCameraMain::getSdkVendorName() const {
  return "Hikrobot MVS";
}

unsigned int HikCameraMain::getMaxWidth() const {
  return img_info_.nWidthMax;
}

unsigned int HikCameraMain::getMaxHeight() const {
  return img_info_.nHeightMax;
}

// ======================== Static Enumeration ========================

std::vector<camera_interface::DeviceInfo>
HikCameraMain::enumerateDevices() {
  std::vector<camera_interface::DeviceInfo> result;

  int n_ret = MV_CC_Initialize();
  if (n_ret != MV_OK) {
    std::cerr << termcolor::red
              << "[HikCameraMain] enumerateDevices: SDK init fail"
              << termcolor::reset << std::endl;
    return result;
  }

  MV_CC_DEVICE_INFO_LIST device_list;
  std::memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  n_ret = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &device_list);

  if (n_ret != MV_OK) {
    MV_CC_Finalize();
    return result;
  }

  for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
    camera_interface::DeviceInfo info;
    MV_CC_DEVICE_INFO* dev = device_list.pDeviceInfo[i];

    if (dev->nTLayerType == MV_GIGE_DEVICE) {
      info.transport = camera_interface::TransportType::GIGE;
      info.model_name =
          reinterpret_cast<const char*>(dev->SpecialInfo.stGigEInfo.chModelName);
      info.serial_number =
          reinterpret_cast<const char*>(dev->SpecialInfo.stGigEInfo.chSerialNumber);
    } else if (dev->nTLayerType == MV_USB_DEVICE) {
      info.transport = camera_interface::TransportType::USB;
      info.model_name =
          reinterpret_cast<const char*>(dev->SpecialInfo.stUsb3VInfo.chModelName);
      info.serial_number =
          reinterpret_cast<const char*>(dev->SpecialInfo.stUsb3VInfo.chSerialNumber);
    } else {
      info.transport = camera_interface::TransportType::UNKNOWN;
    }

    info.manufacturer = "Hikrobot";
    result.push_back(info);
  }

  MV_CC_Finalize();
  return result;
}

// ======================== Connection Helpers ========================

bool HikCameraMain::connectGigE() {
  MV_CC_DEVICE_INFO stDevInfo;
  MV_GIGE_DEVICE_INFO stGigEDev;
  std::memset(&stDevInfo, 0, sizeof(MV_CC_DEVICE_INFO));
  std::memset(&stGigEDev, 0, sizeof(MV_GIGE_DEVICE_INFO));

  std::cout << termcolor::green
            << "[HikCameraMain] GigE — cam IP: " << cam_ip_
            << "  pc IP: " << pc_ip_
            << termcolor::reset << std::endl;

  parseIp(cam_ip_, stGigEDev.nCurrentIp);
  parseIp(pc_ip_,  stGigEDev.nNetExport);

  stDevInfo.nTLayerType = MV_GIGE_DEVICE;
  stDevInfo.SpecialInfo.stGigEInfo = stGigEDev;

  int n_ret = MV_CC_CreateHandle(&camera_handle_, &stDevInfo);
  if (n_ret != MV_OK) {
    std::cerr << termcolor::red
              << "[HikCameraMain] CreateHandle (GigE) fail! nRet[0x"
              << std::hex << n_ret << "]"
              << termcolor::reset << std::endl;
    return false;
  }

  n_ret = MV_CC_OpenDevice(camera_handle_);
  if (n_ret != MV_OK) {
    std::cerr << termcolor::red
              << "[HikCameraMain] OpenDevice (GigE) fail! nRet[0x"
              << std::hex << n_ret << "]"
              << termcolor::reset << std::endl;
    MV_CC_DestroyHandle(&camera_handle_);
    camera_handle_ = nullptr;
    return false;
  }

  // Best packet size for GigE
  int nPacketSize = MV_CC_GetOptimalPacketSize(camera_handle_);
  if (nPacketSize > 0) {
    n_ret = MV_CC_SetIntValue(camera_handle_, "GevSCPSPacketSize", nPacketSize);
    if (n_ret != MV_OK) {
      std::cerr << termcolor::red
                << "[HikCameraMain] Set GevSCPSPacketSize fail! nRet[0x"
                << std::hex << n_ret << "]"
                << termcolor::reset << std::endl;
      return false;
    }
  }

  return true;
}

bool HikCameraMain::connectUSB() {
  // Enumerate USB devices
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  std::memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  int n_ret = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
  if (n_ret != MV_OK) {
    std::cerr << termcolor::red
              << "[HikCameraMain] EnumDevices (USB) fail! nRet[0x"
              << std::hex << n_ret << "]"
              << termcolor::reset << std::endl;
    return false;
  }

  if (stDeviceList.nDeviceNum == 0) {
    std::cerr << termcolor::red
              << "[HikCameraMain] No USB camera found!"
              << termcolor::reset << std::endl;
    return false;
  }

  if (device_index_ >= static_cast<int>(stDeviceList.nDeviceNum)) {
    std::cerr << termcolor::red
              << "[HikCameraMain] Device index " << device_index_
              << " out of range (found " << stDeviceList.nDeviceNum << ")"
              << termcolor::reset << std::endl;
    return false;
  }

  n_ret = MV_CC_CreateHandle(&camera_handle_,
                             stDeviceList.pDeviceInfo[device_index_]);
  if (n_ret != MV_OK) {
    std::cerr << termcolor::red
              << "[HikCameraMain] CreateHandle (USB) fail! nRet[0x"
              << std::hex << n_ret << "]"
              << termcolor::reset << std::endl;
    return false;
  }

  n_ret = MV_CC_OpenDevice(camera_handle_);
  if (n_ret != MV_OK) {
    std::cerr << termcolor::red
              << "[HikCameraMain] OpenDevice (USB) fail! nRet[0x"
              << std::hex << n_ret << "]"
              << termcolor::reset << std::endl;
    MV_CC_DestroyHandle(&camera_handle_);
    camera_handle_ = nullptr;
    return false;
  }

  return true;
}

}  // namespace hik_driver
