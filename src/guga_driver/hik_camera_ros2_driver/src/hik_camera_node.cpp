#include "hik_camera_ros2_driver/hik_camera_node.hpp"

namespace hik_camera_ros2_driver
{

HikCameraRos2DriverNode::HikCameraRos2DriverNode(const rclcpp::NodeOptions & options)
  : Node("hik_camera_ros2_driver", options)
{
  std::cout << "\033[32mStarting HikCamera Ros2 Driver Node!\033[0m" << std::endl;

  // init camera type & ip/usb
  camera_type_ = this->declare_parameter("camera_type", 0); // 0:gige, 1:usb
  pcIp_ = this->declare_parameter("pcIp", "192.168.10.2");
  cameraIp_ = this->declare_parameter("cameraIp", "192.168.10.10");
  deviceIndex_ = this->declare_parameter("device_index", 0);

  // init camera, declare parameters, start camera
  initializeCamera(); declareParameters(); startCamera();

  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&HikCameraRos2DriverNode::dynamicParametersCallback, this, std::placeholders::_1));

  capture_thread_ = std::thread(&HikCameraRos2DriverNode::captureLoop, this);
}

HikCameraRos2DriverNode::~HikCameraRos2DriverNode()
{
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  if (camera_handle_) {
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(&camera_handle_);
    MV_CC_Finalize();
  }
  std::cout << "\033[32mHikCamera Ros2 Driver Node destroyed!\033[0m" << std::endl;
}

bool HikCameraRos2DriverNode::initializeCamera()
{
  // init sdk
  n_ret_ = MV_CC_Initialize();
  if (n_ret_ != MV_OK) {
    std::cerr << "\033[31mInitialize SDK fail! nRet[0x" << std::hex << n_ret_ << "]\033[0m" << std::endl;
    return false;
  }

  // 枚举设备，主要用于检测是否连接相机以及堵塞线程
  MV_CC_DEVICE_INFO_LIST device_list;
  memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  while (rclcpp::ok()) {
    n_ret_ = MV_CC_EnumDevices(MV_USB_DEVICE|MV_GIGE_DEVICE, &device_list);
    if (n_ret_ != MV_OK) {
      std::cout << "\033[33mFailed to enumerate devices, retrying...\033[0m" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else if (device_list.nDeviceNum == 0) {
      std::cout << "\033[33mNo camera found, retrying...\033[0m" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      RCLCPP_INFO(this->get_logger(), "\033[32mFound camera count = %d", device_list.nDeviceNum);
      break;
    }
  }

  // 判断相机类型并连接相机
  if (camera_type_ == GIGE_CAMERA) tryConnectGigE();
  else tryConnectUSB();

  n_ret_ = MV_CC_GetImageInfo(camera_handle_, &img_info_);
  if (n_ret_ != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get camera image info!");
    return false;
  }

  // Init convert param
  image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);
  convert_param_.nWidth = img_info_.nWidthValue;
  convert_param_.nHeight = img_info_.nHeightValue;
  convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

  return true;
}

void HikCameraRos2DriverNode::declareParameters()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  MVCC_FLOATVALUE f_value;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;

  // Acquisition frame rate
  param_desc.description = "Acquisition frame rate in Hz";
  MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &f_value);
  param_desc.integer_range[0].from_value = f_value.fMin;
  param_desc.integer_range[0].to_value = f_value.fMax;
  double acquisition_frame_rate =
    this->declare_parameter("acquisition_frame_rate", 165.0, param_desc);
  MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
  MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", acquisition_frame_rate);
  RCLCPP_INFO(this->get_logger(), "Acquisition frame rate: %f", acquisition_frame_rate);

  // Exposure time
  param_desc.description = "Exposure time in microseconds";
  MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
  param_desc.integer_range[0].from_value = f_value.fMin;
  param_desc.integer_range[0].to_value = f_value.fMax;
  double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
  MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
  RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

  // Gain
  param_desc.description = "Gain";
  MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
  param_desc.integer_range[0].from_value = f_value.fMin;
  param_desc.integer_range[0].to_value = f_value.fMax;
  double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
  MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
  RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);

  int status;

  // Pixel format
  param_desc.description = "Pixel Format";
  std::string pixel_format = this->declare_parameter("pixel_format", "RGB8Packed", param_desc);
  status = MV_CC_SetEnumValueByString(camera_handle_, "PixelFormat", pixel_format.c_str());
  if (status == MV_OK) {
    RCLCPP_INFO(this->get_logger(), "Pixel Format set to %s", pixel_format.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "\033[31mFailed to set Pixel Format, status = %d", status);
  }
}

void HikCameraRos2DriverNode::startCamera()
{
  bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
  camera_name_ = this->declare_parameter("camera_name", "camera");
  frame_id_ = this->declare_parameter("frame_id", camera_name_ + "_optical_frame");
  camera_topic_ = this->declare_parameter("camera_topic", camera_name_ + "/image");

  auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
  camera_pub_ = image_transport::create_camera_publisher(this, camera_topic_, qos);

  MV_CC_StartGrabbing(camera_handle_);

  // Load camera info
  camera_info_manager_ =
    std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
  auto camera_info_url = this->declare_parameter(
    "camera_info_url", "package://hik_camera_ros2_driver/config/camera_info.yaml");
  if (camera_info_manager_->validateURL(camera_info_url)) {
    camera_info_manager_->loadCameraInfo(camera_info_url);
    camera_info_msg_ = camera_info_manager_->getCameraInfo();
  } else {
    RCLCPP_WARN(this->get_logger(), "\033[33mInvalid camera info URL: %s", camera_info_url.c_str());
  }
}

void HikCameraRos2DriverNode::captureLoop()
{
  MV_FRAME_OUT out_frame;
  RCLCPP_INFO(this->get_logger(), "Publishing image!");

  image_msg_.header.frame_id = frame_id_;
  image_msg_.encoding = "rgb8";

  while (rclcpp::ok()) {
    n_ret_ = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
    if (n_ret_ == MV_OK) {
      convert_param_.pDstBuffer = image_msg_.data.data();
      convert_param_.nDstBufferSize = image_msg_.data.size();

      convert_param_.pSrcData = out_frame.pBufAddr;
      convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
      convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

      MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

      image_msg_.header.stamp = this->now();
      image_msg_.height = out_frame.stFrameInfo.nHeight;
      image_msg_.width = out_frame.stFrameInfo.nWidth;
      image_msg_.step = out_frame.stFrameInfo.nWidth * 3;
      image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);

      camera_info_msg_.header = image_msg_.header;
      camera_pub_.publish(image_msg_, camera_info_msg_);

      MV_CC_FreeImageBuffer(camera_handle_, &out_frame);

      static auto last_log_time = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 3) {
        MVCC_FLOATVALUE f_value;
        MV_CC_GetFloatValue(camera_handle_, "ResultingFrameRate", &f_value);
        RCLCPP_DEBUG(this->get_logger(), "\033[33mResultingFrameRate: %f Hz", f_value.fCurValue);
        last_log_time = now;
      }

    } else {
      RCLCPP_WARN(this->get_logger(), "\033[33mGet buffer failed! nRet: [%x]", n_ret_);
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_StartGrabbing(camera_handle_);
      fail_count_++;
    }

    if (fail_count_ > 5) {
      RCLCPP_FATAL(this->get_logger(), "\033[31mCamera failed!");
      rclcpp::shutdown();
    }
  }
}

rcl_interfaces::msg::SetParametersResult HikCameraRos2DriverNode::dynamicParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    const auto & type = param.get_type();
    const auto & name = param.get_name();
    int status = MV_OK;

    if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (name == "gain") {
        status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + name;
        continue;
      }
    } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
      if (name == "exposure_time") {
        status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
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

    if (status != MV_OK) {
      result.successful = false;
      result.reason = "Failed to set " + name + ", status = " + std::to_string(status);
    }
  }

  return result;
}

void HikCameraRos2DriverNode::tryConnectGigE()
{
  MV_CC_DEVICE_INFO stDevInfo; MV_GIGE_DEVICE_INFO stGigEDev;
  memset(&stDevInfo, 0, sizeof(MV_CC_DEVICE_INFO));
  memset(&stGigEDev, 0, sizeof(MV_GIGE_DEVICE_INFO));
  
  // 解析IP地址
  std::cout << "\033[32mPC IP: " << pcIp_ << "\033[0m" <<std::endl;
  std::cout << "\033[32mGigE Camera IP: " << cameraIp_ << "\033[0m" <<std::endl;
  parseIp(cameraIp_, stGigEDev.nCurrentIp); parseIp(pcIp_, stGigEDev.nNetExport);
  stDevInfo.nTLayerType = MV_GIGE_DEVICE; stDevInfo.SpecialInfo.stGigEInfo = stGigEDev;
  
  // 创建句柄
  n_ret_ = MV_CC_CreateHandle(&camera_handle_, &stDevInfo);
  if (n_ret_ != MV_OK) {
    std::cerr << "\033[31mCreate Handle fail! nRet[0x" << std::hex << n_ret_ << "]" << "\033[0m" << std::endl;
    return;
  }
  
  // 打开设备
  n_ret_ = MV_CC_OpenDevice(camera_handle_);
  if (n_ret_ != MV_OK) {
    std::cerr << "\033[31mOpen device fail! nRet[0x" << std::hex << n_ret_ << "]" << "\033[0m" << std::endl;
    MV_CC_DestroyHandle(camera_handle_); camera_handle_ = nullptr;
    return;
  }
  
  // 获取 GigE 相机的最佳数据包大小
  int nPacketSize = MV_CC_GetOptimalPacketSize(camera_handle_);
  if (nPacketSize > 0) {
    n_ret_ = MV_CC_SetIntValue(camera_handle_, "GevSCPSPacketSize", nPacketSize);
    if (n_ret_ != MV_OK) {
      std::cerr << "\033[31mSet Packet Size fail! nRet[0x" << std::hex << n_ret_ << "]" << "\033[0m" << std::endl;
      return;
    }
  }
}

void HikCameraRos2DriverNode::tryConnectUSB()
{
  // 枚举USB设备
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  
  n_ret_ = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
  if (n_ret_ != MV_OK) {
    std::cerr << "\033[31mEnum USB devices fail! nRet[0x" << std::hex << n_ret_ << "]" << "\033[0m" << std::endl;
    return;
  }
  
  if (stDeviceList.nDeviceNum == 0) {
    std::cerr << "\033[31mNo USB camera found!\033[0m" << std::endl;
    return;
  }
  
  if (deviceIndex_ >= static_cast<int>(stDeviceList.nDeviceNum)) {
    std::cerr << "\033[31mDevice index out of range! Found " << stDeviceList.nDeviceNum << " devices.\033[0m" << std::endl;
    return;
  }
  
  // 创建句柄
  n_ret_ = MV_CC_CreateHandle(&camera_handle_, stDeviceList.pDeviceInfo[deviceIndex_]);
  if (n_ret_ != MV_OK) {
    std::cerr << "\033[31mCreate Handle fail! nRet[0x" << std::hex << n_ret_ << "]" << "\033[0m" << std::endl;
    return;
  }

  // 打开设备
  n_ret_ = MV_CC_OpenDevice(camera_handle_);
  if (n_ret_ != MV_OK) {
    std::cerr << "\033[31mOpen device fail! nRet[0x" << std::hex << n_ret_ << "]" << "\033[0m" << std::endl;
    MV_CC_DestroyHandle(camera_handle_); camera_handle_  = nullptr;
    return;
  }
}

} // namespace hik_camera_ros2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera_ros2_driver::HikCameraRos2DriverNode)
