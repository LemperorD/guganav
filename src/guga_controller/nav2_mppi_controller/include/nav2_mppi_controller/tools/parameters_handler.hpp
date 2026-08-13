// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__PARAMETERS_HANDLER_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__PARAMETERS_HANDLER_HPP_

#include <functional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mppi
{
/**
 * @class Parameter Type enum
 */
enum class ParameterType { Dynamic, Static };

/**
 * @class mppi::ParametersHandler
 * @brief 统一处理参数声明、读取和动态更新
 */
class ParametersHandler
{
public:
  using get_param_func_t = void (const rclcpp::Parameter & param);
  using post_callback_t = void ();
  using pre_callback_t = void ();

  /**
    * @brief 构造未绑定节点的参数处理器
    */
  ParametersHandler() = default;

  /**
    * @brief 构造参数处理器并绑定生命周期节点
    * @param parent: 参数所属生命周期节点的弱引用
    */
  explicit ParametersHandler(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent);

  /**
    * @brief 注册并开始处理动态参数更新
    */
  void start();

  /**
    * @brief 校验并应用一批动态参数更新
    * @param parameters: 待处理的参数更新列表
    * @return 返回值: 参数设置结果，ROS 2 参数服务据此接受或拒绝整批更新
    */
  rcl_interfaces::msg::SetParametersResult dynamicParamsCallback(
    std::vector<rclcpp::Parameter> parameters);

  /**
    * @brief 创建绑定指定命名空间的参数读取函数
    * @param ns: 参数命名空间
    * @return 返回值: 参数读取闭包，调用方用它声明、读取并注册动态参数
    */
  inline auto getParamGetter(const std::string & ns);

  /**
    * @brief 注册参数更新完成后的回调
    * @param callback: 更新完成后执行的函数
    */
  template<typename T>
  void addPostCallback(T && callback);

  /**
    * @brief 注册参数更新开始前的回调
    * @param callback: 更新开始前执行的函数
    */
  template<typename T>
  void addPreCallback(T && callback);

  /**
    * @brief 为参数绑定自动写入目标变量的动态更新回调
    * @param setting: 参数更新时要写入的目标变量
    * @param name: 完整参数名
    */
  template<typename T>
  void setDynamicParamCallback(T & setting, const std::string & name);

  /**
    * @brief 获取保护参数更新过程的互斥锁
    * @return 返回值: 互斥锁指针，优化器用它避免计算期间参数被并发修改
    */
  std::mutex * getLock()
  {
    return &parameters_change_mutex_;
  }

  /**
    * @brief 注册自定义动态参数回调
    * @param name: 完整参数名
    * @param callback: 参数变化时执行的函数
    */
  template<typename T>
  void addDynamicParamCallback(const std::string & name, T && callback);

protected:
  /**
    * @brief 声明并读取参数，必要时注册动态更新
    * @param setting: 接收参数值的目标变量
    * @param name: 完整参数名
    * @param default_value: 参数默认值
    * @param param_type: 参数为动态或静态
    */
  template<typename SettingT, typename ParamT>
  void getParam(
    SettingT & setting, const std::string & name, ParamT default_value,
    ParameterType param_type = ParameterType::Dynamic);

  /**
    * @brief 从节点读取参数并转换后写入目标变量
    * @param setting: 接收参数值的目标变量
    * @param name: 完整参数名
    * @param node: 提供参数接口的节点
    */
  template<typename ParamT, typename SettingT, typename NodeT>
  void setParam(SettingT & setting, const std::string & name, NodeT node) const;

  /**
    * @brief 将 ROS 参数转换为目标 C++ 类型
    * @param parameter: 待转换的 ROS 参数
    * @return 返回值: 与模板类型匹配的值，供动态回调写入配置变量
    */
  template<typename T>
  static auto as(const rclcpp::Parameter & parameter);

  std::mutex parameters_change_mutex_;  ///< 保护参数读取和动态更新过程。
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};  ///< 参数处理日志记录器。
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_param_handler_;  ///< 动态参数回调的注册句柄。
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;  ///< 参数所属的生命周期节点。
  std::string node_name_;  ///< 参数所属节点名称，用于日志输出。

  bool verbose_{false};  ///< 是否输出详细的参数变更日志。

  std::unordered_map<std::string, std::function<get_param_func_t>>
  get_param_callbacks_;  ///< 按完整参数名索引的动态更新函数。

  std::vector<std::function<pre_callback_t>> pre_callbacks_;  ///< 参数更新前执行的回调。
  std::vector<std::function<post_callback_t>> post_callbacks_;  ///< 参数更新后执行的回调。
};

inline auto ParametersHandler::getParamGetter(const std::string & ns)
{
  return [this, ns](
    auto & setting, const std::string & name, auto default_value,
    ParameterType param_type = ParameterType::Dynamic) {
           getParam(
             setting, ns.empty() ? name : ns + "." + name,
             std::move(default_value), param_type);
         };
}

template<typename T>
void ParametersHandler::addDynamicParamCallback(const std::string & name, T && callback)
{
  get_param_callbacks_[name] = callback;
}

template<typename T>
void ParametersHandler::addPostCallback(T && callback)
{
  post_callbacks_.push_back(callback);
}

template<typename T>
void ParametersHandler::addPreCallback(T && callback)
{
  pre_callbacks_.push_back(callback);
}

template<typename SettingT, typename ParamT>
void ParametersHandler::getParam(
  SettingT & setting, const std::string & name,
  ParamT default_value,
  ParameterType param_type)
{
  auto node = node_.lock();

  nav2_util::declare_parameter_if_not_declared(
    node, name, rclcpp::ParameterValue(default_value));

  setParam<ParamT>(setting, name, node);

  if (param_type == ParameterType::Dynamic) {
    setDynamicParamCallback(setting, name);
  }
}

template<typename ParamT, typename SettingT, typename NodeT>
void ParametersHandler::setParam(
  SettingT & setting, const std::string & name, NodeT node) const
{
  ParamT param_in{};
  node->get_parameter(name, param_in);
  setting = static_cast<SettingT>(param_in);
}

template<typename T>
void ParametersHandler::setDynamicParamCallback(T & setting, const std::string & name)
{
  if (get_param_callbacks_.find(name) != get_param_callbacks_.end()) {
    return;
  }

  auto callback = [this, &setting, name](const rclcpp::Parameter & param) {
      setting = as<T>(param);

      if (verbose_) {
        RCLCPP_INFO(logger_, "Dynamic parameter changed: %s", std::to_string(param).c_str());
      }
    };

  addDynamicParamCallback(name, callback);

  if (verbose_) {
    RCLCPP_INFO(logger_, "Dynamic Parameter added %s", name.c_str());
  }
}

template<typename T>
auto ParametersHandler::as(const rclcpp::Parameter & parameter)
{
  if constexpr (std::is_same_v<T, bool>) {
    return parameter.as_bool();
  } else if constexpr (std::is_integral_v<T>) {
    return parameter.as_int();
  } else if constexpr (std::is_floating_point_v<T>) {
    return parameter.as_double();
  } else if constexpr (std::is_same_v<T, std::string>) {
    return parameter.as_string();
  } else if constexpr (std::is_same_v<T, std::vector<int64_t>>) {
    return parameter.as_integer_array();
  } else if constexpr (std::is_same_v<T, std::vector<double>>) {
    return parameter.as_double_array();
  } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
    return parameter.as_string_array();
  } else if constexpr (std::is_same_v<T, std::vector<bool>>) {
    return parameter.as_bool_array();
  }
}

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__PARAMETERS_HANDLER_HPP_
