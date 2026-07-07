#ifndef GUGA_UI_PANGOLIN_DATA_SOURCE_HPP
#define GUGA_UI_PANGOLIN_DATA_SOURCE_HPP
/**
 * @file data_source.hpp
 * @brief 封装 ShmReader，提供 UI 渲染所需的统一数据视图。
 *
 * 每帧调用 update() 从共享内存批量读取各槽位数据，
 * 各 Render 模块可直接访问 ui_ 的字段。
 */

#include <cstdint>
#include <string>

#include "guga_ui_common/shm_layout.hpp"
#include "guga_ui_common/shm_reader.hpp"
#include "guga_ui_common/ui_types.hpp"

/**
 * @brief 统一 UI 数据视图，聚合所有 shm 槽位的最新快照。
 *
 * 注意：这些字段由 update() 每帧刷新，
 *       不保证不同字段之间的时序一致性（各模块写入频率不同）。
 */
struct UiData {
  /// 是否有任一模块在向 shm 写入数据
  bool has_data{false};

  guga_ui::UiRobotStatus robot_status{};
  guga_ui::UiGameStatus game_status{};
  guga_ui::UiRfidStatus rfid_status{};
  guga_ui::UiDecision decision{};
  guga_ui::UiEnemy enemy{};
  guga_ui::UiOdom odom{};
  guga_ui::UiYaw yaw{};
  guga_ui::UiPath path{};

  /// 各槽位上次刷新时间戳（update() 调用时的 frame count）
  uint64_t robot_status_age{};
  uint64_t game_status_age{};
  uint64_t decision_age{};
  uint64_t enemy_age{};
  uint64_t odom_age{};
  uint64_t yaw_age{};
  uint64_t path_age{};
};

/**
 * @brief UI 数据源——封装 ShmReader 并按需更新 UiData。
 */
class UiDataSource
{
public:
  /**
   * @brief 打开共享内存。
   * @param shm_name 共享内存名称（与 ShmWriter 使用的名称一致）。
   * @return true 打开成功，false 失败。
   */
  [[nodiscard]] bool open(const std::string& shm_name) {
    return reader_.open(shm_name);
  }

  /**
   * @brief 查询共享内存是否有效。
   */
  [[nodiscard]] bool isValid() const { return reader_.isValid(); }

  /**
   * @brief 每帧调用一次，从共享内存批量读取最新数据。
   *
   * 只读取 seq 发生变化的槽位，跳过未更新的数据。
   * 用帧计数器记录每个槽位的数据年龄（帧数），
   * UI 层可据此判断数据是否过期。
   */
  void update() {
    if (!reader_.isValid()) return;

    ++frame_count_;
    data_.has_data = true;

    // 机器人状态
    if (reader_.checkFresh(guga_ui::UiSlotId::ROBOT_STATUS)) {
      static_cast<void>(reader_.read(guga_ui::UiSlotId::ROBOT_STATUS,
                                     &data_.robot_status,
                                     sizeof(data_.robot_status)));
      data_.robot_status_age = frame_count_;
    }

    // 比赛状态
    if (reader_.checkFresh(guga_ui::UiSlotId::GAME_STATUS)) {
      static_cast<void>(reader_.read(guga_ui::UiSlotId::GAME_STATUS,
                                     &data_.game_status,
                                     sizeof(data_.game_status)));
      data_.game_status_age = frame_count_;
    }

    // RFID 状态
    if (reader_.checkFresh(guga_ui::UiSlotId::RFID_STATUS)) {
      static_cast<void>(reader_.read(guga_ui::UiSlotId::RFID_STATUS,
                                     &data_.rfid_status,
                                     sizeof(data_.rfid_status)));
    }

    // 决策状态
    if (reader_.checkFresh(guga_ui::UiSlotId::DECISION)) {
      static_cast<void>(reader_.read(guga_ui::UiSlotId::DECISION,
                                     &data_.decision,
                                     sizeof(data_.decision)));
      data_.decision_age = frame_count_;
    }

    // 敌方信息
    if (reader_.checkFresh(guga_ui::UiSlotId::ENEMY)) {
      static_cast<void>(reader_.read(guga_ui::UiSlotId::ENEMY, &data_.enemy,
                                     sizeof(data_.enemy)));
      data_.enemy_age = frame_count_;
    }

    // 里程计
    if (reader_.checkFresh(guga_ui::UiSlotId::ODOM)) {
      static_cast<void>(reader_.read(guga_ui::UiSlotId::ODOM, &data_.odom,
                                     sizeof(data_.odom)));
      data_.odom_age = frame_count_;
    }

    // 云台偏航
    if (reader_.checkFresh(guga_ui::UiSlotId::YAW)) {
      static_cast<void>(reader_.read(guga_ui::UiSlotId::YAW, &data_.yaw,
                                     sizeof(data_.yaw)));
      data_.yaw_age = frame_count_;
    }

    // 导航路径
    if (reader_.checkFresh(guga_ui::UiSlotId::PATH)) {
      static_cast<void>(reader_.read(guga_ui::UiSlotId::PATH, &data_.path,
                                     sizeof(data_.path)));
      data_.path_age = frame_count_;
    }
  }

  /**
   * @brief 获取 UI 数据的只读引用。
   */
  [[nodiscard]] const UiData& data() const { return data_; }

  /**
   * @brief 获取当前帧编号（从 update() 首次调用起算）。
   */
  [[nodiscard]] uint64_t frameCount() const { return frame_count_; }

  /**
   * @brief 判断某条数据是否在最近 max_age 帧内更新过。
   * @param age_field 数据的年龄字段（如 data_.robot_status_age）。
   * @param max_age 最大允许的帧数，超过则视为过期。
   * @return true 数据新鲜。
   */
  [[nodiscard]] bool isFresh(uint64_t age_field,
                              uint64_t max_age = 60) const {
    if (frame_count_ < age_field) return false;
    return (frame_count_ - age_field) <= max_age;
  }

private:
  guga_ui::ShmReader reader_;
  UiData data_{};
  uint64_t frame_count_{};
};

#endif  // GUGA_UI_PANGOLIN_DATA_SOURCE_HPP
