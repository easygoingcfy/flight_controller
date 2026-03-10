#ifndef FLIGHT_CONTROLLER_STATE_MANAGER_H
#define FLIGHT_CONTROLLER_STATE_MANAGER_H

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <realtime_tools/realtime_buffer.h>

#include <string>

namespace flight_controller {

/**
 * @brief 状态管理器 —— 订阅 mavros/state 话题并提供线程安全的状态访问
 *
 * 使用 realtime_tools::RealtimeBuffer 保存最新的 mavros 状态消息，
 * 支持在实时线程和非实时线程间安全共享数据。
 * 所有可配置参数从 ROS 参数服务器读取（对应 YAML 配置中的 state_manager 节点）。
 */
class StateManager {
public:
    /**
     * @brief 构造函数
     * @param nh 全局节点句柄，用于订阅话题
     * @param pnh 私有节点句柄，用于读取参数（YAML 中 state_manager 节点）
     */
    StateManager(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    /** @brief 判断 mavros 是否已连接到飞控 */
    bool isConnected() const;

    /** @brief 判断飞控是否已解锁（armed） */
    bool isArmed() const;

    /** @brief 获取当前飞控模式字符串 */
    std::string getMode() const;

    /** @brief 获取完整的 mavros 状态消息 */
    mavros_msgs::State getState() const;

private:
    /// 状态话题回调
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);

    // ---- 可配置参数（从 YAML 读取） ----
    std::string state_topic_;   ///< 状态话题名
    int queue_size_;            ///< 订阅队列大小

    ros::Subscriber state_sub_;
    realtime_tools::RealtimeBuffer<mavros_msgs::State> state_buffer_;
};

}  // namespace flight_controller

#endif  // FLIGHT_CONTROLLER_STATE_MANAGER_H
