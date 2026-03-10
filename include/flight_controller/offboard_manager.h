#ifndef FLIGHT_CONTROLLER_OFFBOARD_MANAGER_H
#define FLIGHT_CONTROLLER_OFFBOARD_MANAGER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include <string>

namespace flight_controller {

/**
 * @brief Offboard 管理器 —— 管理 PX4 飞控模式设置、解锁/锁定和 Offboard 模式维持
 *
 * 所有可配置参数从 ROS 参数服务器读取（对应 YAML 配置中的 offboard_manager 节点）。
 */
class OffboardManager {
public:
    /**
     * @brief 构造函数
     * @param nh 全局节点句柄，用于创建服务客户端和发布者
     * @param pnh 私有节点句柄，用于读取参数（YAML 中 offboard_manager 节点）
     */
    OffboardManager(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    /** @brief 设置飞控模式 */
    bool setMode(const std::string& mode);

    /** @brief 解锁电机（Arm） */
    bool arm();

    /** @brief 锁定电机（Disarm） */
    bool disarm();

    /** @brief 进入 Offboard 模式（自动启动零速度指令流） */
    bool setOffboardMode();

    /** @brief 停止零速度指令发布 */
    void stopOffboard();

    /** @brief 查询零速度指令发布是否处于活跃状态 */
    bool isPublishingSetpoint() const;

private:
    /// 定时器回调：发布零速度指令
    void setpointTimerCallback(const ros::TimerEvent& event);

    // ---- 可配置参数（从 YAML 读取） ----
    std::string set_mode_service_;       ///< set_mode 服务名
    std::string arming_service_;         ///< arming 服务名
    std::string setpoint_velocity_topic_; ///< 速度指令话题名
    int setpoint_pub_queue_size_;        ///< 发布者队列大小
    double setpoint_rate_;               ///< 发布频率（Hz）
    double pre_offboard_wait_;           ///< 进入 OFFBOARD 前的等待时间（秒）

    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;
    ros::Publisher vel_pub_;
    ros::Timer setpoint_timer_;
    bool publishing_setpoint_;
};

}  // namespace flight_controller

#endif  // FLIGHT_CONTROLLER_OFFBOARD_MANAGER_H
