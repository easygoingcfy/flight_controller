#include "flight_controller/offboard_manager.h"

namespace flight_controller {

OffboardManager::OffboardManager(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : publishing_setpoint_(false) {
    // 从参数服务器读取配置（对应 YAML 中的 offboard_manager 节点）
    pnh.param<std::string>("set_mode_service",       set_mode_service_,        "mavros/set_mode");
    pnh.param<std::string>("arming_service",         arming_service_,          "mavros/cmd/arming");
    pnh.param<std::string>("setpoint_velocity_topic", setpoint_velocity_topic_, "mavros/setpoint_velocity/cmd_vel_unstamped");
    pnh.param<int>("setpoint_pub_queue_size",         setpoint_pub_queue_size_, 10);
    pnh.param<double>("setpoint_rate",               setpoint_rate_,           20.0);
    pnh.param<double>("pre_offboard_wait",           pre_offboard_wait_,       0.5);

    ROS_INFO("OffboardManager: 配置 -> set_mode服务=%s, arming服务=%s, 速度话题=%s, "
             "发布频率=%.1fHz, 队列=%d, 预等待=%.2fs",
             set_mode_service_.c_str(), arming_service_.c_str(),
             setpoint_velocity_topic_.c_str(),
             setpoint_rate_, setpoint_pub_queue_size_, pre_offboard_wait_);

    // 创建服务客户端
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>(set_mode_service_);
    arming_client_   = nh.serviceClient<mavros_msgs::CommandBool>(arming_service_);

    // 创建速度指令发布者
    vel_pub_ = nh.advertise<geometry_msgs::Twist>(
        setpoint_velocity_topic_, setpoint_pub_queue_size_);

    // 创建定时器（初始不启动）
    setpoint_timer_ = nh.createTimer(
        ros::Duration(1.0 / setpoint_rate_),
        &OffboardManager::setpointTimerCallback, this,
        false,  // oneshot = false
        false); // autostart = false

    ROS_INFO("OffboardManager: 初始化完成");
}

bool OffboardManager::setMode(const std::string& mode) {
    mavros_msgs::SetMode srv;
    srv.request.custom_mode = mode;

    if (set_mode_client_.call(srv) && srv.response.mode_sent) {
        ROS_INFO("OffboardManager: 模式已设置为 [%s]", mode.c_str());
        return true;
    } else {
        ROS_WARN("OffboardManager: 设置模式 [%s] 失败", mode.c_str());
        return false;
    }
}

bool OffboardManager::arm() {
    mavros_msgs::CommandBool srv;
    srv.request.value = true;

    if (arming_client_.call(srv) && srv.response.success) {
        ROS_INFO("OffboardManager: 解锁成功");
        return true;
    } else {
        ROS_WARN("OffboardManager: 解锁失败");
        return false;
    }
}

bool OffboardManager::disarm() {
    mavros_msgs::CommandBool srv;
    srv.request.value = false;

    if (arming_client_.call(srv) && srv.response.success) {
        ROS_INFO("OffboardManager: 锁定成功");
        return true;
    } else {
        ROS_WARN("OffboardManager: 锁定失败");
        return false;
    }
}

bool OffboardManager::setOffboardMode() {
    // 第一步：启动零速度指令发布（PX4 要求进入 OFFBOARD 前必须有 setpoint 流）
    if (!publishing_setpoint_) {
        setpoint_timer_.start();
        publishing_setpoint_ = true;
        ROS_INFO("OffboardManager: 已启动零速度指令发布");

        // 等待一小段时间，确保 PX4 收到足够的 setpoint
        ros::Duration(pre_offboard_wait_).sleep();
    }

    // 第二步：切换到 OFFBOARD 模式
    return setMode("OFFBOARD");
}

void OffboardManager::stopOffboard() {
    if (publishing_setpoint_) {
        setpoint_timer_.stop();
        publishing_setpoint_ = false;
        ROS_INFO("OffboardManager: 已停止零速度指令发布");
    }
}

bool OffboardManager::isPublishingSetpoint() const {
    return publishing_setpoint_;
}

void OffboardManager::setpointTimerCallback(const ros::TimerEvent& /*event*/) {
    // 发布全零的 Twist 消息（线速度和角速度都为 0）
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
    vel_pub_.publish(cmd);
}

}  // namespace flight_controller
