#include "flight_controller/state_manager.h"

namespace flight_controller {

StateManager::StateManager(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    // 从参数服务器读取配置（对应 YAML 中的 state_manager 节点）
    pnh.param<std::string>("state_topic", state_topic_, "mavros/state");
    pnh.param<int>("queue_size", queue_size_, 10);

    ROS_INFO("StateManager: 配置 -> 话题=%s, 队列大小=%d",
             state_topic_.c_str(), queue_size_);

    // 初始化 RealtimeBuffer 为默认状态（未连接、未解锁）
    mavros_msgs::State init_state;
    init_state.connected = false;
    init_state.armed = false;
    init_state.mode = "";
    state_buffer_.writeFromNonRT(init_state);

    // 订阅 mavros 状态话题
    state_sub_ = nh.subscribe(state_topic_, queue_size_,
                              &StateManager::stateCallback, this);
    ROS_INFO("StateManager: 已订阅话题 [%s]", state_topic_.c_str());
}

void StateManager::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    state_buffer_.writeFromNonRT(*msg);
}

bool StateManager::isConnected() const {
    return state_buffer_.readFromRT()->connected;
}

bool StateManager::isArmed() const {
    return state_buffer_.readFromRT()->armed;
}

std::string StateManager::getMode() const {
    return state_buffer_.readFromRT()->mode;
}

mavros_msgs::State StateManager::getState() const {
    return *(state_buffer_.readFromRT());
}

}  // namespace flight_controller
