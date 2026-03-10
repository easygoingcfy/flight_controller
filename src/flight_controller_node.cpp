#include <ros/ros.h>
#include <signal.h>

#include "flight_controller/state_manager.h"
#include "flight_controller/offboard_manager.h"

bool g_shutdown_requested = false;

void signalHandler(int sig) {
    g_shutdown_requested = true;
    ROS_INFO("收到关闭信号，正在停止控制器...");
}

int main(int argc, char** argv) {
    // 中文支持
    std::setlocale(LC_ALL, "zh_CN.UTF-8");

    // 行缓冲
    setvbuf(stdout, NULL, _IOLBF, 0);

    // 初始化 ROS 节点
    ros::init(argc, argv, "flight_controller_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 读取主节点配置
    int spinner_threads;
    double status_log_interval;
    pnh.param<int>("spinner_threads", spinner_threads, 4);
    pnh.param<double>("status_log_interval", status_log_interval, 1.0);
    spinner_threads = std::max(1, spinner_threads);

    ros::AsyncSpinner spinner(spinner_threads);
    spinner.start();
    ROS_INFO("启用 AsyncSpinner，线程数: %d", spinner_threads);

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // 为每个管理器创建独立的命名空间 NodeHandle，
    // 这样 YAML 配置中的 state_manager / offboard_manager 节点参数会自动映射
    ros::NodeHandle state_nh(pnh, "state_manager");
    ros::NodeHandle offboard_nh(pnh, "offboard_manager");

    flight_controller::StateManager state_manager(nh, state_nh);
    flight_controller::OffboardManager offboard_manager(nh, offboard_nh);

    ROS_INFO("Flight Controller 节点已启动，按 Ctrl+C 停止。");

    while (ros::ok() && !g_shutdown_requested) {
        // 定期输出 mavros 连接状态
        if (state_manager.isConnected()) {
            ROS_INFO("mavros 连接正常 | 模式: %s | 解锁: %s",
                     state_manager.getMode().c_str(),
                     state_manager.isArmed() ? "是" : "否");
        } else {
            ROS_WARN("mavros 未连接");
        }

        ros::Duration(status_log_interval).sleep();
    }

    // 退出前停止 offboard 发布
    offboard_manager.stopOffboard();

    spinner.stop();
    ROS_INFO("Flight Controller 节点已停止。");
    return 0;
}
