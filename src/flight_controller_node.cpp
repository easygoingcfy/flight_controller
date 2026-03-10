#include <ros/ros.h>
#include <signal.h>

bool g_shutdown_requested = false;

void signalHandler(int sig) {
    g_shutdown_requested = true;
    ROS_INFO("Shutdown signal received. Stopping the controller...");
}

int main(int argc, char** argv) {
    // 中文支持
    std::setlocale(LC_ALL, "zh_CN.UTF-8");

    // 行缓冲
    setvbuf(stdout, NULL, _IOLBF, 0);

    // initialize
    ros::init(argc, argv, "flight_controller_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    int spinner_threads = 4;
    pnh.param("spinner_threads", spinner_threads, spinner_threads);
    spinner_threads = std::max(1, spinner_threads);
    ros::AsyncSpinner spinner(spinner_threads);
    spinner.start();
    ROS_INFO("启用AsyncSpinner，线程数: %d", spinner_threads);

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    ROS_INFO("Flight Controller Node started. Press Ctrl+C to stop.");

    while (ros::ok() && !g_shutdown_requested) {
        ros::Duration(1).sleep(); // Sleep for a short duration to reduce CPU usage
        ROS_INFO("Flight Controller is running...");
    }

    spinner.stop();
    ROS_INFO("Flight Controller Node stopped.");
    return 0;
}
