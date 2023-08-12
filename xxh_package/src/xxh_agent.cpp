#include "xxh_agent.h"


void timerCallback(const ros::TimerEvent& event) {
    // 在定时器回调函数中执行的代码
    ROS_INFO("Timer callback triggered!");
}

int main(int argc, char **argv){

    ros::init(argc, argv, "xxh_agent");
    ros::NodeHandle nh_init("~");
    // Task_initial_controller task_initial_controller;
    cout<<"Running xxh agent code"<<endl;
    // ros::Timer timer = nh_init.createTimer(ros::Duration(1.0), timerCallback);
    xxh_agent a;
    ros::spin();
}