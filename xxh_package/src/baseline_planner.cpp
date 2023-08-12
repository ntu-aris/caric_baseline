
#include "baseline_planner.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "baseline_planner");
    ros::NodeHandle nh_init("~");
    Agent a;
    ros::spin();
}