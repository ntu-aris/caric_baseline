#include "baseline_planner.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "baseline_planner");
    ros::NodeHandle nh_init;
    
    ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh_init);

    Agent a(nh_ptr);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    return 0;
}