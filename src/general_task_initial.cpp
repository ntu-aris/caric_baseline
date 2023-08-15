#include "general_task_init.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gcs_task");
    ros::NodeHandle nh_init;
    
    ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh_init);

    gcs_task_assign gcs(nh_ptr);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    return 0;
}