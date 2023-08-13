
#include "baseline_planner.h"

// void thegoddamcallback(const sensor_msgs::PointCloud2ConstPtr &cloud,
//                     //    const sensor_msgs::PointCloud2ConstPtr &Nbr,
//                        const nav_msgs::OdometryConstPtr &msg)
// {
// yolo();
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "baseline_planner");
    ros::NodeHandle nh_init;
    
    ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh_init);

    Agent a(nh_ptr);

    // typedef message_filters::sync_policies::
    //         ApproximateTime<sensor_msgs::PointCloud2,
    //                         // sensor_msgs::PointCloud2,
    //                         nav_msgs::Odometry> MySyncPolicy;

    // message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_(nh_init, "/cloud_inW", 10);
    // // message_filters::Subscriber<sensor_msgs::PointCloud2> nbr_sub_(nh_init, "/nbr_odom_cloud", 10);
    // message_filters::Subscriber<nav_msgs::Odometry> odom_filter_sub_(nh_init, "/ground_truth/odometry", 10);

    // message_filters::Synchronizer<MySyncPolicy> sync_(MySyncPolicy(10), cloud_sub_, odom_filter_sub_);
    // sync_.registerCallback(boost::bind(thegoddamcallback, _1, _2));
    
    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    return 0;
}