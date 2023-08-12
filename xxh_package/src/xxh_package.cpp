#include "xxh_package.h"

Agent::Agent(int dimx,int dimy,int dimz,Eigen::Vector3d& origin,double safed):global_map(dimx,dimy,dimz,origin,safed),nh_()//,
                                                                            // cloud_sub_(nh_, "/cloud_inW", 10)//,
                                                                            // nbr_sub_(nh_,"/nbr_odom_cloud", 10)
{
    nh_.setCallbackQueue(&custom_queue1);
    nh2_.setCallbackQueue(&custom_queue2);
    nh3_.setCallbackQueue(&custom_queue3);
    nh4_.setCallbackQueue(&custom_queue4);
    nh5_.setCallbackQueue(&custom_queue5);

    cloud_sub_=new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/cloud_inW", 10);
    nbr_sub_=new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_,"/nbr_odom_cloud", 10);

    sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *cloud_sub_, *nbr_sub_);
    sync_->registerCallback(boost::bind(&Agent::CloudCallback, this, _1, _2));

    map_pub_ = nh3_.advertise<visualization_msgs::MarkerArray> ("/firefly/map",1);
    Nbr_pub_ = nh4_.advertise<visualization_msgs::MarkerArray> ("/firefly/Nbr_mask",1);

    // octomap_sub_=nh_.subscribe("/cloud_inW",10,&Agent::OctomapCallback,this);
    // neibor_sub_=nh2_.subscribe("/nbr_odom_cloud",10,&Agent::NbrCallback,this);


    odom_sub_ = nh2_.subscribe("/ground_truth/odometry", 10, &Agent::OdomCallback, this);
    timer = nh5_.createTimer(ros::Duration(1.0 / 1.0), &Agent::TimerCallback, this);
    timer2= nh4_.createTimer(ros::Duration(1.0 / 10.0), &Agent::TimerCallback2, this);


    ros::AsyncSpinner spinner1(1, &custom_queue1);  // 1 thread for the custom_queue1 // 0 means threads= # of CPU cores
    ros::AsyncSpinner spinner2(1, &custom_queue2);  // 1 thread for the custom_queue2 // 0 means threads= # of CPU cores
    ros::AsyncSpinner spinner3(1, &custom_queue3);  // 1 thread for the custom_queue3 // 0 means threads= # of CPU cores
    ros::AsyncSpinner spinner4(1, &custom_queue4);  // 1 thread for the custom_queue1 // 0 means threads= # of CPU cores
    ros::AsyncSpinner spinner5(1, &custom_queue5);  // 1 thread for the custom_queue2 // 0 means threads= # of CPU cores

    spinner1.start();  // start spinner of the custom queue 1
    spinner2.start();  // start spinner of the custom queue 2
    spinner3.start();  // start spinner of the custom queue 3
    spinner4.start();  // start spinner of the custom queue 2
    spinner5.start();  // start spinner of the custom queue 3
    ros::waitForShutdown();
};

void Agent::CloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,const sensor_msgs::PointCloud2ConstPtr& Nbr){
    // std::lock_guard<std::mutex> guard(myMutex); 
    double time1=cloud->header.stamp.toSec();
    double time2=Nbr->header.stamp.toSec();
    if(std::fabs(time1-time2)>0.1){
        // printf("time_cloud:%f,time_Nbr:%f,difference:%f.\n",time1,time2,time1-time2);
        return;
    }

    Nbr_mask.clear();
    Nbr_point.clear();
    clean_last_Nbr();
    CloudOdomPtr Nbr_cloud(new CloudOdom());
    pcl::fromROSMsg(*Nbr, *Nbr_cloud);
    for(const auto& point : Nbr_cloud->points)
    {
        double i,j,k=0;
        Eigen::Vector3d cloud_point(point.x,point.y,point.z);
        Nbr_point.push_back(cloud_point);
        i=std::floor((cloud_point(0)-origin_point(0))/safe_distance);
        j=std::floor((cloud_point(1)-origin_point(1))/safe_distance);
        k=std::floor((cloud_point(2)-origin_point(2))/safe_distance);
        Nbr_mask.push_back(Eigen::Vector3d(i,j,k));
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *cloud_cloud);
    // yolo();

    for (const auto& point : cloud_cloud->points)
    {
        Eigen::Vector3d cloud_cloud_point(point.x,point.y,point.z);
        insert_point(cloud_cloud_point);
    }
}


void Agent::NbrCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    return;
    std::lock_guard<std::mutex> guard(myMutex); 
    Nbr_mask.clear();
    Nbr_point.clear();
    clean_last_Nbr();
    CloudOdomPtr cloud(new CloudOdom());
    pcl::fromROSMsg(*msg, *cloud);
    for(const auto& point : cloud->points)
    {
        double i,j,k=0;
        Eigen::Vector3d cloud_point(point.x,point.y,point.z);
        Nbr_point.push_back(cloud_point);
        i=std::floor((cloud_point(0)-origin_point(0))/safe_distance);
        j=std::floor((cloud_point(1)-origin_point(1))/safe_distance);
        k=std::floor((cloud_point(2)-origin_point(2))/safe_distance);
        Nbr_mask.push_back(Eigen::Vector3d(i,j,k));
    }
};
void Agent::OctomapCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    // yolo();
    return;
    // std::lock_guard<std::mutex> guard(myMutex); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    // yolo();

    for (const auto& point : cloud->points)
    {
        Eigen::Vector3d cloud_point(point.x,point.y,point.z);
        insert_point(cloud_point);
    }
    //  yolo();

}

void Agent::TimerCallback2(const ros::TimerEvent &){
    
    visualization_msgs::MarkerArray markers;
    // yolo();
    markers.markers.clear();
    for(int index=0;index<Nbr_mask.size();index++){
        visualization_msgs::Marker marker;
                    marker.header.frame_id = "world";
                    marker.header.stamp = ros::Time::now();
                    marker.ns = "Nbr_marker_array";
                    marker.id =index;
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.action = visualization_msgs::Marker::ADD;
                    // marker.pose.position.x = (Nbr_mask[index](0)+0.5)*safe_distance+origin_point(0);
                    // marker.pose.position.y = (Nbr_mask[index](1)+0.5)*safe_distance+origin_point(1);
                    // marker.pose.position.z = (Nbr_mask[index](2)+0.5)*safe_distance+origin_point(2);
                    marker.pose.position.x = Nbr_point[index](0);
                    marker.pose.position.y = Nbr_point[index](1);
                    marker.pose.position.z = Nbr_point[index](2);
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = safe_distance;
                    marker.scale.y = safe_distance;
                    marker.scale.z = safe_distance;
                    marker.color.a = 0.5; // Don't forget to set the alpha!
                    marker.color.r = 0.50;
                    marker.color.g = 0.0;
                    marker.color.b = 0.50;
                    markers.markers.push_back(marker);
    }
    Nbr_pub_.publish(markers);
}

void Agent::OdomCallback(const nav_msgs::OdometryConstPtr& msg) {
    odom_ = *msg;
}
void Agent::TimerCallback(const ros::TimerEvent &){
    // return;
    visualization_msgs::MarkerArray markers;
    // yolo();
    markers.markers.clear();
    int id=0;
    for (int i = 0; i < xlim; i++){
        for(int j =0;j<ylim;j++){
            for(int k=0;k<zlim;k++){
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "world";
                    marker.header.stamp = ros::Time::now();
                    marker.ns = "cube_marker_array";
                    marker.id =id;
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = (i+0.5)*safe_distance+origin_point(0);
                    marker.pose.position.y = (j+0.5)*safe_distance+origin_point(1);
                    marker.pose.position.z = (k+0.5)*safe_distance+origin_point(2);
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = safe_distance;
                    marker.scale.y = safe_distance;
                    marker.scale.z = safe_distance;
                    if(map[i][j][k].is_occupied()){
                        marker.color.a = 0.5; // Don't forget to set the alpha!
                        marker.color.r = 1.0;
                        marker.color.g = 0.0;
                        marker.color.b = 0.0;
                        markers.markers.push_back(marker);
                        id++;
                    }else if(map[i][j][k].is_Nbr_in()){
                        // yolo();
                        marker.color.a = 0.8; // Don't forget to set the alpha!
                        marker.color.r = 0.0;
                        marker.color.g = 0.0;
                        marker.color.b = 1.0;
                        markers.markers.push_back(marker);
                        id++;
                        continue;

                    }else if(map[i][j][k].is_interest()){
                        marker.color.a = 0.2; // Don't forget to set the alpha!
                        marker.color.r = 0.0;
                        marker.color.g = 1.0;
                        marker.color.b = 0.0;
                        markers.markers.push_back(marker);
                        id++;
                    }
            }
        }
    }
    // Publish the marker array
    map_pub_.publish(markers);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xxh_package");
    ros::NodeHandle nh_init();

    // yolo();
    Eigen::Vector3d origin(-40,-25,0);
    Agent explorer(80,35,35,origin,2);
    // Agent explorer(40,18,18,origin,4);
    // ros::waitForShutdown();
    ros::spin();

    return 0;
}