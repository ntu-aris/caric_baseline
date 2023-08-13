#include "initial_task.h"
#include "sensor_msgs/PointCloud.h"

ros::Time lastCallbackTime(0);

void TimerCallback(const ros::TimerEvent&){

    yolo();
    ros::Time currentCallbackTime = ros::Time::now();

    if (lastCallbackTime.isZero()) {
        ROS_INFO("First timer callback executed");
    } else {
        ros::Duration timeDiff = currentCallbackTime - lastCallbackTime;
        ROS_INFO("Timer callback executed. Time since last callback: %.2f seconds", timeDiff.toSec());
    }
    lastCallbackTime = currentCallbackTime;
}

void Task_initial_controller::bboxCallback(const sensor_msgs::PointCloud::ConstPtr& msg){
    sensor_msgs::PointCloud cloud=*msg;
    int num_points=cloud.points.size();
    if(num_points%8==0&&num_points>8*path_saver.size())
    {
        volumn_total=0;
        path_saver.clear();
        int num_box=num_points/8;
        for(int i=0;i<num_box;i++){
            vector<Eigen::Vector3d> point_vec;
            for(int j=0;j<8;j++){
                if(cloud.points[8*i+j].x>xmax){
                    xmax=cloud.points[8*i+j].x;
                }
                if(cloud.points[8*i+j].x<xmin){
                    xmin=cloud.points[8*i+j].x;
                }
                if(cloud.points[8*i+j].y>ymax){
                    ymax=cloud.points[8*i+j].y;
                }
                if(cloud.points[8*i+j].y<ymin){
                    ymin=cloud.points[8*i+j].y;
                }
                if(cloud.points[8*i+j].z>zmax){
                    zmax=cloud.points[8*i+j].z;
                }
                if(cloud.points[8*i+j].z<zmin){
                    zmin=cloud.points[8*i+j].z;
                }
                point_vec.push_back(Eigen::Vector3d(cloud.points[8*i+j].x,cloud.points[8*i+j].y,cloud.points[8*i+j].z));
            }
            path_saver.push_back(Boundingbox(point_vec,i));
            volumn_total+=path_saver[i].getVolume();
            point_vec.clear();
        }
        finish_bbox_record=true;
        //Calculate the link matrix
        calculate_node_distance();
        find_shortest_link();
        assign_path_calculation();

    }else{
        // printf("Have finish the bbox loading \n");
        return;
    }
    return;
};

void Task_initial_controller::calculate_node_distance(){
    vector_overlap.clear(); 
    vector_overlap.resize(path_saver.size(), std::vector<bool>(path_saver.size()));
    std::cout <<"calculate whether the node is overlap:"<<endl;
    for(int i =0;i<path_saver.size();i++){
        for(int j = 0;j<path_saver.size();j++){
            vector_overlap[i][j]=checkOverlap(path_saver[i],path_saver[j]);
            std::cout << vector_overlap[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void Task_initial_controller::find_shortest_path(){
    shortest_path_not_edit.clear();
    std::vector<int> shortestPath = findShortestPath(path_saver);
    std::cout << "Shortest Path: ";
    for (int i = 0; i < shortestPath.size(); i++) {
        std::cout << shortestPath[i] << " ";
    }
    std::cout << std::endl;
    for(int j=0;j<shortestPath.size();j++)
    {
        std::cout<<"("<<path_saver[shortestPath[j]].getCenter().transpose()<<")"<<std::endl;
        shortest_path_not_edit.push_back(path_saver[shortestPath[j]]);
    }
    printf("finish calculate the shortest path");
};

Task_initial_controller::Task_initial_controller(){
    xmax=-std::numeric_limits<double>::max();
    ymax=-std::numeric_limits<double>::max();
    zmax=-std::numeric_limits<double>::max();
    xmin=std::numeric_limits<double>::max();
    ymin=std::numeric_limits<double>::max();
    zmin=std::numeric_limits<double>::max();
    
    
        nh = ros::NodeHandle();
        nh2 = ros::NodeHandle();
        // timer=nh.create
        client = nh.serviceClient<caric_mission::CreatePPComTopic>("create_ppcom_topic");
        cmd_pub_ = nh.advertise<std_msgs::String> ("/task_assign", 10);
        srv.request.source = "gcs"; 
        srv.request.targets.push_back("all");
        srv.request.topic_name = "/task_assign";
        srv.request.package_name = "std_msgs"; 
        srv.request.message_type = "String";
        while(!serviceAvailable){
            serviceAvailable=ros::service::waitForService("create_ppcom_topic", ros::Duration(10.0));
        }   
        client.call(srv);
        bbox_sub_=nh2.subscribe<sensor_msgs::PointCloud>("/gcs/bounding_box_vertices", 10, &Task_initial_controller::bboxCallback, this);
        timer=nh.createTimer(ros::Duration(1.0 / 10.0), &Task_initial_controller::bboxtimer, this);
    };

int main(int argc, char **argv){

    ros::init(argc, argv, "gcs_talker");
    ros::NodeHandle nh_init("~");
    Task_initial_controller task_initial_controller;
    ros::spin();
}
