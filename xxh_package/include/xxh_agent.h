#include<iostream>
#include <ros/ros.h>
#include<visualization_msgs/Marker.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <mav_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include "Eigen/Dense"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "Astar.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/geometry/distance.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "utility.h"
#include <mutex>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "xxh_initial_task.h"
#include "xxh_planner.h"

struct grid_cell{
    bool occupied=false;
};

class grid_map{
    public:
        grid_map(){}

        grid_map(Boundingbox box,Eigen::Vector3d grid_size_in){
            path_final_global=Eigen::Vector3d(0,0,0);
            target_point_transiction=Eigen::Vector3i(0,0,0);
            rotation_matrix=box.getSearchRotation();
            rotation_matrix_inv=rotation_matrix.inverse();
            rotation_quat=Eigen::Quaterniond(rotation_matrix);
            map_global_center=box.getCenter();
            map_quat_size=box.getRotExtents();
            grid_size=grid_size_in;
            initial_the_convert();
            set_under_ground_occupied();
            ros::NodeHandle nh_;
            map_pub_=nh_.advertise<visualization_msgs::MarkerArray> ("/firefly/map",1);
        }

        grid_map(Eigen::Vector3d grid_size_in){
            path_final_global=Eigen::Vector3d(0,0,0);
            target_point_transiction=Eigen::Vector3i(0,0,0);
            map_global_center=Eigen::Vector3d(0,0,0);
            map_quat_size=Eigen::Vector3d(200,200,50);
            grid_size=grid_size_in;
            rotation_matrix=Eigen::Matrix3d::Identity();
            rotation_matrix_inv=rotation_matrix.inverse();
            rotation_quat=Eigen::Quaterniond(rotation_matrix);
            initial_the_convert();
            set_under_ground_occupied();
            ros::NodeHandle nh_;
            map_pub_=nh_.advertise<visualization_msgs::MarkerArray> ("/firefly/map",1);
        }

        Eigen::Vector3d get_grid_center_global(Eigen::Vector3i grid_index){
            // Eigen::Vector3d result;
            Eigen::Vector3d bias=(grid_index-map_index_center).cast<double>();
            Eigen::Vector3d local_result=bias.cwiseProduct(grid_size);
            Eigen::Vector3d global_result=rotation_matrix_inv*local_result+map_global_center;
            return global_result;
        }

        void set_under_ground_occupied(){
            for(int x=0;x<map_shape.x();x++){
                for(int y=0;y<map_shape.y();y++){
                    for(int z=0;z<map_shape.z();z++){
                        Eigen::Vector3d grid_center_global=get_grid_center_global(Eigen::Vector3i(x,y,z));
                        if(grid_center_global.z()<0.5*grid_size.z()){
                            map[x][y][z]=1;
                        }
                    }
                }
            }
        }


        void initial_the_convert(){
            int x_lim;
            int y_lim;
            int z_lim;
            if(map_quat_size.x()<0.5*grid_size.x()){
                x_lim=0;
            }else{
                x_lim=floor((map_quat_size.x()-0.5*grid_size.x())/grid_size.x())+1;
            }
            if(map_quat_size.y()<0.5*grid_size.y()){
                y_lim=0;
            }else{
                y_lim=floor((map_quat_size.y()-0.5*grid_size.y())/grid_size.y())+1;
            }
            if(map_quat_size.z()<0.5*grid_size.z()){
                z_lim=0;
            }else{
                z_lim=floor((map_quat_size.z()-0.5*grid_size.z())/grid_size.z())+1;
            }

            x_lim=x_lim+1;
            y_lim=y_lim+1;
            z_lim=z_lim+1;

            cout<<"x lim:"<<x_lim<<endl;
            cout<<"y lim:"<<y_lim<<endl;
            cout<<"z lim:"<<z_lim<<endl;

            map_shape=Eigen::Vector3i(2*x_lim+1,2*y_lim+1,2*z_lim+1);

            cout<<"Map shape:"<<map_shape.transpose()<<endl;
            map_index_center=Eigen::Vector3i(x_lim,y_lim,z_lim);
            cout<<"Map Center Index:"<<map_index_center.transpose()<<endl;
            map=vector<vector<vector<int>>>(map_shape.x(), vector<vector<int>>(map_shape.y(), vector<int>(map_shape.z(), 0)));
            astar_planner=AStar(map,map_shape);
            explore_map=xxh_explore_map(map_shape,3);
            
        }
        void update_marker(int i,int j,int k){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "cube_marker_array";
            marker.id =markers.markers.size();
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            Eigen::Vector3d grid_center=get_grid_center_global(Eigen::Vector3i(i,j,k));
            marker.pose.position.x = grid_center.x();
            marker.pose.position.y = grid_center.y();
            marker.pose.position.z = grid_center.z();
            marker.pose.orientation.x = rotation_quat.x();
            marker.pose.orientation.y = rotation_quat.y();
            marker.pose.orientation.z = rotation_quat.z();
            marker.pose.orientation.w = rotation_quat.w();
            marker.scale.x = grid_size.x();
            marker.scale.y = grid_size.y();
            marker.scale.z = grid_size.z();
            if(map[i][j][k]==1&&grid_center.z()>0){
                marker.color.a = 0.5; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                markers.markers.push_back(marker);
            }
        }
        void show_occupied(){
            map_pub_.publish(markers);
        }
        void visual_path(){
            nav_msgs::Path path_msg;
        }
        void plan_to_target(Eigen::Vector3i next_point_index,Eigen::Vector3d target_pointW){
            Eigen::Vector3i start=now_local_index;
            path_final_global=target_pointW;
            // Eigen::Vector3i start=Eigen::Vector3i(0,1,15)+map_index_center;
            // cout<<"now:"<<(now_local_index-map_index_center).transpose()<<endl;
            Eigen::Vector3i end=next_point_index;//+map_index_center;
            list<Eigen::Vector3i> result=astar_planner.get_path(map,start,end);
            Path_index_list=result;
            generate_Path_point_list_global();
        }

        void get_exploration_path(){
            planning_exploration();
            Path_index_list=explore_map.get_path();
            generate_Path_point_list_global();  
        }
        bool whether_find_path(){
            if(Path_index_list.size()==0){
                return false;
            }else{
                return true;
            }
        }


        Eigen::Vector3i get_index(Eigen::Vector3d point_in){
            Eigen::Vector3d point_in_local=rotation_matrix*(point_in-map_global_center);
            now_local_position=point_in_local;
            Eigen::Vector3i bias_index(0,0,0);
            if(fabs(point_in_local.x())<0.5*grid_size.x()){
                bias_index.x()=0;
            }else{
                if(point_in_local.x()>0){
                    bias_index.x()=floor((point_in_local.x()-0.5*grid_size.x())/grid_size.x())+1;
                }else{
                    bias_index.x()=-floor((-point_in_local.x()-0.5*grid_size.x())/grid_size.x())-1;
                }
            }

            if(fabs(point_in_local.y())<0.5*grid_size.y()){
                bias_index.y()=0;
            }else{
                if(point_in_local.y()>0){
                    bias_index.y()=floor((point_in_local.y()-0.5*grid_size.y())/grid_size.y())+1;
                }else{
                    bias_index.y()=-floor((-point_in_local.y()-0.5*grid_size.y())/grid_size.y())-1;
                }
            }

            if(fabs(point_in_local.z())<0.5*grid_size.z()){
                bias_index.z()=0;
            }else{
                if(point_in_local.z()>0){
                    bias_index.z()=floor((point_in_local.z()-0.5*grid_size.z())/grid_size.z())+1;
                }else{
                    bias_index.z()=-floor((-point_in_local.z()-0.5*grid_size.z())/grid_size.z())-1;
                }
            }
            Eigen::Vector3i result=bias_index+map_index_center;
            return result;
        }

        Eigen::Vector3d get_next_point(){
            // cout<<"now:"<<(now_local_index-map_index_center).transpose()<<endl;
            // cout<<"size in get next:"<<Path_point_list_global.size()<<endl;
            if(Path_point_list_global.size()>0){
                // cout<<"Path size:"<<Path_point_list_global.size()<<endl;
                if((now_local_index-map_index_center).z()<=1){
                    // cout<<"Next 3 Point:"<< (now_local_index+Eigen::Vector3i(0,0,1)-map_index_center).transpose()<<endl;
                    // return get_grid_center_global(now_local_index+Eigen::Vector3i(0,0,1));
                    return Path_point_list_global.front();
                }else{
                    // cout<<"Next 2 Point:"<< (Path_index_list.front()-map_index_center).transpose()<<endl;
                    return Path_point_list_global.front();
                }
                
            }else{
                // cout<<"Next 1 Point:"<< (now_local_index-map_index_center).transpose()<<endl;
                if(get_index(path_final_global)==now_local_index)
                {
                    return path_final_global;
                }
                else{
                    return get_grid_center_global(now_local_index);
                }
                // return rotation_matrix_inv*now_local_position+map_global_center;
            }
        }

        nav_msgs::Path get_path_msgs(){
            global_path.header.frame_id="world";
            return global_path;
        }

        void show_path(){

        }
        void update_now_position(Eigen::Vector3d point_in){
            Eigen::Vector3d point_in_local=rotation_matrix*(point_in-map_global_center);
            now_local_position=point_in_local;
            // cout<<"matrix:"<<rotation_matrix<<endl;
            // cout<<"center point:"<<map_global_center.transpose();
            // cout<<point_in_local.transpose()<<endl;
            if(out_of_range(now_local_position,false)){
                in_my_range=false;
                // cout<<"actual position"<<(rotation_matrix*(point_in-map_global_center)).transpose()<<endl;
            }else{
                in_my_range=true;
                Eigen::Vector3i bias_index(0,0,0);
                if(fabs(point_in_local.x())<0.5*grid_size.x()){
                    bias_index.x()=0;
                }else{
                    if(point_in_local.x()>0){
                        bias_index.x()=floor((point_in_local.x()-0.5*grid_size.x())/grid_size.x())+1;
                    }else{
                        bias_index.x()=-floor((-point_in_local.x()-0.5*grid_size.x())/grid_size.x())-1;
                    }
                }

                if(fabs(point_in_local.y())<0.5*grid_size.y()){
                    bias_index.y()=0;
                }else{
                    if(point_in_local.y()>0){
                        bias_index.y()=floor((point_in_local.y()-0.5*grid_size.y())/grid_size.y())+1;
                    }else{
                        bias_index.y()=-floor((-point_in_local.y()-0.5*grid_size.y())/grid_size.y())-1;
                    }
                }

                if(fabs(point_in_local.z())<0.5*grid_size.z()){
                    bias_index.z()=0;
                }else{
                    if(point_in_local.z()>0){
                        bias_index.z()=floor((point_in_local.z()-0.5*grid_size.z())/grid_size.z())+1;
                    }else{
                        bias_index.z()=-floor((-point_in_local.z()-0.5*grid_size.z())/grid_size.z())-1;
                    }
                }
                now_local_index=bias_index+map_index_center;
                explore_map.set_visited(now_local_index);
            }

        }

        bool insert_point(Eigen::Vector3d point_in){
            Eigen::Vector3d point_in_local=rotation_matrix*(point_in-map_global_center);
            if(out_of_range(point_in_local,false)){
                return false;
            }
            Eigen::Vector3i bias_index(0,0,0);
            if(fabs(point_in_local.x())<0.5*grid_size.x()){
                bias_index.x()=0;
            }else{
                if(point_in_local.x()>0){
                    bias_index.x()=floor((point_in_local.x()-0.5*grid_size.x())/grid_size.x())+1;
                }else{
                    bias_index.x()=-floor((-point_in_local.x()-0.5*grid_size.x())/grid_size.x())-1;
                }
            }

            if(fabs(point_in_local.y())<0.5*grid_size.y()){
                bias_index.y()=0;
            }else{
                if(point_in_local.y()>0){
                    bias_index.y()=floor((point_in_local.y()-0.5*grid_size.y())/grid_size.y())+1;
                }else{
                    bias_index.y()=-floor((-point_in_local.y()-0.5*grid_size.y())/grid_size.y())-1;
                }
            }
            
            if(fabs(point_in_local.z())<0.5*grid_size.z()){
                bias_index.z()=0;
            }else{
                if(point_in_local.z()>0){
                    bias_index.z()=floor((point_in_local.z()-0.5*grid_size.z())/grid_size.z())+1;
                }else{
                    bias_index.z()=-floor((-point_in_local.z()-0.5*grid_size.z())/grid_size.z())-1;
                }
            }
            Eigen::Vector3i true_index=bias_index+map_index_center;
            if(map[true_index.x()][true_index.y()][true_index.z()]==1){
                return false;    
            }else{
                map[true_index.x()][true_index.y()][true_index.z()]=1;
                explore_map.insert_map(true_index);
                update_marker(true_index.x(),true_index.y(),true_index.z());
                return true;
            }
        
        }
        bool is_in_boundingbox(){
            return in_my_range;
        }

        Eigen::Vector3d get_target_point_global(){
            return get_grid_center_global(target_point_transiction);
        }

        void update_target_point(bool replan){

            if(map[target_point_transiction.x()][target_point_transiction.y()][target_point_transiction.z()]==1||replan){
                int x=target_point_transiction.x();
                int y=target_point_transiction.y();
                int z=target_point_transiction.z();
                
                int distance=min({abs(x),abs(y),abs(map_shape.x()-x),abs(map_shape.y()-y)});
                int top;
                int bottom;
                int left;
                int right;
                int i=x;int j=y;

                for(int k=z;k<map_shape.z();k++){
                    top=map_shape.y()-distance;
                    bottom=distance;
                    left=distance;
                    right=map_shape.x()-distance;
                    
                    for(i;i<=right&&j==bottom;i++){
                         yolo();
                        if(i<0||j<0||k<0||i>=map_shape.x()||j>=map_shape.y()||k>=map_shape.z()){
                            continue;
                        }
                        if(map[i][j][k]==0){
                            if(target_point_transiction==Eigen::Vector3i(i,j,k)){
                                continue;
                            }else{
                                target_point_transiction=Eigen::Vector3i(i,j,k);
                                return;                                
                            }
                        }

                    }
                    for(j;j<top&&i==right;j++){
                        yolo();
                        if(target_point_transiction==Eigen::Vector3i(i,j,k)){
                            continue;
                        }else{
                            target_point_transiction=Eigen::Vector3i(i,j,k);
                            return;                                
                        }
                    }
                    for(i;i>=left&&j==top;i--){
                        yolo();
                        if(target_point_transiction==Eigen::Vector3i(i,j,k)){
                            continue;
                        }else{
                            target_point_transiction=Eigen::Vector3i(i,j,k);
                            return;                                
                        }
                    }
                    for(j;j>=bottom&&i==left;j--){
                        yolo();
                        if(target_point_transiction==Eigen::Vector3i(i,j,k)){
                            continue;
                        }else{
                            target_point_transiction=Eigen::Vector3i(i,j,k);
                            return;                                
                        }
                    }
                    distance++;
                    i=distance;
                    j=distance;
                    cout<<"Not find point in layer:"<<k<<endl;
                }

            }else{
                yolo();
                //DO SOMETHING                
            }

        }
        bool check_get_point(){
            if(get_index(path_final_global)==now_local_index)
                {
                    return true;
                }else{
                    return false;
                }
        }

        void planning_exploration(){
            explore_map.generate_searching_point();      
        }
        void generate_path(){

            
        }
        bool check_finish(){
            return explore_map.check_finish();
        }


    private:
        xxh_explore_map explore_map;
        bool finish_bottom=false;
        int direction;
        vector<vector<vector<int>>> map;
        Eigen::Vector3d grid_size;
        Eigen::Matrix3d rotation_matrix;
        Eigen::Matrix3d rotation_matrix_inv;
        Eigen::Quaterniond rotation_quat;
        Eigen::Vector3d map_global_center;
        Eigen::Vector3i map_shape;
        Eigen::Vector3i map_index_center;
        Eigen::Vector3d map_quat_size;
        bool ground_initialise=false;
        AStar astar_planner;
        visualization_msgs::MarkerArray markers;
        Eigen::Vector3d now_local_position;
        Eigen::Vector3i now_local_index;
        bool in_my_range=false;
        ros::Publisher map_pub_;
        list<Eigen::Vector3i> Path_index_list;
        list<Eigen::Vector3d> Path_point_list_global;
        nav_msgs::Path global_path;
        bool Developing=true;
        Eigen::Vector3i target_point_transiction;
        Eigen::Vector3d path_final_global;

        bool out_of_range(Eigen::Vector3d point,bool out_put){
            if(fabs(point.x())>(fabs(map_shape.x()/2)+0.5)*grid_size.x()||fabs(point.y())>(fabs(map_shape.y()/2)+0.5)*grid_size.y() ||fabs(point.z())>(fabs(map_shape.z()/2)+0.5)*grid_size.z()){
                if(out_put){
                    cout<<"xbool:"<<(fabs(point.x())>fabs(map_shape.x()/2)*grid_size.x()?"yes":"no")<<endl;
                    cout<<"xlim:"<<fabs(map_shape.x()/2)*grid_size.x()<<endl;
                    cout<<"ylim:"<<fabs(map_shape.y()/2)*grid_size.y()<<endl;
                    cout<<"grid size:"<<grid_size.transpose()<<endl;
                    cout<<"map_shape:"<<map_shape.transpose()<<endl;
                    cout<<"center:"<<map_index_center.transpose()<<endl;
                    cout<<"out range point"<<point.transpose()<<endl;
                    cout<<"Desired point"<<(rotation_matrix*(get_grid_center_global(Eigen::Vector3i(0,0,0))-map_global_center)).transpose()<<endl;
                }
                return true;
            }else{
                return false;
            }            
        }
        void generate_Path_point_list_global(){
            list<Eigen::Vector3i> path_tamp(Path_index_list);
            list<Eigen::Vector3d> point_global_list_tamp;
            // bool findway=false;
            // cout<<"The path size in generate :"<<path_tamp.size()<<endl;
            if(Path_index_list.empty()){

                Path_point_list_global.clear();

                return ;
            }
            // cout<<path_tamp.front().transpose()<<endl;
            // if(path_tamp.size()<=4){
            //     for (const auto& element : path_tamp) {
            //     std::cout << (element-map_index_center).transpose() << endl;
            //     } 
            // }
            nav_msgs::Path global_path_tamp;
            path_tamp.pop_back();
            global_path_tamp.header.frame_id="world";
            while(!path_tamp.empty()){
                Eigen::Vector3i index_current=path_tamp.back();
                Eigen::Vector3d point_current=get_grid_center_global(index_current);
                if(Developing){
                    geometry_msgs::PoseStamped pose;
                    pose.header.frame_id="world";
                    pose.header.stamp=ros::Time::now();
                    pose.pose.position.x=point_current.x();
                    pose.pose.position.y=point_current.y();
                    pose.pose.position.z=point_current.z();
                    pose.pose.orientation.w=1.0;
                    global_path_tamp.poses.push_back(pose);
                }
                point_global_list_tamp.push_back(point_current);
                path_tamp.pop_back();
            }
            point_global_list_tamp.push_back(path_final_global);
            global_path=global_path_tamp;
            Path_point_list_global=point_global_list_tamp;
            // cout<<"The path size2 in generate :"<<point_global_list_tamp.size()<<endl;
            //Delete first point
            // Path_point_list_global.pop_front();
        }
};



class map_manager{
    public:
        map_manager(){cout<<"map_manager"<<endl;}
        map_manager(string str,string name,ros::Publisher cmd,ros::Publisher path):move_pub_(cmd),path_pub_(path){
            nh_=ros::NodeHandle();
            grid_size=Eigen::Vector3d(safe_distance,safe_distance,safe_distance);
            global_map=grid_map(grid_size);
            namespace_=name;
            vector<string> spilited_str;
            std::istringstream iss(str);
            std::string substring;
            // move_pub_= nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory> ("/firefly/command/trajectory", 10);
            // move_pub_= nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory> ("/firefly/command/trajectory_test", 10);
            timer=nh_.createTimer(ros::Duration(1.0 / 10.0), &map_manager::test_timer, this);
            while (std::getline(iss, substring, ';')) {
                spilited_str.push_back(substring);
            }
            generate_global_map(spilited_str[0]);
            if(spilited_str.size()>1){
                for(int j=0;j<path_index.size();j++)
                {
                    map_set.push_back(grid_map(Boundingbox(spilited_str[path_index[j]]),grid_size));
                }
            }else{
                cout<<"Path Assigned Error!!!"<<endl;
            }
            cout<<"size of path assigned:  "<<map_set.size()<<endl;
            finish_init=true;
        }
        void update_position(Eigen::Vector3d position){

            if(!finish_init){
                return;
            }

            my_position=position;
            if(map_set.size()>0){
                
                global_map.update_now_position(position);
                // yolo();
                map_set[0].update_now_position(position);
                // yolo();
            }else{
                global_map.update_now_position(position);
            }

            if(!map_set.size()){
                work_in_box=false;
                return_home=true;
            }else{
                work_in_box=map_set[0].is_in_boundingbox();
                return_home=false;
            }
            // cout<<"now state:"<<work_in_box<<endl;
            return;
        }

        void insert_point(Eigen::Vector3d point_in){
            if(!finish_init){
                // yolo();
                return;
            }
            if(map_set.size()>0){
                global_map.insert_point(point_in);
                for(int i=0;i<map_set.size();i++){
                    map_set[i].insert_point(point_in);
                }

            }else{
                global_map.insert_point(point_in);
            }
        }
        void show_map(){
            if(map_set.size()==0){
                global_map.show_occupied();
            }else{
                map_set[0].show_occupied();
            }
            
        }
        void show_trajactory(){
            //check whether finish the exploration
            // cout<<"Now State:"<<is_transfer<<endl;
            if(!is_transfer){
                if(map_set[0].check_finish()){
                    is_transfer=true;
                    map_set.erase(map_set.begin());
                    yolo();
                }
            }

            if(map_set.size()==0){
                yolo();
                if(namespace_=="jurong"){
                    Eigen::Vector3d target_point=Eigen::Vector3d(0,0,0);//return home
                    Eigen::Vector3i global_target_point=global_map.get_index(target_point);
                    // cout<<global_target_point.transpose()<<endl;
                    global_map.plan_to_target(global_target_point,target_point);
                }
                path_pub_.publish(global_map.get_path_msgs());
                // yolo();
            }else if(is_transfer){
                yolo();
                Eigen::Vector3d target_point=map_set[0].get_target_point_global();
                Eigen::Vector3i global_target_point=global_map.get_index(target_point);
                
                // cout<<global_target_point.transpose()<<endl;
                global_map.plan_to_target(global_target_point,target_point);
                yolo();
                map_set[0].update_target_point(!global_map.whether_find_path());
                yolo();
                cout<<"target point"<<target_point.transpose()<<endl;
                // global_map.plan_to_target(Eigen::Vector3i(20,20,5));
                is_transfer=!global_map.check_get_point();
                // is_transfer=!map_set[0].is_in_boundingbox();
                cout<<"is_transfer"<<is_transfer<<endl;
                path_pub_.publish(global_map.get_path_msgs());
            }else{
                yolo();
                // map_set[0].planning_exploration();
                map_set[0].get_exploration_path();
                path_pub_.publish(map_set[0].get_path_msgs());
            }
            
        }

        void transfer_function(){

            
        }


        void fly_global(){
            trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
            trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;
            // trajset_msg.header.frame_id = "world";
            geometry_msgs::Transform transform_msg;
            geometry_msgs::Twist accel_msg, vel_msg;
            Eigen::Vector3d desired_pos;
            if(work_in_box&&!is_transfer){
                desired_pos=(map_set[0].get_next_point());
            }else{
                desired_pos=(global_map.get_next_point());
            }
            
            // cout<<"out put "<<desired_pos.transpose()<<endl;
            Eigen::Vector3d target_pos=(desired_pos-my_position);
            if(target_pos.norm()<1.5){
                // target_pos=Eigen::Vector3d(0,0,0);
                transform_msg.translation.x = desired_pos.x();
                transform_msg.translation.y = desired_pos.y();
                transform_msg.translation.z = desired_pos.z();
                // transform_msg.translation.x = 0;
                // transform_msg.translation.y = 0;
                // transform_msg.translation.z = 0;
                vel_msg.linear.x = 0;
                vel_msg.linear.y = 0;
                vel_msg.linear.z = 0;
                // transform_msg.translation.x = 0;
                // transform_msg.translation.y = 0;
                // transform_msg.translation.z = 0;
                // vel_msg.linear.x = target_pos.x();
                // vel_msg.linear.y = target_pos.y();
                // vel_msg.linear.z = target_pos.z();

            }else{
                target_pos=1.5*target_pos/target_pos.norm();
                transform_msg.translation.x = 0;
                transform_msg.translation.y = 0;
                transform_msg.translation.z = 0;
                vel_msg.linear.x = target_pos.x();
                vel_msg.linear.y = target_pos.y();
                vel_msg.linear.z = target_pos.z();
            }
            // Eigen::Vector3d target_pos=global_map.get_next_point();
            // transform_msg.translation.x = target_pos(0);
            // transform_msg.translation.y = target_pos(1);
            // transform_msg.translation.z = target_pos(2);

            transform_msg.rotation.x = 0;
            transform_msg.rotation.y = 0;
            transform_msg.rotation.z = 0;
            transform_msg.rotation.w = 1;

            trajpt_msg.transforms.push_back(transform_msg);

            accel_msg.linear.x = 0;
            accel_msg.linear.y = 0;
            accel_msg.linear.z = 0;

            trajpt_msg.velocities.push_back(vel_msg);
            trajpt_msg.accelerations.push_back(accel_msg);
            trajset_msg.points.push_back(trajpt_msg);

            trajset_msg.header.stamp = ros::Time::now();
            // yolo();
            trajset_msg.header.frame_id = "world";
            move_pub_.publish(trajset_msg); 
        }

        void test_timer(const ros::TimerEvent &){
            // yolo();
        }
    private:
        vector<Boundingbox> task_set;
        vector<grid_map> map_set;
        vector<string> teammates;
        double safe_distance=2.5;
        Eigen::Vector3d grid_size;
        string namespace_;
        int Teamid=-1;
        ros::Publisher move_pub_,path_pub_;
        ros::Timer timer;
        ros::NodeHandle nh_;
        bool finish_init=false;
        bool is_transfer=true;
        bool is_exploring_and_scan=false;
        vector<int> teammate;
        vector<int> path_index;
        grid_map global_map;
        Eigen::Vector3d my_position;
        
        //inner boundingbox
        bool work_in_box=false;
        bool return_home=false;


        void generate_global_map(string s){
            vector<string> spilited_str;
            std::istringstream iss(s);
            std::string substring;
            while (std::getline(iss, substring, ',')) {
                spilited_str.push_back(substring);
            }
            Eigen::Vector3d max_region(stod(spilited_str[3]),stod(spilited_str[4]),stod(spilited_str[5]));
            Eigen::Vector3d min_region(stod(spilited_str[0]),stod(spilited_str[1]),stod(spilited_str[2]));
            vector<vector<string>>teams(2);
            vector<int>size_of_path(2);
            for(int i=6;i<spilited_str.size();i++){
                if(spilited_str[i]=="team"){
                    for(int j=i+3;j<i+3+stoi(spilited_str[i+2]);j++){
                        teams[stoi(spilited_str[i+1])].push_back(spilited_str[j]);
                    }
                    i=i+2+stoi(spilited_str[i+2]);
                    continue;
                }
                if(spilited_str[i]=="path_size"){
                    // cout<<"test"<<stoi(spilited_str[i+2])<<endl;
                    size_of_path[stoi(spilited_str[i+1])]=stoi(spilited_str[i+2]);
                    i=i+2;
                    continue;
                }
            }
            for(int i=0;i<2;i++){
                for(int j=0;j<teams[i].size();j++){
                    if("/"+teams[i][j]==namespace_){
                        Teamid=i;
                    }
                    break;
                }
                if(Teamid>-1){
                    break;
                }
            }
            cout<<"TeamID:"<<Teamid<<endl;
            if(Teamid==0){
                for(int i=0;i<size_of_path[0];i++){
                    path_index.push_back(i+1);
                    cout<<i+1<<endl;
                }
            }else{
                for(int i=0;i<size_of_path[1];i++){
                    path_index.push_back(i+1+size_of_path[0]);
                    cout<<i+1+size_of_path[0]<<endl;
                }
            }
        }




};

class  xxh_agent{
    public:
        xxh_agent():global_octree(0.5){
            ros::NodeHandle nh_;
            ros::NodeHandle nh2_;
            ros::NodeHandle nh3_;

            nh_.setCallbackQueue(&custom_queue1);
            nh2_.setCallbackQueue(&custom_queue2);
            nh3_.setCallbackQueue(&custom_queue3);
            // yolo();
            namespace_ = nh_.getNamespace();
            octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("/firefly/octomap", 1);

            // global_octree=octomap::OcTree(3);
            task_sub_=nh_.subscribe("/task_assign/"+namespace_,10,&xxh_agent::TaskCallback,this);
            cloud_sub_=new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/cloud_inW", 10);
            nbr_sub_=new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_,"/nbr_odom_cloud", 10);

            odom_filter_sub_=new message_filters::Subscriber<nav_msgs::Odometry>(nh_,"/ground_truth/odometry", 10);
            traj_pub_= nh2_.advertise<trajectory_msgs::MultiDOFJointTrajectory> ("/firefly/command/trajectory", 10);
            path_pub_=nh_.advertise<nav_msgs::Path> ("/firefly/path_show", 10);
            motion_timer=nh3_.createTimer(ros::Duration(1.0 / 10), &xxh_agent::MotionTimerCallback, this);
            timer=nh_.createTimer(ros::Duration(1.0 / 2), &xxh_agent::TimerCallback, this);
            odom_sub_ = nh_.subscribe("/ground_truth/odometry", 10, &xxh_agent::OdomCallback, this);
            
            my_position=Eigen::Vector3d(0,0,0);
            sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *cloud_sub_, *nbr_sub_,*odom_filter_sub_);
            sync_->registerCallback(boost::bind(&xxh_agent::CloudCallback, this, _1, _2,_3));
            collision_box_size=Eigen::Vector3d (2.5,2.5,2.5);
            ros::AsyncSpinner spinner1(1, &custom_queue1);  // 1 thread for the custom_queue1 // 0 means threads= # of CPU cores
            ros::AsyncSpinner spinner2(1, &custom_queue2);  // 1 thread for the custom_queue2 // 0 means threads= # of CPU cores
            ros::AsyncSpinner spinner3(1, &custom_queue3);
            spinner1.start();  // start spinner of the custom queue 1
            spinner2.start();  // start spinner of the custom queue 2
            spinner3.start();
            // yolo();
            ros::waitForShutdown();

        };
        void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,const sensor_msgs::PointCloud2ConstPtr& Nbr,const nav_msgs::OdometryConstPtr& msg){
            // yolo();
            Eigen::Vector3d sync_my_position=Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,msg->pose.pose.position.z);
            if(!map_initialise){
                return;
            }
            double time1=cloud->header.stamp.toSec();
            double time2=Nbr->header.stamp.toSec();
            if(std::fabs(time1-time2)>0.2){
                return;
            }
            Nbr_point.clear();
            Nbr_point.push_back(sync_my_position);
            CloudOdomPtr Nbr_cloud(new CloudOdom());
            pcl::fromROSMsg(*Nbr, *Nbr_cloud);
            for(const auto& point : Nbr_cloud->points)
            {
                // double i,j,k=0;
                Eigen::Vector3d cloud_point(point.x,point.y,point.z);
                if(std::fabs(point.t-time2)>0.2)
                {
                    cout<<"Missing Nbr"<<endl;
                }else{
                    Nbr_point.push_back(cloud_point);
                }
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud, *cloud_cloud);
            // yolo();

            for (const auto& point : cloud_cloud->points)
            {
                Eigen::Vector3d cloud_cloud_point(point.x,point.y,point.z);
                if(is_Nbr(cloud_cloud_point)||(cloud_cloud_point-my_position).norm()<2.5){
                    continue;
                }else{
                    octomap::point3d endpoint(cloud_cloud_point.x(), cloud_cloud_point.y(),cloud_cloud_point.z());
                    global_octree.updateNode(endpoint, true);
                    mm.insert_point(cloud_cloud_point);
                }
            }
            //
            
            global_octree.updateInnerOccupancy();
            octomap_msgs::Octomap octomap_msg;
            octomap_msgs::fullMapToMsg(global_octree, octomap_msg);
            octomap_msg.header.frame_id="world";
            octomap_pub_.publish(octomap_msg);
            // mm.show_map();
            // yolo();
        }
    private:
        ros::Subscriber odom_sub_,task_sub_;
        ros::Publisher octomap_pub_,traj_pub_,path_pub_;
        octomap::OcTree global_octree;
        string namespace_;
        string pre_task="";

        ros::CallbackQueue custom_queue1;
        ros::CallbackQueue custom_queue2;
        ros::CallbackQueue custom_queue3;
        ros::Timer timer,motion_timer;

        message_filters::Subscriber<sensor_msgs::PointCloud2>* cloud_sub_;
        message_filters::Subscriber<sensor_msgs::PointCloud2>* nbr_sub_;
        message_filters::Subscriber<nav_msgs::Odometry>* odom_filter_sub_;
        // message_filters::Subscriber<nav_msgs::Odometry>* odom_sub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,nav_msgs::Odometry> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy>* sync_;
        // ros::Subscriber odom_sub_;
        Eigen::Vector3d collision_box_size;
        Eigen::Vector3d my_position;
        vector<Eigen::Vector3d> Nbr_point;

        map_manager mm;
        bool planning_finish=false;
        bool map_initialise=false;
        bool is_Nbr(Eigen::Vector3d test){
            if(Nbr_point.size()==0){
                return false;
            }else{
                for(int i=0;i<Nbr_point.size();i++){
                    Eigen::Vector3d Nbr=Nbr_point[i];
                    Eigen::Vector3d diff=Nbr-test;
                    if(fabs(diff[0])<=collision_box_size[0]&&fabs(diff[1])<=collision_box_size[1]&&fabs(diff[2])<=collision_box_size[2]){
                        return true;
                    }
                }
                return false;
            }
        }
        void OdomCallback(const nav_msgs::OdometryConstPtr& msg) {
            //do nothing hhh
            my_position=Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,msg->pose.pose.position.z);
            mm.update_position(my_position);
            // mm.show_trajactory();
        }
        void TaskCallback(const std_msgs::String msg) {
            // yolo();
            if(pre_task==msg.data){
                return;
            }
            mm=map_manager(msg.data,namespace_,traj_pub_,path_pub_);
            pre_task=msg.data;
            map_initialise=true;
        }    
        void TimerCallback(const ros::TimerEvent &){
            if(!map_initialise){
                return;
            }
            mm.show_map();
            mm.show_trajactory();
        }
        void MotionTimerCallback(const ros::TimerEvent&){
            if(!map_initialise){
                return;
            }
            // yolo();
            mm.fly_global();
        }

};

