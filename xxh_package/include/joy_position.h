/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef ROTORS_JOY_INTERFACE_JOY_POSITION_H_
#define ROTORS_JOY_INTERFACE_JOY_POSITION_H_

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <nav_msgs/Odometry.h>
#include "utility.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include "Eigen/Dense"
#include "geometry_msgs/Twist.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/geometry/distance.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
// #include <pcl_conversions/pcl_conversions.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

// #include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


struct Axes {
  int roll;
  int pitch;
  int thrust;
  int roll_direction;
  int pitch_direction;
  int thrust_direction;
};

struct Buttons {
  int takeoff;
  int land;
  int ctrl_enable;
  int ctrl_mode;
  int yaw_left;
  int yaw_right;
};

struct Max {
  double v_xy;
  double roll;
  double pitch;
  double rate_yaw;
  double thrust;
};

struct Box{
    geometry_msgs::Point center_point;
    geometry_msgs::Vector3 size;
    geometry_msgs::Quaternion orientation;
};




class Joy {
  typedef sensor_msgs::Joy::_buttons_type ButtonType;

 private:
  // ros::NodeHandle nh_;
  ros::Publisher ctrl_pub_, cmd_pub_, gim_cmd_pub_;
  ros::Subscriber joy_sub_, odom_sub_, gim_sub_;
  ros::Timer timer,timer2,timer3;
  std::string namespace_;

  //use to planning the motion
  ros::Subscriber octomap_sub_;
  ros::Publisher octomap_pub_,path_pub_;
  Axes axes_;
  Buttons buttons_;
  // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  mav_msgs::RollPitchYawrateThrust control_msg_;
  geometry_msgs::PoseStamped pose_;
  sensor_msgs::Joy current_joy_;
  nav_msgs::Odometry odom_;
  Max max_;

  double current_yaw_vel_;
  double v_yaw_step_;

  bool is_fixed_wing_;
  double yaw_cmd = 0.0;
  Eigen::Vector3d xyz_cmd_world;
  ros::Time time_joy;
  double vertical_speed;

  void StopMav();
  myTf<double> tf_uav;
  void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
  void OdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void GimbalCallback(const geometry_msgs::TwistStamped& msg);
  void OctomapCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void TimerCallback(const ros::TimerEvent &);
  void TimerCallback2(const ros::TimerEvent &);
  void Publish();
  void sqrt();
  bool got_joy_=true;
  void revise_path_point(geometry_msgs::Point& searchPointMsg);
  void generateSpiralPath(const geometry_msgs::Vector3& size, const std::vector<double>& transformation_matrix);
  void generate_voxel_path(const geometry_msgs::Vector3& size, const std::vector<double>& transformation_matrix);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  octomap::OcTree explore_octree; 
  // octomap::OcTree explore_octree=octomap::OcTree(0.05);
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree;
  geometry_msgs::Vector3 center;

  geometry_msgs::Vector3 size;
  std::vector<double> transformation_matrix = {
            0.909586131573, 0.415515899658, 0.000000000000,  38.869224548340,
           -0.415515899658, 0.909586131573, 0.000000000000, -0.340786933899,
            0.000000000000, 0.000000000000, 1.000000357628,  55.821704864502,
            0.000000000000, 0.000000000000, 0.000000000000,  1.000000000000};
    // std::vector<double> transformation_matrix = {
    //         1, 0.0, 0.000000000000,  0,
    //        0.0, 1, 0.000000000000,  0,
    //         0.0, 0.0, 1,  0,
    //         0.0, 0.0, 0.000000000000,  1,};
  double radious(double a ,double b,double theta){
      double r=a*b/(std::sqrt(b*b*std::sin(theta)*std::sin(theta)+a*a*std::cos(theta)*std::cos(theta)));
            // double r=a*b/(std::sqrt(a*a*std::cos(theta)*std::cos(theta)+b*b*std::sin(theta)*std::sin(theta)));
      return r;
  };
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> pcl_octree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
 public:
  Joy(double resolution);

};

pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr octree_to_kdtree(const octomap::OcTree& octree) {
    // 创建一个新的PointCloud用来存储从Octree中提取的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
    // 遍历octree
    for(octomap::OcTree::leaf_iterator it = octree.begin_leafs(), end=octree.end_leafs(); it!= end; ++it)
    {
        // 检查体素是否被占据
        if(octree.isNodeOccupied(*it))
        {
            // 转换体素坐标到全局坐标系
            pcl::PointXYZ point(it.getX(), it.getY(), it.getZ());
            cloud->points.push_back(point);
        }
    }

    // 创建一个新的k-d树并设置输入点云
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud);

    return kdtree;
}

pcl::PointXYZ toPclPointXYZ(const geometry_msgs::Point& point) {
    return pcl::PointXYZ(point.x, point.y, point.z);
}

class Circle {
public:
  Circle(double x1, double y1, double x2, double y2)
    : x1_(x1), y1_(y1), x2_(x2), y2_(y2)
  {
    calculateCircle();
  }
  double cx_;      // 圆心的 x 坐标
  double cy_;      // 圆心的 y 坐标
  double radius_;  // 圆的半径
  double angle1_;  // 第一个点对应的角度
  double angle2_;  // 第二个点对应的角度
  double angle_diff_;
  private:
  double x1_;      // 第一个点的 x 坐标
  double y1_;      // 第一个点的 y 坐标
  double x2_;      // 第二个点的 x 坐标
  double y2_;      // 第二个点的 y 坐标

  void calculateCircle() {
    // 检查两个点是否共线
    if (x1_ * y2_ == x2_ * y1_) {
      cx_ = 0.0;
      cy_ = 0.0;
      radius_ = 0.0;
      angle1_ = 0.0;
      angle2_ = 0.0;
      return;
    }

    double m12x=(x1_+x2_)*0.5;
    double m12y=(y1_+y2_)*0.5;
    double m23x=(x2_+0)*0.5;
    double m23y=(y2_+0)*0.5;

    double k12 = -((x2_ - x1_) / (y2_ - y1_));
    double k23 = -((0 - x2_) / (0 - y2_));


  // 计算圆心坐标
    cx_ = (k12 * m12x - k23 * m23x + m23y - m12y) / (k12 - k23);
    cy_ = m12y + k12 * (cx_ - m12x);



    // // 计算圆心的坐标
    // cx_ = (x1_ * y2_ - x2_ * y1_) / (y2_ - y1_);
    // cy_ = (x1_ * y2_ * y1_ - x2_ * y1_ * y1_) / (x1_ * y2_ - x2_ * y1_);

    // 计算圆的半径
    radius_ = std::hypot(cx_, cy_);

    // 计算两个点对应的角度
    angle1_ = std::atan2(y1_ - cy_, x1_ - cx_);
    angle2_ = std::atan2(y2_ - cy_, x2_ - cx_);

    // // 转换为0到2π的范围
    // if (angle1_ < 0) {
    //   angle1_ += 2 * M_PI;
    // }
    // if (angle2_ < 0) {
    //   angle2_ += 2 * M_PI;
    // }
    double angle_diff = angle2_ - angle1_;
    while (angle_diff < -M_PI) {
      angle_diff += 2 * M_PI;
    }
    while (angle_diff >= M_PI) {
      angle_diff -= 2 * M_PI;
    }
    angle_diff_ = angle_diff;


  }
};





#endif // ROTORS_JOY_INTERFACE_JOY_POSITION_H_
