#include "xxh_package.h"

#include "joy_position.h"







Joy::Joy(double resolution):explore_octree(2.0),pcl_octree(resolution),cloud_global((new pcl::PointCloud<pcl::PointXYZ>)),sor(),kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>()){
  ros::NodeHandle nh_;
  ros::NodeHandle nh2_;
  ros::NodeHandle nh3_;
  ros::NodeHandle nh4_;
  //nh-controller
  //nh2-octomap and planning
  //nh3-commander
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  // ctrl_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust> (
  //   "command/roll_pitch_yawrate_thrust", 10);

  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;
  current_yaw_vel_ = 0;

  pnh.param("axis_roll_", axes_.roll, 0);
  pnh.param("axis_pitch_", axes_.pitch, 1);
  pnh.param("axis_thrust_", axes_.thrust, 2);

  pnh.param("axis_direction_roll", axes_.roll_direction, -1);
  pnh.param("axis_direction_pitch", axes_.pitch_direction, -1);
  pnh.param("axis_direction_thrust", axes_.thrust_direction, 1);

  pnh.param("max_v_xy", max_.v_xy, 1.0);  // [m/s]
  pnh.param("max_roll", max_.roll, 10.0 * M_PI / 180.0);  // [rad]
  pnh.param("max_pitch", max_.pitch, 10.0 * M_PI / 180.0);  // [rad]
  pnh.param("max_yaw_rate", max_.rate_yaw, 45.0 * M_PI / 180.0);  // [rad/s]
  pnh.param("max_thrust", max_.thrust, 30.0);  // [N]

  pnh.param("v_yaw_step", v_yaw_step_, 0.05);  // [rad/s]

  pnh.param("is_fixed_wing", is_fixed_wing_, false);

  pnh.param("button_yaw_left_", buttons_.yaw_left, 3);
  pnh.param("button_yaw_right_", buttons_.yaw_right, 4);
  pnh.param("button_ctrl_enable_", buttons_.ctrl_enable, 5);
  pnh.param("button_ctrl_mode_", buttons_.ctrl_mode, 10);
  pnh.param("button_takeoff_", buttons_.takeoff, 7);
  pnh.param("button_land_", buttons_.land, 8);

  namespace_ = nh_.getNamespace();

  ros::CallbackQueue custom_queue1;
  ros::CallbackQueue custom_queue2;
  ros::CallbackQueue custom_queue3;
  nh_.setCallbackQueue(&custom_queue1);
  nh2_.setCallbackQueue(&custom_queue2);
  nh3_.setCallbackQueue(&custom_queue3);

  size.x =115.09100342;
  size.y = 25.26037979;
  size.z = 12.36612701;

  center.x=38.86922455;
  center.y=-0.34078693;
  center.z=55.82170486;

  sor.setLeafSize(5*resolution, 5*resolution, 5*resolution);

  joy_sub_ = nh_.subscribe("joy", 10, &Joy::JoyCallback, this);
  odom_sub_ = nh_.subscribe("/ground_truth/odometry", 10, &Joy::OdomCallback, this);
  gim_sub_=nh_.subscribe("/gimbal",10,&Joy::GimbalCallback,this);
  cmd_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory> (
  "/firefly/command/trajectory", 10);
  timer = nh_.createTimer(ros::Duration(1.0 / 10.0), &Joy::TimerCallback, this);
  
  octomap_sub_=nh2_.subscribe("/cloud_inW",10,&Joy::OctomapCallback,this);

  gim_cmd_pub_ = nh_.advertise<geometry_msgs::Twist> ("/firefly/command/gimbal", 10);
  octomap_pub_ = nh2_.advertise<octomap_msgs::Octomap>("/firefly/octomap", 1);

  timer2 = nh3_.createTimer(ros::Duration(1.0 / 10.0), &Joy::TimerCallback2, this);
  path_pub_=nh3_.advertise<nav_msgs::Path>("/firefly/planned_path", 10);
  ros::AsyncSpinner spinner1(1, &custom_queue1);  // 1 thread for the custom_queue1 // 0 means threads= # of CPU cores
  ros::AsyncSpinner spinner2(1, &custom_queue2);  // 1 thread for the custom_queue2 // 0 means threads= # of CPU cores
  ros::AsyncSpinner spinner3(1, &custom_queue3);  // 1 thread for the custom_queue3 // 0 means threads= # of CPU cores

  spinner1.start();  // start spinner of the custom queue 1
  spinner2.start();  // start spinner of the custom queue 2
  spinner3.start();  // start spinner of the custom queue 3
  ros::waitForShutdown();
}
void Joy::TimerCallback2(const ros::TimerEvent &){
    // generateSpiralPath(size,transformation_matrix);

}
void Joy::TimerCallback(const ros::TimerEvent &)
{
  
  // yolo();
  if (!got_joy_) return;
  // yolo();

  // if ((ros::Time::now()-time_joy).toSec()>0.5)
  // {
  //   // yolo();
  //   yaw_cmd = tf_uav.yaw();
  //   xyz_cmd_world = Eigen::Vector3d(0.0, 0.0, 0.0);
  //   vertical_speed = 0.0;
  // }

  // Eigen::Quaterniond q = Util::YPR2Quat(yaw_cmd, 0.0, 0.0);
  trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
  trajset_msg.header.stamp = ros::Time::now();
  trajset_msg.header.frame_id = "world";
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;
  geometry_msgs::Transform transform_msg;
  geometry_msgs::Twist accel_msg, vel_msg;
  transform_msg.translation.x = tf_uav.pos(0);
  transform_msg.translation.y = tf_uav.pos(1);
  transform_msg.translation.z = tf_uav.pos(2);
  // printf(transform_msg.translation.z);
  // yolo();
  transform_msg.rotation.x = 0.0;
  transform_msg.rotation.y = 0.0;
  transform_msg.rotation.z = sinf(yaw_cmd/180.0*M_PI*0.5);
  transform_msg.rotation.w = cosf(yaw_cmd/180.0*M_PI*0.5);
  trajpt_msg.transforms.push_back(transform_msg);
  vel_msg.linear.x = xyz_cmd_world(0);
  vel_msg.linear.y = xyz_cmd_world(1);
  vel_msg.linear.z = vertical_speed;
  accel_msg.linear.x = 0.0;
  accel_msg.linear.x = 0.0;
  accel_msg.linear.x = 0.0;
  trajpt_msg.velocities.push_back(vel_msg);
  trajpt_msg.accelerations.push_back(accel_msg);
  trajset_msg.points.push_back(trajpt_msg);
  cmd_pub_.publish(trajset_msg);
  // yolo();
}

void Joy::StopMav() {
  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;
}

void Joy::OdomCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_ = *msg;
}

void Joy::GimbalCallback(const geometry_msgs::TwistStamped& msg) {
  
}
void Joy::OctomapCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  // to pcl
    // yolo();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    for (const auto& point : cloud->points)
    {
        octomap::point3d endpoint(point.x, point.y, point.z);
        explore_octree.updateNode(endpoint, true);
    }
    explore_octree.updateInnerOccupancy();
    octomap_msgs::Octomap octomap_msg;
    octomap_msgs::fullMapToMsg(explore_octree, octomap_msg);
    if(cloud->size()>0)
    {sor.setInputCloud(cloud);
      sor.filter(*cloud);
    }
    if(cloud->size()>0){
      *cloud_global+=*cloud;
    }
    if(cloud->size()>0)
    {sor.setInputCloud(cloud_global);
      sor.filter(*cloud_global);
    }
    octomap_msg.header.frame_id="world";
    octomap_pub_.publish(octomap_msg);
}

void Joy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  // yolo();
  current_joy_ = *msg;
  time_joy = ros::Time::now();
  myTf<double>tf_new(odom_);
  if (!got_joy_) tf_uav = tf_new;
  // yolo();
  control_msg_.roll = msg->axes[axes_.roll] * max_.roll * axes_.roll_direction;
  control_msg_.pitch = msg->axes[axes_.pitch] * max_.pitch * axes_.pitch_direction;
  current_yaw_vel_ = 0;
  if (msg->axes[3]>0.02) {
    current_yaw_vel_ = max_.rate_yaw;
    tf_uav.rot = Util::YPR2Rot(tf_new.yaw(), 0.0, 0.0);
  }
  else if (msg->axes[3]<-0.02) {
    current_yaw_vel_ = -max_.rate_yaw;
    tf_uav.rot = Util::YPR2Rot(tf_new.yaw(), 0.0, 0.0);
  }
  else {
    current_yaw_vel_ = 0;
  }
  control_msg_.yaw_rate = current_yaw_vel_;
  yaw_cmd = tf_uav.yaw()/180.0*M_PI + current_yaw_vel_*0.3;
  yaw_cmd = Util::wrapTo180(yaw_cmd/M_PI*180.0);

  if (fabs(control_msg_.pitch)>0.01 || fabs(control_msg_.roll)>0.01)
  {
    tf_uav.pos(0) = tf_new.pos(0);
    tf_uav.pos(1) = tf_new.pos(1);
  }
     
  Eigen::Vector3d xyz_cmd_body(0.0, 0.0, 0.0);
  xyz_cmd_body(0) = control_msg_.pitch *5.0;
  xyz_cmd_body(1) = -control_msg_.roll * 5.0;

  control_msg_.thrust.z = (msg->axes[axes_.thrust] + 1) * max_.thrust / 2.0 * axes_.thrust_direction;
  vertical_speed = -3.0*msg->axes[axes_.thrust]* axes_.thrust_direction;

  if (fabs(vertical_speed)>0.1) tf_uav.pos(2) = tf_new.pos(2);

  // Eigen::Quaterniond q = YPR2Quat(yaw_cmd, 0.0, 0.0);
  xyz_cmd_world = Util::YPR2Rot(tf_uav.yaw(), 0.0, 0.0)*xyz_cmd_body;
  got_joy_ = true;

  geometry_msgs::Twist gimmsg;
  gimmsg.linear.x = -1;
  gimmsg.linear.y = 0.0;
  gimmsg.linear.z = 0.0;
  gimmsg.angular.x = 0.0;
  gimmsg.angular.y = msg->axes[4];
  gimmsg.angular.z = msg->axes[5]; 
  gim_cmd_pub_.publish(gimmsg);
}

void Joy::Publish() {
  //ctrl_pub_.publish(control_msg_);
}

void Joy::generate_voxel_path(const geometry_msgs::Vector3& size, const std::vector<double>& transformation_matrix) {

    geometry_msgs::Point my_position=odom_.pose.pose.position;
    tf2::Matrix3x3 rot_mat;
    rot_mat.setValue(transformation_matrix[0], transformation_matrix[1], transformation_matrix[2],
                     transformation_matrix[4], transformation_matrix[5], transformation_matrix[6],
                     transformation_matrix[8], transformation_matrix[9], transformation_matrix[10]);
    tf2::Vector3 translation_vec(transformation_matrix[3], transformation_matrix[7], transformation_matrix[11]);

    

    
    



}

void Joy::generateSpiralPath(const geometry_msgs::Vector3& size, const std::vector<double>& transformation_matrix) {
    // 解析输入参数
    double length = size.x+2;
    double width = size.y+2;
    double height = size.z+2;
    if(cloud_global->size()>0)
    {
      // printf("global size: %ld",cloud_global->size());
      // yolo();
       kdtree->setInputCloud(cloud_global);
    }
    // 提取旋转矩阵和平移向量
    tf2::Matrix3x3 rot_mat;
    rot_mat.setValue(transformation_matrix[0], transformation_matrix[1], transformation_matrix[2],
                     transformation_matrix[4], transformation_matrix[5], transformation_matrix[6],
                     transformation_matrix[8], transformation_matrix[9], transformation_matrix[10]);
    tf2::Vector3 translation_vec(transformation_matrix[3], transformation_matrix[7], transformation_matrix[11]);

    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::Point my_position=odom_.pose.pose.position;
    tf2::Vector3 my_position_vec(my_position.x,my_position.y,my_position.z);
    start_pose.pose.position=my_position;
    // tf2::Vector3 my_position_vec=odom_.pose;
    tf2::Vector3 my_position_in_bound=(rot_mat.inverse())*(my_position_vec-translation_vec);
    double begin_angle= std::atan2(my_position_in_bound[1],my_position_in_bound[0]);
    double begin_z=my_position_in_bound[2];

    double x_begin_angle_tra=(std::cos(begin_angle)*radious(width/2,length/2,begin_angle));
    double bow_end_angle;
    // if(std::hypot(my_position_in_bound[1],my_position_in_bound[0])>radious(width/2,length/2,begin_angle))
    //     bow_end_angle=begin_angle;
    // else{
    //     bow_end_angle=std::atan2(my_position_in_bound[1],2*my_position_in_bound[0]-x_begin_angle_tra);
    // }

    
    
    // // double inside=1;
    // // if()

    // if (bow_end_angle<0){
    //   bow_end_angle=bow_end_angle+2*M_PI;
    // }
    // if(begin_angle<0){
    //   begin_angle=begin_angle+2*M_PI;
    // }
    // while(true){
    //     if(begin_angle<=bow_end_angle){break;}else{
    //       bow_end_angle+=2*M_PI;
    //     }
    // }
    
    // if(abs(bow_end_angle-begin_angle)<0.5)
    // {
    //     bow_end_angle=begin_angle+0.5;
    // }

    bow_end_angle=begin_angle+0.5;
    double bow_end_x=(std::cos(bow_end_angle)*radious(width/2,length/2,bow_end_angle));
    double bow_end_y=(std::sin(bow_end_angle)*radious(width/2,length/2,bow_end_angle));
    Circle circle(my_position_in_bound[0],my_position_in_bound[1],bow_end_x,bow_end_y);


    // 生成螺旋路径
    double step_size_abs = 0.01;  // 步长 z 
    double num_steps=std::abs(height/2-begin_z)/step_size_abs;
    // double num_steps = (height-begin_z) / step_size;  // z the longest
    double step_size= (height/2-begin_z) / num_steps; 
    // double radius=std::sqrt(width*width+length*length)/2; 
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world";  // 设置坐标系
    path_msg.poses.resize((int)(num_steps + 2));
    double angleuar_speed=0.01;
    double bow_steps=std::floor(std::abs(bow_end_angle-begin_angle)/angleuar_speed);
    double bow_angular_speed=circle.angle_diff_/bow_steps;

    // printf("Size: %d \n",path_msg.poses.size());
    tf2::Quaternion start_quat;
    rot_mat.getRotation(start_quat);
    start_pose.pose.orientation = tf2::toMsg(start_quat);
    path_msg.poses[0]=(start_pose);
    static TicToc tt_time_out;
    // yolo();
    #pragma omp parallel for num_threads(omp_get_max_threads())
    for (int i = 0; i < num_steps; i++) {
      double x,y;
      if(i>bow_steps){
        x = (std::cos(i*0.01+begin_angle)*radious(width/2,length/2,0.01*i+begin_angle));
        y = (std::sin(i*0.01+begin_angle)*radious(width/2,length/2,0.01*i+begin_angle));
      }else{
        x =(std::cos(i*bow_angular_speed+circle.angle1_)*circle.radius_)+circle.cx_;
        y =(std::sin(i*bow_angular_speed+circle.angle1_)*circle.radius_)+circle.cy_;
      }
        double z = begin_z+ (i * step_size);
        tf2::Vector3 path_no_rotation(x,y,z);
        tf2::Vector3 path_point_after=rot_mat*path_no_rotation+translation_vec;
        // 添加路径点
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = path_point_after[0];
        pose.pose.position.y = path_point_after[1];
        pose.pose.position.z = path_point_after[2];
        // pose.pose.position.x = x;
        // pose.pose.position.y = y;
        // pose.pose.position.z = z;
        if(i<10)
        {
          // revise_path_point(pose.pose.position);
        }
        tf2::Quaternion quat;
        rot_mat.getRotation(quat);
        pose.pose.orientation = tf2::toMsg(quat);

        path_msg.poses[i+1] = pose;

    }
    // printf("time %f,\n",tt_time_out.Toc());
    path_pub_.publish(path_msg);
};
void Joy::revise_path_point(geometry_msgs::Point& searchPointMsg){

    pcl::PointXYZ searchPoint;
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    while(true){
    searchPoint.x = searchPointMsg.x;
    searchPoint.y = searchPointMsg.y;
    searchPoint.z = searchPointMsg.z;

    // printf("cloud global kd size: %d \n",cloud_global->size());
    if(cloud_global->size()>0){
      if (kdtree->nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {

          pcl::PointXYZ nearestPoint = cloud_global->points[pointIdxNKNSearch[0]];
          // yolo();
          Eigen::Vector3f vec1(searchPoint.x, searchPoint.y, searchPoint.z);
          Eigen::Vector3f vec2(nearestPoint.x, nearestPoint.y, nearestPoint.z);
          Eigen::Vector3f vec3(center.x,center.y,center.z);
          double distance_12=(vec1 - vec2).norm();
          double distance_23=(vec2 - vec3).norm();
          double distance_13=(vec1 - vec3).norm();
          
          double safe_dis=3;



          if(distance_13>distance_23){
            searchPointMsg.x=-(vec2[0]-vec1[0])/distance_12*safe_dis+vec2[0];
            searchPointMsg.y=-(vec2[1]-vec1[1])/distance_12*safe_dis+vec2[1];
            searchPointMsg.z=-(vec2[2]-vec1[2])/distance_12*safe_dis+vec2[2];
            return;
          }
          else{
            searchPointMsg.x=(vec2[0]-vec1[0])/distance_12*safe_dis+vec2[0];
            searchPointMsg.y=(vec2[1]-vec1[1])/distance_12*safe_dis+vec2[1];
            searchPointMsg.z=(vec2[2]-vec1[2])/distance_12*safe_dis+vec2[2];
          }
      } 
    }else{
      return;
    }
    }
  return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xxh_package");
    // ros::NodeHandle nh;
    // ros::NodeHandle pnh("~");
    


    Joy joy(0.1);
    ros::waitForShutdown();
    ros::spin();

    return 0;
}