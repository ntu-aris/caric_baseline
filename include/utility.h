/**
* This file is part of CARIC_MISSION.
* 
* Copyright (C) 2020 Thien-Minh Nguyen <thienminh.nguyen at ntu dot edu dot sg>,
* School of EEE
* Nanyang Technological Univertsity, Singapore
* 
* For more information please see <https://britsknguyen.github.io>.
* or <https://github.com/britsknguyen/CARIC_MISSION>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* CARIC_MISSION is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* CARIC_MISSION is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with CARIC_MISSION.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by Thien-Minh Nguyen on 15/12/20.
//

#pragma once

#ifndef _Util_CARIC_MISSION_MANAGER_H_
#define _Util_CARIC_MISSION_MANAGER_H_

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <random>
#include <mutex>

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// #include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/impl/search.hpp>
// #include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/common/common.h>
// #include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/filters/filter.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "glob.h"

// #include <ufo/math/vector3.h>
// #include <ufo/map/point_cloud.h>
// #include <ufo/map/surfel_map.h>

// #include <sophus/se3.hpp>

// Ceres
// #include <ceres/ceres.h>

using namespace std;
using namespace Eigen;

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"

#define yolo() printf("Hello line: %s:%d. \n", __FILE__ , __LINE__);
#define yolos(...) printf("Hello line: %s:%d. ", __FILE__, __LINE__); printf(__VA_ARGS__); std::cout << std::endl;
#define MAX_THREADS std::thread::hardware_concurrency()

/* #region  Custom point type definition --------------------------------------------------------*/

struct PointOuster
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t  ring;
    // uint16_t ambient; // Available in NTU VIRAL and multicampus datasets
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointOuster,
                                 (float, x, x) (float, y, y) (float, z, z)
                                 (float, intensity, intensity)
                                 (uint32_t, t, t)
                                 (uint16_t, reflectivity, reflectivity)
                                 (uint8_t,  ring, ring)
                                //  (uint16_t, ambient, ambient)
                                 (uint32_t, range, range))

struct PointVelodyne
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointVelodyne,
                                  (float, x, x) (float, y, y) (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (float, time, time))

struct PointHesai
{
    PCL_ADD_POINT4D
    float intensity;
    double timestamp;
    uint16_t ring;                   ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointHesai,
                                 (float, x, x) (float, y, y) (float, z, z)
                                 (float, intensity, intensity)
                                 (double, timestamp, timestamp)
                                 (uint16_t, ring, ring))

struct PointTQXYZI 
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;              // preferred way of adding a XYZ+padding
    double t;
    float  qx;
    float  qy;
    float  qz;
    float  qw;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment
POINT_CLOUD_REGISTER_POINT_STRUCT(PointTQXYZI,
                                 (float,  x, x) (float,  y, y) (float,  z, z)
                                 (float,  intensity, intensity)
                                 (double, t,  t)
                                 (float,  qx, qx)
                                 (float,  qy, qy)
                                 (float,  qz, qz)
                                 (float,  qw, qw))
struct PointOdom
{
    PCL_ADD_POINT4D                 // X Y Z position
    PCL_ADD_INTENSITY;              // preferred way of adding a XYZ+padding
    double t;
    float  qx;                      // Quaternion x
    float  qy;                      // Quaternion y
    float  qz;                      // Quaternion z
    float  qw;                      // Quaternion w
    float  vx;                      // Linear Velocity x
    float  vy;                      // Linear Velocity y
    float  vz;                      // Linear Velocity z
    float  wx;                      // Angular Velocity x
    float  wy;                      // Angular Velocity y
    float  wz;                      // Angular Velocity z
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment
POINT_CLOUD_REGISTER_POINT_STRUCT(PointOdom,
                                 (float,  x, x) (float,  y, y) (float,  z, z)
                                 (float,  intensity, intensity)
                                 (double, t,  t)
                                 (float,  qx, qx)
                                 (float,  qy, qy)
                                 (float,  qz, qz)
                                 (float,  qw, qw)
                                 (float,  vx, vx)
                                 (float,  vy, vy)
                                 (float,  vz, vz)
                                 (float,  wx, wx)
                                 (float,  wy, wy)
                                 (float,  wz, wz))

struct PointXYZIT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    double t;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
                                 (float, x, x) (float, y, y) (float, z, z)
                                 (float, intensity, intensity) (double, t, t))

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointXYZI PointXYZI;
typedef PointTQXYZI PointPose;

typedef pcl::PointCloud<PointXYZ> CloudXYZ;
typedef pcl::PointCloud<PointXYZI> CloudXYZI;
typedef pcl::PointCloud<PointXYZIT> CloudXYZIT;
typedef pcl::PointCloud<PointPose> CloudPose;
typedef pcl::PointCloud<PointOdom> CloudOdom;
typedef pcl::PointCloud<PointOuster> CloudOuster;
typedef pcl::PointCloud<PointVelodyne> CloudVelodyne;

typedef pcl::PointCloud<PointXYZ>::Ptr CloudXYZPtr;
typedef pcl::PointCloud<PointXYZI>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointXYZI>::Ptr CloudXYZIPtr;
typedef pcl::PointCloud<PointXYZIT>::Ptr CloudXYZITPtr;
typedef pcl::PointCloud<PointPose>::Ptr CloudPosePtr;
typedef pcl::PointCloud<PointOdom>::Ptr CloudOdomPtr;
typedef pcl::PointCloud<PointOuster>::Ptr CloudOusterPtr;
typedef pcl::PointCloud<PointVelodyne>::Ptr CloudVelodynePtr;

typedef pcl::KdTreeFLANN<PointXYZI> KdFLANN;
typedef pcl::KdTreeFLANN<PointXYZI>::Ptr KdFLANNPtr;

/* #endregion  Custom point type definition -----------------------------------------------------*/


/* #region  Image pointer shortened name -------*/

typedef sensor_msgs::Image::Ptr RosImgPtr;
typedef sensor_msgs::Image::ConstPtr RosImgConstPtr;
// typedef map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> VisFeatureFrame;
// typedef pair<double, VisFeatureFrame> StampedVisFeatureFrame;

/* #endregion Image pointer shortened name -----*/

// Shortened typedef matching character length of Vector3d and Matrix3d
typedef Eigen::Quaterniond Quaternd;
typedef Eigen::Quaterniond Quaternf;

template <typename T>
void pointConverter(Eigen::MatrixXd &base2LidarTf, T &point)
{
    Eigen::Vector3d p_lidar(point.x, point.y, point.z);
    Eigen::Vector3d p_baselink = base2LidarTf.block<3, 3>(0, 0) * p_lidar + base2LidarTf.block<3, 1>(0, 3);

    point.x = p_baselink.x();
    point.y = p_baselink.y();
    point.z = p_baselink.z();
}

class TicToc
{
public:
    TicToc()
    {
        Tic();
    }

    void Tic()
    {
        start_ = std::chrono::system_clock::now();
    }

    double Toc()
    {
        end_ = std::chrono::system_clock::now();
        elapsed_seconds_ = end_ - start_;
        return elapsed_seconds_.count() * 1000;
    }

    double GetLastStop()
    {
        return elapsed_seconds_.count() * 1000;
    }

#define LASTSTOP(x) printf(#x".LastStop : %f\n", x.GetLastStop());
#define TOCPRINT(x) printf(#x".Toc : %f\n", x.Toc());

private:
    std::chrono::time_point<std::chrono::system_clock> start_, end_;
    std::chrono::duration<double> elapsed_seconds_;
};

template <typename T = double>
struct myTf
{
    Eigen::Quaternion<T>   rot;
    Eigen::Matrix<T, 3, 1> pos;

    myTf Identity()
    {
        return myTf();
    }
    
    myTf(const myTf<T> &other)
    {
        this->rot = other.rot;
        this->pos = other.pos;
    }

    myTf()
    {
        this->rot = Quaternd(1, 0, 0, 0);
        this->pos = Vector3d(0, 0, 0);
    }

    myTf(Eigen::Quaternion<T> rot_in, Eigen::Matrix<T, 3, 1> pos_in)
    {
        this->rot = rot_in;
        this->pos = pos_in;
    }

    myTf(Eigen::Matrix<T, 3, 3> rot_in, Eigen::Matrix<T, 3, 1> pos_in)
    {
        this->rot = Quaternion<T>(rot_in);
        this->pos = pos_in;
    }

    template <typename Tin>
    myTf(Eigen::Matrix<Tin, 4, 4> tfMat)
    {
        Eigen::Matrix<T, 3, 3> M = tfMat.block(0, 0, 3, 3).template cast<T>();
        this->rot = Quaternion<T>(M);
        this->pos = tfMat.block(0, 3, 3, 1).template cast<T>();
    }

    myTf(PointPose point)
    {
        this->rot = Quaternion<T>(point.qw, point.qx, point.qy, point.qz);
        this->pos << point.x, point.y, point.z;
    }

    myTf(nav_msgs::Odometry odom)
    {
        this->rot = Quaternion<T>(odom.pose.pose.orientation.w,
                                  odom.pose.pose.orientation.x,
                                  odom.pose.pose.orientation.y,
                                  odom.pose.pose.orientation.z);
                                    
        this->pos << odom.pose.pose.position.x,
                     odom.pose.pose.position.y,
                     odom.pose.pose.position.z;
    }

    myTf(Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform)
    {
        this->rot = Eigen::Quaternion<T>{transform.linear()}.normalized();
        this->pos = transform.translation();
    }

    myTf interpolate(T s)
    {
        // Interpolate this transform from the identity to the dataset
        return myTf<T>(Eigen::Quaternion<T>::Identity().slerp(s, this->rot), s * this->pos);
    }

    myTf interpolate(T s, myTf tf_final)
    {
        // Interpolate this transform from this transform to the target
        return myTf<T>(this->rot.slerp(s, tf_final.rot), (1-s)*this->pos + s*tf_final.pos);
    }

    // template <typename Tin>
    // myTf(Sophus::SE3<Tin> se3)
    // {
    //     this->rot = se3.so3().unit_quaternion();
    //     this->pos = se3.translation();
    // }

    Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform() const
    {
        Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform;
        transform.linear() = rot.normalized().toRotationMatrix();
        transform.translation() = pos;
        return transform;
    }

    Eigen::Matrix<T, 4, 4> tfMat() const
    {
        Eigen::Matrix<T, 4, 4> M = Matrix<T, 4, 4>::Identity();
        M.block(0, 0, 3, 3) = rot.normalized().toRotationMatrix();
        M.block(0, 3, 3, 1) = pos;
        return M;
    }

    PointXYZI Point3D() const
    {
        PointXYZI p;

        p.x = (float)pos.x();
        p.y = (float)pos.y();
        p.z = (float)pos.z();

        p.intensity = -1;

        return p;
    }
    
    PointPose Pose6D() const
    {
        PointPose p;

        p.t = -1;

        p.x = (float)pos.x();
        p.y = (float)pos.y();
        p.z = (float)pos.z();

        p.qx = (float)rot.x();
        p.qy = (float)rot.y();
        p.qz = (float)rot.z();
        p.qw = (float)rot.w();

        p.intensity = -1;

        return p;
    }

    PointPose Pose6D(double time) const
    {
        PointPose p;

        p.t = time;
        
        p.x = (float)pos.x();
        p.y = (float)pos.y();
        p.z = (float)pos.z();

        p.qx = (float)rot.x();
        p.qy = (float)rot.y();
        p.qz = (float)rot.z();
        p.qw = (float)rot.w();

        p.intensity = -1;

        return p;
    }

    // template <typename Tout = double>
    // Sophus::SE3<Tout> getSE3() const
    // {
    //     return Sophus::SE3<Tout>(this->rot.template cast<Tout>(),
    //                              this->pos.template cast<Tout>());
    // }

    double roll() const
    {
        return atan2(rot.x()*rot.w() + rot.y()*rot.z(), 0.5 - (rot.x()*rot.x() + rot.y()*rot.y()))/M_PI*180.0;
    }

    double pitch() const
    {
        return asin(-2*(rot.x()*rot.z() - rot.w()*rot.y()))/M_PI*180.0;
    }

    double yaw() const
    {
        return atan2(rot.x()*rot.y() + rot.w()*rot.z(), 0.5 - (rot.y()*rot.y() + rot.z()*rot.z()))/M_PI*180.0;
    }

    myTf inverse() const
    {
        Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform_inv = this->transform().inverse();
        myTf tf_inv;
        tf_inv.rot = transform_inv.linear();
        tf_inv.pos = transform_inv.translation();
        return tf_inv;
    }

    myTf operator*(const myTf &other) const
    {
        Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform_out = this->transform() * other.transform();
        return myTf(transform_out);
    }

    Vector3d operator*(const Vector3d &v) const
    {
        return (rot*v + pos);
    }

    template <typename NewType>
    myTf<NewType> cast() const
    {
        myTf<NewType> tf_new{this->rot.template cast<NewType>(), this->pos.template cast<NewType>()};
        return tf_new;
    }

    friend std::ostream &operator<<(std::ostream &os, const myTf &tf)
    {
        os << tf.pos.x() << " " << tf.pos.y() << " " << tf.pos.z() << " " << tf.rot.w() << " "
           << tf.rot.x() << " " << tf.rot.y() << " " << tf.rot.z();
        return os;
    }
}; // class myTf

typedef myTf<> mytf;

namespace Util
{
    template <typename PointType>
    sensor_msgs::PointCloud2 publishCloud(ros::Publisher &thisPub,
                                          pcl::PointCloud<PointType> &thisCloud,
                                          ros::Time thisStamp, std::string thisFrame)
    {
        sensor_msgs::PointCloud2 tempCloud;
        pcl::toROSMsg(thisCloud, tempCloud);
        tempCloud.header.stamp = thisStamp;
        tempCloud.header.frame_id = thisFrame;
        // if (thisPub.getNumSubscribers() != 0)
            thisPub.publish(tempCloud);
        return tempCloud;
    }

    float wrapTo360(float angle)
    {
        angle = fmod(angle, 360);
        if (angle < 0)
            angle += 360;
        return angle;
    }

    double wrapTo180(double angle)
    {
        angle = fmod(angle + 180,360);
        if (angle < 0)
            angle += 360;
        return angle - 180;
    }

    template <typename PoinT>
    float pointDistance(PoinT p)
    {
        return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    template <typename Derived>
    typename Derived::Scalar angleDiff(const Eigen::QuaternionBase<Derived> &q1, const Eigen::QuaternionBase<Derived> &q2)
    {
        return (Eigen::AngleAxis<typename Derived::Scalar>(q1.inverse()*q2).angle()*180.0/M_PI);
    }

    template <typename Derived>
    Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;

        Scalar_t theta_nrm = theta.norm();

        if (theta_nrm < 1e-5)
        {
            Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
            half_theta /= static_cast<Scalar_t>(2.0);
            dq.w() = static_cast<Scalar_t>(1.0);
            dq.x() = half_theta.x();
            dq.y() = half_theta.y();
            dq.z() = half_theta.z();
        }
        else
        {
            Scalar_t costheta = cos(theta_nrm / 2);
            Scalar_t sintheta = sin(theta_nrm / 2);
            Eigen::Matrix<Scalar_t, 3, 1> quat_vec = theta / theta_nrm * sintheta;

            dq.w() = costheta;
            dq.vec() = quat_vec;
        }

        // printf("dq: %f, %f, %f, %f. norm: %f\n", dq.x(), dq.y(), dq.z(), dq.w(), dq.norm());

        return dq;
    }

    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        ans << typename Derived::Scalar(0), -q(2), q(1),
               q(2), typename Derived::Scalar(0), -q(0),
              -q(1), q(0), typename Derived::Scalar(0);
        return ans;
    }

    template <typename Derived>
    Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
    {
        //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
        //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
        //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
        //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
        return q;
    }

    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
    {
        Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
        ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
        return ans;
    }

    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
    {
        Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
        ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
        return ans;
    }

    Eigen::Matrix3d SO3Jright(const Eigen::AngleAxis<double> &phi)
    {
        Eigen::Matrix3d ans;
        double theta = phi.angle();
        Eigen::Matrix3d ux = skewSymmetric(phi.axis());

        if (theta == 0.0)
        {
            ans = Eigen::Matrix3d::Identity();
        }
        else
        {
            ans = Eigen::Matrix3d::Identity() - (1 - cos(theta)) / theta * ux + (theta - sin(theta)) / theta * ux * ux;
        }

        return ans;
    }

    Eigen::Matrix3d SO3JrightInv(const Eigen::AngleAxis<double> &phi)
    {
        Eigen::Matrix3d ans;
        double theta = phi.angle();
        Eigen::Matrix3d ux = skewSymmetric(phi.axis());

        if (theta == 0.0)
        {
            ans = Eigen::Matrix3d::Identity();
        }
        else
        {
            ans = Eigen::Matrix3d::Identity() + 0.5 * theta * ux + (1 - theta * (1 + cos(theta)) / 2 / sin(theta)) * ux * ux;
        }

        return ans;
    }

    Eigen::Matrix3d SO3Jleft(const Eigen::AngleAxis<double> &phi)
    {
        Eigen::Matrix3d ans;
        double theta = phi.angle();
        Eigen::Matrix3d ux = skewSymmetric(phi.axis());

        if (theta == 0.0)
        {
            ans = Eigen::Matrix3d::Identity();
        }
        else
        {
            ans = Eigen::Matrix3d::Identity() + (1 - cos(theta)) / theta * ux + (theta - sin(theta)) / theta * ux * ux;
        }

        return ans;
    }

    Eigen::Matrix3d SO3JleftInv(const Eigen::AngleAxis<double> &phi)
    {
        Eigen::Matrix3d ans;
        double theta = phi.angle();
        Eigen::Matrix3d ux = skewSymmetric(phi.axis());

        if (theta == 0.0)
        {
            ans = Eigen::Matrix3d::Identity();
        }
        else
        {
            ans = Eigen::Matrix3d::Identity() - 0.5 * theta * ux + (1 - theta * (1 + cos(theta)) / 2 / sin(theta)) * ux * ux;
        }

        return ans;
    }

    Eigen::Matrix3d YPR2Rot(Eigen::Vector3d ypr)
    {
        double y = ypr(0) / 180.0 * M_PI;
        double p = ypr(1) / 180.0 * M_PI;
        double r = ypr(2) / 180.0 * M_PI;

        Eigen::Matrix3d Rz;
        Rz << cos(y), -sin(y), 0.0,
              sin(y),  cos(y), 0.0,
              0.0,     0.0,    1;

        Eigen::Matrix3d Ry;
        Ry << cos(p), 0.0, sin(p),
              0.0,    1.0, 0.0,
             -sin(p), 0.0, cos(p);

        Eigen::Matrix3d Rx;
        Rx << 1.0, 0.0,     0.0,
              0.0, cos(r), -sin(r),
              0.0, sin(r),  cos(r);

        return Rz * Ry * Rx;
    }

    Eigen::Matrix3d YPR2Rot(double yaw, double pitch, double roll)
    {
        double y = yaw / 180.0 * M_PI;
        double p = pitch / 180.0 * M_PI;
        double r = roll / 180.0 * M_PI;

        Eigen::Matrix3d Rz;
        Rz << cos(y), -sin(y), 0.0,
              sin(y),  cos(y), 0.0,
              0.0,     0.0,    1;

        Eigen::Matrix3d Ry;
        Ry << cos(p), 0.0, sin(p),
              0.0,    1.0, 0.0,
             -sin(p), 0.0, cos(p);

        Eigen::Matrix3d Rx;
        Rx << 1.0, 0.0,     0.0,
              0.0, cos(r), -sin(r),
              0.0, sin(r),  cos(r);

        return Rz * Ry * Rx;
    }

    Eigen::Vector3d Rot2YPR(const Eigen::Matrix3d R)
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }

    Eigen::Vector3d Quat2YPR(const Eigen::Quaterniond Q)
    {
        return Rot2YPR(Q.toRotationMatrix());
    }

    Eigen::Vector3d Quat2YPR(double qx, double qy, double qz, double qw)
    {
        return Rot2YPR(Quaternd(qw, qx, qy, qz).toRotationMatrix());
    }

    Eigen::Quaterniond YPR2Quat(Eigen::Vector3d ypr)
    {
        return Quaterniond(YPR2Rot(ypr));
    }

    Eigen::Quaterniond YPR2Quat(double y, double p, double r)
    {
        return Quaterniond(YPR2Rot(y, p, r));
    }

    Vector3d transform_pointVec(const mytf &tf, const PointXYZI &pi)
    {
        Vector3d bodyPoint = tf.rot * Vector3d(pi.x, pi.y, pi.z) + tf.pos;
        return bodyPoint;
    }

    template <typename PointT>
    PointT transform_point(const mytf &tf, const PointT &pi)
    {
        Vector3d pos = tf.rot * Vector3d(pi.x, pi.y, pi.z) + tf.pos;
        
        PointT po = pi;
        po.x = pos.x();
        po.y = pos.y();
        po.z = pos.z();

        return po;
    }

    PointPose transform_point(const mytf &tf, const PointPose &pi)
    {
        Vector3d pos = tf.rot * Vector3d(pi.x, pi.y, pi.z) + tf.pos;
        Quaternd rot = tf.rot * Quaternd(pi.qw, pi.qx, pi.qy, pi.qz);
        
        PointPose po;
        po.x  = pos.x();
        po.y  = pos.y();
        po.z  = pos.z();
        po.qx = rot.x();
        po.qy = rot.y();
        po.qz = rot.z();
        po.qw = rot.w();
        po.t  = pi.t;
        po.intensity = pi.intensity;

        return po;
    }

    PointXYZI Extract3DFrom6D(const PointPose &p6D)
    {
        PointXYZI p3D;

        p3D.x = p6D.x;
        p3D.y = p6D.y;
        p3D.z = p6D.z;
        p3D.intensity = p6D.intensity;

        return p3D;
    }

    template <typename PointT>
    bool PointIsValid(const PointT &p)
    {
        return (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)
                && !std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z));
    }

    template <size_t N>
    struct uint_
    {
    };

    template <size_t N, typename Lambda, typename IterT>
    void unroller(const Lambda &f, const IterT &iter, uint_<N>)
    {
        unroller(f, iter, uint_<N - 1>());
        f(iter + N);
    }

    template <typename Lambda, typename IterT>
    void unroller(const Lambda &f, const IterT &iter, uint_<0>)
    {
        f(iter);
    }

    template <typename T>
    T normalizeAngle(const T& angle_degrees) {
      T two_pi(2.0 * 180);
      if (angle_degrees > 0)
      return angle_degrees -
          two_pi * std::floor((angle_degrees + T(180)) / two_pi);
      else
        return angle_degrees +
            two_pi * std::floor((-angle_degrees + T(180)) / two_pi);
    };

}; // namespace Util

#define _CRT_NO_VA_START_VALIDATION
std::string myprintf(const std::string& format, ...)
{
    va_list args;
    va_start(args, format);
    size_t len = std::vsnprintf(NULL, 0, format.c_str(), args);
    va_end(args);
    std::vector<char> vec(len + 1);
    va_start(args, format);
    std::vsnprintf(&vec[0], len + 1, format.c_str(), args);
    va_end(args);
    
    return string(vec.begin(), vec.end() - 1);
}

#endif