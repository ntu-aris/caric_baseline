#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include "Eigen/Dense"

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

class map_cube
{
public:
    map_cube()
    {
        center = Eigen::Vector3d(0, 0, 0);
    };
    ~map_cube(){};
    void set_center(double x, double y, double z) { center = Eigen::Vector3d(x, y, z); };
    void set_center(const Eigen::Vector3d &origin, double x, double y, double z) { center = Eigen::Vector3d(x, y, z) + origin; };
    void get_search_target(){};
    void insert_target(const Eigen::Vector3d &target)
    {
        if (isInTargetVec(target))
        {
            return;
        }
        else
        {
            search_target_vec.push_back(target);
            return;
        }
    }
    bool isEqual(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
    {
        return v1.isApprox(v2);
    }
    bool isInTargetVec(const Eigen::Vector3d &v1)
    {
        auto it = std::find_if(search_target_vec.begin(), search_target_vec.end(), [&](const Eigen::Vector3d &v)
                               { return isEqual(v, v1); });
        if (it != search_target_vec.end())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void set_interest(bool interest_in) { interest = interest_in; };
    void set_visit(bool visit_in)
    {
        visit = visit_in;
        if (interest == true)
        {
            used_interest = true;
        }
        interest = false;
    };
    void set_Nbr(bool Nbr) { Nbr_in = Nbr; };
    void set_occupied(bool occupied_in) { occupied = occupied_in; };
    bool is_interest() { return interest; };
    bool is_occupied() { return occupied; };
    bool is_visit() { return visit; };
    bool is_used_interest() { return used_interest; };
    bool is_Nbr_in() { return Nbr_in; };
    void reset_Nbr() { Nbr_in = false; };

private:
    // search target vec;
    std::vector<Eigen::Vector3d> search_target_vec;
    // cube center
    Eigen::Vector3d center;
    // cube occupied
    bool occupied = false;
    // cube visit
    bool visit = false;
    // cube interest
    bool interest = false;
    bool used_interest = false;
    bool Nbr_in = false;
};

class global_map
{
public:
    global_map(int dimx, int dimy, int dimz, Eigen::Vector3d &origin, double safed) : origin_point(origin), map(dimx, std::vector<std::vector<map_cube>>(dimy, std::vector<map_cube>(dimz)))
    {
        xlim = dimx;
        ylim = dimy;
        zlim = dimz;
        safe_distance = safed;
        origin_point = origin;
        for (int i = 0; i < dimx; i++)
        {
            for (int j = 0; j < dimy; j++)
            {
                for (int k = 0; k < dimz; k++)
                {
                    map[i][j][k].set_center(origin, (i + 0.5) * safe_distance, (j + 0.5) * safe_distance, (k + 0.5) * safe_distance);
                }
            }
        }
    };

    void clean_last_Nbr()
    {
        for (const auto &vector : Nbr_last)
        {
            map[vector(0)][vector(1)][vector(2)].reset_Nbr();
        }
        Nbr_last.clear();
    }

    void insert_point(Eigen::Vector3d &pos)
    {
        // yolo();
        double i, j, k = 0;
        i = std::floor((pos(0) - origin_point(0)) / safe_distance);
        j = std::floor((pos(1) - origin_point(1)) / safe_distance);
        k = std::floor((pos(2) - origin_point(2)) / safe_distance);

        if (vectorExists(Eigen::Vector3d(i, j, k), Nbr_mask))
        {
            if (i >= 0 && i < xlim && j >= 0 && j < ylim && k >= 0 && k < zlim)
            {
                if (!vectorExists(Eigen::Vector3d(i, j, k), Nbr_last))
                {
                    map[i][j][k].set_Nbr(true);
                    Nbr_last.push_back(Eigen::Vector3d(i, j, k));
                    return;
                }
                return;
            }
        }
        else
        {
            if (vectorclose(pos, Nbr_point))
            {
                if (i >= 0 && i < xlim && j >= 0 && j < ylim && k >= 0 && k < zlim)
                {
                    if (!vectorExists(Eigen::Vector3d(i, j, k), Nbr_last))
                    {
                        // yolo();
                        map[i][j][k].set_Nbr(true);
                        Nbr_last.push_back(Eigen::Vector3d(i, j, k));
                    }
                }
                return;
            }
        }
        if (i >= 0 && i < xlim && j >= 0 && j < ylim && k >= 0 && k < zlim)
        {
            map[i][j][k].set_occupied(true);
            edit_map(i, j, k);
            return;
        }
        else
        {
            return;
        }
    };

    void edit_map(int i, int j, int k)
    {
        // yolo();
        Eigen::Vector3d target(0, 0, 0);
        for (int x = i - 1; x < i + 2; x++)
        {
            for (int y = j - 1; y < j + 2; y++)
            {
                for (int z = k - 1; z < k + 2; z++)
                {
                    if (x >= 0 && x < xlim && y >= 0 && y < ylim && z >= 0 && z < zlim)
                    {
                        if ((!map[x][y][z].is_occupied()) && (!map[x][y][z].is_visit()))
                        {
                            map[x][y][z].set_interest(true);
                        }
                        if (map[x][y][z].is_interest() || map[x][y][z].is_used_interest())
                        {
                            target = Eigen::Vector3d(i - x, j - y, k - z);
                            map[x][y][z].insert_target(target);
                        }
                    }
                }
            }
        }
    };

    void visit_point(Eigen::Vector3d &visit_pos)
    {
        double i, j, k = 0;
        i = std::floor((visit_pos(0) - origin_point(0)) / safe_distance);
        j = std::floor((visit_pos(1) - origin_point(1)) / safe_distance);
        k = std::floor((visit_pos(2) - origin_point(2)) / safe_distance);
        map[i][j][k].set_visit(true);
    }
    bool vectorExists(const Eigen::Vector3d &target, const std::vector<Eigen::Vector3d> &vectors)
    {
        for (const auto &vector : vectors)
        {
            if (vector.isApprox(target))
            {
                return true;
            }
        }
        return false;
    }
    bool vectorclose(const Eigen::Vector3d &target, const std::vector<Eigen::Vector3d> &vectors)
    {
        for (const auto &vector : vectors)
        {
            if ((vector - target).norm() < 2 * safe_distance)
            {
                return true;
            }
        }
        return false;
    }
    int sign(float a)
    {
        if (a > 0)
        {
            return 1;
        }
        else if (a < 0)
        {
            return -1;
        }
        else
        {
            return 0;
        }
    }
    ~global_map(){};

protected:
    std::vector<std::vector<std::vector<map_cube>>> map;
    // std::vector<std::vector<std::vector<map_cube>>> map;
    double safe_distance = 3.0;
    Eigen::Vector3d &origin_point;
    int xlim = 0;
    int ylim = 0;
    int zlim = 0;
    std::vector<Eigen::Vector3d> Nbr_mask;
    std::vector<Eigen::Vector3d> Nbr_point;
    std::vector<Eigen::Vector3d> Nbr_last;
};

class Agent : public global_map
{
public:
    Agent(int dimx, int dimy, int dimz, Eigen::Vector3d &origin, double safed);
    ~Agent() { delete sync_; };

private:
    void CloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud, const sensor_msgs::PointCloud2ConstPtr &Nbr);

    void OctomapCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

    void OdomCallback(const nav_msgs::OdometryConstPtr &msg);

    void TimerCallback(const ros::TimerEvent &);
    void TimerCallback2(const ros::TimerEvent &);
    void NbrCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *nbr_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> *sync_;
    ros::CallbackQueue custom_queue1;
    ros::CallbackQueue custom_queue2;
    ros::CallbackQueue custom_queue3;
    ros::CallbackQueue custom_queue4;
    ros::CallbackQueue custom_queue5;

    ros::NodeHandle nh_, nh2_, nh3_, nh4_, nh5_, nh6_;

    ros::Publisher map_pub_, Nbr_pub_;
    // ros::Subscriber octomap_sub_,odom_sub_,neibor_sub_;
    ros::Subscriber odom_sub_;
    ros::Timer timer, timer2;
    nav_msgs::Odometry odom_;
    std::mutex myMutex;
};