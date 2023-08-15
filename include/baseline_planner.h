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
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <pcl/kdtree/kdtree_flann.h>
#include "utility.h"
#include <mutex>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <caric_mission/CreatePPComTopic.h>
#include <std_msgs/String.h>

#include "general_task_init.h"
#include "Astar.h"
#include <map>
struct agent_local
{
    bool in_bounding_box = false;
    bool planning_in_bounding_box = false;
    Eigen::Vector3i position_index;
    Eigen::Vector3i planning_index;
    double time = 0;
    double state = 0;
    double priority = 0;
};

struct info
{
    bool get_info = false;
    double message_time = 0;
    Eigen::Vector3d global_point;
    list<Eigen::Vector3d> global_path = {};
    int state = 0;
    int priority = 0;
};

class info_agent
{
public:
    
    info_agent()
    {
        namelist = {"/jurong", "/raffles", "/changi", "/sentosa", "/nanyang"};
        Agent_dict["/jurong"] = {false, 0, Eigen::Vector3d(0, 0, 1), {}, 0, 5};
        Agent_dict["/raffles"] = {false, 0, Eigen::Vector3d(0, 0, 2), {}, 0, 4};
        Agent_dict["/changi"] = {false, 0, Eigen::Vector3d(0, 0, 3), {}, 0, 3};
        Agent_dict["/sentosa"] = {false, 0, Eigen::Vector3d(0, 0, 4), {}, 0, 2};
        Agent_dict["/nanyang"] = {false, 0, Eigen::Vector3d(0, 0, 5), {}, 0, 1};
    }
    
    info_agent(vector<string> teammate)
    {
        namelist = {"/jurong", "/raffles", "/changi", "/sentosa", "/nanyang"};
        Agent_dict["/jurong"] = {false, 0, Eigen::Vector3d(0, 0, 1), {}, 0, 5};
        Agent_dict["/raffles"] = {false, 0, Eigen::Vector3d(0, 0, 2), {}, 0, 4};
        Agent_dict["/changi"] = {false, 0, Eigen::Vector3d(0, 0, 3), {}, 0, 3};
        Agent_dict["/sentosa"] = {false, 0, Eigen::Vector3d(0, 0, 4), {}, 0, 2};
        Agent_dict["/nanyang"] = {false, 0, Eigen::Vector3d(0, 0, 5), {}, 0, 1};
        for (int i = 0; i < teammate.size(); i++)
        {
            if (teammate[i] == "jurong")
            {
                leader = "/jurong";
            }
            else if (teammate[i] == "raffles")
            {
                leader = "/raffles";
            }
        }
    }
    
    int get_leader_state()
    {
        return Agent_dict[leader].state;
    }
    
    string get_leader()
    {
        return leader;
    }
    
    void get_leader_position(Eigen::Vector3d &target)
    {
        // cout<<"leader:"<<leader<<endl;
        target = Agent_dict[leader].global_point;
    }
    
    void update_state(string name, int state_in)
    {

        Agent_dict[name].state = state_in;
    }
    
    void reset_position_path(istringstream &str)
    {
        string name;
        getline(str, name, ';');
        if (name != "/jurong" && name != "/raffles" && name != "/changi" && name != "sentosa" && name != "/nanyang")
        {
            return;
        }
        else
        {
            string position_str;
            getline(str, position_str, ';');
            info info_temp;
            info_temp.get_info = true;
            info_temp.global_point = str2point(position_str);
            string path_point;
            while (getline(str, path_point, ';'))
            {
                info_temp.global_path.push_back(str2point(path_point));
            }
            info_temp.message_time = ros::Time::now().toSec();
            info_temp.state = Agent_dict[name].state;
            info_temp.priority = Agent_dict[name].priority;
            Agent_dict[name] = info_temp;
        }
    }

private:
    
    list<string> namelist;
    // list<Eigen::Vector3d> path_list;
    
    string leader;
    
    map<string, info> Agent_dict;
    
    void cout_name(string name)
    {
        cout << name << endl;
        info info_in = Agent_dict[name];
        cout << "Priority:" << info_in.priority << endl;
        cout << "State:" << info_in.state << endl;
        cout << "Get info:" << info_in.get_info << endl;
        cout << "Time:" << info_in.message_time << endl;
        cout << "Global position:" << info_in.global_point.transpose() << endl;
        cout << "Path point:" << endl;
        for (auto &point : info_in.global_path)
        {
            cout << "node:" << point.transpose() << endl;
        }
        cout << endl;
    }
    
    Eigen::Vector3d str2point(string input)
    {
        Eigen::Vector3d result;
        std::vector<string> value;
        boost::split(value, input, boost::is_any_of(","));
        // cout<<input<<endl;
        if (value.size() == 3)
        {
            result = Eigen::Vector3d(stod(value[0]), stod(value[1]), stod(value[2]));
        }
        else
        {
            cout << input << endl;
            cout << "error use str2point 2" << endl;
        }
        return result;
    }
};

class grid_map
{
public:
    grid_map() {}

    // Function use boundingbox message to build map
    grid_map(Boundingbox box, Eigen::Vector3d grid_size_in, int Teamsize_in, vector<string> team_list)
    {
        local_dict["/jurong"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 5};
        local_dict["/raffles"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 4};
        local_dict["/sentosa"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 3};
        local_dict["/changi"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 2};
        local_dict["/nanyang"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 1};
        namelist = {"/jurong", "/raffles", "/changi", "/sentosa", "/nanyang"};
        for (auto &name : team_list)
        {
            if (name == "jurong" || name == "raffles")
            {
                continue;
            }
            follower.push_back("/" + name);
        }
        team_size = Teamsize_in;
        fly_in_index = Eigen::Vector3i(0, 0, 0);
        rotation_matrix = box.getSearchRotation();
        rotation_matrix_inv = rotation_matrix.inverse();
        rotation_quat = Eigen::Quaterniond(rotation_matrix_inv);
        map_global_center = box.getCenter();
        map_quat_size = box.getRotExtents();
        grid_size = grid_size_in;
        initial_the_convert();
        interval = floor(map_shape.z() / team_size);
        cout << "Teamsize:"
             << "team_size" << endl; // test
        for (int i = 1; i < Teamsize_in; i++)
        {
            region_slice_layer.push_back(i * interval);
            finish_flag.push_back(0);
            finish_exp_flag.push_back(0);
        }
        set_under_ground_occupied();
    }

    // Function use grid size to build map used in construct the global map
    grid_map(Eigen::Vector3d grid_size_in)
    {
        local_dict["/jurong"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 5};
        local_dict["/raffles"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 4};
        local_dict["/sentosa"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 3};
        local_dict["/changi"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 2};
        local_dict["/nanyang"] = {false, false, Eigen::Vector3i(0, 0, -1), Eigen::Vector3i(0, 0, -1), 0, 0, 1};
        namelist = {"/jurong", "/raffles", "/changi", "/sentosa", "/nanyang"};
        fly_in_index = Eigen::Vector3i(0, 0, 0);
        map_global_center = Eigen::Vector3d(0, 0, 0);
        map_quat_size = Eigen::Vector3d(200, 200, 100);
        grid_size = grid_size_in;
        rotation_matrix = Eigen::Matrix3d::Identity();
        rotation_matrix_inv = rotation_matrix.inverse();
        rotation_quat = Eigen::Quaterniond(rotation_matrix_inv);
        initial_the_convert();
        set_under_ground_occupied();
    }

    // Function for update the map and interest point
    void insert_point(Eigen::Vector3d point_in)
    {
        Eigen::Vector3d point_in_local = rotation_matrix * (point_in - map_global_center);
        if (out_of_range(point_in_local, false))
        {
            return;
        }
        Eigen::Vector3i bias_index(0, 0, 0);
        if (fabs(point_in_local.x()) < 0.5 * grid_size.x())
        {
            bias_index.x() = 0;
        }
        else
        {
            if (point_in_local.x() > 0)
            {
                bias_index.x() = floor((point_in_local.x() - 0.5 * grid_size.x()) / grid_size.x()) + 1;
            }
            else
            {
                bias_index.x() = -floor((-point_in_local.x() - 0.5 * grid_size.x()) / grid_size.x()) - 1;
            }
        }

        if (fabs(point_in_local.y()) < 0.5 * grid_size.y())
        {
            bias_index.y() = 0;
        }
        else
        {
            if (point_in_local.y() > 0)
            {
                bias_index.y() = floor((point_in_local.y() - 0.5 * grid_size.y()) / grid_size.y()) + 1;
            }
            else
            {
                bias_index.y() = -floor((-point_in_local.y() - 0.5 * grid_size.y()) / grid_size.y()) - 1;
            }
        }
        if (fabs(point_in_local.z()) < 0.5 * grid_size.z())
        {
            bias_index.z() = 0;
        }
        else
        {
            if (point_in_local.z() > 0)
            {
                bias_index.z() = floor((point_in_local.z() - 0.5 * grid_size.z()) / grid_size.z()) + 1;
            }
            else
            {
                bias_index.z() = -floor((-point_in_local.z() - 0.5 * grid_size.z()) / grid_size.z()) - 1;
            }
        }
        Eigen::Vector3i true_index = bias_index + map_index_center;
        if (map[true_index.x()][true_index.y()][true_index.z()] == 1)
        {
            return;
        }
        else
        {
            map[true_index.x()][true_index.y()][true_index.z()] = 1;
            occupied_num++;
            map_cloud_massage = point3i2str(true_index) + ";" + map_cloud_massage;
            for (int x = true_index.x() - 1; x < true_index.x() + 2; x++)
            {
                for (int y = true_index.y() - 1; y < true_index.y() + 2; y++)
                {
                    for (int z = true_index.z() - 1; z < true_index.z() + 2; z++)
                    {
                        if (out_of_range_index(Eigen::Vector3i(x, y, z)))
                        {
                            continue;
                        }
                        if (abs(x - true_index.x()) + abs(y - true_index.y()) + abs(z - true_index.z()) == 1)
                        {
                            if (map[x][y][z] == 0 && visited_map[x][y][z] == 0)
                            {
                                interest_map[x][y][z] = 1;
                            }
                            else
                            {
                                interest_map[x][y][z] = 0;
                            }
                        }
                    }
                }
            }
            return;
        }
    }

    visualization_msgs::MarkerArray Draw_map()
    {
        visualization_msgs::MarkerArray markers;
        for (int x = 0; x < map_shape.x(); x++)
        {
            for (int y = 0; y < map_shape.y(); y++)
            {
                for (int z = 0; z < map_shape.z(); z++)
                {
                    if (map[x][y][z] == 1)
                    {
                        markers.markers.push_back(generate_marker(Eigen::Vector3i(x, y, z), 0, markers.markers.size()));
                    }
                    else if (interest_map[x][y][z] == 1)
                    {
                        markers.markers.push_back(generate_marker(Eigen::Vector3i(x, y, z), 1, markers.markers.size()));
                    }
                }
            }
        }

        return markers;
    }

    void update_position(Eigen::Vector3d point)
    {
        Eigen::Vector3d point_local = rotation_matrix * (point - map_global_center);
        if (out_of_range(point_local, false))
        {
            in_my_range = false;
            return;
        }
        Eigen::Vector3i index = get_index(point);
        if (now_position_index != index && visited_map[index.x()][index.y()][index.z()] == 0 && search_direction.empty())
        {
            search_direction = get_search_target(index);
            time_start=ros::Time::now().toSec();
        }
        if (search_direction.empty())
        {
            visited_map[index.x()][index.y()][index.z()] = 1;
        }
        if(fabs(ros::Time::now().toSec()-time_start)>3)
        {
            visited_map[index.x()][index.y()][index.z()] = 1;
        }
        now_position_global = point;
        now_position_index = index;
        now_position_local = point_local;
        in_my_range = true;
    }

    Eigen::Vector3i get_index(Eigen::Vector3d point_in)
    {
        Eigen::Vector3d point_in_local = rotation_matrix * (point_in - map_global_center);
        Eigen::Vector3i bias_index(0, 0, 0);
        if (fabs(point_in_local.x()) < 0.5 * grid_size.x())
        {
            bias_index.x() = 0;
        }
        else
        {
            if (point_in_local.x() > 0)
            {
                bias_index.x() = floor((point_in_local.x() - 0.5 * grid_size.x()) / grid_size.x()) + 1;
            }
            else
            {
                bias_index.x() = -floor((-point_in_local.x() - 0.5 * grid_size.x()) / grid_size.x()) - 1;
            }
        }

        if (fabs(point_in_local.y()) < 0.5 * grid_size.y())
        {
            bias_index.y() = 0;
        }
        else
        {
            if (point_in_local.y() > 0)
            {
                bias_index.y() = floor((point_in_local.y() - 0.5 * grid_size.y()) / grid_size.y()) + 1;
            }
            else
            {
                bias_index.y() = -floor((-point_in_local.y() - 0.5 * grid_size.y()) / grid_size.y()) - 1;
            }
        }

        if (fabs(point_in_local.z()) < 0.5 * grid_size.z())
        {
            bias_index.z() = 0;
        }
        else
        {
            if (point_in_local.z() > 0)
            {
                bias_index.z() = floor((point_in_local.z() - 0.5 * grid_size.z()) / grid_size.z()) + 1;
            }
            else
            {
                bias_index.z() = -floor((-point_in_local.z() - 0.5 * grid_size.z()) / grid_size.z()) - 1;
            }
        }
        Eigen::Vector3i result = bias_index + map_index_center;
        return result;
    }
    
    void Astar_local(Eigen::Vector3d target, string myname, string leader_name, bool &flag, bool islong)
    {
        vector<vector<vector<int>>> map_temp = map;
        if (myname == "/jurong" || myname == "/raffles")
        {
            if (true)
            { // Here condition should be whether need waiting;
                for (auto &name : namelist)
                {
                    if (myname == name)
                    {
                        continue;
                    }
                    else
                    {
                        if (fabs(ros::Time::now().toSec() - local_dict[name].time) < 1 || true)
                        {
                            if (local_dict[name].in_bounding_box)
                            {
                                Eigen::Vector3i tar = local_dict[name].position_index;
                                map_temp[tar.x()][tar.y()][tar.z()] = 1;
                            }
                            if (local_dict[name].planning_in_bounding_box)
                            {
                                Eigen::Vector3i tar = local_dict[name].planning_index;
                                map_temp[tar.x()][tar.y()][tar.z()] = 1;
                            }
                        }
                    }
                }
                Eigen::Vector3i tar_index = get_index(target);
                list<Eigen::Vector3i> path_tamp;
                if (!islong)
                {
                    path_tamp = astar_planner.get_path(map_temp, now_position_index, tar_index);
                }
                else
                {
                    path_tamp = astar_planner.get_path_long(map_temp, now_position_index, tar_index);
                }

                if (path_tamp.empty())
                {
                    path_final_global = now_position_global;
                    path_index = path_tamp;
                    flag = true;
                }
                else
                {
                    flag = false;
                    path_final_global = target;
                    path_index = path_tamp;
                }
                generate_the_global_path();
                return;
            }
            else
            {
                path_index = {};
                generate_the_global_path();
                return;
            }
        }
        else
        {
            for (auto &name : namelist)
            {
                if (myname == name || name == leader_name)
                {
                    continue;
                }
                else
                {
                    if (fabs(ros::Time::now().toSec() - local_dict[name].time) < 1 || true)
                    {
                        if (local_dict[name].in_bounding_box)
                        {
                            Eigen::Vector3i tar = local_dict[name].position_index;
                            map_temp[tar.x()][tar.y()][tar.z()] = 1;
                        }
                        if (local_dict[name].planning_in_bounding_box)
                        {
                            Eigen::Vector3i tar = local_dict[name].planning_index;
                            map_temp[tar.x()][tar.y()][tar.z()] = 1;
                        }
                    }
                }
            }
            Eigen::Vector3i tar_index = get_index(target);
            list<Eigen::Vector3i> path_tamp;
            if (!islong)
            {
                path_tamp = astar_planner.get_path(map_temp, now_position_index, tar_index);
            }
            else
            {
                path_tamp = astar_planner.get_path_long(map_temp, now_position_index, tar_index);
            }
            if (path_tamp.empty())
            {
                path_final_global = now_position_global;
                path_index = path_tamp;
                flag = true;
            }
            else
            {
                if (fabs(ros::Time::now().toSec() - local_dict[leader_name].time) < 1)
                {
                    path_tamp.pop_front();
                }
                flag = false;
                path_final_global = target;
                path_index = path_tamp;
            }
            generate_the_global_path();
            return;
        }
    }
    
    void Astar_photo(Eigen::Vector3d target, string myname, bool &flag)
    {
        vector<vector<vector<int>>> map_temp = map;
        for (auto &name : namelist)
        {
            if (myname == name)
            {
                continue;
            }
            else
            {
                if (fabs(ros::Time::now().toSec() - local_dict[name].time) < 1||true)
                {
                    if (local_dict[name].in_bounding_box)
                    {
                        Eigen::Vector3i tar = local_dict[name].position_index;
                        map_temp[tar.x()][tar.y()][tar.z()] = 1;
                    }
                    if (local_dict[name].planning_in_bounding_box)
                    {
                        Eigen::Vector3i tar = local_dict[name].planning_index;
                        map_temp[tar.x()][tar.y()][tar.z()] = 1;
                    }
                }
            }
        }
        Eigen::Vector3i tar_index = get_index(target);
        list<Eigen::Vector3i> path_tamp;
        path_tamp = astar_planner.get_path(map_temp, now_position_index, tar_index);
        if (path_tamp.empty())
        {
            path_final_global = now_position_global;
            path_index = path_tamp;
            flag = true;
        }
        else
        {
            flag = false;
            path_final_global = target;
            path_index = path_tamp;
        }
        generate_the_global_path();
    }
    
    Eigen::Vector3d get_fly_in_point_global()
    {
        // cout<<"fly in output"<<fly_in_index.transpose()<<endl;//test debug
        return get_grid_center_global(fly_in_index);
        // return get_grid_center_global(Eigen::Vector3i(0,0,1));
    }
    
    bool check_whether_fly_in(bool print)
    {
        if (print)
        {
            cout << "now position" << now_position_index.transpose() << endl;
            cout << "fly in" << fly_in_index.transpose() << endl;
        }
        if ((now_position_index - fly_in_index).norm() < 2 && in_my_range)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    
    void update_fly_in_index(bool replan)
    {
        if (map[fly_in_index.x()][fly_in_index.y()][fly_in_index.z()] == 1 || replan)
        {
            int x = fly_in_index.x();
            int y = fly_in_index.y();
            int z = fly_in_index.z();
            int distance = min({abs(x), abs(y), abs(map_shape.x() - x), abs(map_shape.y() - y)});
            int top;
            int bottom;
            int left;
            int right;
            int i = x;
            int j = y;

            for (int k = z; k < map_shape.z(); k++)
            {
                for (distance; distance <= 3; distance++)
                {
                    top = map_shape.y() - distance;
                    bottom = distance;
                    left = distance;
                    right = map_shape.x() - distance;

                    while (i < right && j == bottom)
                    {
                        if (x == i && y == j && z == k)
                        {
                            i++;
                            continue;
                        }
                        if (i < 0 || j < 0 || k < 0 || i >= map_shape.x() || j >= map_shape.y() || k >= map_shape.z())
                        {
                            i++;
                            continue;
                        }
                        if (map[i][j][k] == 0)
                        {
                            if (fly_in_index == Eigen::Vector3i(i, j, k))
                            {
                                i++;
                                continue;
                            }
                            else
                            {
                                fly_in_index = Eigen::Vector3i(i, j, k);
                                return;
                            }
                        }
                        i++;
                    }
                    while (j < top && i == right)
                    {
                        if (x == i && y == j && z == k)
                        {
                            j++;
                            continue;
                        }
                        if (i < 0 || j < 0 || k < 0 || i >= map_shape.x() || j >= map_shape.y() || k >= map_shape.z())
                        {
                            j++;
                            continue;
                        }
                        if (map[i][j][k] == 0)
                        {
                            if (fly_in_index == Eigen::Vector3i(i, j, k))
                            {
                                j++;
                                continue;
                            }
                            else
                            {
                                fly_in_index = Eigen::Vector3i(i, j, k);
                                return;
                            }
                        }
                        j++;
                    }
                    while (i > left && j == top)
                    {
                        if (x == i && y == j && z == k)
                        {
                            i--;
                            continue;
                        }
                        if (i < 0 || j < 0 || k < 0 || i >= map_shape.x() || j >= map_shape.y() || k >= map_shape.z())
                        {
                            i--;
                            continue;
                        }
                        if (map[i][j][k] == 0)
                        {
                            if (fly_in_index == Eigen::Vector3i(i, j, k))
                            {
                                i--;
                                continue;
                            }
                            else
                            {
                                fly_in_index = Eigen::Vector3i(i, j, k);
                                return;
                            }
                        }
                        i--;
                    }
                    while (j > bottom && i == left)
                    {
                        if (x == i && y == j && z == k)
                        {
                            j--;
                            continue;
                        }
                        if (i < 0 || j < 0 || k < 0 || i >= map_shape.x() || j >= map_shape.y() || k >= map_shape.z())
                        {
                            j--;
                            continue;
                        }
                        if (map[i][j][k] == 0)
                        {
                            if (fly_in_index == Eigen::Vector3i(i, j, k))
                            {
                                j--;
                                continue;
                            }
                            else
                            {
                                fly_in_index = Eigen::Vector3i(i, j, k);
                                return;
                            }
                        }
                        j--;
                    }
                    i = distance + 1;
                    j = distance + 1;
                }
                cout << "Not find point in layer:" << k << endl;
                distance = 0;
            }
        }
    }
    
    Eigen::Vector3d get_next_point(bool global)
    {
        if (!path_global.empty())
        {
            Eigen::Vector3i index = get_index(path_global.front());
            if (map[index.x()][index.y()][index.z()] == 0)
            {
                return path_global.front();
            }
            else
            {
                return now_position_global;
            }
        }
        else
        {
            if (map[now_position_index.x()][now_position_index.y()][now_position_index.z()] == 1 && global)
            {
                return now_position_global;
            }
            else
            {
                if (get_index(path_final_global) == now_position_index)
                {
                    return path_final_global;
                }
                else
                {
                    return get_grid_center_global(now_position_index);
                }
            }
        }
    }
    
    nav_msgs::Path get_path_show()
    {
        return path_global_show_message;
    }
    
    void set_state(int a)
    {
        mystate = a;
    }
    
    int get_state()
    {
        return mystate;
    }
    
    int get_state_leader()
    {
        bool flag_state = true;
        if (mystate == 3)
        {
            return mystate;
        }
        for (auto &name : follower)
        {
            if (local_dict[name].state != 2)
            {
                flag_state = false;
                return mystate;
            }
        }
        if (flag_state && mystate == 2)
        {
            mystate = 3;
        }
        return mystate;
    }
    
    bool get_whether_pop()
    {
        if (mystate == 3)
        {
            // cout<<"mystate:"<<mystate<<endl;
            // return true;
        }
        if (mystate != 3)
        {
            return false;
        }
        else
        { 
            for (auto &name : follower)
            {
                if (local_dict[name].state != 0)
                {
                    return false;
                }
            }
            if(follower.size()==0&&mystate!=3){
                return false;
            }
        }
        return true;
    }
    
    void exploration_layer(string myname, int region_index)
    {
        list<Eigen::Vector3i> path_index_temp;
        if (is_not_empty())
        {
            path_index_temp = Dijkstra_search_2D_with_3D(height, region_slice_layer[region_index], myname);
        }
        else
        {
            path_index_temp = Dijkstra_search_edge(height, region_slice_layer[region_index], myname);
        }
        if (path_index_temp.empty())
        {
            finish_exp_flag[region_index] = 1;
            if (height < region_slice_layer[region_index])
            {
                height = region_slice_layer[region_index];
            }
            else
            {
                if (local_dict[follower[region_index]].state != 1)
                {
                    finish_exp_flag[region_index] = 1;
                    path_index_temp = Dijkstra_search_fly_in_xy(interval * (region_index - 1), height, myname);
                    if (path_index_temp.empty() || path_index_temp.size() == 1)
                    {
                        path_index_temp = Dijkstra_search_edge(height, region_slice_layer[region_index], myname);
                        if (path_index_temp.empty())
                        {
                            height++;
                        }
                    }
                }
                else
                {
                    finish_flag[region_index] = 1;
                }
            }
        }
        path_index_temp.reverse();
        path_index = path_index_temp;
        generate_the_global_path();
    }
    
    void exploration(string myname)
    {
       ////yolo();
        for (int i = 0; i < finish_flag.size(); i++)
        {
            if (finish_flag[i] == 0)
            {
                exploration_layer(myname, i);
                return;
            }
        }
        take_photo(myname);
    }
    
    void take_photo(string myname)
    {
        if (!init_task_id)
        {
            int i = 0;
            while (i < follower.size())
            {
                if (follower[i] == myname)
                {
                    break;
                }
                i++;
            }
            task_id = i;
        }
        if(follower.size()==0){
            //yolo();
            take_photo_layer(0, map_shape.z()-1, myname);
            //yolo();
            return;
        }


        if (task_id == 0)
        {
            take_photo_layer(0, region_slice_layer[0], myname);
            return;
        }
        else if (task_id == follower.size())
        {
            take_photo_layer(region_slice_layer[task_id - 1], map_shape.z() - 1, myname);
        }
        else
        {
            take_photo_layer(region_slice_layer[task_id - 1], region_slice_layer[task_id], myname);
        }
    }
    
    void take_photo_layer(int low, int high, string myname)
    {
        list<Eigen::Vector3i> path_index_temp;
        if (height < low + 1)
        {
            height = low + 1;
        }
        if (myname == "/raffles" || myname == "/jurong")
        {
            if (finish_flag_leader)
            {

            }
            else
            {
                bool tamp = true;
                if(follower.size()==0){
                    tamp=false;
                }
                for (auto &name : follower)
                {
                    if (local_dict[name].state != 2)
                    {
                        tamp = false;
                        break;
                    }
                }
                finish_flag_leader = tamp;
            }
        }
        else
        {
            finish_flag_leader = false;
        }

        if (height >= high || finish_flag_leader)
        {
            path_index_temp = Dijkstra_search_fly_in_xy(low + 1, high - 1, myname);
            // cout<<"Path size:"<<path_index_temp.size()<<endl;
            if (path_index_temp.empty() || path_index_temp.size() == 1 || finish_flag_leader)
            {
                if (mystate != 3)
                {
                    mystate = 2;
                }
            }
            path_index_temp.reverse();
            path_index = path_index_temp;
            generate_the_global_path();
            return;
        }

        path_index_temp = Dijkstra_search_2D_with_3D(height, high - 1, myname);
        if (path_index_temp.empty())
        {
            if (myname == "/raffles" || myname == "/jurong")
            {
                height += 4;
            }
            else
            {
                height += 4;
            }
        }
        path_index_temp.reverse();
        path_index = path_index_temp;
        generate_the_global_path();
        return;
    }

    bool is_not_empty()
    {
        for (int x = 0; x < map_shape.x(); x++)
        {
            for (int y = 0; y < map_shape.y(); y++)
            {
                if (map[x][y][height] == 1)
                {
                    return true;
                }
            }
        }
        return false;
    }

    void update_gimbal(Eigen::Vector3d direction_global, bool print)
    {
        if (print)
        {
            cout << "direction_global:" << direction_global.transpose() << endl;
            // cout<<"matrix:"<<endl;
            // cout<<Rpy2Rot(direction_global)<<endl;
            if (!search_direction.empty())
            {
                cout << "target:" << search_direction.front().transpose() << endl;
                // cout<<"matrix:"<<endl;
                // cout<<Rpy2Rot(search_direction.front())<<endl;
            }
        }
        if (search_direction.empty())
        {
            return;
        }
        else if ((direction_global - search_direction.front()).norm() < 0.30)
        {
            search_direction.pop_front();
            // cout<<"pop front search"<<endl;// test
        }
    }

    void insert_cloud_from_str(istringstream &msg)
    {
        string number_of_map;
        getline(msg, number_of_map, ';');
        int number_map = stoi(number_of_map);

        while (occupied_num < number_map)
        {
            string index_occupied;
            getline(msg, index_occupied, ';');

            Eigen::Vector3i index_occ_tamp;
            if (str2point3i(index_occupied, index_occ_tamp))
            {
                insert_map_index(index_occ_tamp);
            }
        }
    }

    void get_gimbal_rpy(Eigen::Vector3d &result)
    {
        list<Eigen::Vector3d> search_temp = search_direction;

        if (search_temp.empty())
        {
        }
        else
        {
            result = search_temp.front();
        }
    }
    bool get_mission_finished()
    {
        return is_finished;
    }
    string get_num_str()
    {
        string result;
        result = to_string(occupied_num) + ";";
        return result;
    }
    string get_map_str()
    {
        return map_cloud_massage;
    }
    void set_fly_in_index(string tar)
    {
        try
        {
            Eigen::Vector3i vec_fly;
            if (str2point3i(tar, vec_fly))
            {
                fly_in_index = vec_fly;
            }
        }
        catch (const std::invalid_argument &e)
        {
            cout << "Invalid argument" << e.what() << endl;
            return;
        }
        catch (const std::out_of_range &e)
        {
            cout << "Out of range" << e.what() << endl;
            return;
        }
    }
    string get_fly_in_str()
    {
        return (point3i2str(fly_in_index) + ";");
    }
    void update_local_dict(istringstream &str)
    {
        string name;
        getline(str, name, ';');
        if (name != "/jurong" && name != "/raffles" && name != "/changi" && name != "/sentosa" && name != "/nanyang")
        {
            return;
        }
        else
        {
            string position_str;
            getline(str, position_str, ';');
            agent_local info_temp;
            // cout<<"position_str:"<<position_str<<endl;
            Eigen::Vector3d global_nbr_position_point = str2point(position_str);
            if (!out_of_range_global(global_nbr_position_point, false))
            {
                info_temp.in_bounding_box = true;
                info_temp.position_index = get_index(global_nbr_position_point);
            }
            string path_point;
            getline(str, path_point, ';');
            // cout<<"path_point:"<<path_point<<endl;
            Eigen::Vector3d next_nbr_path_po = str2point(path_point);
            if (!out_of_range_global(next_nbr_path_po, false))
            {
                info_temp.planning_in_bounding_box = true;
                info_temp.planning_index = get_index(next_nbr_path_po);
            }

            info_temp.time = ros::Time::now().toSec();
            info_temp.state = local_dict[name].state;
            info_temp.priority = local_dict[name].priority;
            local_dict[name] = info_temp;
        }
    }
    void update_state(string name, int state_in)
    {
        local_dict[name].state = state_in;
    }
    list<string> get_state_string_list()
    {
        list<string> result;
        for (int i = 0; i < finish_exp_flag.size(); i++)
        {
            if (finish_exp_flag[i] == 1 && finish_flag[i] == 0)
            {
                string str_state_set = follower[i] + ";1;";
                // cout<<"str_state_set:"<<str_state_set<<endl;
                result.push_back(str_state_set);
            }
        }
        return result;
    }

private:
    // communication part
    list<string> namelist;
    std::map<string, agent_local> local_dict;
    int mystate = 0;
    //
    int team_size;
    bool is_finished = false;
    AStar astar_planner;
    double time_start=0;

    bool init_task_id = false;
    int task_id = 0;
    string map_cloud_massage;
    int occupied_num = 0;
    int exploration_state = 0;
    vector<vector<vector<int>>> map;
    vector<vector<vector<int>>> interest_map;
    vector<vector<vector<int>>> visited_map;
    Eigen::Vector3d grid_size;
    Eigen::Matrix3d rotation_matrix;
    Eigen::Matrix3d rotation_matrix_inv;
    Eigen::Quaterniond rotation_quat;
    Eigen::Vector3d map_global_center;
    Eigen::Vector3i map_shape;
    Eigen::Vector3i map_index_center;
    Eigen::Vector3d map_quat_size;
    Eigen::Vector3d now_position_global;
    Eigen::Vector3d now_position_local;
    Eigen::Vector3i now_position_index;
    Eigen::Vector3i fly_in_index;
    Eigen::Vector3d path_final_global;
    list<Eigen::Vector3d> path_global;
    list<Eigen::Vector3i> path_index;
    list<Eigen::Vector3d> search_direction;
    vector<int> region_slice_layer;
    vector<int> finish_flag;
    vector<int> finish_exp_flag;
    vector<string> follower;
    bool Developing = true;
    bool in_my_range = false;
    nav_msgs::Path path_global_show_message;
    int height = 0;
    int interval = 0;
    bool finish_flag_leader = false;
    void initial_the_convert()
    {
        int x_lim;
        int y_lim;
        int z_lim;
        if (map_quat_size.x() < 0.5 * grid_size.x())
        {
            x_lim = 0;
        }
        else
        {
            x_lim = floor((map_quat_size.x() - 0.5 * grid_size.x()) / grid_size.x()) + 1;
        }
        if (map_quat_size.y() < 0.5 * grid_size.y())
        {
            y_lim = 0;
        }
        else
        {
            y_lim = floor((map_quat_size.y() - 0.5 * grid_size.y()) / grid_size.y()) + 1;
        }
        if (map_quat_size.z() < 0.5 * grid_size.z())
        {
            z_lim = 0;
        }
        else
        {
            z_lim = floor((map_quat_size.z() - 0.5 * grid_size.z()) / grid_size.z()) + 1;
        }
        x_lim = x_lim + 1;
        y_lim = y_lim + 1;
        z_lim = z_lim + 1;
        cout << "x lim:" << x_lim << endl;
        cout << "y lim:" << y_lim << endl;
        cout << "z lim:" << z_lim << endl;
        map_shape = Eigen::Vector3i(2 * x_lim + 1, 2 * y_lim + 1, 2 * z_lim + 1);
        cout << "Map shape:" << map_shape.transpose() << endl;
        map_index_center = Eigen::Vector3i(x_lim, y_lim, z_lim);
        cout << "Map Center Index:" << map_index_center.transpose() << endl;
        map = vector<vector<vector<int>>>(map_shape.x(), vector<vector<int>>(map_shape.y(), vector<int>(map_shape.z(), 0)));
        interest_map = vector<vector<vector<int>>>(map_shape.x(), vector<vector<int>>(map_shape.y(), vector<int>(map_shape.z(), 0)));
        visited_map = vector<vector<vector<int>>>(map_shape.x(), vector<vector<int>>(map_shape.y(), vector<int>(map_shape.z(), 0)));
        astar_planner = AStar(map, map_shape);
    }
    void set_under_ground_occupied()
    {
        for (int x = 0; x < map_shape.x(); x++)
        {
            for (int y = 0; y < map_shape.y(); y++)
            {
                for (int z = 0; z < map_shape.z(); z++)
                {
                    Eigen::Vector3d grid_center_global = get_grid_center_global(Eigen::Vector3i(x, y, z));
                    if (grid_center_global.z() < 0.5 * grid_size.z())
                    {
                        map[x][y][z] = 1;
                    }
                }
            }
        }
    }
    // Function whether a local point is out of range
    bool out_of_range(Eigen::Vector3d point, bool out_put)
    {
        if (fabs(point.x()) > (fabs(map_shape.x() / 2) + 0.5) * grid_size.x() || fabs(point.y()) > (fabs(map_shape.y() / 2) + 0.5) * grid_size.y() || fabs(point.z()) > (fabs(map_shape.z() / 2) + 0.5) * grid_size.z())
        {
            if (out_put)
            {
                cout << "xbool:" << (fabs(point.x()) > fabs(map_shape.x() / 2) * grid_size.x() ? "yes" : "no") << endl;
                cout << "xlim:" << fabs(map_shape.x() / 2) * grid_size.x() << endl;
                cout << "ylim:" << fabs(map_shape.y() / 2) * grid_size.y() << endl;
                cout << "grid size:" << grid_size.transpose() << endl;
                cout << "map_shape:" << map_shape.transpose() << endl;
                cout << "center:" << map_index_center.transpose() << endl;
                cout << "out range point" << point.transpose() << endl;
                cout << "Desired point" << (rotation_matrix * (get_grid_center_global(Eigen::Vector3i(0, 0, 0)) - map_global_center)).transpose() << endl;
            }
            return true;
        }
        else
        {
            return false;
        }
    }

        // Function whether a local point is out of range
    bool out_of_range_global(Eigen::Vector3d point_in, bool out_put)
    {
        Eigen::Vector3d point=rotation_matrix*(point_in-map_global_center);
        if (fabs(point.x()) > (fabs(map_shape.x() / 2) + 0.5) * grid_size.x() || fabs(point.y()) > (fabs(map_shape.y() / 2) + 0.5) * grid_size.y() || fabs(point.z()) > (fabs(map_shape.z() / 2) + 0.5) * grid_size.z())
        {
            if (out_put)
            {
                cout << "xbool:" << (fabs(point.x()) > fabs(map_shape.x() / 2) * grid_size.x() ? "yes" : "no") << endl;
                cout << "xlim:" << fabs(map_shape.x() / 2) * grid_size.x() << endl;
                cout << "ylim:" << fabs(map_shape.y() / 2) * grid_size.y() << endl;
                cout << "grid size:" << grid_size.transpose() << endl;
                cout << "map_shape:" << map_shape.transpose() << endl;
                cout << "center:" << map_index_center.transpose() << endl;
                cout << "out range point" << point.transpose() << endl;
                cout << "Desired point" << (rotation_matrix * (get_grid_center_global(Eigen::Vector3i(0, 0, 0)) - map_global_center)).transpose() << endl;
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    // Function whether a local point_index is out of range
    bool out_of_range_index(Eigen::Vector3i point)
    {
        if (point.x() >= 0 && point.x() < map_shape.x() && point.y() >= 0 && point.y() < map_shape.y() && point.z() >= 0 && point.z() < map_shape.z())
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    // Function whether a local point_index is out of range and whether the z in limitation
    bool out_of_range_index(Eigen::Vector3i point, int top_z, int bottom_z)
    {
        if (top_z <= bottom_z)
        {
            cout << "Error use function" << endl;
        }
        if (point.x() >= 0 && point.x() < map_shape.x() && point.y() >= 0 && point.y() < map_shape.y() && point.z() > bottom_z && point.z() < top_z && point.z() >= 0 && point.z() < map_shape.z())
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    // Function to get the grid center point in global
    Eigen::Vector3d get_grid_center_global(Eigen::Vector3i grid_index)
    {
        Eigen::Vector3d bias = (grid_index - map_index_center).cast<double>();
        Eigen::Vector3d local_result = bias.cwiseProduct(grid_size);
        Eigen::Vector3d global_result = rotation_matrix_inv * local_result + map_global_center;
        return global_result;
    }
    // Function to generate map marker
    visualization_msgs::Marker generate_marker(Eigen::Vector3i index, int type, int id)
    {
        // type- 0:occupied 1:interest
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "cube_marker_array";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        Eigen::Vector3d grid_center = get_grid_center_global(index);
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
        if (type == 0 && grid_center.z() > 0)
        {
            marker.color.a = 0.5; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        else if (type == 1 && grid_center.z() > 0)
        {
            marker.color.a = 0.5; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        else
        {
            marker.color.a = 0.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        return marker;
    }
    // Function to generate 2D layer search path with 3D Dijkstra
    list<Eigen::Vector3i> Dijkstra_search_2D_with_3D(int layer, int upper, string myname)
    {
        vector<vector<vector<int>>> grid = map;
        for (auto &name : namelist)
        {
            if (myname == name)
            {
                continue;
            }
            else
            {
                if (fabs(ros::Time::now().toSec() - local_dict[name].time) < 1||true)
                {
                    if (local_dict[name].in_bounding_box)
                    {
                        Eigen::Vector3i tar = local_dict[name].position_index;
                        // cout<<"position"<<tar.transpose()<<endl;
                        grid[tar.x()][tar.y()][tar.z()] = 1;
                    }
                    if (local_dict[name].planning_in_bounding_box)
                    {
                        Eigen::Vector3i tar = local_dict[name].planning_index;
                        // cout<<"motion"<<tar.transpose()<<endl;
                        grid[tar.x()][tar.y()][tar.z()] = 1;
                    }
                }
            }
        }
        Eigen::Vector3i start = now_position_index;
        vector<Vector3i> directions = {Vector3i(0, 1, 0), Vector3i(0, -1, 0), Vector3i(1, 0, 0), Vector3i(-1, 0, 0), Vector3i(0, 0, 1), Vector3i(0, 0, -1)};

        // Initialize the queue and visited flag.
        queue<list<Eigen::Vector3i>> q;
        q.push({start});
        vector<vector<vector<bool>>> visited(map_shape.x(), vector<vector<bool>>(map_shape.y(), vector<bool>(map_shape.z(), false)));
        visited[start.x()][start.y()][start.z()] = true;

        while (!q.empty())
        {
            list<Eigen::Vector3i> path = q.front();
            q.pop();

            Eigen::Vector3i curr = path.back();
            if (interest_map[curr.x()][curr.y()][curr.z()] == 1 && visited_map[curr.x()][curr.y()][curr.z()] == 0 && curr.z() == layer)
            {
                return path; // Found the path, return the complete path.
            }

            for (const auto &dir : directions)
            {
                int nextRow = curr.x() + dir.x();
                int nextCol = curr.y() + dir.y();
                int nextHeight = curr.z() + dir.z();
                if (isValidMove(nextRow, nextCol, nextHeight,grid) && !visited[nextRow][nextCol][nextHeight] && nextHeight <= upper)
                {
                    list<Vector3i> newPath = path;
                    newPath.push_back(Vector3i(nextRow, nextCol, nextHeight));
                    q.push(newPath);
                    visited[nextRow][nextCol][nextHeight] = true;
                }
            }
        }
        // If the path is not found, return an empty list.
        return {};
    }
    // Function to generate 3D layer search/path planning with 3D Dijkstra
    list<Eigen::Vector3i> Dijkstra_search_edge(int layer, int upper, string myname)
    {
        vector<vector<vector<int>>> grid = map;
        for (auto &name : namelist)
        {
            if (myname == name)
            {
                continue;
            }
            else
            {
                if (fabs(ros::Time::now().toSec() - local_dict[name].time) < 1||true)
                {
                    // cout<<"name:"<<name<<endl;
                    if (local_dict[name].in_bounding_box)
                    {
                        // cout<<"pos insert"<<endl;
                        Eigen::Vector3i tar = local_dict[name].position_index;
                        grid[tar.x()][tar.y()][tar.z()] = 1;
                    }
                    if (local_dict[name].planning_in_bounding_box)
                    {
                        // cout<<"motion insert"<<endl;
                        Eigen::Vector3i tar = local_dict[name].planning_index;
                        grid[tar.x()][tar.y()][tar.z()] = 1;
                    }
                }
            }
        }
        Eigen::Vector3i start = now_position_index;
        vector<Vector3i> directions = {Vector3i(0, 1, 0), Vector3i(0, -1, 0), Vector3i(1, 0, 0), Vector3i(-1, 0, 0), Vector3i(0, 0, 1), Vector3i(0, 0, -1)};

        // Initialize the queue and visited flag.
        queue<list<Eigen::Vector3i>> q;
        q.push({start});
        vector<vector<vector<bool>>> visited(map_shape.x(), vector<vector<bool>>(map_shape.y(), vector<bool>(map_shape.z(), false)));
        visited[start.x()][start.y()][start.z()] = true;

        while (!q.empty())
        {
            list<Eigen::Vector3i> path = q.front();
            q.pop();

            Eigen::Vector3i curr = path.back();
            if ((curr.x() == 0 || curr.y() == 0 || curr.x() == map_shape.x() - 1 || curr.y() == map_shape.y() - 1) && visited_map[curr.x()][curr.y()][curr.z()] == 0 && curr.z() == layer)
            {
                return path; // Found the path, return the complete path.
            }

            for (const auto &dir : directions)
            {
                int nextRow = curr.x() + dir.x();
                int nextCol = curr.y() + dir.y();
                int nextHeight = curr.z() + dir.z();
                if (isValidMove(nextRow, nextCol, nextHeight,grid) && !visited[nextRow][nextCol][nextHeight] && nextHeight <= upper)
                {
                    list<Vector3i> newPath = path;
                    newPath.push_back(Vector3i(nextRow, nextCol, nextHeight));
                    q.push(newPath);
                    visited[nextRow][nextCol][nextHeight] = true;
                }
            }
        }
        // If the path is not found, return an empty list.
        return {};
    }
    list<Eigen::Vector3i> Dijkstra_search_fly_in_xy(int lower, int upper, string myname)
    {
        vector<vector<vector<int>>> grid = map;
        for (auto &name : namelist)
        {
            if (myname == name)
            {
                continue;
            }
            else
            {
                if (fabs(ros::Time::now().toSec() - local_dict[name].time) < 1||true)
                {   
                    if (local_dict[name].in_bounding_box)
                    {
                        Eigen::Vector3i tar = local_dict[name].position_index;
                        grid[tar.x()][tar.y()][tar.z()] = 1;
                    }
                    if (local_dict[name].planning_in_bounding_box)
                    {
                        Eigen::Vector3i tar = local_dict[name].planning_index;
                        grid[tar.x()][tar.y()][tar.z()] = 1;
                    }
                }
            }
        }
        Eigen::Vector3i start = now_position_index;
        vector<Vector3i> directions = {Vector3i(0, 1, 0), Vector3i(0, -1, 0), Vector3i(1, 0, 0), Vector3i(-1, 0, 0), Vector3i(0, 0, 1), Vector3i(0, 0, -1)};

        // Initialize the queue and visited flag.
        queue<list<Eigen::Vector3i>> q;
        q.push({start});
        vector<vector<vector<bool>>> visited(map_shape.x(), vector<vector<bool>>(map_shape.y(), vector<bool>(map_shape.z(), false)));
        visited[start.x()][start.y()][start.z()] = true;

        while (!q.empty())
        {
            list<Eigen::Vector3i> path = q.front();
            q.pop();

            Eigen::Vector3i curr = path.back();
            if ((curr.x() == fly_in_index.x() && curr.y() == fly_in_index.y()) && curr.z() >= lower && curr.z() <= upper)
            {
                return path; // Found the path, return the complete path.
            }

            for (const auto &dir : directions)
            {
                int nextRow = curr.x() + dir.x();
                int nextCol = curr.y() + dir.y();
                int nextHeight = curr.z() + dir.z();
                if (isValidMove(nextRow, nextCol, nextHeight,grid) && !visited[nextRow][nextCol][nextHeight] && nextHeight <= upper && nextHeight >= lower)
                {
                    list<Vector3i> newPath = path;
                    newPath.push_back(Vector3i(nextRow, nextCol, nextHeight));
                    q.push(newPath);
                    visited[nextRow][nextCol][nextHeight] = true;
                }
            }
        }
        // If the path is not found, return an empty list.
        return {};
    }

    bool isValidMove(int x, int y, int z)
    {
        if (x >= 0 && x < map_shape.x() && y >= 0 && y < map_shape.y() && z < map_shape.z() && z >= 0)
        {
            if (map[x][y][z] == 1)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        else
        {
            return false;
        }
    }
    bool isValidMove(int x,int y,int z,vector<vector<vector<int>>> grid)
    {
        if (x >= 0 && x < map_shape.x() && y >= 0 && y < map_shape.y() && z < map_shape.z() && z >= 0)
        {
            if (grid[x][y][z] == 1)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        else
        {
            return false;
        }
    }    

    void generate_the_global_path()
    {
        list<Eigen::Vector3i> path_tamp(path_index);
        list<Eigen::Vector3d> point_global_list_tamp;
        if (path_index.empty())
        {
            path_global.clear();
            return;
        }
        nav_msgs::Path global_path_tamp;

        // if (path_tamp.size() == 1)
        // {
        //     path_tamp.pop_back();
        // }
        // else if (path_tamp.size() >= 2)
        // {
        //     Eigen::Vector3i my_index = path_tamp.back();
        //     path_tamp.pop_back();
        //     if ((now_position_global - get_grid_center_global(path_tamp.back())).norm() > (get_grid_center_global(path_tamp.back()) - get_grid_center_global(my_index)).norm())
        //     {
        //         path_tamp.push_back(my_index);
        //     }
        // }

        path_tamp.pop_back();

        // path_tamp.pop_back();
        global_path_tamp.header.frame_id = "world";
        while (!path_tamp.empty())
        {
            Eigen::Vector3i index_current = path_tamp.back();
            Eigen::Vector3d point_current = get_grid_center_global(index_current);
            if (Developing)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "world";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = point_current.x();
                pose.pose.position.y = point_current.y();
                pose.pose.position.z = point_current.z();
                pose.pose.orientation.w = 1.0;
                global_path_tamp.poses.push_back(pose);
            }
            point_global_list_tamp.push_back(point_current);
            path_tamp.pop_back();
        }
        // point_global_list_tamp.push_back(path_final_global);
        path_global_show_message = global_path_tamp;
        path_global = point_global_list_tamp;
    }
    list<Eigen::Vector3d> get_search_target(Eigen::Vector3i true_index)
    {
        list<Eigen::Vector3d> point_list;
        // cout<<"begin:"<<endl;//test
        for (int x = true_index.x() - 1; x < true_index.x() + 2; x++)
        {
            for (int y = true_index.y() - 1; y < true_index.y() + 2; y++)
            {
                for (int z = true_index.z() - 1; z < true_index.z() + 2; z++)
                {
                    if (out_of_range_index(Eigen::Vector3i(x, y, z)))
                    {
                        continue;
                    }
                    if (abs(x - true_index.x()) + abs(y - true_index.y()) + abs(z - true_index.z()) == 1)
                    {
                        if (map[x][y][z] == 1)
                        {
                            point_list.push_back(get_rpy_limited_global(Eigen::Vector3d(x - true_index.x(), y - true_index.y(), z - true_index.z())));
                            // cout<<(Eigen::Vector3i(x,y,z)-true_index).transpose()<<endl;//test
                            // cout<<get_rpy_limited_global(Eigen::Vector3d(x-true_index.x(),y-true_index.y(),z-true_index.z())).transpose()<<endl;//test
                        }
                    }
                }
            }
        }
        // cout<<"end"<<endl;//test

        return point_list;
    }
    double get_rad(Eigen::Vector3d v1, Eigen::Vector3d v2)
    {
        return atan2(v1.cross(v2).norm(), v1.transpose() * v2);
    }
    Eigen::Vector3d get_rpy_limited_global(Eigen::Vector3d target_direction)
    {
        Eigen::Vector3d global_target_direction = rotation_matrix_inv * target_direction;
        // Eigen::Vector3d global_target_direction=target_direction;
        Eigen::Quaterniond quaternion;
        quaternion.setFromTwoVectors(Eigen::Vector3d(1, 0, 0), global_target_direction);
        Eigen::Matrix3d rotation_matrix_here = quaternion.toRotationMatrix();
        Eigen::Vector3d rpy = Rot2rpy(rotation_matrix_here);
        if (abs(rpy.x() + rpy.z()) < 1e-6 || abs(rpy.x() - rpy.z()) < 1e-6)
        {
            rpy.x() = 0;
            rpy.z() = 0;
        }

        if (rpy.y() > M_PI * 4 / 9)
        {
            rpy.y() = M_PI * 4 / 9;
        }
        else if (rpy.y() < -M_PI * 4 / 9)
        {
            rpy.y() = -M_PI * 4 / 9;
        }
        return rpy;
    }
    Eigen::Vector3d Rot2rpy(Eigen::Matrix3d R)
    {
        // Eigen::Vector3d euler_angles=R.eulerAngles(2,1,0);
        // Eigen:;Vector3d result(euler_angles.z(),euler_angles.y(),euler_angles.x());
        // return result;
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d rpy(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        rpy(0) = r;
        rpy(1) = p;
        rpy(2) = y;

        return rpy;
    }
    Eigen::Matrix3d Rpy2Rot(Eigen::Vector3d rpy)
    {
        Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
        result = Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix() * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
        return result;
    }
    string point3i2str(Eigen::Vector3i point)
    {
        string result;
        result = to_string(point.x()) + "," + to_string(point.y()) + "," + to_string(point.z());
        return result;
    }
    bool str2point3i(string str, Eigen::Vector3i &result)
    {
        std::vector<string> value;
        Eigen::Vector3i result_tamp;
        boost::split(value, str, boost::is_any_of(","));
        if (value.size() == 3)
        {
            // cout<<"str:"<<str<<endl;
            try
            {
                result_tamp = Eigen::Vector3i(stoi(value[0]), stoi(value[1]), stoi(value[2]));
            }
            catch (const std::invalid_argument &e)
            {
                return false;
                cout << "Invalid argument" << e.what() << endl;
            }
            catch (const std::out_of_range &e)
            {
                cout << "Out of range" << e.what() << endl;
                return false;
            }
            result = result_tamp;
            return true;
        }
        else
        {
            return false;
            cout << "error use str2point 3" << endl;
            // cout<<"str:"<<str<<endl;
        }
    }
    void insert_map_index(Eigen::Vector3i true_index)
    {
        map[true_index.x()][true_index.y()][true_index.z()] = 1;
        occupied_num++;
        for (int x = true_index.x() - 1; x < true_index.x() + 2; x++)
        {
            for (int y = true_index.y() - 1; y < true_index.y() + 2; y++)
            {
                for (int z = true_index.z() - 1; z < true_index.z() + 2; z++)
                {
                    if (out_of_range_index(Eigen::Vector3i(x, y, z)))
                    {
                        continue;
                    }
                    if (abs(x - true_index.x()) + abs(y - true_index.y()) + abs(z - true_index.z()) == 1)
                    {
                        if (map[x][y][z] == 0 && visited_map[x][y][z] == 0)
                        {
                            interest_map[x][y][z] = 1;
                        }
                        else
                        {
                            interest_map[x][y][z] = 0;
                        }
                    }
                }
            }
        }
    }
    Eigen::Vector3d str2point(string input)
    {
        Eigen::Vector3d result;
        std::vector<string> value;
        boost::split(value, input, boost::is_any_of(","));
        if (value.size() == 3)
        {
            result = Eigen::Vector3d(stod(value[0]), stod(value[1]), stod(value[2]));
        }
        else
        {
            // cout<<"error use str2point 4"<<endl;
            result = Eigen::Vector3d(1000, 1000, 1000);
        }
        return result;
    }
};

// class mainbrain is built for the transfer of the map
class mainbrain
{
public:
    mainbrain() {}
    mainbrain(string str, string name)
    {
        drone_rotation_matrix = Eigen::Matrix3d::Identity();
        grid_size = Eigen::Vector3d(safe_distance, safe_distance, safe_distance);
        global_map = grid_map(grid_size);
        namespace_ = name;
        if (namespace_ == "/jurong" || namespace_ == "/raffles")
        {
            is_leader = true;
        }
        vector<string> spilited_str;
        std::istringstream iss(str);
        std::string substring;
        while (std::getline(iss, substring, ';'))
        {
            spilited_str.push_back(substring);
        }
        generate_global_map(spilited_str[0]);
        if (spilited_str.size() > 1)
        {
            for (int j = 0; j < path_index.size(); j++)
            {
                map_set.push_back(grid_map(Boundingbox(spilited_str[path_index[j]]), grid_size, teammates_name.size(), teammates_name));
            }
        }
        else
        {
            cout << "Path Assigned Error!!!" << endl;
        }

        cout << "size of path assigned:  " << map_set.size() << endl;
        finish_init = true;
    }

    void update_map(const sensor_msgs::PointCloud2ConstPtr &cloud, const sensor_msgs::PointCloud2ConstPtr &Nbr, const nav_msgs::OdometryConstPtr &msg)
    {
        if (!is_leader)
        {
            return;
        }
        Eigen::Vector3d sync_my_position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        vector<Eigen::Vector3d> Nbr_point;
        Nbr_point.push_back(sync_my_position);
        CloudOdomPtr Nbr_cloud(new CloudOdom());
        pcl::fromROSMsg(*Nbr, *Nbr_cloud);
        for (const auto &point : Nbr_cloud->points)
        {
            Eigen::Vector3d cloud_point(point.x, point.y, point.z);
            if (std::fabs(point.t - Nbr->header.stamp.toSec()) > 0.2)
            {
                cout << "Missing Nbr" << endl;
            }
            else
            {
                Nbr_point.push_back(cloud_point);
            }
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud, *cloud_cloud);
        for (const auto &point : cloud_cloud->points)
        {
            Eigen::Vector3d cloud_cloud_point(point.x, point.y, point.z);
            if (is_Nbr(cloud_cloud_point, Nbr_point))
            {
                continue;
            }
            else
            {
                insert_point(cloud_cloud_point);
            }
        }
    }
    
    void update_gimbal(Eigen::Vector3d gimbal_position)
    {
        if (!finish_init)
        {
            return;
        }
        Eigen::Matrix3d gimbal_rotation_matrix = Rpy2Rot(gimbal_position);
        Eigen::Matrix3d now_rot = gimbal_rotation_matrix * drone_rotation_matrix;
        Eigen::Vector3d rpy = Rot2rpy(now_rot);
        rpy.x() = 0;

        if (map_set.size() > now_id && !is_transfer)
        {
            map_set[now_id].update_gimbal(rpy, false);
        }
        else
        {
            global_map.update_gimbal(rpy, false);
        }
    }
    
    void update_position(Eigen::Vector3d point, Eigen::Matrix3d rotation)
    {
        if (!odom_get)
        {
            initial_position = point;
            initial_position = initial_position + Eigen::Vector3d(0, 0, 3);
        }
        drone_rotation_matrix = rotation;
        now_global_position = point;
        if (!finish_init)
        {
            return;
        }

        if (map_set.size() > now_id)
        {
            global_map.update_position(point);
            map_set[now_id].update_position(point);
        }
        else
        {
            global_map.update_position(point);
        }
        odom_get = true;
    }
    
    void replan()
    {
        if (!finish_init)
        {
            return;
        }
        if (map_set.size() == now_id && is_transfer)
        {
            // fly home
            if (is_leader)
            {   
                state = 0;
                bool flag = false;
                global_map.Astar_local(initial_position, namespace_, info_mannager.get_leader(), flag, true);
                get_way_point = update_target_waypoint();
                path_show = global_map.get_path_show();
                if (flag)
                {
                    initial_position.z()= initial_position.z()+2;
                }
                return;
            }
            else
            {
                state = 0;
                if (state == 0)
                {
                    is_transfer = true;
                    if (info_mannager.get_leader_state() == 0 && !not_delete)
                    {
                        not_delete = true;
                    }
                }
                if (now_global_position.z() < 8)
                {
                    bool flag = false;

                    global_map.Astar_local(initial_position, namespace_, info_mannager.get_leader(), flag, false);
                    if (flag)
                    {
                        global_map.Astar_local(initial_position, namespace_, info_mannager.get_leader(), flag, true);
                    }
                    get_way_point = update_target_waypoint();
                    path_show = global_map.get_path_show();
                    if (flag)
                    {
                        initial_position.z() += 1;
                    }

                    return;
                }
                else
                {
                    Eigen::Vector3d target;
                    info_mannager.get_leader_position(target);

                    bool flag = false;
                    global_map.Astar_local(target, namespace_, info_mannager.get_leader(), flag, false);
                    // if (flag)
                    // {
                    //     global_map.Astar_local(initial_position, namespace_, info_mannager.get_leader(), flag, true);
                    // }
                    get_way_point = update_target_waypoint();
                    path_show = global_map.get_path_show();

                    return;
                }
            }
        }
        else if (finish_init && is_transfer)
        {
            if (namespace_ == "/jurong" || namespace_ == "/raffles")
            {
               ////yolo();
                Eigen::Vector3d target = map_set[now_id].get_fly_in_point_global();
                bool flag = false;
                global_map.Astar_local(target, namespace_, info_mannager.get_leader(), flag, false);
                get_way_point = update_target_waypoint();
                path_show = global_map.get_path_show();
                map_set[now_id].update_fly_in_index(flag);
                is_transfer = !map_set[now_id].check_whether_fly_in(false);
                if (!is_transfer)
                {
                    map_set[now_id].set_state(1);
                    state = map_set[now_id].get_state();
                }
               ////yolo();
            }
            else
            {
                if (state == 0)
                {
                    is_transfer = true;
                    if (info_mannager.get_leader_state() == 0 && !not_delete)
                    {
                        not_delete = true;
                    }
                }
                if (info_mannager.get_leader_state() == 0)
                {
                    Eigen::Vector3d target;
                    info_mannager.get_leader_position(target);
                    bool flag = false;
                    global_map.Astar_local(target, namespace_, info_mannager.get_leader(), flag, false);
                    get_way_point = update_target_waypoint();
                    path_show = global_map.get_path_show();
                    return;
                }
                else if (state == 1)
                {
                    Eigen::Vector3d target = map_set[now_id].get_fly_in_point_global();
                    bool flag = false;
                    global_map.Astar_photo(target, namespace_, flag);
                    get_way_point = update_target_waypoint();
                    path_show = global_map.get_path_show();
                    is_transfer = !map_set[now_id].check_whether_fly_in(false);
                    if (!is_transfer)
                    {
                        map_set[now_id].set_state(1);
                        state = map_set[now_id].get_state();
                    }
                    return;
                }
                else
                {
                    Eigen::Vector3d target = map_set[now_id].get_fly_in_point_global();
                    bool flag = false;
                    global_map.Astar_photo(target, namespace_, flag);
                    get_way_point = update_target_waypoint();
                    path_show = global_map.get_path_show();
                    return;
                    // bool flag = false;
                    // global_map.Astar_photo(now_global_position, namespace_, flag);
                    // get_way_point = update_target_waypoint();
                    // path_show = global_map.get_path_show();
                    // return;

                }
            }
        }
        else
        {
            if (namespace_ == "/jurong" || namespace_ == "/raffles")
            {
               ////yolo();
                state = map_set[now_id].get_state_leader();
                map_set[now_id].exploration(namespace_);
                path_show = map_set[now_id].get_path_show();
                get_way_point = update_target_waypoint();
               ////yolo();
                if (map_set[now_id].get_whether_pop())
                {
                    now_id++;
                    state = 0;
                    is_transfer = true;
                    return;
                }
            }
            else
            {
                if (map_set.size() > now_id)
                {
                    state = map_set[now_id].get_state();
                }
                else
                {
                    state = 0;
                }
                if (state == 0)
                {
                    is_transfer = true;
                    return;
                }
                if (map_set.size() > now_id)
                {
                    map_set[now_id].take_photo(namespace_);
                    path_show = map_set[now_id].get_path_show();
                    get_way_point = update_target_waypoint();
                }
                else
                {
                    get_way_point = update_target_waypoint();
                }
            }
        }
    }
    
    bool update_target_waypoint()
    {
        if (odom_get && finish_init)
        {
            if (is_transfer || map_set.size() == now_id)
            {
                target_position = global_map.get_next_point(true);
                finish_first_planning = true;
                return true;
            }
            else
            {
                target_position = map_set[now_id].get_next_point(false);
                finish_first_planning = true;
                return true;
            }
        }
        else
        {
            return false;
        }
    }
    
    bool get_cmd(trajectory_msgs::MultiDOFJointTrajectory &cmd, geometry_msgs::Twist &gimbal)
    {
        if (!finish_first_planning)
        {
            return false;
        }
        if (get_way_point)
        {
            if (is_transfer || map_set.size() == now_id)
            {
                global_map.get_gimbal_rpy(target_angle_rpy);
                cmd = position_msg_build(now_global_position, target_position, target_angle_rpy.z());
            }
            else
            {
                try
                {
                    map_set[now_id].get_gimbal_rpy(target_angle_rpy);
                }
                catch (...)
                {
                }

                cmd = position_msg_build(now_global_position, target_position, target_angle_rpy.z());
            }
            gimbal = gimbal_msg_build(target_angle_rpy);
            return true;
        }
        else
        {
            return false;
        }
    }
    
    visualization_msgs::MarkerArray Draw_map()
    {
        if (!finish_init)
        {
            visualization_msgs::MarkerArray nullobj;
            return nullobj;
        }
        if (map_set.size() == now_id)
        {
            return global_map.Draw_map();
        }
        else
        {
            return map_set[now_id].Draw_map();
        }
    }
    nav_msgs::Path Draw_Path()
    {
        path_show.header.frame_id = "world";
        return path_show;
    }
    // communication part
    bool get_position_plan_msg(string &msg)
    {
        if (!odom_get || !finish_init)
        {
            return false;
        }
        else if (!is_planned)
        {
            msg = "position;" + namespace_ + ";" + point2str(now_global_position) + ";";
            return true;
        }
        else
        {
            msg = "position;" + namespace_ + ";" + point2str(now_global_position) + ";" + point2str(planning_point);
            return true;
        }
    }

    bool get_global_massage(string &msg)
    {
        if (!is_leader || !finish_init)
        {
            return false;
        }
        msg = "mapglobal;" + to_string(Teamid) + ";" + global_map.get_num_str() + global_map.get_map_str();
        return true;
    }
    bool get_local_massage(string &msg)
    {
        if (!is_leader || !finish_init || state >= 2 || map_set.size() == now_id)
        {
            return false;
        }
        msg = "map;" + to_string(Teamid) + ';' + map_set[now_id].get_num_str() + map_set[now_id].get_map_str();
        return true;
    }
    bool get_fly_in_massage(string &msg)
    {
        if (!is_leader || !finish_init || map_set.size() == now_id)
        {
            return false;
        }
        msg = "flyin;" + to_string(Teamid) + ';' + map_set[now_id].get_fly_in_str();
        return true;
    }
    bool get_state_set_msg_list(list<string> &string_list)
    {
        if (!is_leader || !finish_init || map_set.size() == now_id)
        {
            return false;
        }
        list<string> string_list_tamp = map_set[now_id].get_state_string_list();
        if (string_list_tamp.empty())
        {
            return false;
        }
        else
        {
            for (auto &str : string_list_tamp)
            {
                string_list.push_back("state_set;" + to_string(Teamid) + ";" + str);
            }
            return true;
        }
    }
    bool get_state_massage(string &msg)
    {
        if (!finish_init)
        {
            return false;
        }
        msg = "state;" + namespace_ + ";" + to_string(state) + ";";
        return true;
    }

    void communicate(string str)
    {
        if (!finish_init)
        {
            return;
        }

        istringstream msg_stream(str);
        string topic;
        getline(msg_stream, topic, ';');

        if (topic == "position")
        {
            if (not_delete == false || (!is_leader && state == 2))
            {
                return;
            }
            istringstream global_po_str(str);
            istringstream local_po_str(str);
            getline(global_po_str, topic, ';');
            getline(local_po_str, topic, ';');
            info_mannager.reset_position_path(msg_stream);
            if (map_set.size() > now_id)
            {
                global_map.update_local_dict(global_po_str);
                map_set[now_id].update_local_dict(local_po_str);
            }
            else
            {
                global_map.update_local_dict(global_po_str);
            }

            return;
        }
        else if (topic == "state")
        {
            string orin;
            getline(msg_stream, orin, ';');
            string state_str;
            getline(msg_stream, state_str, ';');
            if (map_set.size() > now_id)
            {
                try
                {
                    if (orin == info_mannager.get_leader() && stoi(state_str) == 3 && state == 2 && not_delete)
                    {
                        not_delete = false;
                        now_id++;
                        return;
                    }
                    map_set[now_id].update_state(orin, stoi(state_str));
                    info_mannager.update_state(orin, stoi(state_str));
                }
                catch (const std::invalid_argument &e)
                {

                    cout << "Invalid argument" << e.what() << endl;
                }
                catch (const std::out_of_range &e)
                {
                    cout << "Out of range" << e.what() << endl;
                }
            }
            else
            {
                try
                {
                    info_mannager.update_state(orin, stoi(state_str));
                }
                catch (const std::invalid_argument &e)
                {
                    cout << "Invalid argument" << e.what() << endl;
                }
                catch (const std::out_of_range &e)
                {
                    cout << "Out of range" << e.what() << endl;
                }
                return;
            }
        }
        else if (topic == "state_set")
        {
            string target_team;
            if (not_delete == false || (!is_leader && state == 2 || map_set.size() == now_id))
            {
                return;
            }
            getline(msg_stream, target_team, ';');
            if (stoi(target_team) == Teamid)
            {
                string target_name;
                getline(msg_stream, target_name, ';');
                if (namespace_ == target_name)
                {
                    string state_str;
                    getline(msg_stream, state_str, ';');
                    state = stoi(state_str);
                    return;
                }
                else
                {
                    return;
                }
            }
            else
            {
                return;
            }
        }
        else if (topic == "map")
        {
            if (not_delete == false || (!is_leader && state == 2) || map_set.size() == now_id)
            {
                return;
            }
            string target_team;
            getline(msg_stream, target_team, ';');
            // cout<<"target team:"<<target_team<<endl;
            if (stoi(target_team) == Teamid)
            {
                // insert map_front from string
                if (map_set.size() > now_id)
                {
                    map_set[now_id].insert_cloud_from_str(msg_stream);
                }
                else
                {
                    return;
                }
            }
            else
            {
                return;
            }
        }
        else if (topic == "mapglobal")
        {
            string target_team;
            if (not_delete == false || (!is_leader && state == 2))
            {
                return;
            }

            getline(msg_stream, target_team, ';');

            if (stoi(target_team) == Teamid)
            {
                global_map.insert_cloud_from_str(msg_stream);
            }
            else
            {
                return;
            }
        }
        else if (topic == "visit")
        {
            string target_team;
            getline(msg_stream, target_team, ';');
            if (stoi(target_team) == Teamid)
            {
                // insert map_front from string
                // not develop this function now
            }
            else
            {
                return;
            }
        }
        else if (topic == "flyin")
        {
            if (not_delete == false || (!is_leader && state == 2) || map_set.size() == now_id)
            {

                return;
            }
            string target_team;
            getline(msg_stream, target_team, ';');
            if (stoi(target_team) == Teamid)
            {
                if (map_set.size() >= now_id)
                {
                    // insert fly_in_index
                    string fly_in_index;
                    getline(msg_stream, fly_in_index, ';');
                    map_set[now_id].set_fly_in_index(fly_in_index);
                }
                else
                {
                    return;
                }
            }
            else
            {

                return;
            }
        }
    }

private:
    int now_id = 0;
    int map_set_use = 0;
    int state = 0;
    bool not_delete = true;
    bool is_planned = false;
    Eigen::Vector3d planning_point;
    Eigen::Vector3d initial_position;
    bool is_leader = false;
    vector<grid_map> map_set;
    grid_map global_map;

    int lowest_bound = 0;
    int highest_bound = 0;

    Eigen::Vector3d grid_size;
    vector<int> path_index;
    double safe_distance = 2.5;
    bool finish_init = false;
    bool odom_get = false;
    string namespace_;
    int Teamid;
    info_agent info_mannager;
    Eigen::Vector3d now_global_position;
    list<Eigen::Vector3i> global_index_path;
    list<Eigen::Vector3d> global_path;
    Eigen::Vector3d now_gimbal_position;
    nav_msgs::Path path_show;
    Eigen::Matrix3d gimbal_rotation_matrix;
    Eigen::Matrix3d drone_rotation_matrix;
    bool is_transfer = true;
    bool is_exploration = false;
    Eigen::Vector3d target_angle_rpy;
    Eigen::Vector3d target_position;
    bool get_way_point = false;
    bool finish_first_planning = false;
    vector<string> teammates_name;
    void generate_global_map(string s)
    {
        vector<string> spilited_str;
        std::istringstream iss(s);
        std::string substring;
        while (std::getline(iss, substring, ','))
        {
            spilited_str.push_back(substring);
        }
        Eigen::Vector3d max_region(stod(spilited_str[3]), stod(spilited_str[4]), stod(spilited_str[5]));
        Eigen::Vector3d min_region(stod(spilited_str[0]), stod(spilited_str[1]), stod(spilited_str[2]));
        vector<vector<string>> teams(2);
        vector<int> size_of_path(2);
        for (int i = 6; i < spilited_str.size(); i++)
        {
            if (spilited_str[i] == "team")
            {
                for (int j = i + 3; j < i + 3 + stoi(spilited_str[i + 2]); j++)
                {
                    teams[stoi(spilited_str[i + 1])].push_back(spilited_str[j]);
                }
                i = i + 2 + stoi(spilited_str[i + 2]);
                continue;
            }
            if (spilited_str[i] == "path_size")
            {
                size_of_path[stoi(spilited_str[i + 1])] = stoi(spilited_str[i + 2]);
                i = i + 2;
                continue;
            }
        }
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < teams[i].size(); j++)
            {
                if ("/" + teams[i][j] == namespace_)
                {
                    Teamid = i;
                    break;
                }
            }
        }
        cout << "TeamID:" << Teamid << endl;
        if (Teamid == 0)
        {
            for (int i = 0; i < size_of_path[0]; i++)
            {
                path_index.push_back(i + 1);
                cout << i + 1 << endl;
            }
            teammates_name = teams[Teamid];
        }
        else
        {
            for (int i = 0; i < size_of_path[1]; i++)
            {
                path_index.push_back(i + 1 + size_of_path[0]);
                cout << i + 1 + size_of_path[0] << endl;
            }
            teammates_name = teams[Teamid];
        }
        info_mannager = info_agent(teammates_name);
    }
    void insert_point(Eigen::Vector3d point_in)
    {
        if (map_set.size() > now_id)
        {
            global_map.insert_point(point_in);
            for (auto &element : map_set)
            {
                element.insert_point(point_in);
            }
        }
        else
        {
            global_map.insert_point(point_in);
        }
    }
    bool is_Nbr(Eigen::Vector3d test, vector<Eigen::Vector3d> Nbr_point)
    {
        if (Nbr_point.size() == 0)
        {
            return false;
        }
        else
        {
            Eigen::Vector3d collision_box_size = grid_size;
            for (int i = 0; i < Nbr_point.size(); i++)
            {
                Eigen::Vector3d Nbr = Nbr_point[i];
                Eigen::Vector3d diff = Nbr - test;
                if (fabs(diff[0]) <= collision_box_size[0] && fabs(diff[1]) <= collision_box_size[1] && fabs(diff[2]) <= collision_box_size[2])
                {
                    return true;
                }
            }
            return false;
        }
    }
    trajectory_msgs::MultiDOFJointTrajectory position_msg_build(Eigen::Vector3d position, Eigen::Vector3d target, double target_yaw)
    {
        is_planned = true;
        planning_point = target;
        if (fabs(target_yaw) < M_PI / 2)
        {
            target_yaw = 0;
        }
        trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;
        trajset_msg.header.frame_id = "world";
        geometry_msgs::Transform transform_msg;
        geometry_msgs::Twist accel_msg, vel_msg;

        Eigen::Vector3d difference = (target - position);
        if (difference.norm() < 2)
        {
            transform_msg.translation.x = target.x();
            transform_msg.translation.y = target.y();
            transform_msg.translation.z = target.z();
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_msg.linear.z = 0;
        }
        else
        {
            Eigen::Vector3d target_pos = 2 * difference / difference.norm();
            transform_msg.translation.x = 0;
            transform_msg.translation.y = 0;
            transform_msg.translation.z = 0;
            vel_msg.linear.x = target_pos.x();
            vel_msg.linear.y = target_pos.y();
            vel_msg.linear.z = target_pos.z();
        }
        transform_msg.rotation.x = 0;
        transform_msg.rotation.y = 0;
        transform_msg.rotation.z = sinf(target_yaw * 0.5);
        transform_msg.rotation.w = cosf(target_yaw * 0.5);

        trajpt_msg.transforms.push_back(transform_msg);

        accel_msg.linear.x = 0;
        accel_msg.linear.y = 0;
        accel_msg.linear.z = 0;

        trajpt_msg.velocities.push_back(vel_msg);
        trajpt_msg.accelerations.push_back(accel_msg);
        trajset_msg.points.push_back(trajpt_msg);

        trajset_msg.header.frame_id = "world";
        return trajset_msg;
    }
    geometry_msgs::Twist gimbal_msg_build(Eigen::Vector3d target_euler_rpy)
    {
        geometry_msgs::Twist gimbal_msg;
        gimbal_msg.linear.x = 1.0; // setting linear.x to -1.0 enables velocity control mode.
        if (fabs(target_euler_rpy.z()) < M_PI / 2)
        {
            gimbal_msg.linear.y = target_euler_rpy.y(); // if linear.x set to 1.0, linear,y and linear.z are the
            gimbal_msg.linear.z = target_euler_rpy.z(); // target pitch and yaw angle, respectively.
        }
        else
        {
            gimbal_msg.linear.y = target_euler_rpy.y(); // if linear.x set to 1.0, linear,y and linear.z are the
            gimbal_msg.linear.z = 0;                    // target pitch and yaw angle, respectively.
        }
        gimbal_msg.angular.x = 0.0;
        gimbal_msg.angular.y = 0.0; // in velocity control mode, this is the target pitch velocity
        gimbal_msg.angular.z = 0.0; // in velocity control mode, this is the target yaw velocity
        return gimbal_msg;
    }
    Eigen::Matrix3d Rpy2Rot(Eigen::Vector3d rpy)
    {
        Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
        result = Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix() * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
        return result;
    }
    Eigen::Vector3d Rot2rpy(Eigen::Matrix3d R)
    {

        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d rpy(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        rpy(0) = r;
        rpy(1) = p;
        rpy(2) = y;

        return rpy;
    }
    string point2str(Eigen::Vector3d point)
    {
        string result;
        result = to_string(point.x()) + "," + to_string(point.y()) + "," + to_string(point.z());
        return result;
    }
    Eigen::Vector3d str2point(string input)
    {
        Eigen::Vector3d result;
        std::vector<string> value;
        boost::split(value, input, boost::is_any_of(","));
        if (value.size() == 3)
        {
            result = Eigen::Vector3d(stod(value[0]), stod(value[1]), stod(value[2]));
        }
        else
        {
            cout << "error use str2point 1" << endl;
        }
        return result;
    }
};

class Agent
{
public:
    Agent(ros::NodeHandlePtr &nh_ptr_)
    : nh_ptr(nh_ptr_)
    {

        TimerProbeNbr = nh_ptr->createTimer(ros::Duration(1.0 / 10.0), &Agent::TimerProbeNbrCB, this);
        TimerPlan     = nh_ptr->createTimer(ros::Duration(1.0 / 2.0),  &Agent::TimerPlanCB,     this);
        TimerCmdOut   = nh_ptr->createTimer(ros::Duration(1.0 / 10.0), &Agent::TimerCmdOutCB,   this);
        TimerViz      = nh_ptr->createTimer(ros::Duration(1.0 / 1.0),  &Agent::TimerVizCB,      this);

        task_sub_ = nh_ptr->subscribe("/task_assign" + nh_ptr->getNamespace(), 10, &Agent::TaskCallback, this);
        com_sub_  = nh_ptr->subscribe("/broadcast" + nh_ptr->getNamespace(), 10, &Agent::ComCallback, this);
        client    = nh_ptr->serviceClient<caric_mission::CreatePPComTopic>("/create_ppcom_topic");
        communication_pub_ = nh_ptr->advertise<std_msgs::String>("/broadcast", 10);

        string str = nh_ptr->getNamespace();
        str.erase(0, 1);
        srv.request.source = str;
        srv.request.targets.push_back("all");
        srv.request.topic_name = "/broadcast";
        srv.request.package_name = "std_msgs";
        srv.request.message_type = "String";
        while (!serviceAvailable)
        {
            serviceAvailable = ros::service::waitForService("/create_ppcom_topic", ros::Duration(10.0));
        }
        string result = "Begin";
        while (result != "success lah!")
        {
            client.call(srv);
            result = srv.response.result;
            printf(KYEL "%s\n" RESET, result.c_str());
            std::this_thread::sleep_for(chrono::milliseconds(1000));
        }
        communication_initialise = true;

        odom_sub_        = nh_ptr->subscribe("/ground_truth/odometry", 10, &Agent::OdomCallback, this);
        gimbal_sub_      = nh_ptr->subscribe("/firefly/gimbal", 10, &Agent::GimbalCallback, this);

        cloud_sub_       = new message_filters::Subscriber<sensor_msgs::PointCloud2>(*nh_ptr, "/cloud_inW", 10);
        nbr_sub_         = new message_filters::Subscriber<sensor_msgs::PointCloud2>(*nh_ptr, "/nbr_odom_cloud", 10);
        odom_filter_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(*nh_ptr, "/ground_truth/odometry", 10);
        sync_            = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *cloud_sub_, *nbr_sub_, *odom_filter_sub_);

        sync_->registerCallback(boost::bind(&Agent::MapCallback, this, _1, _2, _3));

        motion_pub_     = nh_ptr->advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 1);
        gimbal_pub_     = nh_ptr->advertise<geometry_msgs::Twist>("/firefly/command/gimbal", 1);

        map_marker_pub_ = nh_ptr->advertise<visualization_msgs::MarkerArray>("/firefly/map", 1);
        path_pub_       = nh_ptr->advertise<nav_msgs::Path>("/firefly/path_show", 10);
    }

    void MapCallback(const sensor_msgs::PointCloud2ConstPtr &cloud,
                     const sensor_msgs::PointCloud2ConstPtr &Nbr,
                     const nav_msgs::OdometryConstPtr &msg)
    {

        // ensure the map initialization finished
        if (!map_initialise)
        {
            return;
        }
        // ensure time of messages sync
        if (std::fabs(cloud->header.stamp.toSec() - Nbr->header.stamp.toSec()) > 0.2)
        {
            return;
        }
        mm.update_map(cloud, Nbr, msg);
    }

private:
    ros::NodeHandlePtr nh_ptr;  // nodehandle for communication

    ros::Timer TimerProbeNbr;   // To request updates from neighbours
    ros::Timer TimerPlan;       // To design a trajectory
    ros::Timer TimerCmdOut;     // To issue control setpoint to unicon
    ros::Timer TimerViz;        // To vizualize internal states

    // callback Q1
    caric_mission::CreatePPComTopic srv; // This PPcom create for communication between neibors;
    ros::ServiceClient client;           // The client to create ppcom
    ros::Publisher communication_pub_;   // PPcom publish com
    bool serviceAvailable = false;       // The flag whether the communication service is ready
    ros::Subscriber task_sub_;
    ros::Subscriber com_sub_;
    string pre_task;

    // callback Q2
    ros::Subscriber odom_sub_;   // Get neibor_info update
    ros::Subscriber gimbal_sub_; // Get gimbal info update;
    // callback Q3
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *nbr_sub_;
    message_filters::Subscriber<nav_msgs::Odometry>       *odom_filter_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                            sensor_msgs::PointCloud2,
                                                            nav_msgs::Odometry> MySyncPolicy;
    // // boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    message_filters::Synchronizer<MySyncPolicy> *sync_;

    // callback Q4
    ros::Publisher motion_pub_; // motion command pub
    ros::Publisher gimbal_pub_; // motion gimbal pub

    // callback Q5
    ros::Publisher map_marker_pub_;
    ros::Publisher path_pub_;

    mainbrain mm;

    // variable for static map
    vector<Eigen::Vector3d> Nbr_point;

    bool map_initialise = false;
    bool communication_initialise = false;
    // Callback function

    void TaskCallback(const std_msgs::String msg)
    {

        // cout<<nh_ptr->getNamespace()<<"Task begin"<<endl;
        if (pre_task == msg.data && pre_task != "")
        {
            map_initialise = true;
            return;
        }

        mm = mainbrain(msg.data, nh_ptr->getNamespace());
        pre_task = msg.data;
        map_initialise = true;
    }

    void ComCallback(const std_msgs::String msg)
    {
        if(!map_initialise)
        {
            return;
        }
        mm.communicate(msg.data);
        if (!serviceAvailable || !communication_initialise)
        {
            return;
        }
        std_msgs::String msg_map;
        if (mm.get_global_massage(msg_map.data))
        {
            communication_pub_.publish(msg_map);
        }
        if (mm.get_local_massage(msg_map.data))
        {
            communication_pub_.publish(msg_map);
        }
        if (mm.get_fly_in_massage(msg_map.data))
        {
            communication_pub_.publish(msg_map);
        }
        if (mm.get_state_massage(msg_map.data))
        {
            communication_pub_.publish(msg_map);
        }
        list<string> msg_list;
        if (mm.get_state_set_msg_list(msg_list))
        {
            for (auto &str : msg_list)
            {
                std_msgs::String msg_list_item;
                msg_list_item.data = str;
                communication_pub_.publish(msg_list_item);
            }
        }

    }

    void OdomCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        if(!map_initialise)
        {
            Eigen::Vector3d initial_position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
            std_msgs::String init_position_msg;
            init_position_msg.data="init_pos;"+nh_ptr->getNamespace()+";"+to_string(initial_position.x())+","+to_string(initial_position.y())+","+to_string(initial_position.z());
            if(communication_initialise)
            {
                communication_pub_.publish(init_position_msg);
            }
            return;
        }

        Eigen::Vector3d my_position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Eigen::Matrix3d R = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();
        mm.update_position(my_position, R);
    }

    void GimbalCallback(const geometry_msgs::TwistStamped &msg)
    {
        if(!map_initialise)
        {
            return;
        }
        Eigen::Vector3d position = Eigen::Vector3d(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
        mm.update_gimbal(position);
    }

    void TimerProbeNbrCB(const ros::TimerEvent &)
    {
        if (!serviceAvailable || !map_initialise)
        {
            return;
        }
        std_msgs::String msg;
        if (mm.get_position_plan_msg(msg.data))
        {
            communication_pub_.publish(msg);
        }
        else
        {
            return;
        }
        return;
    }
    void TimerPlanCB(const ros::TimerEvent &)
    {
        if (!map_initialise)
        {
            return;
        }
       ////yolo();
        mm.replan();
       ////yolo();
        return;
    }

    void TimerCmdOutCB(const ros::TimerEvent &)
    {
        if (!map_initialise)
        {
            return;
        }
        
        trajectory_msgs::MultiDOFJointTrajectory position_cmd;
        geometry_msgs::Twist gimbal_msg;
        
        if (mm.get_cmd(position_cmd, gimbal_msg))
        {
            position_cmd.header.stamp = ros::Time::now();
            motion_pub_.publish(position_cmd);
            gimbal_pub_.publish(gimbal_msg);
        }

        return;
    }
    void TimerVizCB(const ros::TimerEvent &)
    {

        if (!map_initialise)
        {
            return;
        }

        map_marker_pub_.publish(mm.Draw_map());
        path_pub_.publish(mm.Draw_Path());

        return;
    }
};
