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
#include <pcl/kdtree/kdtree_flann.h>

#include "utility.h"
#include <mutex>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <caric_mission/CreatePPComTopic.h>
#include <std_msgs/String.h>

struct position_info{
    Eigen::Vector3d position;
    bool update=false;
};

class Boundingbox
{
public:

    Boundingbox()
    {
        center = Eigen::Vector3d(0, 0, 0);
        volume = 0;
        id = -1;
    };

    Boundingbox(string str)
    {
        vector<string> spilited_str;
        std::istringstream iss(str);
        std::string substring;
        while (std::getline(iss, substring, ','))
        {
            spilited_str.push_back(substring);
        }
        
        int i = 0;
        while (i < 24)
        {
            vertice.push_back(Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2])));
            i = i + 3;
        }
        
        while (i < 33)
        {
            rotation_matrix(i - 24) = stod(spilited_str[i]);
            i++;
        }
        while (i < 36)
        {
            center = Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2]));
            i = i + 3;
        }
        while (i < 39)
        {
            size_vector = Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2]));
            i = i + 3;
        }
        while (i < 42)
        {
            global_in_out.push_back(Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2])));
            i = i + 3;
        }
        while (i < 45)
        {
            global_in_out.push_back(Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2])));
            i = i + 3;
        }
        
        xsize = stod(spilited_str[i]);
        i++;
        ysize = stod(spilited_str[i]);
        i++;
        zsize = stod(spilited_str[i]);
        i++;
        id = stod(spilited_str[i]);
        i++;
        state = stod(spilited_str[i]);
        i++;
        volume = stod(spilited_str[i]);
        i++;
        use_x = stod(spilited_str[i]);
        i++;
        use_y = stod(spilited_str[i]);
        i++;
        use_z = stod(spilited_str[i]);
        i++;
    }

    Boundingbox(const std::vector<Eigen::Vector3d> &vec, int id_in, Eigen::Vector3d &start, Eigen::Vector3d &end, int state_in, bool x, bool y, bool z)
    {
        vertice = vec;
        id = id_in;
        center = Eigen::Vector3d(0, 0, 0);
        for (int index = 0; index < vec.size(); index++)
        {
            center = center + vec[index];
        }
        center = center / 8;
        xsize = (vec[1] - vec[0]).norm();
        ysize = (vec[3] - vec[0]).norm();
        zsize = (vec[4] - vec[0]).norm();
        volume = xsize * ysize * zsize;
        size_vector = Eigen::Vector3d(xsize / 2, ysize / 2, zsize / 2);
        Eigen::Vector3d xaxis, yaxis, zaxis;
        xaxis = (vec[1] - vec[0]).normalized();
        yaxis = (vec[3] - vec[0]).normalized();
        zaxis = (vec[4] - vec[0]).normalized();
        rotation_matrix << xaxis, yaxis, zaxis;
        global_in_out.push_back(start);
        global_in_out.push_back(end);
        state = state_in;
        use_x = x;
        use_y = y;
        use_z = z;
    }
    
    Boundingbox(const std::vector<Eigen::Vector3d> &vec, int id_in)
    {
        vertice = vec;
        id = id_in;
        center = Eigen::Vector3d(0, 0, 0);
        for (int index = 0; index < vec.size(); index++)
        {
            center = center + vec[index];
        }
        center = center / 8;
        xsize = (vec[1] - vec[0]).norm();
        ysize = (vec[3] - vec[0]).norm();
        zsize = (vec[4] - vec[0]).norm();

        volume = xsize * ysize * zsize;
        size_vector = Eigen::Vector3d(xsize / 2, ysize / 2, zsize / 2);

        Eigen::Vector3d xaxis, yaxis, zaxis;
        xaxis = (vec[1] - vec[0]).normalized();
        yaxis = (vec[3] - vec[0]).normalized();
        zaxis = (vec[4] - vec[0]).normalized();

        Eigen::Vector3d xplus, xminus, yplus, yminus, zplus, zminus;
        yminus = (vec[0] + vec[1] + vec[4] + vec[5]) / 4;
        yplus = (vec[2] + vec[3] + vec[6] + vec[7]) / 4;

        xminus = (vec[0] + vec[3] + vec[4] + vec[7]) / 4;
        xplus = (vec[1] + vec[2] + vec[5] + vec[6]) / 4;

        zminus = (vec[0] + vec[1] + vec[2] + vec[3]) / 4;
        zplus = (vec[4] + vec[5] + vec[6] + vec[7]) / 4;

        rotation_matrix << xaxis, yaxis, zaxis;
        if ((zsize >= ysize) && (zsize >= xsize))
        {
            use_z = true;
            global_in_out.push_back(zminus);
            global_in_out.push_back(zplus);
        }
        else if ((xsize >= ysize) && (xsize >= zsize))
        {
            if (global_in_out.size() == 0)
            {
                use_x = true;
                global_in_out.push_back(xminus);
                global_in_out.push_back(xplus);
            }
        }
        else if ((ysize >= xsize) && (ysize >= zsize))
        {
            if (global_in_out.size() == 0)
            {
                use_y = true;
                global_in_out.push_back(yminus);
                global_in_out.push_back(yplus);
            }
        }
        else
        {
            if (global_in_out.size() == 0)
            {
                use_z = true;
                global_in_out.push_back(zminus);
                global_in_out.push_back(zplus);
            }
        }
    };

    ~Boundingbox(){};
    
    const Matrix3d getSearchRotation() const
    {
        Eigen::Vector3d axis_rotation_along(0.0, 0.0, 1.0);
        Eigen::Matrix3d transfer_matrix;
        Eigen::Matrix3d result;
        double angle = 0;
        if (use_x)
        {
            if (state == 0)
            {
                cout << "x+" << endl;
                axis_rotation_along = Eigen::Vector3d(0.0, 1.0, 0.0);
                angle = -M_PI / 2;
            }
            else if (state == 1)
            {
                cout << "x-" << endl;
                axis_rotation_along = Eigen::Vector3d(0.0, 1.0, 0.0);
                angle = M_PI / 2;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else if (use_y)
        {
            if (state == 0)
            {
                cout << "y+" << endl;
                axis_rotation_along = Eigen::Vector3d(1.0, 0.0, 0.0);
                angle = M_PI / 2;
            }
            else if (state == 1)
            {
                cout << "y-" << endl;
                axis_rotation_along = Eigen::Vector3d(1.0, 0.0, 0.0);
                angle = -M_PI / 2;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else if (use_z)
        {
            if (state == 0)
            {
                cout << "z+" << endl;
            }
            else if (state == 1)
            {
                cout << "z-" << endl;
                axis_rotation_along = Eigen::Vector3d(1.0, 0.0, 0.0);
                angle = M_PI;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else
        {
            cout << "noting happen" << endl;
        }

        transfer_matrix = Eigen::AngleAxisd(angle, axis_rotation_along);
        result = transfer_matrix * rotation_matrix.inverse();
        return result;
        
    }

    int getState()
    {
        return state;
    }

    double getVolume() const
    {
        return volume;
    }
    const Vector3d getCenter() const
    {
        return center;
    }
    const Matrix3d getRotation() const
    {
        return rotation_matrix;
    }
    const Vector3d getExtents() const
    {
        return size_vector;
    }
    const Vector3d getRotExtents() const
    {
        Eigen::Vector3d result(0, 0, 0);
        if (use_x)
        {
            if (state == 0)
            {
                // cout<<"x+"<<endl;
                result.x() = size_vector.z();
                result.y() = size_vector.y();
                result.z() = size_vector.x();
            }
            else if (state == 1)
            {
                result.x() = size_vector.z();
                result.y() = size_vector.y();
                result.z() = size_vector.x();
                // cout<<"x-"<<endl;
                // axis_rotation_along=Eigen::Vector3d(0.0,1.0,0.0);
                // angle=M_PI/2;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else if (use_y)
        {
            if (state == 0)
            {
                // cout<<"y+"<<endl;
                result.x() = size_vector.x();
                result.y() = size_vector.z();
                result.z() = size_vector.y();
            }
            else if (state == 1)
            {
                // cout<<"y-"<<endl;
                result.x() = size_vector.x();
                result.y() = size_vector.z();
                result.z() = size_vector.y();
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else if (use_z)
        {
            if (state == 0)
            {
                // cout<<"z+"<<endl;
                result = size_vector;
            }
            else if (state == 1)
            {
                // cout<<"z-"<<endl;
                result = size_vector;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else
        {
            cout << "noting happen" << endl;
        }
        return result;
    }
    vector<Eigen::Vector3d> getVertices() const
    {
        return vertice;
    }
    Eigen::Vector3d get_global_in_out(int state) const
    {
        Eigen::Vector3d result = global_in_out[state];
        return result;
    }
    void edit_state(int state_in)
    {
        state = state_in;
    }
    void edit_id()
    {
        id = id + 1;
    }
    int getId()
    {
        return id;
    }

    void generate_start(double scale, Boundingbox &start, Boundingbox &end)
    {
        if (use_z)
        {
            vector<Eigen::Vector3d> new_vertice_start(8);
            vector<Eigen::Vector3d> new_vertice_end(8);
            if (state == 0)
            {
                new_vertice_start[0] = vertice[0];
                new_vertice_start[1] = vertice[1];
                new_vertice_start[2] = vertice[2];
                new_vertice_start[3] = vertice[3];
                new_vertice_start[4] = vertice[0] + (vertice[4] - vertice[0]) * scale;
                new_vertice_start[5] = vertice[1] + (vertice[5] - vertice[1]) * scale;
                new_vertice_start[6] = vertice[2] + (vertice[6] - vertice[2]) * scale;
                new_vertice_start[7] = vertice[3] + (vertice[7] - vertice[3]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[4] - vertice[0]) * scale;
                new_vertice_end[1] = vertice[1] + (vertice[5] - vertice[1]) * scale;
                new_vertice_end[2] = vertice[2] + (vertice[6] - vertice[2]) * scale;
                new_vertice_end[3] = vertice[3] + (vertice[7] - vertice[3]) * scale;

                new_vertice_end[4] = vertice[4];
                new_vertice_end[5] = vertice[5];
                new_vertice_end[6] = vertice[6];
                new_vertice_end[7] = vertice[7];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[1] + new_vertice_start[2] + new_vertice_start[3]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[4] + new_vertice_start[5] + new_vertice_start[6] + new_vertice_start[7]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[1] + new_vertice_end[2] + new_vertice_end[3]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[4] + new_vertice_end[5] + new_vertice_end[6] + new_vertice_end[7]) / 4;

                start = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
            else
            {
                scale = 1 - scale;
                new_vertice_start[0] = vertice[0];
                new_vertice_start[1] = vertice[1];
                new_vertice_start[2] = vertice[2];
                new_vertice_start[3] = vertice[3];
                new_vertice_start[4] = vertice[0] + (vertice[4] - vertice[0]) * scale;
                new_vertice_start[5] = vertice[1] + (vertice[5] - vertice[1]) * scale;
                new_vertice_start[6] = vertice[2] + (vertice[6] - vertice[2]) * scale;
                new_vertice_start[7] = vertice[3] + (vertice[7] - vertice[3]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[4] - vertice[0]) * scale;
                new_vertice_end[1] = vertice[1] + (vertice[5] - vertice[1]) * scale;
                new_vertice_end[2] = vertice[2] + (vertice[6] - vertice[2]) * scale;
                new_vertice_end[3] = vertice[3] + (vertice[7] - vertice[3]) * scale;

                new_vertice_end[4] = vertice[4];
                new_vertice_end[5] = vertice[5];
                new_vertice_end[6] = vertice[6];
                new_vertice_end[7] = vertice[7];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[1] + new_vertice_start[2] + new_vertice_start[3]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[4] + new_vertice_start[5] + new_vertice_start[6] + new_vertice_start[7]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[1] + new_vertice_end[2] + new_vertice_end[3]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[4] + new_vertice_end[5] + new_vertice_end[6] + new_vertice_end[7]) / 4;

                start = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
        }
        else if (use_x)
        {
            vector<Eigen::Vector3d> new_vertice_start(8);
            vector<Eigen::Vector3d> new_vertice_end(8);
            if (state == 0)
            {
                new_vertice_start[0] = vertice[0];
                new_vertice_start[3] = vertice[3];
                new_vertice_start[4] = vertice[4];
                new_vertice_start[7] = vertice[7];
                new_vertice_start[1] = vertice[0] + (vertice[1] - vertice[0]) * scale;
                new_vertice_start[2] = vertice[3] + (vertice[2] - vertice[3]) * scale;
                new_vertice_start[5] = vertice[4] + (vertice[5] - vertice[4]) * scale;
                new_vertice_start[6] = vertice[7] + (vertice[6] - vertice[7]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[1] - vertice[0]) * scale;
                new_vertice_end[3] = vertice[3] + (vertice[2] - vertice[3]) * scale;
                new_vertice_end[4] = vertice[4] + (vertice[5] - vertice[4]) * scale;
                new_vertice_end[7] = vertice[7] + (vertice[6] - vertice[7]) * scale;

                new_vertice_end[1] = vertice[1];
                new_vertice_end[2] = vertice[2];
                new_vertice_end[5] = vertice[5];
                new_vertice_end[6] = vertice[6];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[3] + new_vertice_start[4] + new_vertice_start[7]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[1] + new_vertice_start[2] + new_vertice_start[5] + new_vertice_start[6]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[3] + new_vertice_end[4] + new_vertice_end[7]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[1] + new_vertice_end[2] + new_vertice_end[5] + new_vertice_end[6]) / 4;

                start = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
            else
            {
                scale = 1 - scale;
                new_vertice_start[0] = vertice[0];
                new_vertice_start[3] = vertice[3];
                new_vertice_start[4] = vertice[4];
                new_vertice_start[7] = vertice[7];
                new_vertice_start[1] = vertice[0] + (vertice[1] - vertice[0]) * scale;
                new_vertice_start[2] = vertice[3] + (vertice[2] - vertice[3]) * scale;
                new_vertice_start[5] = vertice[4] + (vertice[5] - vertice[4]) * scale;
                new_vertice_start[6] = vertice[7] + (vertice[6] - vertice[7]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[1] - vertice[0]) * scale;
                new_vertice_end[3] = vertice[3] + (vertice[2] - vertice[3]) * scale;
                new_vertice_end[4] = vertice[4] + (vertice[5] - vertice[4]) * scale;
                new_vertice_end[7] = vertice[7] + (vertice[6] - vertice[7]) * scale;

                new_vertice_end[1] = vertice[1];
                new_vertice_end[2] = vertice[2];
                new_vertice_end[5] = vertice[5];
                new_vertice_end[6] = vertice[6];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[3] + new_vertice_start[4] + new_vertice_start[7]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[1] + new_vertice_start[2] + new_vertice_start[5] + new_vertice_start[6]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[3] + new_vertice_end[4] + new_vertice_end[7]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[1] + new_vertice_end[2] + new_vertice_end[5] + new_vertice_end[6]) / 4;

                start = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
        }
        else
        {
            vector<Eigen::Vector3d> new_vertice_start(8);
            vector<Eigen::Vector3d> new_vertice_end(8);
            if (state == 0)
            {
                new_vertice_start[0] = vertice[0];
                new_vertice_start[1] = vertice[1];
                new_vertice_start[5] = vertice[5];
                new_vertice_start[4] = vertice[4];
                new_vertice_start[3] = vertice[0] + (vertice[3] - vertice[0]) * scale;
                new_vertice_start[2] = vertice[1] + (vertice[2] - vertice[1]) * scale;
                new_vertice_start[6] = vertice[5] + (vertice[6] - vertice[5]) * scale;
                new_vertice_start[7] = vertice[4] + (vertice[7] - vertice[4]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[3] - vertice[0]) * scale;
                new_vertice_end[1] = vertice[1] + (vertice[2] - vertice[1]) * scale;
                new_vertice_end[5] = vertice[5] + (vertice[6] - vertice[5]) * scale;
                new_vertice_end[4] = vertice[4] + (vertice[7] - vertice[4]) * scale;

                new_vertice_end[3] = vertice[3];
                new_vertice_end[2] = vertice[2];
                new_vertice_end[6] = vertice[6];
                new_vertice_end[7] = vertice[7];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[1] + new_vertice_start[5] + new_vertice_start[4]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[3] + new_vertice_start[2] + new_vertice_start[6] + new_vertice_start[7]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[1] + new_vertice_end[5] + new_vertice_end[4]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[3] + new_vertice_end[2] + new_vertice_end[6] + new_vertice_end[7]) / 4;

                start = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
            else
            {
                scale = 1 - scale;
                new_vertice_start[0] = vertice[0];
                new_vertice_start[1] = vertice[1];
                new_vertice_start[5] = vertice[5];
                new_vertice_start[4] = vertice[4];
                new_vertice_start[3] = vertice[0] + (vertice[3] - vertice[0]) * scale;
                new_vertice_start[2] = vertice[1] + (vertice[2] - vertice[1]) * scale;
                new_vertice_start[6] = vertice[5] + (vertice[6] - vertice[5]) * scale;
                new_vertice_start[7] = vertice[4] + (vertice[7] - vertice[4]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[3] - vertice[0]) * scale;
                new_vertice_end[1] = vertice[1] + (vertice[2] - vertice[1]) * scale;
                new_vertice_end[5] = vertice[5] + (vertice[6] - vertice[5]) * scale;
                new_vertice_end[4] = vertice[4] + (vertice[7] - vertice[4]) * scale;

                new_vertice_end[3] = vertice[3];
                new_vertice_end[2] = vertice[2];
                new_vertice_end[6] = vertice[6];
                new_vertice_end[7] = vertice[7];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[1] + new_vertice_start[5] + new_vertice_start[4]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[3] + new_vertice_start[2] + new_vertice_start[6] + new_vertice_start[7]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[1] + new_vertice_end[5] + new_vertice_end[4]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[3] + new_vertice_end[2] + new_vertice_end[6] + new_vertice_end[7]) / 4;

                start = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
        }
    }

    string generate_string_version() const
    {
        string result = "";
        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                result = result + to_string(vertice[i][j]) + ",";
            }
        }
        for (int i = 0; i < 9; i++)
        {
            result = result + to_string(rotation_matrix(i)) + ",";
        }
        for (int j = 0; j < 3; j++)
        {
            result = result + to_string(center[j]) + ",";
        }
        for (int j = 0; j < 3; j++)
        {
            result = result + to_string(size_vector[j]) + ",";
        }

        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                result = result + to_string(global_in_out[i][j]) + ",";
            }
        }
        result = result + to_string(xsize) + ",";
        result = result + to_string(ysize) + ",";
        result = result + to_string(zsize) + ",";
        result = result + to_string(id) + ",";
        result = result + to_string(state) + ",";
        result = result + to_string(volume) + ",";
        result = result + to_string(use_x) + ",";
        result = result + to_string(use_y) + ",";
        result = result + to_string(use_z) + ",";
        // cout<<result<<endl;
        return result;
    }
    
private:
    vector<Eigen::Vector3d> vertice; 
    double volume = 0;
    Eigen::Vector3d center;                
    Eigen::Matrix3d rotation_matrix;       
    Eigen::Vector3d size_vector;           
    double xsize, ysize, zsize;            
    int id = 0;                            
    vector<Eigen::Vector3d> global_in_out; 
    int state = 0;                         
    bool use_x = false;                    
    bool use_y = false;                    
    bool use_z = false;                    
};


class gcs_task_assign{
    public:
        gcs_task_assign(){

        }
        gcs_task_assign(ros::NodeHandlePtr &nh_ptr_)
        : nh_ptr_(nh_ptr_)
        {
            namelist = {"/jurong", "/raffles", "/changi", "/sentosa", "/nanyang"};
            position_pair["/jurong"] = {Eigen::Vector3d(0, 0, 1), false};
            position_pair["/raffles"] = {Eigen::Vector3d(0, 0, 1), false};
            position_pair["/changi"] = {Eigen::Vector3d(0, 0, 1), false};
            position_pair["/sentosa"] = {Eigen::Vector3d(0, 0, 1), false};
            position_pair["/nanyang"]={Eigen::Vector3d(0, 0, 1), false};



            xmax = -std::numeric_limits<double>::max();
            ymax = -std::numeric_limits<double>::max();
            zmax = -std::numeric_limits<double>::max();
            xmin = std::numeric_limits<double>::max();
            ymin = std::numeric_limits<double>::max();
            zmin = std::numeric_limits<double>::max();

            client = nh_ptr_->serviceClient<caric_mission::CreatePPComTopic>("create_ppcom_topic");
            cmd_pub_ = nh_ptr_->advertise<std_msgs::String>("/task_assign", 10);
            srv.request.source = "gcs";
            srv.request.targets.push_back("all");
            srv.request.topic_name = "/task_assign";
            srv.request.package_name = "std_msgs";
            srv.request.message_type = "String";
            while (!serviceAvailable)
            {
                serviceAvailable = ros::service::waitForService("create_ppcom_topic", ros::Duration(10.0));
            }
            client.call(srv);
            
            bbox_sub_ = nh_ptr_->subscribe<sensor_msgs::PointCloud>("/gcs/bounding_box_vertices", 10, &gcs_task_assign::bboxCallback, this);
            agent_position_sub_=nh_ptr_->subscribe<std_msgs::String>("/broadcast/gcs", 10, &gcs_task_assign::positionCallback, this);

            Agent_ensure_Timer=nh_ptr_->createTimer(ros::Duration(1.0 / 10.0),  &gcs_task_assign::TimerEnsureCB,     this);
            Massage_publish_Timer=nh_ptr_->createTimer(ros::Duration(1.0 / 10.0),  &gcs_task_assign::TimerMessageCB,     this);
        }
    private:
        ros::NodeHandlePtr nh_ptr_;
        
        //communication related param
        bool serviceAvailable = false;
        caric_mission::CreatePPComTopic srv;
        ros::ServiceClient client;

        ros::Publisher cmd_pub_;

        ros::Subscriber agent_position_sub_;
        ros::Subscriber bbox_sub_;
        
        ros::Timer Agent_ensure_Timer;
        ros::Timer Massage_publish_Timer;


        bool get_bbox=false;
        bool finish_massage_generate=false;
        bool agent_info_get=false;

        list<string> namelist;
        map<string,position_info> position_pair;
        double update_time;

        double volumn_total=0;
        bool finish_bbox_record = false;
        vector<Boundingbox> box_set;
        
        vector<int> box_index;
        vector<int> state_vec;


        double xmax;
        double ymax;
        double zmax;
        double xmin;
        double ymin;
        double zmin;

        vector<vector<string>> team_info;
        vector<vector<Boundingbox>> output_path;

        string result;

        void bboxCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
        {
            if(finish_bbox_record||!agent_info_get)
            {
                // cout<<"now agent get"<<endl;
                // for(auto& name:namelist)
                // {
                //     if(position_pair[name].update){
                //         cout<<name<<endl;
                //     }
                // }
                return;
            }
            if(finish_massage_generate)
            {
                return;
            }

            sensor_msgs::PointCloud cloud = *msg;
            int num_points = cloud.points.size();
            if (num_points % 8 == 0 && num_points > 8 * box_set.size())
            {
                volumn_total = 0;
                int num_box = num_points / 8;
                for (int i = 0; i < num_box; i++)
                {
                    vector<Eigen::Vector3d> point_vec;
                    for (int j = 0; j < 8; j++)
                    {
                        if (cloud.points[8 * i + j].x > xmax)
                        {
                            xmax = cloud.points[8 * i + j].x;
                        }
                        if (cloud.points[8 * i + j].x < xmin)
                        {
                            xmin = cloud.points[8 * i + j].x;
                        }
                        if (cloud.points[8 * i + j].y > ymax)
                        {
                            ymax = cloud.points[8 * i + j].y;
                        }
                        if (cloud.points[8 * i + j].y < ymin)
                        {
                            ymin = cloud.points[8 * i + j].y;
                        }
                        if (cloud.points[8 * i + j].z > zmax)
                        {
                            zmax = cloud.points[8 * i + j].z;
                        }
                        if (cloud.points[8 * i + j].z < zmin)
                        {
                            zmin = cloud.points[8 * i + j].z;
                        }
                        point_vec.push_back(Eigen::Vector3d(cloud.points[8 * i + j].x, cloud.points[8 * i + j].y, cloud.points[8 * i + j].z));
                    }
                    box_set.push_back(Boundingbox(point_vec, i));
                    volumn_total += box_set[i].getVolume();
                    point_vec.clear();
                }
                finish_bbox_record = true;
                //Team allocate
                Team_allocate();
                //Use best first to generate the global trajactory
                Best_first_search();
                //Clip the best path
                Clip_the_task();
                //Generate the massage
                generate_massage();


            }
            else
            {
                return;
            }
            return;
        }

        void positionCallback(const std_msgs::String msg)
        {
            istringstream str(msg.data);
            string type;
            getline(str,type,';');
            if(type=="init_pos")
            {
                string origin;
                getline(str,origin,';');
                string position_str;
                getline(str,position_str,';');
                if(!position_pair[origin].update){
                    update_time=ros::Time::now().toSec();
                    position_pair[origin].position=str2point(position_str);
                    position_pair[origin].update=true;
                }
            }
        }

        void TimerEnsureCB(const ros::TimerEvent &)
        {   
            if(agent_info_get)
            {
                return;
            }
            bool finish_agent_record=false; 
            for(auto& name:namelist)
            {
                if(position_pair[name].update){
                    finish_agent_record=true;
                    break;
                }
            }
            if(!finish_agent_record)
            {
                return;
            }
            double time_now=ros::Time::now().toSec();

            if(fabs(time_now-update_time)>10)
            {
                agent_info_get=true;
            }
        }
        void TimerMessageCB(const ros::TimerEvent &){
            if(finish_massage_generate)
            {
                std_msgs::String task;
                task.data=result;
                cmd_pub_.publish(task);

            }
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

        void Team_allocate()
        {
            team_info=vector<vector<string>>(2);

            if(position_pair["/jurong"].update){
                team_info[0].push_back("/jurong");
            }
            if(position_pair["/raffles"].update){
                team_info[1].push_back("/raffles");
            }
            for(auto name:namelist){
                if(name=="/jurong"||name=="/raffles"){
                    continue;
                }
                if(!position_pair[name].update){
                    continue;
                }
                if(team_info[0].size()>0&&team_info[1].size()>0){
                    double jurong_dis=(position_pair[name].position-position_pair[team_info[0][0]].position).norm();
                    double raffles_dis=(position_pair[name].position-position_pair[team_info[1][0]].position).norm();
                    if(jurong_dis<=raffles_dis){
                        team_info[0].push_back(name);
                    }else{
                        team_info[1].push_back(name);
                    }
                    continue;
                }else if(team_info[0].size()>0){
                    team_info[0].push_back(name);
                    continue;
                }else if(team_info[1].size()>0){
                    team_info[1].push_back(name);
                    continue;
                }else{
                    continue;
                }
            }
            for(int i=0;i<2;i++){
                cout<<"team"<<i<<": "<<endl;
                for(int j=0;j<team_info[i].size();j++){
                    cout<<team_info[i][j]<<endl;
                }
                cout<<"team"<<i<<" finished"<<endl;
            }

            
        }
    void Best_first_search(){
        Eigen::Vector3d start_point;
        if(team_info[0].size()>0&&team_info[1].size()>0){
            start_point=position_pair[team_info[0][0]].position;
        }else if(team_info[0].size()>0&&team_info[1].size()==0){
            start_point=position_pair[team_info[0][0]].position;
        }else if(team_info[0].size()==0&&team_info[1].size()>0){
            start_point=position_pair[team_info[1][0]].position;
        }

        while(box_index.size()<box_set.size()){
            int index;
            int state;
            double mindis=std::numeric_limits<double>::max();
            for(int i=0;i<box_set.size();i++){
                if(find(box_index.begin(),box_index.end(),i)!=box_index.end()&&box_index.size()>0){
                    continue;
                }
                for(int j=0;j<2;j++){  
                    double dis=(box_set[i].get_global_in_out(j)-start_point).norm();
                    if(dis<mindis){
                        mindis=dis;
                        state=j;
                        index=i;
                    }
                }
            }
            box_index.push_back(index);
            state_vec.push_back(state);
            start_point=box_set[index].get_global_in_out(1-state);
        }
        cout<<"Path:"<<endl;
        for(int i=0;i<box_set.size();i++)
        {
            cout<<"bbox index:"<<box_index[i];
            cout<<" state:"<<state_vec[i]<<endl;
        }

    }
    void Clip_the_task()
    {
        double volum_path = 0;
        int clip_index = -1;
        bool clip_in_boundingbox = false;

        Boundingbox replaced_in;
        Boundingbox replaced_out;
        vector<Boundingbox> BFS_result;
        for(int i=0;i<box_index.size();i++)
        {
            BFS_result.push_back(box_set[box_index[i]]);
            BFS_result[i].edit_state(state_vec[i]);
        }

        output_path.resize(2);

        if(team_info[0].size()==0&&team_info[1].size()>0)
        {
            output_path[1]=BFS_result;
            return;

        }else if(team_info[0].size()>0&&team_info[1].size()==0)
        {
            output_path[0]=BFS_result;
            return;
        }else if(team_info[0].size()>0&&team_info[1].size()>0)
        {
            double factor=double(team_info[0].size())/double(team_info[0].size()+team_info[1].size());
            cout<<"factor"<<to_string(factor)<<endl;
            for(int j=0;j<BFS_result.size();j++)
            {
                volum_path +=BFS_result[j].getVolume();
                if (abs(volum_path / volumn_total - factor) <= 0.05)
                {
                    clip_index = j;
                    clip_in_boundingbox = false;
                    break;
                }
                if (volum_path / volumn_total - factor > 0.05)
                {
                
                    clip_index = j;
                    clip_in_boundingbox = true;
                    double volum_more = volum_path - volumn_total * factor;
                    double scaled_param = 1 - volum_more / (BFS_result[j].getVolume());
                    BFS_result[j].generate_start(scaled_param, replaced_in, replaced_out);
                    break;
                }
            }

            if(clip_in_boundingbox){
                for (int i = 0; i < BFS_result.size(); i++)
                {
                    if (i < clip_index)
                    {
                        output_path[0].push_back(BFS_result[i]);
                    }
                    else if (i == clip_index)
                    {
                        output_path[0].push_back(replaced_in);
                        output_path[1].push_back(replaced_out);
                    }
                    else
                    {
                        output_path[1].push_back(BFS_result[i]);
                    }
                }
                return;
            }else{
                for (int i = 0; i < BFS_result.size(); i++)
                {
                    if (i <= clip_index)
                    {
                        output_path[0].push_back(BFS_result[i]);
                    }
                    else
                    {
                        output_path[1].push_back(BFS_result[i]);
                    }
                } 
                return;       

            }
                
        }

    }

    void generate_massage()
    {
        result="";
        double loose_length=6.0;
        result = result + to_string(xmin - loose_length) + ",";
        result = result + to_string(ymin - loose_length) + ",";
        result = result + to_string(zmin - loose_length) + ",";

        result = result + to_string(xmax + loose_length) + ",";
        result = result + to_string(ymax + loose_length) + ",";
        result = result + to_string(zmax + loose_length) + ",";

        result=result+"team"+",0,"+to_string(team_info[0].size())+",";

        for(int i=0;i<team_info[0].size();i++){
            string str=team_info[0][i];
            str.erase(0, 1);
            result=result+str+",";
        }
        result=result+"team"+",1,"+to_string(team_info[1].size())+",";
        for(int i=0;i<team_info[1].size();i++){
            string str=team_info[1][i];
            str.erase(0, 1);
            result=result+str+",";
        }
        result = result + "path_size" + "," + "0" + "," + to_string(output_path[0].size()) + ",";
        result = result + "path_size" + "," + "1" + "," + to_string(output_path[1].size()) + ",";
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < output_path[i].size(); j++)
            {
                result = result + ";";
                result = result + output_path[i][j].generate_string_version();
            }
        }
        finish_massage_generate=true;

    }


};
