#include<iostream>
#include <ros/ros.h>
#include<visualization_msgs/Marker.h>
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

#include <caric_mission/CreatePPComTopic.h>
// #include "xxh_package.h"
#include <std_msgs/String.h>
#include "Astar.h"
class Boundingbox{
    public:
        Boundingbox(){
          center=Eigen::Vector3d(0,0,0);
          volume=0;
          id=-1;
        };
        Boundingbox(string str){
            vector<string> spilited_str;
            std::istringstream iss(str);
            std::string substring;
            while (std::getline(iss, substring, ',')) {
                spilited_str.push_back(substring);
            }
            // yolo();
            int i=0;
            while(i<24){
                vertice.push_back(Eigen::Vector3d(stod(spilited_str[i]),stod(spilited_str[i+1]),stod(spilited_str[i+2])));
                i=i+3;
            }
            // yolo();
            while(i<33){
                rotation_matrix(i-24)=stod(spilited_str[i]);
                i++;
            }
            while(i<36){
                center=Eigen::Vector3d(stod(spilited_str[i]),stod(spilited_str[i+1]),stod(spilited_str[i+2]));
                i=i+3;
            }
            while(i<39){
                size_vector=Eigen::Vector3d(stod(spilited_str[i]),stod(spilited_str[i+1]),stod(spilited_str[i+2]));
                i=i+3;
            }
            while(i<42){
                global_in_out.push_back(Eigen::Vector3d(stod(spilited_str[i]),stod(spilited_str[i+1]),stod(spilited_str[i+2])));
                i=i+3;
            }
            while(i<45){
                global_in_out.push_back(Eigen::Vector3d(stod(spilited_str[i]),stod(spilited_str[i+1]),stod(spilited_str[i+2])));
                i=i+3;
            }
            // yolo();
            xsize=stod(spilited_str[i]);
            i++;
            ysize=stod(spilited_str[i]);
            i++;
            zsize=stod(spilited_str[i]);
            i++;
            id=stod(spilited_str[i]);
            i++;
            state=stod(spilited_str[i]);
            i++;
            volume=stod(spilited_str[i]);
            i++;
            use_x=stod(spilited_str[i]);
            i++;
            use_y=stod(spilited_str[i]);
            i++;
            use_z=stod(spilited_str[i]);
            i++;
        }

        Boundingbox(const std::vector<Eigen::Vector3d>& vec,int id_in,Eigen::Vector3d& start,Eigen::Vector3d& end,int state_in,bool x,bool y, bool z){
            vertice=vec;
            id=id_in;
            center=Eigen::Vector3d(0,0,0);
            for(int index=0;index<vec.size();index++){
                center=center+vec[index];
            }
            center=center/8;
            xsize=(vec[1]-vec[0]).norm();
            ysize=(vec[3]-vec[0]).norm();
            zsize=(vec[4]-vec[0]).norm();
            volume=xsize*ysize*zsize;
            size_vector=Eigen::Vector3d(xsize/2,ysize/2,zsize/2);
            Eigen::Vector3d xaxis,yaxis,zaxis;
            xaxis=(vec[1]-vec[0]).normalized();
            yaxis=(vec[3]-vec[0]).normalized();
            zaxis=(vec[4]-vec[0]).normalized();
            rotation_matrix<< xaxis,yaxis,zaxis;
            global_in_out.push_back(start);
            global_in_out.push_back(end);
            state=state_in;
            use_x=x;
            use_y=y;
            use_z=z;
        }
        Boundingbox(const std::vector<Eigen::Vector3d>& vec,int id_in){
            vertice=vec;
            id=id_in;
            center=Eigen::Vector3d(0,0,0);
            for(int index=0;index<vec.size();index++){
                center=center+vec[index];
            }
            center=center/8;
            xsize=(vec[1]-vec[0]).norm();
            ysize=(vec[3]-vec[0]).norm();
            zsize=(vec[4]-vec[0]).norm();
            // volume=(vec[1]-vec[0]).norm()*(vec[3]-vec[0]).norm()*(vec[4]-vec[0]).norm();
            volume=xsize*ysize*zsize;
            size_vector=Eigen::Vector3d(xsize/2,ysize/2,zsize/2);

            Eigen::Vector3d xaxis,yaxis,zaxis;
            xaxis=(vec[1]-vec[0]).normalized();
            yaxis=(vec[3]-vec[0]).normalized();
            zaxis=(vec[4]-vec[0]).normalized();

            Eigen::Vector3d xplus,xminus,yplus,yminus,zplus,zminus;
            yminus=(vec[0]+vec[1]+vec[4]+vec[5])/4;
            yplus=(vec[2]+vec[3]+vec[6]+vec[7])/4;

            xminus=(vec[0]+vec[3]+vec[4]+vec[7])/4;
            xplus=(vec[1]+vec[2]+vec[5]+vec[6])/4;

            zminus=(vec[0]+vec[1]+vec[2]+vec[3])/4;
            zplus=(vec[4]+vec[5]+vec[6]+vec[7])/4;

            rotation_matrix<< xaxis,yaxis,zaxis;
            if((zsize>=ysize) && (zsize>=xsize)){
                use_z=true;
                global_in_out.push_back(zminus);
                global_in_out.push_back(zplus);
            }else if((xsize>=ysize) && (xsize>=zsize)){
                if(global_in_out.size()==0){
                    use_x=true;
                    global_in_out.push_back(xminus);
                    global_in_out.push_back(xplus);
                }

            }else if((ysize>=xsize) && (ysize>=zsize)){
                if(global_in_out.size()==0){
                    use_y=true;
                    global_in_out.push_back(yminus);
                    global_in_out.push_back(yplus);
                }
            }else{
                if(global_in_out.size()==0){
                    use_z=true;
                    global_in_out.push_back(zminus);
                    global_in_out.push_back(zplus);
                }

            }
            // std::cout<<rotation_matrix<<std::endl;
            // std::cout<<center.transpose()<<std::endl;
            // std::cout<<"size: "<<size_vector.transpose()<<std::endl;
        };
        ~Boundingbox(){};
        const Matrix3d getSearchRotation() const {
            Eigen::Vector3d axis_rotation_along(0.0,0.0,1.0);
            Eigen::Matrix3d transfer_matrix;
            Eigen::Matrix3d result;
            double angle=0;
            if(use_x){
                if(state==0){
                    cout<<"x+"<<endl;
                    axis_rotation_along=Eigen::Vector3d(0.0,1.0,0.0);
                    angle=-M_PI/2;
                }else if(state==1){
                    cout<<"x-"<<endl;
                    axis_rotation_along=Eigen::Vector3d(0.0,1.0,0.0);
                    angle=M_PI/2;
                }else{
                    cout<<"Error State"<<endl;
                }
            }else if(use_y){
                if(state==0){
                    cout<<"y+"<<endl;
                    axis_rotation_along=Eigen::Vector3d(1.0,0.0,0.0);
                    angle=M_PI/2;
                }else if(state==1){
                    cout<<"y-"<<endl;
                    axis_rotation_along=Eigen::Vector3d(1.0,0.0,0.0);
                    angle=-M_PI/2;
                }else{
                    cout<<"Error State"<<endl;
                }
            }else if(use_z){
                if(state==0){
                    cout<<"z+"<<endl;
                }else if(state==1){
                    cout<<"z-"<<endl;
                    axis_rotation_along=Eigen::Vector3d(1.0,0.0,0.0);
                    angle=M_PI;
                }else{
                    cout<<"Error State"<<endl;
                }
            }else{
                cout<<"noting happen"<<endl;
            }
            
            transfer_matrix=Eigen::AngleAxisd(angle,axis_rotation_along);
            result=transfer_matrix*rotation_matrix.inverse();
            return result;
            
            // result=rotation_matrix.inverse();
            // Eigen::Vector3d vectorafter(0,0,1);

            // Eigen::Vector3d vectorwanna=transfer_matrix.inverse()*vectorafter;
            // Eigen::Vector3d vectorafter=global_in_out[0];

            // Eigen::Vector3d vectorwanna=result*(vectorafter-center);
            
            // cout<<"test vector:"<<endl;
            // cout<<vectorwanna.transpose()<<endl;
            // yolo();
            // cout<<rotation_matrix<<endl;
            // cout<<transfer_matrix<<endl;
            
            // yolo();
            
        }


        int getState(){
            return state;
        }

        double getVolume() const {
            return volume;
        }
        const Vector3d getCenter() const {
            return center;
        }
        const Matrix3d getRotation() const {
            return rotation_matrix;
        }
        const Vector3d getExtents() const {
            return size_vector;
        }
        const Vector3d getRotExtents() const{
            Eigen::Vector3d result(0,0,0);
            if(use_x){
                if(state==0){
                    // cout<<"x+"<<endl;
                    result.x()=size_vector.z();
                    result.y()=size_vector.y();
                    result.z()=size_vector.x();
                }else if(state==1){
                    result.x()=size_vector.z();
                    result.y()=size_vector.y();
                    result.z()=size_vector.x();
                    // cout<<"x-"<<endl;
                    // axis_rotation_along=Eigen::Vector3d(0.0,1.0,0.0);
                    // angle=M_PI/2;
                }else{
                    cout<<"Error State"<<endl;
                }
            }else if(use_y){
                if(state==0){
                    // cout<<"y+"<<endl;
                    result.x()=size_vector.x();
                    result.y()=size_vector.z();
                    result.z()=size_vector.y();
                }else if(state==1){
                    // cout<<"y-"<<endl;
                    result.x()=size_vector.x();
                    result.y()=size_vector.z();
                    result.z()=size_vector.y();
                }else{
                    cout<<"Error State"<<endl;
                }
            }else if(use_z){
                if(state==0){
                    // cout<<"z+"<<endl;
                    result=size_vector;
                }else if(state==1){
                    // cout<<"z-"<<endl;
                    result=size_vector;
                }else{
                    cout<<"Error State"<<endl;
                }
            }else{
                cout<<"noting happen"<<endl;
            }
            return result;

        }
        vector<Eigen::Vector3d> getVertices() const{
            return vertice;
        }
        Eigen::Vector3d get_global_in_out(int state) const{
            Eigen::Vector3d result=global_in_out[state];
            return result;
        }
        void edit_state(int state_in){
            state=state_in;
        }
        void edit_id(){
            id=id+1;
        }
        int getId(){
            return id;
        }

        void generate_start(double scale,Boundingbox& start,Boundingbox& end){
            if(use_z){
                vector<Eigen::Vector3d> new_vertice_start(8);
                vector<Eigen::Vector3d> new_vertice_end(8);
                if (state==0){
                    new_vertice_start[0]=vertice[0];
                    new_vertice_start[1]=vertice[1];
                    new_vertice_start[2]=vertice[2];
                    new_vertice_start[3]=vertice[3];
                    new_vertice_start[4]=vertice[0]+(vertice[4]-vertice[0])*scale;
                    new_vertice_start[5]=vertice[1]+(vertice[5]-vertice[1])*scale;
                    new_vertice_start[6]=vertice[2]+(vertice[6]-vertice[2])*scale;
                    new_vertice_start[7]=vertice[3]+(vertice[7]-vertice[3])*scale;

                    new_vertice_end[0]=vertice[0]+(vertice[4]-vertice[0])*scale;
                    new_vertice_end[1]=vertice[1]+(vertice[5]-vertice[1])*scale;
                    new_vertice_end[2]=vertice[2]+(vertice[6]-vertice[2])*scale;
                    new_vertice_end[3]=vertice[3]+(vertice[7]-vertice[3])*scale;
                    
                    new_vertice_end[4]=vertice[4];
                    new_vertice_end[5]=vertice[5];
                    new_vertice_end[6]=vertice[6];
                    new_vertice_end[7]=vertice[7];

                    Eigen::Vector3d new_start=(new_vertice_start[0]+new_vertice_start[1]+new_vertice_start[2]+new_vertice_start[3])/4;
                    Eigen::Vector3d new_end=(new_vertice_start[4]+new_vertice_start[5]+new_vertice_start[6]+new_vertice_start[7])/4;
                    Eigen::Vector3d end_start=(new_vertice_end[0]+new_vertice_end[1]+new_vertice_end[2]+new_vertice_end[3])/4;
                    Eigen::Vector3d end_end=(new_vertice_end[4]+new_vertice_end[5]+new_vertice_end[6]+new_vertice_end[7])/4;
                    
                    start=Boundingbox(new_vertice_start,id,end_start,end_end,state,use_x,use_y,use_z);
                    end=Boundingbox(new_vertice_end,id,end_start,end_end,state,use_x,use_y,use_z);
                    return;
                    
                }else{
                    scale=1-scale;
                    new_vertice_start[0]=vertice[0];
                    new_vertice_start[1]=vertice[1];
                    new_vertice_start[2]=vertice[2];
                    new_vertice_start[3]=vertice[3];
                    new_vertice_start[4]=vertice[0]+(vertice[4]-vertice[0])*scale;
                    new_vertice_start[5]=vertice[1]+(vertice[5]-vertice[1])*scale;
                    new_vertice_start[6]=vertice[2]+(vertice[6]-vertice[2])*scale;
                    new_vertice_start[7]=vertice[3]+(vertice[7]-vertice[3])*scale;

                    new_vertice_end[0]=vertice[0]+(vertice[4]-vertice[0])*scale;
                    new_vertice_end[1]=vertice[1]+(vertice[5]-vertice[1])*scale;
                    new_vertice_end[2]=vertice[2]+(vertice[6]-vertice[2])*scale;
                    new_vertice_end[3]=vertice[3]+(vertice[7]-vertice[3])*scale;
                    
                    new_vertice_end[4]=vertice[4];
                    new_vertice_end[5]=vertice[5];
                    new_vertice_end[6]=vertice[6];
                    new_vertice_end[7]=vertice[7];
                    
                    Eigen::Vector3d new_start=(new_vertice_start[0]+new_vertice_start[1]+new_vertice_start[2]+new_vertice_start[3])/4;
                    Eigen::Vector3d new_end=(new_vertice_start[4]+new_vertice_start[5]+new_vertice_start[6]+new_vertice_start[7])/4;
                    Eigen::Vector3d end_start=(new_vertice_end[0]+new_vertice_end[1]+new_vertice_end[2]+new_vertice_end[3])/4;
                    Eigen::Vector3d end_end=(new_vertice_end[4]+new_vertice_end[5]+new_vertice_end[6]+new_vertice_end[7])/4;

                    start=Boundingbox(new_vertice_end,id,end_start,end_end,state,use_x,use_y,use_z);
                    end=Boundingbox(new_vertice_start,id,end_start,end_end,state,use_x,use_y,use_z);
                    return;
                }
                
            }
            else if(use_x){
                vector<Eigen::Vector3d> new_vertice_start(8);
                vector<Eigen::Vector3d> new_vertice_end(8);
                if (state==0){
                    new_vertice_start[0]=vertice[0];
                    new_vertice_start[3]=vertice[3];
                    new_vertice_start[4]=vertice[4];
                    new_vertice_start[7]=vertice[7];
                    new_vertice_start[1]=vertice[0]+(vertice[1]-vertice[0])*scale;
                    new_vertice_start[2]=vertice[3]+(vertice[2]-vertice[3])*scale;
                    new_vertice_start[5]=vertice[4]+(vertice[5]-vertice[4])*scale;
                    new_vertice_start[6]=vertice[7]+(vertice[6]-vertice[7])*scale;

                    new_vertice_end[0]=vertice[0]+(vertice[1]-vertice[0])*scale;
                    new_vertice_end[3]=vertice[3]+(vertice[2]-vertice[3])*scale;
                    new_vertice_end[4]=vertice[4]+(vertice[5]-vertice[4])*scale;
                    new_vertice_end[7]=vertice[7]+(vertice[6]-vertice[7])*scale;
                    
                    new_vertice_end[1]=vertice[1];
                    new_vertice_end[2]=vertice[2];
                    new_vertice_end[5]=vertice[5];
                    new_vertice_end[6]=vertice[6];

                    Eigen::Vector3d new_start=(new_vertice_start[0]+new_vertice_start[3]+new_vertice_start[4]+new_vertice_start[7])/4;
                    Eigen::Vector3d new_end=(new_vertice_start[1]+new_vertice_start[2]+new_vertice_start[5]+new_vertice_start[6])/4;
                    Eigen::Vector3d end_start=(new_vertice_end[0]+new_vertice_end[3]+new_vertice_end[4]+new_vertice_end[7])/4;
                    Eigen::Vector3d end_end=(new_vertice_end[1]+new_vertice_end[2]+new_vertice_end[5]+new_vertice_end[6])/4;
                    
                    start=Boundingbox(new_vertice_start,id,end_start,end_end,state,use_x,use_y,use_z);
                    end=Boundingbox(new_vertice_end,id,end_start,end_end,state,use_x,use_y,use_z);
                    return;
                    
                }else{
                    scale=1-scale;
                    new_vertice_start[0]=vertice[0];
                    new_vertice_start[3]=vertice[3];
                    new_vertice_start[4]=vertice[4];
                    new_vertice_start[7]=vertice[7];
                    new_vertice_start[1]=vertice[0]+(vertice[1]-vertice[0])*scale;
                    new_vertice_start[2]=vertice[3]+(vertice[2]-vertice[3])*scale;
                    new_vertice_start[5]=vertice[4]+(vertice[5]-vertice[4])*scale;
                    new_vertice_start[6]=vertice[7]+(vertice[6]-vertice[7])*scale;

                    new_vertice_end[0]=vertice[0]+(vertice[1]-vertice[0])*scale;
                    new_vertice_end[3]=vertice[3]+(vertice[2]-vertice[3])*scale;
                    new_vertice_end[4]=vertice[4]+(vertice[5]-vertice[4])*scale;
                    new_vertice_end[7]=vertice[7]+(vertice[6]-vertice[7])*scale;
                    
                    new_vertice_end[1]=vertice[1];
                    new_vertice_end[2]=vertice[2];
                    new_vertice_end[5]=vertice[5];
                    new_vertice_end[6]=vertice[6];

                    Eigen::Vector3d new_start=(new_vertice_start[0]+new_vertice_start[3]+new_vertice_start[4]+new_vertice_start[7])/4;
                    Eigen::Vector3d new_end=(new_vertice_start[1]+new_vertice_start[2]+new_vertice_start[5]+new_vertice_start[6])/4;
                    Eigen::Vector3d end_start=(new_vertice_end[0]+new_vertice_end[3]+new_vertice_end[4]+new_vertice_end[7])/4;
                    Eigen::Vector3d end_end=(new_vertice_end[1]+new_vertice_end[2]+new_vertice_end[5]+new_vertice_end[6])/4;
                    
                    start=Boundingbox(new_vertice_end,id,end_start,end_end,state,use_x,use_y,use_z);
                    end=Boundingbox(new_vertice_start,id,end_start,end_end,state,use_x,use_y,use_z);
                    return;
                }
            }else{
                vector<Eigen::Vector3d> new_vertice_start(8);
                vector<Eigen::Vector3d> new_vertice_end(8);
                if (state==0){
                     new_vertice_start[0]=vertice[0];
                    new_vertice_start[1]=vertice[1];
                    new_vertice_start[5]=vertice[5];
                    new_vertice_start[4]=vertice[4];
                    new_vertice_start[3]=vertice[0]+(vertice[3]-vertice[0])*scale;
                    new_vertice_start[2]=vertice[1]+(vertice[2]-vertice[1])*scale;
                    new_vertice_start[6]=vertice[5]+(vertice[6]-vertice[5])*scale;
                    new_vertice_start[7]=vertice[4]+(vertice[7]-vertice[4])*scale;

                    new_vertice_end[0]=vertice[0]+(vertice[3]-vertice[0])*scale;
                    new_vertice_end[1]=vertice[1]+(vertice[2]-vertice[1])*scale;
                    new_vertice_end[5]=vertice[5]+(vertice[6]-vertice[5])*scale;
                    new_vertice_end[4]=vertice[4]+(vertice[7]-vertice[4])*scale;
                    
                    new_vertice_end[3]=vertice[3];
                    new_vertice_end[2]=vertice[2];
                    new_vertice_end[6]=vertice[6];
                    new_vertice_end[7]=vertice[7];
                    
                    Eigen::Vector3d new_start=(new_vertice_start[0]+new_vertice_start[1]+new_vertice_start[5]+new_vertice_start[4])/4;
                    Eigen::Vector3d new_end=(new_vertice_start[3]+new_vertice_start[2]+new_vertice_start[6]+new_vertice_start[7])/4;
                    Eigen::Vector3d end_start=(new_vertice_end[0]+new_vertice_end[1]+new_vertice_end[5]+new_vertice_end[4])/4;
                    Eigen::Vector3d end_end=(new_vertice_end[3]+new_vertice_end[2]+new_vertice_end[6]+new_vertice_end[7])/4;

                    
                    start=Boundingbox(new_vertice_start,id,end_start,end_end,state,use_x,use_y,use_z);
                    end=Boundingbox(new_vertice_end,id,end_start,end_end,state,use_x,use_y,use_z);
                    return;
                    
                }else{
                    scale=1-scale;
                    new_vertice_start[0]=vertice[0];
                    new_vertice_start[1]=vertice[1];
                    new_vertice_start[5]=vertice[5];
                    new_vertice_start[4]=vertice[4];
                    new_vertice_start[3]=vertice[0]+(vertice[3]-vertice[0])*scale;
                    new_vertice_start[2]=vertice[1]+(vertice[2]-vertice[1])*scale;
                    new_vertice_start[6]=vertice[5]+(vertice[6]-vertice[5])*scale;
                    new_vertice_start[7]=vertice[4]+(vertice[7]-vertice[4])*scale;

                    new_vertice_end[0]=vertice[0]+(vertice[3]-vertice[0])*scale;
                    new_vertice_end[1]=vertice[1]+(vertice[2]-vertice[1])*scale;
                    new_vertice_end[5]=vertice[5]+(vertice[6]-vertice[5])*scale;
                    new_vertice_end[4]=vertice[4]+(vertice[7]-vertice[4])*scale;
                    
                    new_vertice_end[3]=vertice[3];
                    new_vertice_end[2]=vertice[2];
                    new_vertice_end[6]=vertice[6];
                    new_vertice_end[7]=vertice[7];
                    
                    Eigen::Vector3d new_start=(new_vertice_start[0]+new_vertice_start[1]+new_vertice_start[5]+new_vertice_start[4])/4;
                    Eigen::Vector3d new_end=(new_vertice_start[3]+new_vertice_start[2]+new_vertice_start[6]+new_vertice_start[7])/4;
                    Eigen::Vector3d end_start=(new_vertice_end[0]+new_vertice_end[1]+new_vertice_end[5]+new_vertice_end[4])/4;
                    Eigen::Vector3d end_end=(new_vertice_end[3]+new_vertice_end[2]+new_vertice_end[6]+new_vertice_end[7])/4;

                    start=Boundingbox(new_vertice_end,id,end_start,end_end,state,use_x,use_y,use_z);
                    end=Boundingbox(new_vertice_start,id,end_start,end_end,state,use_x,use_y,use_z);
                    return;
                }

            }
        }

        string generate_string_version()const {
            string result="";
            for(int i=0;i<8;i++){
                for(int j=0;j<3;j++){
                    result=result+to_string(vertice[i][j])+",";
                }
            }
            for(int i=0;i<9;i++){
                result=result+to_string(rotation_matrix(i))+",";
            }
            for(int j=0;j<3;j++){
                result=result+to_string(center[j])+",";
            }
            for(int j=0;j<3;j++){
                result=result+to_string(size_vector[j])+",";
            }
            
            for(int i=0;i<2;i++){
                for(int j=0;j<3;j++){
                    result=result+to_string(global_in_out[i][j])+",";
                }
            }
            result=result+to_string(xsize)+",";
            result=result+to_string(ysize)+",";
            result=result+to_string(zsize)+",";
            result=result+to_string(id)+",";
            result=result+to_string(state)+",";
            result=result+to_string(volume)+",";
            result=result+to_string(use_x)+",";
            result=result+to_string(use_y)+",";
            result=result+to_string(use_z)+",";
            // cout<<result<<endl;
            return result;
        }

    private:
        vector<Eigen::Vector3d> vertice;//1
        double volume=0;
        Eigen::Vector3d center;//1
        Eigen::Matrix3d rotation_matrix;//1
        Eigen::Vector3d size_vector;//1
        double xsize,ysize,zsize;//1
        int id=0;//1
        vector<Eigen::Vector3d> global_in_out;//1
        int state=0;//1
        bool use_x=false;//1
        bool use_y=false;//1
        bool use_z=false;//1
};

class Task_initial_controller{
    
    public:
    Task_initial_controller();
    void bboxtimer(const ros::TimerEvent &){
        // yolo();
        if(serviceAvailable && finished_massage_generate){
            std_msgs::String message;
            // message.data = "Hello, world!";
            message.data =global_massage;
            cmd_pub_.publish(message);
        }
        // yolo();
        return;
    }

    void bboxCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    void find_shortest_path();
    void assign_the_task();
    void calculate_node_distance();
    private:
        bool serviceAvailable = false;
        caric_mission::CreatePPComTopic srv;
        ros::ServiceClient client;
        ros::Publisher cmd_pub_;
        ros::Subscriber bbox_sub_;
        ros::Timer timer;
        vector<std::vector<Boundingbox>> shortest_path_camp;
        vector<Boundingbox> shortest_path_not_edit;
        vector<int>shortest_state_not_edit;
        vector<vector<Boundingbox>> output_path;
        vector<Boundingbox> path_saver;
        double xmax;
        double ymax;
        double zmax;
        double xmin;
        double ymin;
        double zmin;
        string global_massage;
        bool finished_massage_generate=false;
        ros::NodeHandle nh;
        ros::NodeHandle nh2;
        bool finish_bbox_record =false;
        vector<vector<bool>> vector_overlap;
        double volumn_total;
        void backtrack(std::vector<Boundingbox>& vertices, std::vector<int>& path, std::vector<int>& bestPath, double& minLength, double currLength, int currentIndex) {
        if (path.size() == vertices.size()) {
            // 已经遍历完所有顶点，更新最短路径
            if (currLength < minLength) {
                minLength = currLength;
                bestPath = path;
            }
            return;
        }

        for (int i = 0; i < vertices.size(); i++) {
            if (std::find(path.begin(), path.end(), i) == path.end()) {
                // 如果顶点尚未访问，则将其添加到路径中
                path.push_back(i);

                // 计算当前顶点与前一个顶点之间的距离
                if (path.size() > 1) {
                    Boundingbox& currentVertex = vertices[i];
                    Boundingbox& prevVertex = vertices[path[path.size() - 2]];
                    double distance = (currentVertex.getCenter() - prevVertex.getCenter()).norm();
                    currLength += distance;
                }

                // 递归搜索下一个顶点
                backtrack(vertices, path, bestPath, minLength, currLength, i);

                // 回溯，将当前顶点从路径中移除
                path.pop_back();

                // 还原当前顶点与前一个顶点之间的距离
                if (path.size() > 0) {
                    Boundingbox& prevVertex = vertices[path[path.size() - 1]];
                    currLength -= (prevVertex.getCenter() - vertices[i].getCenter()).norm();
                }
            }
        }
    }

    // 最小路径搜索函数
    std::vector<int> findShortestPath(std::vector<Boundingbox>& vertices) {
        std::vector<int> path; // 当前路径
        std::vector<int> bestPath; // 最短路径
        double minLength = std::numeric_limits<double>::max(); // 最短路径长度
        double currLength = 0.0; // 当前路径长度

        // 从每个顶点开始进行回溯
        for (int i = 0; i < vertices.size(); i++) {
            for(int j=0;j<2;j++){
                path.push_back(i);
                backtrack(vertices, path, bestPath, minLength, currLength, i);
                path.pop_back();
            }        
        }

        return bestPath;
    }

    void find_shortest_link() {
        std::vector<int> path;
        std::vector<int> bestPath;
        std::vector<int>bestState;
        shortest_path_not_edit.resize(path_saver.size());
        double minLength = std::numeric_limits<double>::max(); // 最短路径长度
        double currLength = 0.0; // 当前路径长度
        for(int i=0;i<path_saver.size();i++)
        {
            path.push_back(i);
        }
        sort(path.begin(),path.end());
        double min_distance=std::numeric_limits<double>::max();
        do{
            bestPath_finder(min_distance,path,bestState,bestPath);    

        }while(std::next_permutation(path.begin(),path.end()));
        // yolo();
        cout<<bestPath.size()<<endl;
        cout<<bestState.size()<<endl;
        
        for(int i=0;i<bestPath.size();i++){
            cout<<bestPath[i]<<endl;
            path_saver[bestPath[i]].edit_state(bestState[i]);
            shortest_path_not_edit[i]=path_saver[bestPath[i]];
            cout<<i<<endl;
        }
        shortest_state_not_edit=bestState;
        cout<<"bestPath"<<" :  "<<endl;
        for(int i=0;i<bestPath.size();i++){
            cout<<bestPath[i]<<" ";
        }
        cout<<endl;
        cout<<"bestState"<<" :  "<<endl;
        for(int i=0;i<bestState.size();i++){
            cout<<bestState[i]<<" ";
        }
        cout<<endl;
        cout<<"mindis"<<" :  "<<min_distance<<endl;

    }

    void bestPath_finder(double& min_distance,vector<int>& path,vector<int>& best_state,vector<int>& best_path){
        std::vector<int>sequence(path_saver.size());
        int distance=0;
        int N=path_saver.size();
        N=1<<N;
        for(int i=0;i<N;i++){
            distance=0;
            // yolo();
            for(int j=0;j<path_saver.size();j++){
                sequence[j]=(i>>j)&1;
            }
            // for(int n=0;n<path_saver.size();n++){
            //     cout<<sequence[n]<<" ";
            // }
            // cout<<endl;
            for(int m=1;m<path_saver.size();m++){
                distance+=(path_saver[path[m-1]].get_global_in_out(1-sequence[m-1])-path_saver[path[m]].get_global_in_out(sequence[m])).norm();
                if(distance>min_distance){
                    break;
                }
            }
            if(distance<min_distance){
                min_distance=distance;
                best_state=sequence;
                best_path=path;
            }
        }
    }

    void assign_path_calculation(){
        // yolo();
        double volum_path=0;
        int clip_index=-1;
        bool clip_in_boundingbox= false;
        Boundingbox replaced_in;
        
        Boundingbox replaced_out;
        // yolo();
        for(int i=0;i<shortest_path_not_edit.size();i++)
        {  
            cout<<i<<endl;
            volum_path+=shortest_path_not_edit[i].getVolume();
            // yolo();
            if(abs(volum_path/volumn_total-0.6)<=0.05){
                clip_index=i;
                clip_in_boundingbox=false;
                break;
            }
            if(volum_path/volumn_total-0.6>0.05){
                // yolo();
                clip_index=i;
                clip_in_boundingbox=true;
                double volum_more=volum_path-volumn_total*0.6;
                double scaled_param=1-volum_more/(shortest_path_not_edit[i].getVolume());
                // scaled_param=0.5;
                shortest_path_not_edit[i].generate_start(scaled_param,replaced_in,replaced_out);
                // yolo();
           }
        //    yolo();
        }
        // yolo();
        if(clip_in_boundingbox){
            // cout<<"clip index:  "<<clip_index<<endl;
            // cout<<"state:   "<<shortest_path_not_edit[clip_index].getState()<<endl;
            // cout<<"start_center:  "<<endl;
            // cout<<replaced_in.getCenter().transpose()<<endl;
            // cout<<"out_center:  "<<endl;
            // cout<<replaced_out.getCenter().transpose()<<endl;
            output_path.resize(2);            
            for(int i=0;i<shortest_path_not_edit.size();i++){
                yolo();
                if(i<clip_index){
                    output_path[0].push_back(shortest_path_not_edit[i]);
                }else if(i== clip_index){
                    output_path[0].push_back(replaced_in);
                    // replaced_out.edit_id();
                    output_path[1].push_back(replaced_out);
                }else{
                    // shortest_path_not_edit[i].edit_id();
                    output_path[1].push_back(shortest_path_not_edit[i]);
                }

            }
            cout<<"Finish the assign waiting for message generated "<<endl;
        }else{
            output_path.resize(2); 
            for(int i=0;i<shortest_path_not_edit.size();i++){
                if(i<=clip_index){
                    output_path[0].push_back(shortest_path_not_edit[i]);
                }else{
                    // shortest_path_not_edit[i].edit_id();
                    output_path[1].push_back(shortest_path_not_edit[i]);
                }
            }
            cout<<"Finish the assign waiting for message generated"<<endl;
        }
        // yolo();
        global_massage=generate_global_string();
        cout<<"global message"<<endl;
        // decode_global_string(global_massage);
        for(int i=0;i<2;i++){
            for(int j=0;j<output_path[i].size();j++)
            {
                global_massage=global_massage+";";
                global_massage=global_massage+output_path[i][j].generate_string_version();
            }
        }
        finished_massage_generate=true;
        
        // string path_string=output_path[0][0].generate_string_version();
        // Boundingbox A(path_string);
    }

    string generate_global_string(){
        string result="";
        int loose_length=6;
        result=result+to_string(xmin-loose_length)+",";
        result=result+to_string(ymin-loose_length)+",";
        result=result+to_string(zmin-loose_length)+",";
        
        result=result+to_string(xmax+loose_length)+",";
        result=result+to_string(ymax+loose_length)+",";
        result=result+to_string(zmax+loose_length)+",";

        result=result+"team"+",0,"+"3"+","+"jurong"+","+"changi,nanyang"+",";
        result=result+"team"+",1,"+"2"+","+"raffles"+","+"sentosa"+",";
        result=result+"path_size"+","+"0"+","+to_string(output_path[0].size())+",";
        result=result+"path_size"+","+"1"+","+to_string(output_path[1].size())+",";
        // cout<<result<<endl;
        
        return result;
    }

    void decode_global_string(string s){
        vector<string> spilited_str;
        std::istringstream iss(s);
        std::string substring;
        while (std::getline(iss, substring, ',')) {
            spilited_str.push_back(substring);
            // std::cout << substring << std::endl;
        }
        // cout<<"vector:   "<<endl;
        // for(int i=0;i<spilited_str.size();i++){
        //     std::cout << spilited_str[i] << std::endl;
        // }
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
        cout<<"verify:  "<<endl;
        // cout<<"size1: "<<size_of_path[0]<<endl;
        // cout<<"size2: "<<size_of_path[1]<<endl;
        cout<<"size1: "<<output_path[0].size()<<endl;
        cout<<"size2: "<<output_path[1].size()<<endl;
        for(int i=0;i<2;i++){
            cout<<"team "<<i<<":   "<<endl;
            for(int j=0;j<teams[i].size();j++){
                cout<<teams[i][j]<<",";
            }
            cout<<endl;
        }
    }
};
void computeProjectionRange(const Boundingbox& obb, const Eigen::Vector3d& axis, double& minv, double& maxv) {
    Eigen::Vector3d obbAxis = obb.getRotation().transpose() * axis;  // 将轴变换到OBB的局部坐标系
    Eigen::Vector3d center = obb.getCenter();
    Eigen::Matrix3d rotation = obb.getRotation();

    minv = std::numeric_limits<double>::max();
    maxv = -std::numeric_limits<double>::max();
    // yolo();
    int i=0;
    for (const auto& vertex : obb.getVertices()) {
        i++;
        Eigen::Vector3d localVertex = rotation.transpose() * (vertex - center);
        double projectionValue = localVertex.dot(obbAxis);
        minv = std::min(minv, projectionValue);
        maxv = std::max(maxv, projectionValue);
    }

    minv=minv+center.dot(axis);
    maxv=maxv+center.dot(axis);
}

// 检查两个OBBs是否重叠，并计算重叠体积
bool checkOverlap(const Boundingbox& obbA, const Boundingbox& obbB) {
    double overlap = 1.0;
    
    // 对于每个OBB的三个轴，检查是否有分离轴
    for (int i = 0; i < 3; ++i) {
        Eigen::Vector3d axisA = obbA.getRotation().col(i).normalized();
        double minA, maxA, minB, maxB;
        computeProjectionRange(obbA, axisA, minA, maxA);
        computeProjectionRange(obbB, axisA, minB, maxB);
        // 如果在任何一个轴上的投影范围不重叠，则两个OBBs不重叠
        if (maxA < minB || minA > maxB) {
            overlap = 0.0;
            break;
        }
    }
    return overlap > 0.0;
}