#include "utility.h"
#include "Eigen/Dense"
#pragma once
#include<vector>
#include<list>
#include<memory>
#include<algorithm>
#include<iostream>
#include "Astar.h"

class xxh_explore_map{
    public:
        xxh_explore_map(){
        }
        xxh_explore_map(Eigen::Vector3i shape_in,int number_of_teammate){
            // yolo();
            num=number_of_teammate;
            map_shape=shape_in;
            int interval=floor(shape_in.z()/number_of_teammate);
            map=vector<vector<vector<int>>>(map_shape.x(), vector<vector<int>>(map_shape.y(), vector<int>(map_shape.z(), 0)));
            interest_map=vector<vector<vector<int>>>(map_shape.x(), vector<vector<int>>(map_shape.y(), vector<int>(map_shape.z(), 0)));
            height=vector<vector<vector<int>>>(map_shape.x(), vector<vector<int>>(map_shape.y(), vector<int>(map_shape.z(), 0)));
            visited_map=vector<vector<vector<int>>>(map_shape.x(), vector<vector<int>>(map_shape.y(), vector<int>(map_shape.z(), 0)));
            lowest_highest_index=Eigen::Vector3i(0,0,0);
            for(int i=0;i<num;i++){
                area.push_back(interval*i);                
            }
            area.push_back(shape_in.z());
            // yolo();
        }
        void set_visited(Eigen::Vector3i index){
            // yolo();
            int x=index.x();
            int y=index.y();
            int z=index.z();
            // yolo();
            my_position=index;
            if(x>=0&&x<map_shape.x()&&y>=0&&y<map_shape.y()&&z<map_shape.z()&&z>=0){
                // yolo();
                visited_map[x][y][z]=1;
                my_position=index;
                get_position=true;
                // layer=
            }
            // yolo();
        }
        void insert_map(Eigen::Vector3i index){
            int x=index.x();
            int y=index.y();
            int z=index.z();
            map[x][y][z]=1;
            if(z>height[x][y][0]){
                height[x][y][0]=z;
                if(z<lowest){
                    lowest=z;
                    lowest_highest_index=Eigen::Vector3i(x,y,z);
                }
            }
            // cout<<lowest<<endl;

            for(int i=x-1;i<x+1;i++){
                for(int j=y-1;j<y+1;j++){
                    for(int k=z-1;k<z+1;k++){
                        if(i>=0&&i<map_shape.x()&&j>=0&&j<map_shape.y()&&k<map_shape.z()&&k>=0){
                            if(map[i][j][k]==0){
                                interest_map[i][j][k]=1;
                            }else{
                                interest_map[i][j][k]=0;
                            }
                        }
                    }
                }
            }
        }
        void generate_searching_point(){
            list<Eigen::Vector3i> path_tamp;
            if(get_position){
                if(state_1&&explore_index<num){
                    path_tamp=BFS(area[explore_index]);
                    path_tamp.reverse();
                    path=path_tamp;
                    if(path.size()==0){
                        state_2=true;
                        state_1=false;
                    }
                }else if(state_2&&explore_index<num-1){

                    path_tamp=BFS2d3d(area[explore_index],area[explore_index+1]);
                    path_tamp.reverse();
                    path=path_tamp;
                    if(path.size()==0){
                        state_2=false;
                        state_1=false;
                        explore_index++;
                        cout<<"Region:"<<explore_index<<endl;
                        explore_layer=area[explore_index];
                    }
                }else if(explore_index<num-1){
                    path_tamp=BFS2d3d(area[explore_index],area[explore_index+1]);
                    path_tamp.reverse();
                    path=path_tamp;
                    if(my_position.z()==area[explore_index]||path.size()==0){
                        state_2=false;
                        state_1=true;
                    }     
                }else{
                    cout<<"Doing explorer scaned layer:"<<explore_layer<<endl;
                    finish=true;
                    path_tamp=BFS2d3d(explore_layer,explore_layer+1);
                    path_tamp.reverse();
                    if(path_tamp.size()>0){
                        path=path_tamp;
                    }else{
                        path=path_tamp;
                        if(explore_layer>=map_shape.z()){
                            finish=true;
                            cout<<"finish_searching waiting for photographer"<<endl;
                            return;
                        }
                        explore_layer+=2;
                        path_tamp=BFS2d3d(explore_layer,map_shape.z()-1);

                    }
                    return;
                }
                
            }
        }

        bool check_finish(){
            return finish;
        }

        list<Eigen::Vector3i> get_path(){
            return path;
        }
        bool isValidMove(int x,int y,int z){
            if(x>=0&&x<map_shape.x()&&y>=0&&y<map_shape.y()&&z<map_shape.z()&&z>=0){
                if(map[x][y][z]==1){
                    return false;
                }else{
                    return true;    
                }
                
            }else{
                return false;
            }
        }

        list<Eigen::Vector3i> BFS(int layer) {
            // Define the four directions of movement.
            vector<vector<vector<int>>> grid=map;
            Eigen::Vector3i start=my_position;
            vector<Vector3i> directions = { Vector3i(0, 1,0), Vector3i(0, -1,0), Vector3i(1, 0,0), Vector3i(-1, 0,0) };
            
            // Initialize the queue and visited flag.
            queue<list<Eigen::Vector3i>> q;
            q.push({start});
            vector<vector<vector<bool>>> visited(map_shape.x(), vector<vector<bool>>(map_shape.y(), vector<bool>(map_shape.z(), false)));
            visited[start.x()][start.y()][layer] = true;
            
            while (!q.empty()) {
                list<Eigen::Vector3i> path = q.front();
                q.pop();
                
                Eigen::Vector3i curr = path.back();
                if (interest_map[curr.x()][curr.y()][curr.z()]==1&&visited_map[curr.x()][curr.y()][curr.z()]==0) {
                    return path; // Found the path, return the complete path.
                }
                
                for (const auto& dir : directions) {
                    int nextRow = curr.x() + dir.x();
                    int nextCol = curr.y() + dir.y();
                    
                    if (isValidMove(nextRow, nextCol,layer) && !visited[nextRow][nextCol][layer]) {
                        list<Vector3i> newPath = path;
                        newPath.push_back(Vector3i(nextRow, nextCol,layer));
                        q.push(newPath);
                        visited[nextRow][nextCol][layer] = true;
                    }
                }
            }
            // If the path is not found, return an empty list.
            return {};
        }

        list<Eigen::Vector3i> BFS3d(int layer) {
            // Define the four directions of movement.
            vector<vector<vector<int>>> grid=map;
            Eigen::Vector3i start=my_position;
            vector<Vector3i> directions = { Vector3i(0, 1,0), Vector3i(0, -1,0), Vector3i(1, 0,0), Vector3i(-1, 0,0),Vector3i(0,0,1),Vector3i(0,0,-1) };
            
            // Initialize the queue and visited flag.
            queue<list<Eigen::Vector3i>> q;
            q.push({start});
            vector<vector<vector<bool>>> visited(map_shape.x(), vector<vector<bool>>(map_shape.y(), vector<bool>(map_shape.z(), false)));
            visited[start.x()][start.y()][start.z()] = true;
            
            while (!q.empty()) {
                list<Eigen::Vector3i> path = q.front();
                q.pop();
                
                Eigen::Vector3i curr = path.back();
                if (interest_map[curr.x()][curr.y()][curr.z()]==1&&visited_map[curr.x()][curr.y()][curr.z()]==0&&curr.z()>=layer) {
                    return path; // Found the path, return the complete path.
                }
                
                for (const auto& dir : directions) {
                    int nextRow = curr.x() + dir.x();
                    int nextCol = curr.y() + dir.y();
                    int nextHeight=curr.z()+dir.z();
                    if (isValidMove(nextRow, nextCol,nextHeight) && !visited[nextRow][nextCol][nextHeight]) {
                        list<Vector3i> newPath = path;
                        newPath.push_back(Vector3i(nextRow, nextCol,nextHeight));
                        q.push(newPath);
                        visited[nextRow][nextCol][nextHeight] = true;
                    }
                }
            }
            // If the path is not found, return an empty list.
            return {};
        }

        list<Eigen::Vector3i> BFS2d3d(int layer,int upper) {
            // Define the four directions of movement.
            vector<vector<vector<int>>> grid=map;
            Eigen::Vector3i start=my_position;
            vector<Vector3i> directions = { Vector3i(0, 1,0), Vector3i(0, -1,0), Vector3i(1, 0,0), Vector3i(-1, 0,0),Vector3i(0,0,1),Vector3i(0,0,-1) };
            
            // Initialize the queue and visited flag.
            queue<list<Eigen::Vector3i>> q;
            q.push({start});
            vector<vector<vector<bool>>> visited(map_shape.x(), vector<vector<bool>>(map_shape.y(), vector<bool>(map_shape.z(), false)));
            visited[start.x()][start.y()][start.z()] = true;
            
            while (!q.empty()) {
                list<Eigen::Vector3i> path = q.front();
                q.pop();
                
                Eigen::Vector3i curr = path.back();
                if (interest_map[curr.x()][curr.y()][curr.z()]==1&&visited_map[curr.x()][curr.y()][curr.z()]==0&&curr.z()==layer) {
                    return path; // Found the path, return the complete path.
                }
                
                for (const auto& dir : directions) {
                    int nextRow = curr.x() + dir.x();
                    int nextCol = curr.y() + dir.y();
                    int nextHeight=curr.z()+dir.z();
                    if (isValidMove(nextRow, nextCol,nextHeight) && !visited[nextRow][nextCol][nextHeight]&&nextHeight<=upper) {
                        list<Vector3i> newPath = path;
                        newPath.push_back(Vector3i(nextRow, nextCol,nextHeight));
                        q.push(newPath);
                        visited[nextRow][nextCol][nextHeight] = true;
                    }
                }
            }
            // If the path is not found, return an empty list.
            return {};
        }
        
    private:
        vector<vector<vector<int>>> map;
        vector<vector<vector<int>>> interest_map;
        vector<vector<vector<int>>> visited_map;
        vector<vector<vector<int>>> height;
        Eigen::Vector3i map_shape;
        list<Eigen::Vector3i> path;
        int lowest=300;
        Eigen::Vector3i lowest_highest_index;
        Eigen::Vector3i my_position;
        bool get_position=false;
        bool state_1=true;
        bool state_2=false;
        int num=1;
        vector<int> area;
        int explore_index=0;
        int explore_layer=0;
        bool finish=false;
};





