#include "utility.h"
#include <vector>
#include <utility>
#include <map>
#include "Eigen/Dense"
#include <list>
#include <memory>
#include <algorithm>
#include <iostream>

#pragma once
#include <vector>
#include <list>
#include <memory>
#include <algorithm>
#include <iostream>
struct ANode
{
    ANode(int X, int Y, int Z, std::shared_ptr<ANode> p = nullptr) : x(X), y(Y), z(Z), prev(p) {}
    int x;                     // 点的x坐标
    int y;                     // 点的y坐标
    int z;                     // 点的z坐标
    int G = 0;                 // 起点到该点的欧拉距离
    int H = 0;                 // 该点到终点的曼哈顿距离
    int F = 0;                 // G+H
    std::weak_ptr<ANode> prev; // 指向的前一个节点!!!改成了weak_ptr不然数据多的时候析构报错
};

class AStar
{
public:

    AStar(){};

    AStar(vector<vector<std::vector<int>>> m, Eigen::Vector3i shape) : maps(m)
    {
        map_shape = shape;
    }

    std::shared_ptr<ANode> findPath(std::shared_ptr<ANode> beg, std::shared_ptr<ANode> end);
    
    std::shared_ptr<ANode> findPathlong(std::shared_ptr<ANode> beg, std::shared_ptr<ANode> end);
    
    void PrintAStarPath(Eigen::Vector3i sp, Eigen::Vector3i ep);
    
    list<Eigen::Vector3i> get_path_long(vector<vector<std::vector<int>>> m, Eigen::Vector3i sp, Eigen::Vector3i ep);

    list<Eigen::Vector3i> get_path(vector<vector<std::vector<int>>> m, Eigen::Vector3i sp, Eigen::Vector3i ep);
    ~AStar()
    {
        openlist.clear();
        closeist.clear();
    }

private:
    Eigen::Vector3i start_point;
    Eigen::Vector3i end_point;

    void refreshOpenList(std::shared_ptr<ANode>, std::shared_ptr<ANode> end);
    int calculateH(std::shared_ptr<ANode>, std::shared_ptr<ANode>) const;
    int calculateF(std::shared_ptr<ANode>, std::shared_ptr<ANode>) const;

private:
    Eigen::Vector3i map_shape;
    std::vector<vector<std::vector<int>>> maps; // 地图
    std::list<std::shared_ptr<ANode>> openlist; // 保存还未遍历过的节点
    std::list<std::shared_ptr<ANode>> closeist; // 保存已经找到最短路径的节点
    const static int costLow;                   // 6
    const static int costHigh;                  // 8
    const static int costLargest;               // 8
};

const int AStar::costLow     = 10;
const int AStar::costHigh    = 14;
const int AStar::costLargest = 17;

int AStar::calculateH(std::shared_ptr<ANode> point, std::shared_ptr<ANode> end) const
{
    return costLow * (std::abs(point->x - end->x) + std::abs(point->y - end->y) + std::abs(point->z - end->z));
}

int AStar::calculateF(std::shared_ptr<ANode> point, std::shared_ptr<ANode> end) const
{
    return point->G + calculateH(point, end);
}

std::shared_ptr<ANode> AStar::findPath(std::shared_ptr<ANode> beg, std::shared_ptr<ANode> end)
{
    
    int literate_num = 0;
    // cout<<end->x<<","<<end->y<<","<<end->z<<endl;
    refreshOpenList(beg, end);
    while (!openlist.empty())
    {
        
        // cout<<end->x<<","<<end->y<<","<<end->z<<endl;
        auto iter = std::min_element(openlist.cbegin(), openlist.cend(), [](std::shared_ptr<ANode> p1, std::shared_ptr<ANode> p2)
                                     { return p1->F <= p2->F; });
        closeist.push_back(*iter);
        std::shared_ptr<ANode> iter_temp = *iter;
        openlist.erase(iter);
        refreshOpenList(iter_temp, end);
        auto iter2 = std::find_if(openlist.cbegin(), openlist.cend(), [end](std::shared_ptr<ANode> sp)
                                  { return (sp->x == end->x) && (sp->y == end->y) && (sp->z == end->z); });
        if (iter2 != openlist.end())
            return *iter2;
        literate_num++;
        if (literate_num > 1000)
        {
            return nullptr;
        }
    }
    
    return nullptr;
}

std::shared_ptr<ANode> AStar::findPathlong(std::shared_ptr<ANode> beg, std::shared_ptr<ANode> end)
{
    
    int literate_num = 0;
    // cout<<end->x<<","<<end->y<<","<<end->z<<endl;
    refreshOpenList(beg, end);
    while (!openlist.empty())
    {
        
        // cout<<end->x<<","<<end->y<<","<<end->z<<endl;
        auto iter = std::min_element(openlist.cbegin(), openlist.cend(), [](std::shared_ptr<ANode> p1, std::shared_ptr<ANode> p2)
                                     { return p1->F <= p2->F; });
        closeist.push_back(*iter);
        std::shared_ptr<ANode> iter_temp = *iter;
        openlist.erase(iter);
        refreshOpenList(iter_temp, end);
        auto iter2 = std::find_if(openlist.cbegin(), openlist.cend(), [end](std::shared_ptr<ANode> sp)
                                  { return (sp->x == end->x) && (sp->y == end->y) && (sp->z == end->z); });
        if (iter2 != openlist.end())
            return *iter2;
        literate_num++;
        if (literate_num > 50000)
        {
            return nullptr;
        }
    }
    
    return nullptr;
}

void AStar::refreshOpenList(std::shared_ptr<ANode> point, std::shared_ptr<ANode> end)
{
    if (openlist.empty() && closeist.empty())
    {
        
        closeist.push_back(point);
        for (int i = point->x - 1; i <= point->x + 1; ++i)
        {
            for (int j = point->y - 1; j <= point->y + 1; ++j)
            {
                for (int k = point->z - 1; k <= point->z + 1; ++k)
                {
                    if (i >= 0 && j >= 0 && k >= 0 && i < map_shape.x() && j < map_shape.y() && k < map_shape.z())
                    {
                        if (maps[i][j][k] == 1 || (i == point->x && j == point->y && k == point->z))
                        {
                            
                            continue;
                        }
                        if (abs(i - point->x) == 1 && abs(j - point->y) == 1 && abs(k - point->z) == 1)
                        {
                            
                            if (maps[i][j][point->z] == 0 && maps[i][point->y][k] == 0 && maps[point->x][j][k] == 0)
                            {
                                auto cur = std::make_shared<ANode>(i, j, k, point);
                                cur->G = costLargest;
                                cur->H = calculateH(cur, end);
                                cur->F = calculateF(cur, end);
                                openlist.push_back(cur);
                            }
                        }
                        else if ((i == point->x && j == point->y) || (i == point->x && k == point->z) || (k == point->z && j == point->y))
                        {
                            auto cur = std::make_shared<ANode>(i, j, k, point);
                            cur->G = costLow;
                            cur->H = calculateH(cur, end);
                            cur->F = calculateF(cur, end);
                            openlist.push_back(cur);
                            // cout<<k<<endl;
                        }
                        else
                        {
                            if (i == point->x && (maps[i][j][point->z] == 1 || maps[i][point->y][k] == 1))
                            {
                                continue;
                            }
                            else if (j == point->y && (maps[point->x][j][k] == 1 || maps[i][j][point->z] == 1))
                            {
                                continue;
                            }
                            else if (k == point->z && (maps[point->x][j][k] == 1 || maps[i][point->y][k] == 1))
                            {
                                continue;
                            }
                            else
                            {
                                auto cur = std::make_shared<ANode>(i, j, k, point);
                                cur->G = costHigh;
                                cur->H = calculateH(cur, end);
                                cur->F = calculateF(cur, end);
                                openlist.push_back(cur);
                            }
                        }
                    }
                }
            }
        }
    }
    // 刷新每一次找到的起点到当前点的最短路径中的当前点的周围节点情况
    else
    {
        for (int i = point->x - 1; i <= point->x + 1; ++i)
        {
            for (int j = point->y - 1; j <= point->y + 1; ++j)
            {
                for (int k = point->z - 1; k <= point->z + 1; ++k)
                {
                    if (i >= 0 && j >= 0 && k >= 0 && i < map_shape.x() && j < map_shape.y() && k < map_shape.z())
                    {
                        if (maps[i][j][k] == 1 || (i == point->x && j == point->y && k == point->z))
                        {
                            continue;
                        }
                        if (abs(i - point->x) == 1 && abs(j - point->y) == 1 && abs(k - point->z) == 1)
                        {
                            if (maps[i][j][point->z] == 0 && maps[i][point->y][k] == 0 && maps[point->x][j][k] == 0)
                            {
                                auto cur = std::make_shared<ANode>(i, j, k, point);
                                cur->G = costLargest + point->G;
                                ;
                                cur->H = calculateH(cur, end);
                                cur->F = calculateF(cur, end);
                                openlist.push_back(cur);
                                auto iter_close = std::find_if(closeist.cbegin(), closeist.cend(), [i, j, k](std::shared_ptr<ANode> sp)
                                                               { return (sp->x == i) && (sp->y == j) && (sp->z == k); });
                                if (iter_close == closeist.end())
                                {
                                    auto iter_open = std::find_if(openlist.cbegin(), openlist.cend(), [i, j, k](std::shared_ptr<ANode> sp)
                                                                  { return (sp->x == i) && (sp->y == j) && (sp->z == k); });
                                    if (iter_open != openlist.end())
                                    {
                                        if ((*iter_open)->G > cur->G)
                                        {
                                            (*iter_open)->G = cur->G;
                                            (*iter_open)->F = (*iter_open)->G + (*iter_open)->H;
                                            (*iter_open)->prev = point;
                                        }
                                    }
                                    else
                                    {
                                        openlist.push_back(cur);
                                    }
                                }
                            }
                        }
                        else if ((i == point->x && j == point->y) || (i == point->x && k == point->z) || (k == point->z && j == point->y))
                        {
                            auto cur = std::make_shared<ANode>(i, j, k, point);
                            cur->G = costLow + point->G;
                            ;
                            cur->H = calculateH(cur, end);
                            cur->F = calculateF(cur, end);
                            openlist.push_back(cur);
                            auto iter_close = std::find_if(closeist.cbegin(), closeist.cend(), [i, j, k](std::shared_ptr<ANode> sp)
                                                           { return (sp->x == i) && (sp->y == j) && (sp->z == k); });
                            if (iter_close == closeist.end())
                            {
                                auto iter_open = std::find_if(openlist.cbegin(), openlist.cend(), [i, j, k](std::shared_ptr<ANode> sp)
                                                              { return (sp->x == i) && (sp->y == j) && (sp->z == k); });
                                if (iter_open != openlist.end())
                                {
                                    if ((*iter_open)->G > cur->G)
                                    {
                                        (*iter_open)->G = cur->G;
                                        (*iter_open)->F = (*iter_open)->G + (*iter_open)->H;
                                        (*iter_open)->prev = point;
                                    }
                                }
                                else
                                {
                                    openlist.push_back(cur);
                                }
                            }
                        }
                        else
                        {
                            if (i == point->x && (maps[i][j][point->z] == 1 || maps[i][point->y][k] == 1))
                            {
                                continue;
                            }
                            else if (j == point->y && (maps[point->x][j][k] == 1 || maps[i][j][point->z] == 1))
                            {
                                continue;
                            }
                            else if (k == point->z && (maps[point->x][j][k] == 1 || maps[i][point->y][k] == 1))
                            {
                                continue;
                            }
                            else
                            {
                                auto cur = std::make_shared<ANode>(i, j, k, point);
                                cur->G = costHigh + point->G;
                                ;
                                cur->H = calculateH(cur, end);
                                cur->F = calculateF(cur, end);
                                openlist.push_back(cur);
                                auto iter_close = std::find_if(closeist.cbegin(), closeist.cend(), [i, j, k](std::shared_ptr<ANode> sp)
                                                               { return (sp->x == i) && (sp->y == j) && (sp->z == k); });
                                if (iter_close == closeist.end())
                                {
                                    auto iter_open = std::find_if(openlist.cbegin(), openlist.cend(), [i, j, k](std::shared_ptr<ANode> sp)
                                                                  { return (sp->x == i) && (sp->y == j) && (sp->z == k); });
                                    if (iter_open != openlist.end())
                                    {
                                        if ((*iter_open)->G > cur->G)
                                        {
                                            (*iter_open)->G = cur->G;
                                            (*iter_open)->F = (*iter_open)->G + (*iter_open)->H;
                                            (*iter_open)->prev = point;
                                        }
                                    }
                                    else
                                    {
                                        openlist.push_back(cur);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
void AStar::PrintAStarPath(Eigen::Vector3i sp, Eigen::Vector3i ep)
{

    auto start_sp = std::make_shared<ANode>(sp.x(), sp.y(), sp.z()), end_sp = std::make_shared<ANode>(ep.x(), ep.y(), ep.z());
    // yolo()''
    std::shared_ptr<ANode> final = findPath(start_sp, end_sp);
    
    if (!final)
        std::cout << "Not Find Path In 1000 Steps" << std::endl;
    else
    {
        std::cout << "Find Path:" << std::endl;
        while (final)
        {
            std::cout << "(" << final->x << "," << final->y << "," << final->z << ")" << endl;
            final = final->prev.lock();
        }
    }
}
list<Eigen::Vector3i> AStar::get_path(vector<vector<std::vector<int>>> m, Eigen::Vector3i sp, Eigen::Vector3i ep)
{
    maps = m;
    // if(maps[sp.x()][sp.y()][sp.z()]>0){
    //     cout<<"sp!=0"<<": "<<maps[sp.x()][sp.y()][sp.z()]<<endl;
    // }
    // if(maps[ep.x()][ep.y()][ep.z()]>0){
    //     cout<<"ep!=0"<<": "<<maps[ep.x()][ep.y()][ep.z()]<<endl;
    // } //debug test
    if (sp == ep)
    {
        return {sp};
    }
    list<Eigen::Vector3i> result;
    openlist.clear();
    closeist.clear();
    // cout<<"start:"<<sp.transpose()<<endl;//test debug
    // cout<<"end:"<<ep.transpose()<<endl;//test debug
    auto start_sp = std::make_shared<ANode>(sp.x(), sp.y(), sp.z()), end_sp = std::make_shared<ANode>(ep.x(), ep.y(), ep.z());
    // yolo()''
    std::shared_ptr<ANode> final = findPath(start_sp, end_sp);
    
    if (!final)
        std::cout << "Not Find Path In 1000 Steps" << std::endl;
    else
    {
        // std::cout<<"Find Path:"<<std::endl;
        while (final)
        {
            if (Eigen::Vector3i(final->x, final->y, final->z) == sp)
            {
                // cout<<"break"<<"("<<final->x<<","<<final->y<<","<<final->z<<")"<<endl;
                break;
            }
            result.push_back(Eigen::Vector3i(final->x, final->y, final->z));
            // std::cout<<"("<<final->x<<","<<final->y<<","<<final->z<<")"<<endl;
            final = final->prev.lock();
        }
    }
    return result;
}

list<Eigen::Vector3i> AStar::get_path_long(vector<vector<std::vector<int>>> m, Eigen::Vector3i sp, Eigen::Vector3i ep)
{
    maps = m;
    // if(maps[sp.x()][sp.y()][sp.z()]>0){
    //     cout<<"sp!=0"<<": "<<maps[sp.x()][sp.y()][sp.z()]<<endl;
    // }
    // if(maps[ep.x()][ep.y()][ep.z()]>0){
    //     cout<<"ep!=0"<<": "<<maps[ep.x()][ep.y()][ep.z()]<<endl;
    // } //debug test
    if (sp == ep)
    {
        return {sp};
    }
    list<Eigen::Vector3i> result;
    openlist.clear();
    closeist.clear();
    // cout<<"start:"<<sp.transpose()<<endl;//test debug
    // cout<<"end:"<<ep.transpose()<<endl;//test debug
    auto start_sp = std::make_shared<ANode>(sp.x(), sp.y(), sp.z()), end_sp = std::make_shared<ANode>(ep.x(), ep.y(), ep.z());
    // yolo()''
    std::shared_ptr<ANode> final = findPathlong(start_sp, end_sp);
    
    if (!final)
        std::cout << "Not Find Path In 50000 Steps" << std::endl;
    else
    {
        // std::cout<<"Find Path:"<<std::endl;
        while (final)
        {
            if (Eigen::Vector3i(final->x, final->y, final->z) == sp)
            {
                // cout<<"break"<<"("<<final->x<<","<<final->y<<","<<final->z<<")"<<endl;
                break;
            }
            result.push_back(Eigen::Vector3i(final->x, final->y, final->z));
            // std::cout<<"("<<final->x<<","<<final->y<<","<<final->z<<")"<<endl;
            final = final->prev.lock();
        }
    }
    return result;
}