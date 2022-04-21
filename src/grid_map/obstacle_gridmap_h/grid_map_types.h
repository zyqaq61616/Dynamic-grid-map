/**
 * @file holo_grid_map_types.h
 * @author zhang yuqi (you@domain.com)
 * @brief Define map and grid base classes and function
 * @version 0.1
 * @date 2022-02-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

//公司头文件
#pragma once 
#include <holo/common/timestamp.h>
#include <holo/common/classification.h>
#include <holo/geometry/point2.h>
#include <holo/common/odometry.h>
#include <holo/obstacle/obstacle_list.h>
#include <holo/common/freespace.h>
#include <holo/container/vector.h>
#include "holo/perception/obstacle_gridmap/merge.h"
//基础头文件
#include<vector>
#include<map>
#include<queue>
#include<string>
#include<unordered_map>
#include <iostream>
#include <sstream>

namespace holo
{
namespace perception
{
using Box3f = holo::geometry::Box3f;
using Point2f = holo::geometry::Point2f;
using Vector3f = holo::numerics::Vector3f;


#define PI 3.1415926f
#define THRESHOLD 5.0f 
#define MAPFRONT 20.0F
#define MAPBACK -5.0F
#define MAPLEFT 10.0F
#define MAPRIGHT -10.0F 
// ///////////////////////////////////////////////////                                                                                          定义基础参数类
class Params
{
public:
    //有参构造
    Params(uint32_t in_height = 250,uint32_t in_width = 200,
    float32_t in_resolution = 0.10f,float32_t in_threshold = 5.0f)
    : height(in_height),width(in_width),resolution(in_resolution),
    threshold(in_threshold){};
    ~Params();
    Params & operator=(Params const& other)
    {
        this->height = other.height;
        this->width = other.width;
        this->resolution = other.resolution;
        this->threshold = this->threshold;
    }
public:
    uint32_t height; //地图高度
    uint32_t width;  //地图水平宽度
    float32_t resolution; //地图分辨率
    float32_t threshold;   //判断为占有的阈值根据置信度
};

/////////////////////////////////////////////////////                                                                                            汽车坐标系(连续)
struct MapPoint
{
public:
    MapPoint(float32_t x = 0.0f,float32_t y = 0.0f) : x(x),y(y){};
    MapPoint &operator=(MapPoint const& point)
    {
        this->x = point.x;
        this->y = point.y;
        return *this;
    }

    bool_t operator==(MapPoint const& point)
    {
        if(this->x == point.x && this->y == point.y)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    //是否为无效点
    inline bool_t IsNull()
    {
        return x ==0.0f && y == 0.0f;
    }
public:
    float32_t x;
    float32_t y;
};
    
//////////////////////////////////////////////////////////////////                                                                      地图坐标(离散)
struct MapPointGrid
{
public:
    MapPointGrid(uint32_t x = 0,uint32_t y = 0) : x(x),y(y){};    using Point2f = holo::geometry::Point2f;

    MapPointGrid &operator=(MapPointGrid const& point)
    {
        this->x = point.x;
        this->y = point.y;
        return *this;
    }

    bool_t operator==(MapPointGrid const& point)
    {
        if(this->x == point.x && this->y == point.y)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    //是否为无效点
    inline bool_t IsNull()
    {
        return x ==0 && y == 0;
    }
public:
    uint32_t x;
    uint32_t y;
};

    /////////////////////////////////////////////////////////////////                                                                           功能函数

    //输入序号和宽度可以生成在栅格地图中的坐标(已验证)
    MapPointGrid OrderToGridCoordinate(uint32_t index,uint32_t width)
    {
            MapPointGrid result;
            index ++;
            int remainder = index % width;
            if(remainder ==0)
            {
                result.x = index /width;
                result.y = width;
            }
            else
            {
                result.x = index /width +1;
                result.y = remainder;
            }
            return result;
    }

    //输入序号和宽度可以生成在栅格地图中的坐标(已验证)
    inline uint32_t GridCoordinateToOrder(MapPointGrid point,uint32_t width)
    {
        return (point.x - 1) * width+point.y - 1; 
    }
    //栅格坐标系转汽车坐标系(中心点)(已验证)
    MapPoint GridToVehicle(MapPointGrid const& point,uint32_t width, uint32_t height,float32_t resolution)
    {
        MapPoint result;
        if(point.x == 0 || point.y == 0 || point.x > height || point.y > width)
        {
            return result;
        }
        float32_t temp_x = (point.x - 1) * resolution + resolution / 2;
        float32_t temp_y = (point.y - 1) * resolution + resolution / 2;
        result.x = (temp_x + MAPBACK);
        result.y = (temp_y - (width * resolution) / 2);
        return result;
    }

    //汽车坐标系转栅格坐标系(已验证,但是有一个关于浮点数运算的小bug)
    MapPointGrid VehicleToGrid(MapPoint const& point,uint32_t width, uint32_t height,float32_t resolution)
    {
        //先把汽车坐标系原点移动到地图的右下角
        MapPointGrid result;
        float32_t temp_x = point.x - MAPBACK;
        float32_t temp_y = point.y + (width * resolution) / 2;
        if(temp_x >= 0.0f && temp_y >= 0.0f)
        {
            if(temp_y < width *resolution && temp_x < height *resolution)
            {
                //左边界和上边界不包括在该网格中
                result.x = (uint32_t(temp_x / resolution) + 1);
                result.y = (uint32_t(temp_y / resolution) + 1);
            }
        }
        return result;
    }

    //  汽车坐标系变换导致的点转换(栅格点)
    MapPointGrid CoordinateSystemTF(Pose2f const& movement,MapPointGrid const& point,uint32_t width, uint32_t height,float32_t resolution)
    {
        MapPointGrid result;
        //如果传入的栅格已经越界了返回(0,0)
        if(point.x == 0 || point.y == 0 || point.x > height || point.y > width)
        {
            return result;
        }
        //栅格转汽车坐标系
        float32_t temp_x = (point.x - 1) * resolution + resolution / 2;
        float32_t temp_y = (point.y - 1) * resolution + resolution / 2;

        float32_t primal_x = (temp_x + MAPBACK);
        float32_t primal_y = (temp_y - (width * resolution) / 2);
        // std::cout<<"汽车坐标系X:"<< primal_x<<std::endl;
        // std::cout<<"汽车坐标系Y:"<< primal_y<<std::endl;

        //x y yaw 增量(默认正方向,且顺时针方向为正方向)
        float32_t movement_x = movement.GetX();
        float32_t movement_y = movement.GetY();
        float32_t movement_yaw = movement.GetTheta();

        //在新坐标系下的X和Y
        float32_t after_x = (primal_x - movement_x) * cos(movement_yaw) + (primal_y - movement_y) * sin(movement_yaw);
        float32_t after_y = (primal_y - movement_y) * cos(movement_yaw) - (primal_x - movement_x) * sin(movement_yaw);

        // std::cout<<"变换后的汽车坐标系X:"<< after_x<<std::endl;
        // std::cout<<"变换后的汽车坐标系Y:"<< after_y<<std::endl;

        //转到栅格坐标系下
        float32_t result_temp_x = after_x - MAPBACK;
        float32_t result_temp_y = after_y + (width * resolution) / 2;
        if(temp_x >= 0 && temp_y >= 0)
        {
            if(result_temp_y < width *resolution && result_temp_x < height *resolution)
            {
                //左边界和上边界不包括在该网格中
                result.x = (uint32_t(result_temp_x / resolution) + 1);
                result.y = (uint32_t(result_temp_y / resolution) + 1);
            }
        }
        // std::cout<<"变换后的栅格坐标系X:"<< after_x<<std::endl;
        // std::cout<<"变换后的栅格坐标系Y:"<< after_y<<std::endl;
        return result;
    }


} //namespace perception
} //namespace holo