/**
 * @file obstacle_gridmap_parking_pipline.cpp
 * @author zhang yuqi (zhangyuqi1@holomatic.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "holo/perception/obstacle_gridmap/obstacle_gridmap_parking_pipline.h"
#include <holo/utils/holo_root.h>

namespace holo
{
namespace perception
{

ObstacleGridMapParkingPipline::ObstacleGridMapParkingPipline()
{

}

void ObstacleGridMapParkingPipline::SetParams(std::string config_file)
{
    holo::yaml::Node node = holo::yaml::LoadFile(
        GetConfigFileAbsolutePath(config_file));
    StaticParams s_params;
    Params d_params;
    this->static_layer_.SetParams(s_params);
    this->dynamic_layer_.SetParams(d_params);
    
    this->static_layer_.Initialize();
    this->dynamic_layer_.Initialize();
}

ObstacleGridMapParkingPipline::Bool ObstacleGridMapParkingPipline::UpdateMap(OdometryMessage  const& odo_msg,FreeSpaceMessage const& free_msg,
                                                                                                                                                                            ObstacleGeneralPtrListMessage const& obs_msg)
{
    ObstacleGeneralPtrList obstacle_list_ = *obs_msg.get();
    Freespace freespace_ = *free_msg.get();
    OdometryType odometry_ = *odo_msg.get();


    std::vector<std::vector<Obstacle>> obstacles = ObstacleClassification(obstacle_list_);
    if(obstacles.size() != 2)
    {
        std::cout<<"障碍物分类有错误!"<<std::endl;
        return;
    }
    //生成两幅地图的观测数组
    std::map<uint32_t,DynamicMeasurementCell> dynamic_measurement = this->dynamic_layer_.InputToMeasurement(obstacles[1]);
    std::map<uint32_t,StaticMeasurementCell> static_measurement = this->static_layer_.InputToMeasurement(freespace_,obstacles[0]);

    //更新两幅地图的pose
    this->dynamic_layer_.UpdatePose(odometry_);
    this->static_layer_.UpdatePose(odometry_);

    //更新两幅地图的观测栅格
    this->dynamic_layer_.UpdateMeasurement(dynamic_measurement);
    this->static_layer_.UpdateMeasurement(static_measurement);

    //更新地图
    this->dynamic_layer_.UpdateMap();
    this->static_layer_.UpdateMap();

    //合并
    std::vector<StaticGridMapCell*> all_static_cell = this->static_layer_.GetAllCell();
    std::vector<DynamicGridMapCell*> all_dynamic_cell = this->dynamic_layer_.GetAllCell();
    holo::bool_t flag = holo::perception::MergeMap(all_static_cell, all_dynamic_cell,this->master_layer_,THRESHOLD);
    if(flag)
    {
        //发布
        std::cout<<"地图更新完成!"<<std::endl;
        return true;
    }
    else
    {
         std::cout<<"地图更新失败!"<<std::endl;
        return false;
    }
}


}
}