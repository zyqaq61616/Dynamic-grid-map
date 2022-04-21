/**
 * @file static_grid_map.h
 * @author your name (you@domain.com)
 * @brief  定义静态栅格地图的类
 * @version 0.1
 * @date 2022-02-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once 
#include "holo/perception/obstacle_gridmap/grid_map_types.h"
namespace holo
{
namespace perception
{


//静态栅格地图参数类
class StaticParams : public Params
{

#define FROZENTHRESHOLD 5U
public:
    //有参构造函数
    StaticParams(uint32_t height = 250U,uint32_t width = 200U,
    float32_t resolution = 0.10F,float32_t threshold = 5.0F,
    float32_t  initial_value = 50.0F,float32_t log_free_increment = -2.0F,
    float32_t log_occ_increment = 2.0F)
    {
        this->height = height;
        this->width = width;
        this->resolution = resolution;
        this->threshold = threshold;
        this->initial_value = initial_value;
        this->log_free_increment = log_free_increment;
        this->log_occ_increment = log_occ_increment;
    }
    ~StaticParams();
    StaticParams & operator=(StaticParams const& param)
    {
        this->height = param.height;
        this->initial_value = param.initial_value;
        this->log_free_increment = param.log_free_increment;
        this->log_occ_increment = param.log_occ_increment;
        this->resolution = param.resolution;
        this->threshold = param.threshold;
        this->width = param.width;
        return *this;
    }

public:
    float32_t  initial_value;
    float32_t log_free_increment;
    float32_t log_occ_increment;
};

//////////////////////////////////                                                                                                                  静态栅格地图观测值类
class StaticMeasurementCell
{
public:
    //有参构造
    StaticMeasurementCell(MapPointGrid const& point,Classification const& class_index,uint32_t id,float32_t value)
    {
        this->point = point;
        this->class_index = class_index;
        this->value = value;
        this->obstacle_id = id;
    }

    //默认构造
    StaticMeasurementCell()
    {
        this->obstacle_id = 0U;
    };

    //重载等号
    StaticMeasurementCell & operator=(StaticMeasurementCell const & cell)
    {
        this->point = cell.point;
        this->class_index = cell.class_index;
        this->value = cell.value;
        this->obstacle_id = this->class_index;
        return *this;
    }

    //设置栅格坐标
    void SetPoint(MapPointGrid const& point)
    {
        this->point.x = point.x;
        this->point.y = point.y;
    }

    //设置初始值
    void SetValue(float32_t value)
    {
        this->value = value;
    }

    //设置类别
    void SetClassIndex(Classification const& class_index)
    {

        this->class_index.SetMainType(class_index.GetMainType());
        this->class_index.SetSubType (class_index.GetSubType());
    }
    //设置障碍物ID
    void SetID(uint32_t const id)
    {
        this->obstacle_id = id;
    }
    //初始化类别和观测值
    void Initialize(float32_t value)
    {
        this->value = value;
        Classification temp;
        this->class_index = temp;
        this->obstacle_id = 0U;
    }
public:
    MapPointGrid point;
    Classification class_index;
    uint32_t obstacle_id;
    float32_t value;
};

//////////////////////////////////////////                                                                                                          静态栅格地图栅格类
class StaticGridMapCell
{
public:
    
    StaticGridMapCell(MapPointGrid const& point,float32_t value)
    {
       this->point_ = point;
       this->value_ = value;
       this->initial_value_ = value;
    }
    StaticGridMapCell();
    ~StaticGridMapCell();

    //是不是未知点
    bool_t IsUnknown()
    {
        if(classindex_queue_.size() == 0 && value_ == initial_value_ )
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    //是不是空点(需要设定阈值 默认小于初始值就是空点也就是一次标记为空)
    bool_t IsFree(float32_t thread)
    {
        //类别队列为空 并且value值小于阈值(可以设)
        if(classindex_queue_.size() != 0 && value_< thread)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    //是不是空点(如果不设置阈值默认小于初始值就为空)
    bool_t IsFree()
    {
        return value_< initial_value_;
    }

    //获取栅格坐标
    inline MapPointGrid GetPoint()
    {
        return point_;
    }

    //获取当前值
    inline float32_t GetCurrentValue()
    {
        return value_;
    }
    //获取初始值
    inline float32_t GetInitialValue()
    {
        return initial_value_;
    }
    //获取获取类别
    Classification GetClassification()
    {
        //如果为空就返回一个未知类 如果不为空就返回最新加入的类
        if(classindex_queue_.size() == 0)
        {
            Classification result;
            return  result;
        }
        else
        {
            return classindex_queue_.back();
        }
    }
    //获取障碍物ID
    uint32_t GetObstacleID()
    {
        //如果为空就返回一个未知类 如果不为空就返回最新加入的类
        if(obstacle_id_queue_.size() == 0)
        {
            return  0U;
        }
        else
        {
            return obstacle_id_queue_.back();
        }
    }
    //获取障碍物ID队列
    inline std::queue<uint32_t> GetObstacleIDQueue()
    {
        return this->obstacle_id_queue_;
    }
    //获取类别队列
    inline std::queue<Classification> GetClassificationQueue()
    {
        return this->classindex_queue_;
    }
    //更新value值
    void SetValue(float32_t value)
    {
        this->initial_value_ = value;
        this->value_ = value;
    }
    //设置栅格坐标
    void SetPoint(MapPointGrid const& point)
    {
        this->point_ = point;
    }
    //初始化(数值和类别)
    void Initialize(float32_t value)
    {
        this->initial_value_ = value;
        this->value_ = value;
        std::queue<Classification> temp;
        this->classindex_queue_ = temp; 
        std::queue<uint32_t> temp_id;
        this->obstacle_id_queue_ = temp_id;
    }
    //用观测值更新原值(如果free只更新值,占用需要更新概率)
    bool_t UpDateCell(StaticMeasurementCell const* measurement_cell)
    {
        //如果栅格连续五次保持空闲或者占用,则不再更新
        float32_t frozen = abs(initial_value_ -value_);
        if(frozen >= abs(FROZENTHRESHOLD * measurement_cell->value))
        {
            return true;
        }

        //如果两个栅格标号相同,则更新
        if(point_ == measurement_cell->point)
        {
            //如果是占用状态 那么就把观测到的类别放入
            if(measurement_cell->value > 0)
            {
                this->value_ += measurement_cell->value;
                classindex_queue_.push(measurement_cell->class_index);
                obstacle_id_queue_.push(measurement_cell->obstacle_id);
            }
            else
            {
                //如果是空闲状态,那么队列为空前出队一个类别
                this->value_ += measurement_cell->value;
                if(this->classindex_queue_.size() != 0)
                {
                    this->classindex_queue_.pop();
                    this->obstacle_id_queue_.pop();
                }
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    //重载赋值运算符
    StaticGridMapCell & operator=(StaticGridMapCell & cell)
    {
        this->point_ = cell.GetPoint();
        this->value_ = cell.GetCurrentValue();
        this->initial_value_ = cell.GetInitialValue();
        this->classindex_queue_ = cell.GetClassificationQueue();
        this->obstacle_id_queue_ = cell.GetObstacleIDQueue();
    }
    
    private:
    MapPointGrid point_;
    std::queue<Classification> classindex_queue_;
    std::queue<uint32_t> obstacle_id_queue_;
    float32_t value_;
    float32_t initial_value_;
};


//////////////////////////////////////////////                                                                                                                  静态栅格地图类
class StaticGridMap
{
using FreeSpace = holo::common::AutoFreespace<Point2f>;
using Odometry = holo::common::Odometryf;
using ObstacleGeneralPtrList = holo::obstacle::ObstaclePtrListT<holo::obstacle::ObstacleGeneralT<holo::float32_t>, 128UL>;
using Obstacle = holo::obstacle::ObstacleGeneralT<float32_t>;
public:
    //用的是float类型的Odometry

    
    StaticGridMap(StaticParams const& param);
    StaticGridMap();
    ~StaticGridMap();
    
    //初始化地图
    void Initialize();
    ///////////////////////////////////                                                                                                                                                         freespace和障碍物列表to观测栅格
    std::map<uint32_t,StaticMeasurementCell> InputToMeasurement(FreeSpace const& freespace,std::vector<Obstacle> const& static_obstacle);
    //更新观测栅格列表
    void UpdateMeasurement(std::map<uint32_t,StaticMeasurementCell> const& current_measurement);

    //更新栅格的新下标,时间戳,和里程计信息
    void UpdatePose(Odometry const& current_odom);

    //更新地图
    void UpdateMap();

    void SetParams(StaticParams const& params);


    //获取时间戳
    inline Timestamp GetTimestamp()
    {
        return timestamp_;
    }

    //获取里程计数据
    inline Odometry GetOdometry()
    {
        return this->odometry_;
    }
    //
    inline bool_t IsInitialized()
    {
        return this->initialized_;
    }
    //获取地图
    std::vector<StaticGridMapCell*> GetAllCell();
    inline float32_t GetThreshold()
    {
        return this->param_.threshold;
    }

private:
    Timestamp timestamp_;
    Odometry  odometry_;
    StaticParams param_;
    std::vector<StaticGridMapCell*> grid_map_master_;
    std::vector<StaticMeasurementCell*> static_measurement_master_;
    bool_t initialized_;
    bool_t is_first_;
};
}
}