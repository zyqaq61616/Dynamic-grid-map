/**
 * @file Dynamic_grid_map.h
 * @author zhang yuqi (zhangyuqi1@holomatic.com)
 * @brief  定义动态栅格地图的类
 * @version 1.0
 * @date 2022-02-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "holo/perception/obstacle_gridmap/grid_map_types.h"
namespace holo
{
namespace perception
{

//用枚举类表示占用状态
enum class OccStatus : uint8_t
{
    UNKNOWN = 0U,
    OCCUPY = 1U,
    FREE = 2U
};

////////////////////////////////////////////////////////                                                                                                        动态栅格地图观测值类
class DynamicMeasurementCell
{
public:
    //有参构造
    DynamicMeasurementCell(MapPointGrid const& point,Classification const& class_index,
                                                            MotionStatus const& m_status,MapPoint const& velocity,
                                                            OccStatus const& o_status,uint32_t id)
    {
        this->point = point;
        this->class_index = class_index;
        this->motionstatus = m_status;
        this->velocity = velocity;
        this->status = o_status;
        this->obstacle_id = id;
    }

    //默认构造
    DynamicMeasurementCell()
    {
        this->motionstatus = MotionStatus::UNKNOWN;
        this->status =  OccStatus::UNKNOWN;
        this->obstacle_id = 0U;
    }

    //重载等号
    DynamicMeasurementCell & operator=(DynamicMeasurementCell const& cell)
    {
        this->point = cell.point;
        this->class_index = cell.class_index;
        this->status = cell.status;
        this->velocity = cell.velocity;
        this->motionstatus = cell.motionstatus;
        this->obstacle_id = cell.obstacle_id;
        return *this;
    }

    //设置栅格坐标
    void SetPoint(MapPointGrid const& point)
    {
        this->point = point;
    }

    //设置初始值
    void SetStatus(OccStatus const& status)
    {
        this->status = status;
    }

    //设置类别
    void SetClassIndex(Classification const& class_index ,MotionStatus &statue)
    {

        this->class_index.SetMainType(class_index.GetMainType());
        this->class_index.SetSubType (class_index.GetSubType());
        this->motionstatus = statue;
    }
    //设置id
    void SetObstacleID(uint32_t id)
    {
        this->obstacle_id = id;
    }
    //设置速度
    void SetVelocity(MapPoint const& velocity)
    {
        this->velocity = velocity;
    }
    //设置运动状态
    void SetMotionStatus(MotionStatus &status)
    {
        this->motionstatus = status;
    }
    //设置占用状态
    void SetOccupyStatus(OccStatus const& status)
    {
        this->status = status;
    }
    //初始化类别和观测值(初始化了占用情况,速度,类别)
    void Initialize()
    {
        this->status = OccStatus::UNKNOWN;
        this->velocity.x = 0.0f;
        this->velocity.y = 0.0f;
        Classification temp_cla;
        this->class_index = temp_cla;
        this->motionstatus =MotionStatus::UNKNOWN;
        this->obstacle_id = 0U;
    }
public:
    MapPointGrid point;
    Classification class_index;
    uint32_t obstacle_id;
    MotionStatus motionstatus;
    MapPoint velocity;
    OccStatus status;
};

////////////////////////////////////////                                                                                                                    动态栅格地图栅格类
class DynamicGridMapCell
{
public:
    //有参构造
    DynamicGridMapCell(Classification const& class_index,MotionStatus const& motionstatus,
                                                OccStatus const& status,MapPoint const& velocity,MapPointGrid const& point,
                                                uint32_t id)
    {
        this->point_ = point;
        this->velocity_ = velocity;
        this->classindex_ = class_index;
        this->motionstatus_=motionstatus;
        this->status_ = status;
        this->obstacle_id_ = 0U;
    }
    //默认构造
    DynamicGridMapCell()
    {
        this->motionstatus_= MotionStatus::UNKNOWN;
        this->status_ = OccStatus::UNKNOWN;
        this->obstacle_id_ = 0U;
    }
    ~DynamicGridMapCell(){};

    //是不是未知点
    bool_t IsUnknown()
    {
        if(this->status_ == OccStatus::UNKNOWN)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    //获取栅格坐标
    inline MapPointGrid GetPoint()
    {
        return point_;
    }

    //获取速度
    inline MapPoint GetVelocity()
    {
        return this->velocity_;
    }

    //获取运动状态
    inline MotionStatus GetMotionStatus()
    {
        return this->motionstatus_;
    }

    //获取类别
    inline Classification GetClassification()
    {
        return this->classindex_;
    }

    //获取ID
    inline uint32_t GetObstacleID()
    {
        return this->obstacle_id_;
    }
    //获取占用状态
    OccStatus GetOccupyStatus()
    {
        return this->status_;
    }

    //设置速度值
    void SetVelocity(MapPoint const& velocity)
    {
        this->velocity_ = velocity;
    }

    //设置栅格坐标
    void SetPoint(MapPointGrid const& point)
    {
        this->point_ = point;
    }

    //设置运动状态
    void SetMotionStatus(MotionStatus const& status)
    {
        this->motionstatus_ = status;
    }

    //设置观测状态
    void SetOccupyStatus(OccStatus const& status)
    {
        this->status_ = status;
    }

    //设置类别
    void SetClassification(Classification const& classification)
    {
        this->classindex_ = classification;
    }
    //设置障碍物ID
    void SetObstacleID(uint32_t id)
    {
        this->obstacle_id_ = id;
    }
    //初始化(数值和类别)
    void Initialize()
    {
        Classification temp_cla;
        MapPoint temp_vel(0.0f,0.0f);
        this->classindex_ = temp_cla;
        this->obstacle_id_ = 0U;
        this->velocity_ = temp_vel;
        this->motionstatus_ = MotionStatus::UNKNOWN;
        this->status_ = OccStatus::UNKNOWN;
    }

    //用观测值更新原值
    bool_t UpDateCell(DynamicMeasurementCell const* measurement_cell)
    {
        if(!(this->point_ == measurement_cell->point))
        {
            return false;
        }
        else
        {
                this->classindex_ = measurement_cell->class_index;
                this->velocity_ = measurement_cell->velocity;
                this->status_ = measurement_cell->status;
                this->motionstatus_ = measurement_cell->motionstatus;
                this->obstacle_id_ = measurement_cell->obstacle_id;
        }
    }

    //重载赋值运算符
    DynamicGridMapCell & operator=(DynamicGridMapCell & cell)
    {
        this->point_ = cell.GetPoint();
        this->velocity_ = cell.GetVelocity();
        this->classindex_ = cell.GetClassification();
        this->status_ = cell.GetOccupyStatus();
        this->motionstatus_ = cell.GetMotionStatus();
        this->obstacle_id_ = cell.GetObstacleID();
    }
    private:
    MapPointGrid point_;
    MapPoint velocity_; 
    Classification classindex_;
    uint32_t obstacle_id_;
    MotionStatus motionstatus_;
    OccStatus status_;
};


/////////////////////////////////////////////////////////                                                                               动态栅格地图类
class DynamicGridMap
{
public:
    //用的是float类型的Odometry
    using FreeSpace = holo::common::AutoFreespace<Point2f>;
    using Odometry = holo::common::Odometryf;
    using ObstacleGeneralPtrList = holo::obstacle::ObstaclePtrListT<holo::obstacle::ObstacleGeneralT<holo::float32_t>, 128UL>;
    using Obstacle = holo::obstacle::ObstacleGeneralT<float32_t>;
    //
    DynamicGridMap(Params const& param);
    DynamicGridMap();
    ~DynamicGridMap();
    
    //初始化地图
    void Initialize();

    //更新观测栅格列表
    void UpdateMeasurement(std::map<uint32_t,DynamicMeasurementCell> const & current_measurement);

    //更新栅格的新下标,时间戳,和里程计信息
    void UpdatePose(Odometry const& current_odom);

    //更新地图
    void UpdateMap();

    void SetParams(Params const& params);

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
    std::vector<DynamicGridMapCell*> GetAllCell();
    //开发障碍物列表和freespace观测栅格
    std::map<uint32_t,DynamicMeasurementCell> DynamicGridMap::InputToMeasurement(std::vector<Obstacle> const& dynamic_obstacle);

    //重载一下
    std::map<uint32_t,DynamicMeasurementCell> DynamicGridMap::InputToMeasurement(ObstacleGeneralPtrList const& obstacle);

private:
    Timestamp timestamp_; //时间戳
    Odometry  odometry_;    //里程计
    Params param_;                   //参数
    std::vector<DynamicGridMapCell*> grid_map_master_; //主栅格地图 
    std::vector<DynamicMeasurementCell*> dynamic_measurement_master_; //观测栅格地图
    bool_t initialized_;    //是否初始化
};

}//namespace holo
}//namespace perception