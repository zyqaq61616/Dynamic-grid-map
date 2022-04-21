#include "holo/perception/obstacle_gridmap/dynamic/dynamic_grid_map.h"

namespace holo
{
namespace perception
{
    //构造函数应该没啥问题
    DynamicGridMap::DynamicGridMap(Params const& param)
    {
       this->param_ = param;
        this->initialized_ = false;
    }

     DynamicGridMap::DynamicGridMap()
     {
        this->initialized_ = false;
     }

    //析构函数应该没啥问题
    DynamicGridMap::~DynamicGridMap()
    {
        //delete空间   
        uint32_t map_size = this->grid_map_master_.size();
        for(int i = 0;i < map_size;i++)
        {
            if(this->dynamic_measurement_master_[i] != NULL)
            {
                delete this->dynamic_measurement_master_[i];
                this->dynamic_measurement_master_[i] = NULL;
            }
            if( this->grid_map_master_[i] != NULL)
            {
                delete this->grid_map_master_[i];
                this->grid_map_master_[i] = NULL;
            }
        }
        std::cout<<"成功销毁地图!"<<std::endl;
    }
    void DynamicGridMap::SetParams(Params const& params)
    {
        this->param_ = params;
    }
    //初始化函数(主要更新地图的下标点)
    void DynamicGridMap::Initialize()
    {
        uint32_t map_size = this->param_.height * this->param_.width;
        this->dynamic_measurement_master_.reserve(map_size);
        this->grid_map_master_.reserve(map_size);
        //在堆上开辟空间
        for(int i = 0;i < map_size;i++)
        {
            this->dynamic_measurement_master_[i] = new DynamicMeasurementCell;
            this->grid_map_master_[i] = new DynamicGridMapCell;

            //更新栅格地图列表(坐标点,初始value值,Classification有默认值)
            MapPointGrid grid_coordinate = OrderToGridCoordinate(i , this->param_.width);
            this->grid_map_master_[i]->SetPoint(grid_coordinate);
            this->grid_map_master_[i]->Initialize();

            //更新观测列表(坐标点)
            this->dynamic_measurement_master_[i]->SetPoint(grid_coordinate);
            this->dynamic_measurement_master_[i]->Initialize();
        }
        //构造以后直接初始化
        this->Initialize();
        std::cout<<"成功创建地图!"<<std::endl;
        std::cout<<"地图的尺寸为:"<<map_size<<std::endl;

        this->initialized_ =true;
        std::cout<<"动态观测列表和栅格地图列表已初始化完成!"<<std::endl;

    }

    //更新观测栅格列表
    void DynamicGridMap::UpdateMeasurement(std::map<uint32_t,DynamicMeasurementCell> const& current_measurement)
    {
        for(auto it = current_measurement.begin();it != current_measurement.end();it++)
        {
            uint32_t order = it->first;
            *(this->dynamic_measurement_master_[order]) = it->second;
        }
        std::cout<<"观测栅格列表更新完成!"<<std::endl;
    }

    //更新时间戳和里程计
    void DynamicGridMap::UpdatePose(Odometry const& current_odom)
    {
        //把里程计和时间戳的信息记录下来
            this->timestamp_ = current_odom.GetTimestamp();
            this->odometry_ = current_odom;
            return;
    }

    //更新地图
    void DynamicGridMap::UpdateMap()
    {
        for(int i = 0; i < this->grid_map_master_.size();i++)
        {
            //先把主栅格都初始化
            this->grid_map_master_[i]->Initialize();
            //读观测栅格列表的值
            DynamicMeasurementCell * measurement_cell = this->dynamic_measurement_master_[i];
            //更新主栅格
            bool_t flag = this->grid_map_master_[i]->UpDateCell(measurement_cell);
            if(!flag)
            {
                std::cout<<"动态栅格地图 :"<< i <<"更新失败!"<<std::endl;
            }
            //把观测栅格列表初始化为全部未观测状态
            this->dynamic_measurement_master_[i]->Initialize();

        }
        std::cout<<"动态地图地图更新完成!"<<std::endl;
    }

    std::map<uint32_t,DynamicMeasurementCell> DynamicGridMap::InputToMeasurement(std::vector<Obstacle> const& dynamic_obstacle)
    {
        std::map<uint32_t,DynamicMeasurementCell> result;
          //处理障碍物列表
        for(int i = 0; i < dynamic_obstacle.size();i++)
        {
            //拿到当前的障碍物
            Obstacle obstacle_temp = dynamic_obstacle[i];
            //计算障碍物的四个角点
            std::vector<Point2f> box_corner = ObstaclePolygon(obstacle_temp);
            //找到近似的左上角和右下角点
            std::vector<Point2f> box_boundary_corner = FindBoundaryPoint(box_corner);
            MapPoint box_left_top_conor(box_boundary_corner[0].GetX(),box_boundary_corner[0].GetY());
            MapPoint box_right_bottom_cornor(box_boundary_corner[1].GetX(),box_boundary_corner[1].GetY());
            //计算左上角和右下角两个点在栅格坐标系下的坐标
            MapPointGrid box_left_top = VehicleToGrid(box_left_top_conor,this->param_.width,
                                                                                this->param_.height,this->param_.resolution);
            MapPointGrid box_right_bottom = VehicleToGrid(box_right_bottom_cornor,this->param_.width,
                                                                                this->param_.height,this->param_.resolution);
            //遍历
            for(int i = box_right_bottom.x;i <= box_left_top.x;i++)
            {
                for(int j = box_right_bottom.y;j <= box_left_top.y;j++)
                {
                    bool_t flag = IsInteriorPoint(MapPointGrid(i,j),box_boundary_corner,this->param_.width,this->param_.height,this->param_.resolution);
                    //如果是freespace的内点
                    if(flag)
                    {
                        //点,速度,障碍物id,障碍物类别
                        DynamicMeasurementCell temp;
                        temp.point.x = i;
                        temp.point.y =j;
                        temp.velocity.x = obstacle_temp.GetRelativeVelocity().At(0);
                        temp.velocity.y = obstacle_temp.GetRelativeVelocity().At(1);
                        temp.obstacle_id =obstacle_temp.GetObstacleId();
                        temp.class_index = obstacle_temp.GetClassification();
                        uint32_t index = GridCoordinateToOrder(temp.point,this->param_.width);
                        result[index] = temp;
                    }
                }
            }                                                  
        }
    return result;
    }

    std::map<uint32_t,DynamicMeasurementCell> DynamicGridMap::InputToMeasurement(ObstacleGeneralPtrList const& obstacle)
    {
        
    }
}// namespace perception
}//namespace holo