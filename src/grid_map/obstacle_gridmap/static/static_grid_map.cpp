#include "holo/perception/obstacle_gridmap/static/static_grid_map.h"

namespace holo
{
namespace perception
{
    StaticGridMap::StaticGridMap(StaticParams const& param)
    {
        //初始化观测数组和地图大小
        this->param_ = param;
        this->initialized_ = false;
    }

     StaticGridMap::StaticGridMap()
    {
        this->initialized_ = false;
    }

    StaticGridMap::~StaticGridMap()
    {
        //delete空间   
        uint32_t map_size = this->grid_map_master_.size();
        for(int i = 0;i < map_size;i++)
        {
            if(this->static_measurement_master_[i] != NULL)
            {
                delete this->static_measurement_master_[i];
                this->static_measurement_master_[i] = NULL;
            }
            if( this->grid_map_master_[i] != NULL)
            {
                delete this->grid_map_master_[i];
                this->grid_map_master_[i] = NULL;
            }
        }
        std::cout<<"成功销毁地图!"<<std::endl;
    }

    void StaticGridMap::SetParams(StaticParams const& params)
    {
        this->param_ = params;
    }
    //初始化函数
    void StaticGridMap::Initialize()
    {

        uint32_t map_size = this->param_.height * this->param_.width;
        this->static_measurement_master_.reserve(map_size);
        this->grid_map_master_.reserve(map_size);
        //在堆上开辟空间
        for(int i = 0;i < map_size;i++)
        {
            this->static_measurement_master_[i] = new StaticMeasurementCell;
            this->grid_map_master_[i] = new StaticGridMapCell;

            //更新栅格地图列表(坐标点,初始value值,Classification有默认值)
            MapPointGrid grid_coordinate = OrderToGridCoordinate(i , this->param_.width);
            this->grid_map_master_[i]->SetPoint(grid_coordinate);
            this->grid_map_master_[i]->SetValue(this->param_.initial_value);

            //更新观测列表(坐标点)
            this->static_measurement_master_[i]->SetPoint(grid_coordinate);
            this->static_measurement_master_[i]->SetValue(0.0f);
            this->static_measurement_master_[i]->SetID(0U);
        }

        std::cout<<"成功创建地图!"<<std::endl;
        std::cout<<"地图的尺寸为:"<<map_size<<std::endl;


        this->is_first_ = true;
        this->initialized_ =true;
        std::cout<<"静态观测列表和栅格地图列表已初始化完成!"<<std::endl;

    }

    //更新观测栅格列表(有一个关于内联函数和const的bug)
    void StaticGridMap::UpdateMeasurement(std::map<uint32_t,StaticMeasurementCell> const& current_measurement)
    {
        for(auto it = current_measurement.begin();it != current_measurement.end();it++)
        {
            uint32_t order = it->first;
            *(this->static_measurement_master_[order]) = it->second;
        }
        std::cout<<"观测栅格列表更新完成!"<<std::endl;
    }

    //更新栅格的坐标
    void StaticGridMap::UpdatePose(Odometry const& current_odom)
    {
        //如果是第一帧,那么只是把里程计和时间戳的信息记录下来
        if(this->is_first_ == true)
        {
            this->timestamp_ = current_odom.GetTimestamp();
            this->odometry_ = current_odom;
            this->is_first_ = false;
            return;
        }
        //计算坐标原点移动距离(默认里程计使用车身坐标系)
        Timestamp current_time = current_odom.GetTimestamp();
        Duration time_difference = current_time - this->timestamp_;
        //输入帧位置和当前帧位置差
        Point3f movement_translation = current_odom.GetPose().GetTranslation() - this->odometry_.GetPose().GetTranslation();
        //当前的偏航
        float32_t current_yaw = current_odom.GetPose().GetRotation().YPR()[0];
        //地图的偏航
        float32_t last_yaw = this ->odometry_.GetPose().GetRotation().YPR()[0];
        //偏航差
        float32_t  yaw;

         //如果发生跳变
         if((current_yaw * last_yaw) < 0)
         {
            yaw = current_yaw - last_yaw;
             if(yaw > 0)
             {
                 yaw = yaw - 2*PI;
             }
             else
             {
                 yaw = yaw + 2*PI;
             }
         }
         else
        {
        //旋转角(默认向逆时针旋转为正角)
        yaw = current_yaw - last_yaw;
        }

        //2d的位置 平移加旋转
        Pose2f movement(yaw,movement_translation.GetX(),movement.GetY());
        //创建一个map里面放StaticGridMapCell
        std::vector<std::pair<uint32_t,StaticGridMapCell>> new_cell_vector;
        for(int i = 0; i < this->grid_map_master_.size();i++)
        {
            //对于未知的栅格不处理
            if(this->grid_map_master_[i]->IsUnknown())
            {
                continue;
            }
            else
            {
                MapPointGrid old_point = grid_map_master_[i]->GetPoint();
                //如果转换以后越界了 就变为无效点
                MapPointGrid new_point = CoordinateSystemTF(movement,old_point,this->param_.width,this->param_.height,this->param_.resolution);
                uint32_t new_order = GridCoordinateToOrder(new_point,this->param_.width);
                //如果这个格无效
                if(!new_point.IsNull())
                {
                    //如果是有效点,就放到新的vector里
                    StaticGridMapCell temp = *grid_map_master_[i];
                    temp.SetPoint(new_point);
                    new_cell_vector.emplace_back(std::make_pair(new_order,temp));
                }
                //保证所有的栅格都是初始状态
                grid_map_master_[i]->Initialize(this->param_.initial_value);
            }
        }
        for(int i = 0;i<new_cell_vector.size();i++)
        {
            uint32_t index = new_cell_vector[i].first;
            *grid_map_master_[index] =  new_cell_vector[i].second;
        }
        std::cout<<"栅格地图坐标已更新!"<<std::endl;

        //更新时间戳,更新里程计信息
        this->timestamp_ = current_time;
        this->odometry_ = current_odom;
    }

    //更新地图
    void StaticGridMap::UpdateMap()
    {
        for(int i = 0; i < this->grid_map_master_.size();i++)
        {
            StaticMeasurementCell * measurement_cell = this->static_measurement_master_[i];
            this->grid_map_master_[i]->UpDateCell(measurement_cell);
            //把观测栅格列表初始化为全部未观测状态
            this->static_measurement_master_[i]->Initialize(this->param_.log_free_increment);

        }
        std::cout<<"静态地图更新完成!"<<std::endl;
    }

    //freespace和obstacle_list映射为栅格
    std::map<uint32_t,StaticMeasurementCell> StaticGridMap::InputToMeasurement(FreeSpace const& freespace,std::vector<Obstacle> const& static_obstacle)
    {
        //获取freespace的多边形点
        std::vector<Point2f> polygon_corner;
        for(int i=0; i < freespace.size();i++)
        {
            polygon_corner.emplace_back(freespace.at(i));
        }

        //将多边形处理为长方型返回角点
        std::vector<Point2f> boundary_corner = FindBoundaryPoint(polygon_corner);
        std::map<uint32_t,StaticMeasurementCell> result;

        if(boundary_corner.size() ==2)
        {
            MapPoint freespace_left_top_conor(boundary_corner[0].GetX(),boundary_corner[0].GetY());
            MapPoint freespace_right_bottom_cornor(boundary_corner[1].GetX(),boundary_corner[1].GetY());
            //计算左上角和右下角两个点
            MapPointGrid freespace_left_top = VehicleToGrid(freespace_left_top_conor,this->param_.width,
                                                                                this->param_.height,this->param_.resolution);
            MapPointGrid freespace_right_bottom = VehicleToGrid(freespace_right_bottom_cornor,this->param_.width,
                                                                                            this->param_.height,this->param_.resolution);

            //把freespace全都放在map里
            for(int i =  freespace_right_bottom.x; i <= freespace_left_top.x; i++)
            {
                for(int j = freespace_right_bottom.y; j<= freespace_left_top.y; j++)
                {
                    
                    bool_t flag = IsInteriorPoint(MapPointGrid(i,j),boundary_corner,this->param_.width,this->param_.height,this->param_.resolution);
                    //如果是freespace的内点
                    if(flag)
                    {
                        StaticMeasurementCell temp;
                        temp.point.x = i;
                        temp.point.y =j;
                        temp.value = this->param_.log_free_increment;
                        temp.obstacle_id = 0U;
                        uint32_t index = GridCoordinateToOrder(temp.point,this->param_.width);
                        result[index] = temp;
                    }
                }
            }
        }
        
        //处理障碍物列表
        for(int i = 0; i < static_obstacle.size();i++)
        {
            //拿到当前的障碍物
            Obstacle obstacle_temp = static_obstacle[i];
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
                        StaticMeasurementCell temp;
                        temp.point.x = i;
                        temp.point.y =j;
                        temp.value = this->param_.log_occ_increment;
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
} //namespace perception
}//namespace holo