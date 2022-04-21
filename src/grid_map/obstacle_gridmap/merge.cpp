#include "holo/perception/obstacle_gridmap/merge.h"
namespace holo
{
namespace perception
{

bool_t MergeMap(std::vector<StaticGridMapCell*> const& static_array,std::vector<DynamicGridMapCell*> const& dynamic_array,
                                        holo::gridmap::GridMap <holo::gridmap::GridMapCell,200U,250U> &grid_map,float32_t threshold)
{
    if(static_array.size() != dynamic_array.size())
    {
        std::cout<<"静态地图和动态地图的大小不同!"<<std::endl;
        return false;
    }
    else
    {
        //按车前20m车后5m,左右各10m,分辨率为0.1m开发
        if(static_array.size() == grid_map.HEIGHT_VALUE * grid_map.WIDTH_VALUE)
        {
            for(int i = 0; i < static_array.size();i++)
            {
                StaticGridMapCell sta_cell = *static_array[i];
                DynamicGridMapCell dyn_cell = *dynamic_array[i];
                float32_t value = sta_cell.GetCurrentValue() - sta_cell.GetInitialValue();
                MapPoint real_point;
                MapPointGrid grid_point;
                grid_point = holo::perception::OrderToGridCoordinate(i,200);
                real_point = holo::perception::GridToVehicle(grid_point,200,250,0.1);
                holo::gridmap::GridMapCell cell = grid_map.at(grid_point.x,grid_point.y);
                cell.center_.SetX(real_point.x);
                cell.center_.SetY(real_point.x);
                OccStatus sta_statue;
                OccStatus dyn_statue = dyn_cell.GetOccupyStatus();
                uint8_t flag=0;
                //判断静态栅格的占用状态
                if(value > threshold)
                {
                    //表示为free
                    sta_statue = OccStatus::FREE;
                }
                else if(value < -threshold)
                {
                    //表示为occ
                    sta_statue = OccStatus::OCCUPY;
                }
                else
                {
                    //表示为unknown
                    sta_statue = OccStatus::UNKNOWN;
                }
                //融合策略
                if(sta_statue==OccStatus::FREE)
                {
                    if(dyn_statue == OccStatus::FREE)
                    {
                        //static : free,dynamic : free
                        cell.is_occupied_ = false;
                        cell.relative_velocity_.SetX(0);
                        cell.relative_velocity_.SetY(0);
                        cell.motion_status = MotionStatus::UNKNOWN;
                        cell.classification_ = Classification();
                        cell.obstacle_id_ = 0U;
                    }
                    else if(dyn_statue == OccStatus::OCCUPY)
                    {
                        //static : free,dynamic : occupy
                        cell.is_occupied_ = true;
                        cell.relative_velocity_.SetX(dyn_cell.GetVelocity().x);
                        cell.relative_velocity_.SetY(dyn_cell.GetVelocity().y);
                        cell.motion_status = dyn_cell.GetMotionStatus();
                        cell.classification_ = dyn_cell.GetClassification();
                        cell.obstacle_id_= dyn_cell.GetObstacleID();
                    }
                    else
                    {
                        //static : free,dynamic : unknown
                        cell.is_occupied_ = false;
                        cell.relative_velocity_.SetX(0);
                        cell.relative_velocity_.SetY(0);
                        cell.motion_status = holo::MotionStatus::UNKNOWN;
                        cell.classification_ = holo::common::Classification();
                        cell.obstacle_id_ = 0U;
                    }
                }
                else if(sta_statue ==   OccStatus::UNKNOWN)
                {
                    if(dyn_statue == OccStatus::FREE)
                        {
                            //static : unknown,dynamic : free
                            cell.is_occupied_ = false;
                        }
                        else
                        {
                            ////static : unknown,dynamic : occupy/unknown
                            cell.is_occupied_ = true;
                        }
                        cell.relative_velocity_.SetX(dyn_cell.GetVelocity().x);
                        cell.relative_velocity_.SetY(dyn_cell.GetVelocity().y);
                        cell.motion_status = dyn_cell.GetMotionStatus();
                        cell.classification_ = dyn_cell.GetClassification();
                        cell.obstacle_id_ = dyn_cell.GetObstacleID();
                }
                else
                {
                    //static :occupy ,dynamic : unknow/occupy/free
                    cell.is_occupied_ = true;
                    cell.relative_velocity_.SetX(0U);
                    cell.relative_velocity_.SetY(0U);
                    cell.motion_status = holo::MotionStatus::STATIONARY;
                    cell.classification_ = sta_cell.GetClassification();
                    cell.obstacle_id_= sta_cell.GetObstacleID();
                }
            }
            return true;
        }
        else
        {
            std::cout<<"栅格地图尺寸出现问题"<<std::endl;
            return false;
        }
    }
}

//寻找左上角和右下角点(验证了没问题)
std::vector<Point2f> FindBoundaryPoint(std::vector<Point2f>& point_vector)
{
    //寻找边界点左上角和右下角
    std::vector<Point2f> result;
    if(point_vector.size() <= 2)
    {
        return result;
    }
    float32_t x_max = point_vector[0].GetX();
    float32_t x_min =  point_vector[0].GetX();
    float32_t y_max =  point_vector[0].GetY();
    float32_t y_min = point_vector[0].GetY();
    for(int i = 0; i < point_vector.size(); i++)
    {
        if(x_max <  point_vector[i].GetX())
        {
            x_max = point_vector[i].GetX();
        }
        if(x_min > point_vector[i].GetX())
        {
            x_min = point_vector[i].GetX();
        }
        if(y_max <  point_vector[i].GetY())
        {
            y_max = point_vector[i].GetY();
        }
        if(y_min > point_vector[i].GetY())
        {
            y_min = point_vector[i].GetY();
        }
    }
    //保证不越界
    x_max = std::min(x_max,(MAPFRONT - 0.1F));
    x_min = std::max(x_min,MAPBACK);
    y_max = std::min(y_max,(MAPLEFT - 0.1F));
    y_min = std::max(y_min,MAPRIGHT);
    result.reserve(2);
    result[0].SetX(x_max);
    result[0].SetY(y_max);
    result[1].SetX(x_min);
    result[1].SetY(y_min);
    return result;    
}

//是不是多边形内点
bool_t IsInteriorPoint(MapPointGrid  grid_point, std::vector<Point2f> const& corner_vector,
                                        float32_t width,float32_t height,float32_t resulution)
{
    //先转换到车身坐标系下
    MapPoint temp = GridToVehicle(grid_point,width,height,resulution);
    Point2f point(temp.x,temp.y);
    int count = corner_vector.size();
	bool_t inside = false;
	for (int i = 1; i <= count; i++)
	{
		Point2f A = corner_vector[i - 1];
		Point2f B =  corner_vector[i%count];
		if ((B.GetY() <= point.GetY() && point.GetY() < A.GetY()) || (A.GetY() <= point.GetY() && point.GetY() < B.GetY()))
		{
		    float32_t t = (point.GetX() - B.GetX())*(A.GetY() - B.GetY()) - (A.GetX() - B.GetX())*(point.GetY() - B.GetY());
		    if (A.GetY() < B.GetY())
            {
			    t = -t;
            }
		    if (t < 0)
            {
		    	inside = !inside;
            }
		}
	}
    return inside;
}

//计算障碍物角点
std::vector<Point2f> ObstaclePolygon( holo::obstacle::ObstacleGeneralT<float32_t> const& obstacle)
{
    Box3f box = obstacle.GetObstacleBBox3D();
    Point2f center_point(box.GetCenter().GetX(),box.GetCenter().GetY());
    float32_t width = box.GetWidth();
    float32_t height = box.GetDepth();
    float32_t yaw = box.GetPose().GetRotation().YPR()[0];
    std::vector<Point2f> result = GetRectVertex(center_point,yaw,height,width);
    return result;
}

//矩形计算
std::vector<Point2f> GetRectVertex(Point2f center,float32_t theta,float32_t height , float32_t width)
{
    std::vector<Point2f> result;
    Point2f a;
    a.SetX(center.GetX() + height / 2 * cos(theta) - width / 2 * sin(theta));
    a.SetY(center.GetY() - height / 2 * sin(theta) - width / 2 * cos(theta));
    Point2f b;
    b.SetX(center.GetX() - height / 2 * cos(theta) - width / 2 * sin(theta));
    b.SetY(center.GetY() + height / 2 * sin(theta) - width / 2 * cos(theta));
    Point2f c;
    c.SetX(center.GetX() - height / 2 * cos(theta) + width / 2 * sin(theta));
    c.SetY(center.GetY() + height / 2 * sin(theta) + width / 2 * cos(theta));
    Point2f d;
    d.SetX(center.GetX() + height / 2 * cos(theta) + width / 2 * sin(theta));
    d.SetY(center.GetY() - height / 2 * sin(theta) + width / 2 * cos(theta));
    result.emplace_back(a);
    result.emplace_back(b);
    result.emplace_back(c);
    result.emplace_back(d);
    return result;
}

//障碍物列表分类
std::vector<std::vector< holo::obstacle::ObstacleGeneralT<float32_t>>> ObstacleClassification(
    holo::obstacle::ObstaclePtrListT<holo::obstacle::ObstacleGeneralT<holo::float32_t>, 128UL> obstacle_list)
{
    std::vector<std::vector< holo::obstacle::ObstacleGeneralT<float32_t>>> result;
    std::vector< holo::obstacle::ObstacleGeneralT<float32_t>> static_obstacle;
    std::vector< holo::obstacle::ObstacleGeneralT<float32_t>> dynamic_obstacle;
    //创建一个array接受障碍物列表的智能指针
    std::array<std::shared_ptr< holo::obstacle::ObstacleGeneralT<float32_t>>, 128UL> obstacle = obstacle_list.GetObstacleList();
    //遍历整个列表 找出属性为stationary 的障碍物放到static_vector里面
    for(auto it = obstacle.begin();it != obstacle.end();it++)
    {
         holo::obstacle::ObstacleGeneralT<float32_t> temp = *it->get();
        if(temp.GetMotionStatus() == MotionStatus::STATIONARY)
        {
            static_obstacle.push_back(temp);
        }
        else
        {
            dynamic_obstacle.push_back(temp);
        }
    }
    //输入静态和动态障碍物对象
    result.emplace_back(static_obstacle);
    result.emplace_back(dynamic_obstacle);
    return result;
}
} //namespace perception
}//namepsace holo
