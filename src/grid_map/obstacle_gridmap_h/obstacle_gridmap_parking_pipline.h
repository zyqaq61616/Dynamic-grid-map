/**
 * @file obstacle_gridmap_parking_pipline.h
 * @author zhang yuqi (zhangyuqi1@holomatic.com)
 * @brief Grid map generation process
 * @version 0.1
 * @date 2022-02-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include"holo/perception/obstacle_gridmap/merge.h"
#include <holo/utils/yaml.h>

namespace holo
{
namespace perception
{

/**
 * @brief Class for grid map generation
 * 
 */
class ObstacleGridMapParkingPipline
{
public:
    using Scalar       = holo::float32_t;
    using Bool         = bool_t;
    using Timestamp    = holo::common::Timestamp;

    using Obstacle = holo::obstacle::ObstacleGeneralT<float32_t>;
    using ObstacleGeneralPtrList = holo::obstacle::ObstaclePtrListT<holo::obstacle::ObstacleGeneralT<holo::float32_t>, 128UL>;//obstacle_list
    using ObstacleGeneralPtrListMessage       = std::shared_ptr<ObstacleGeneralPtrList>;

    using Point2f = holo::geometry::Point2f;
    using Freespace           = holo::common::AutoFreespace<Point2f>; //freespace
    using FreeSpaceMessage       = std::shared_ptr<Freespace>;

    using OdometryType = holo::common::Odometryf;//odometry
    using OdometryMessage       = std::shared_ptr<OdometryType>;

    using GridMapCell       = holo::gridmap::GridMapCell;//grid_map
    using GridMap = holo::gridmap::GridMap<GridMapCell,200,250>;

    using PublishCallback       = std::function<void(GridMap const&)>;//publish callback


    /**
     * @brief Construct a new Obstacle Grid Map Parking Pipline object
     * 
     */
    ObstacleGridMapParkingPipline();

    /**
     * @brief Destroy the Obstacle Grid Map Parking Pipline object
     * 
     */
    ~ObstacleGridMapParkingPipline();

    /**
     * @brief Set the Params object
     * 
     * @param config_file Path to configuration file
     */
    void SetParams(std::string config_file);

    /**
     * @brief Update the main function of grid map
     * 
     * @param odo_msg odometyr
     * @param free_msg freespace
     * @param obs_msg obstacle list
     * @return Bool 
     */
    Bool UpdateMap(OdometryMessage  const& odo_msg,FreeSpaceMessage const& free_msg,ObstacleGeneralPtrListMessage const& obs_msg);

    /**
     * @brief Get the Map object
     * 
     * @return GridMap Return to grid map
     */
    inline GridMap & GetMap()
    {
        return master_layer_;
    }

private:

    DynamicGridMap dynamic_layer_;//dynamci gridmap layer
    StaticGridMap static_layer_;//static gridmap layer
    GridMap master_layer_;//master gridmap layer
};
}//namespace perception
}//namesapce holo