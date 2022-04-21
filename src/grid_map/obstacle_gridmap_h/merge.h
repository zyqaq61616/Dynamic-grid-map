#include"holo/perception/obstacle_gridmap/dynamic/dynamic_grid_map.h"
#include"holo/perception/obstacle_gridmap/static/static_grid_map.h"
#include "holo/perception/obstacle_gridmap/gridmap.h"
namespace holo
{
namespace perception
{

/**
 * @brief Integration of static layer and dynamic layer
 * 
 * @param static_array Static map array
 * @param dynamic_array Dynamic map array
 * @param grid_map output gridmap
 * @param threshold Threshold to determine whether it is occupied or idle
 * @return bool_t Judge map size
 */
bool_t MergeMap(std::vector<StaticGridMapCell*> const& static_array,std::vector<DynamicGridMapCell*> const& dynamic_array,
                                        holo::gridmap::GridMap <holo::gridmap::GridMapCell,200U,250U> & grid_map,float32_t threshold);

/**
 * @brief Find the corner of the box fitted by polygon
 * 
 * @param point_vector Polygon corner
 * @return std::vector<Point2f> Upper left corner and lower right corner
 */
std::vector<Point2f> FindBoundaryPoint(std::vector<Point2f>& point_vector);

/**
 * @brief Find the corner of the box fitted by polygon
 * 
 * @param grid_point grid point
 * @param corner_vector Polygon corner
 * @param width width of grid map
 * @param height height of grid map
 * @param resulution resulution of grid map
 * @return bool_t 
 */
bool_t IsInteriorPoint(MapPointGrid  grid_point, std::vector<Point2f> const& corner_vector,float32_t width,float32_t height,float32_t resulution);

/**
 * @brief Calculate the grounding point of the box
 * 
 * @param obstacle obstacle
 * @return std::vector<Point2f> Polygon corner
 */
std::vector<Point2f> ObstaclePolygon( holo::obstacle::ObstacleGeneralT<float32_t> const& obstacle);

/**
 * @brief The obstacle list is divided into dynamic and static obstacles
 * 
 * @param obstacle_list obstacle_list 
 * @return std::vector<std::vector< holo::obstacle::ObstacleGeneralT<float32_t>>>  obstacle vector [0] static [1]dynamic
 */
std::vector<std::vector< holo::obstacle::ObstacleGeneralT<float32_t>>> ObstacleClassification(
    holo::obstacle::ObstaclePtrListT<holo::obstacle::ObstacleGeneralT<holo::float32_t>, 128UL> obstacle_list);
    
} //namespace perception
}//namepsace holo

