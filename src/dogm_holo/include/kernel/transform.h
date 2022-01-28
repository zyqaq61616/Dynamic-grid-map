#include"dogm_holo_types.h"

#include <opencv2/opencv.hpp>

using namespace std;

namespace dogm_holo
{
    
    /**
     * @brief Map freesapce to a 2D grid map
     * @return StaticMeasurementCell* 
     */
    vector<Point2d_grid> freespace2grid(vector<Point2d>& point_vector , StaticMapParams& param);

    vector<Point2d> turn_3Dbounxingbox22Dgrid(Obstacle<float>& obstacle,StaticMapParams& param);

        /**
     * @brief Map boundingbox to a 2D grid map
     * @return StaticMeasurementCell* 
     */
        /**
     * @brief Calculate the occupied grid according to the four corners of box on the grid map
     * @param point_vector  Corner array
     * @param class_index Class_index
     * @param param Static map parameter
     * @return StaticMeasurementCell* 
     */
     vector<StaticMeasurementCell> boundingbox2grid(vector<Point2d>& point_vector ,int class_index, StaticMapParams& param);

    /**
     * @brief Judge whether a point is an inner point of a polygon
     * @param point Points to be tested
     * @param corner_vector Polygon corner
     * @return true The point to be detected is the inner point of the polygon
     * @return false  The point to be detected is not the inner point of the polygon
     */
    bool is_interior_point(Point2d & const point, vector<Point2d>& const corner_list );

        /**
     * @brief Merge freespace and boundingbox
     * @return map<int,StaticMeasurementCell>& 
     */
    map<int,StaticMeasurementCell> merge_cell_array(vector<Point2d_grid>& freespace_grid,vector<StaticMeasurementCell>& boundingbox_grid,StaticMapParams & const param);

        /**
     * @brief Calculate the rotation and translation matrix from the odometer information
     * @return  MovingMatrix 
     */
    MovingMatrix calculate_movement_matrix(nav_msgs::Odometry& current_data);

    /**
     * @brief Convert grid coordinate system to array sequence number
     * @param point Grid coordinate system point
     * @param width Number of horizontal grids
     * @return int Sequence number in array

     */
    int  grid_coordinate2sequence_number(Point2d_grid point, int width);

    /**
     * @brief Convert grid array sequence number to grid coordinate system
     * @param index Sequence number in array
     * @param width Number of horizontal grids
     * @return Point2d_grid [x]:Grid coordinate system line number ; [y] : Grid coordinate system column number
     */
    Point2d_grid sequence_number2grid_coordinate(int index,int width); 

    /**
     * @brief Grid map coordinate system to real coordinate system
     * @param point Grid coordinate system point
     * @param resolution Map resolution
     * @param width Number of horizontal grids
     * @param height Number of vertical grids
     * @return Point2d [x]: Real coordinate system system line number ; [y] :  Real coordinate system column number
     */
    Point2d grid_coordinate2real_coordinate(Point2d_grid point, float resolution, int width, int height);

    /**
     * @brief Real coordinate system to grid map coordinate
     * @param point Real coordinate system point
     * @param width Number of horizontal grids
     * @param height Number of vertical grids
     * @param resolution Map resolution
     * @return Point2d_grid [x]:Grid coordinate system line number ; [y] : Grid coordinate system column number
     */
    Point2d_grid real_coordinate2grid_coordinate(Point2d real_point,float resolution, int width, int height);

    /**
     * @brief Grid coordinate system conversion
     * @param primal Primal grid coordinate system point
     * @param moving_vector Translation rotation vector in real coordinate system
     * @param resolution Map resolution
     * @param width Number of horizontal grids
     * @param height Number of vertical grids
     * @return Point2d_grid [x]Converted grid coordinate system line number ; [y] : Converted grid coordinate system column number
     */
    Point2d_grid Grid_coordinate_system_conversion(Point2d_grid primal ,MovingMatrix moving_vector,float resolution);

    /**
     * @brief Update the grid map array sequence number according to the change of coordinate system
     * @param index Sequence number in the original array
     * @param moving_vector Translation rotation vector in real coordinate system
     * @param resolution Map resolution
     * @param width Number of horizontal grids
     * @param height Number of vertical grids
     * @return int Subscript in the updated grid map array
     */
    int update_grid_coordinates(int index,MovingMatrix moving_vector,float resolution, int width, int height);

}