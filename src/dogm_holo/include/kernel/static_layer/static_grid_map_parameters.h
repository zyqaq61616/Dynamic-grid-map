#include "kernel/dogm_holo_types.h"

#include <eigen3/Eigen/Core>

using namespace std;
namespace dogm_holo
{
class StaticGridMap
{
    public:

    /**
     * @brief 
     * Construct a new Static Grid Map:: Static Grid Map object
     * @param params Static grid map parameters
     */
    StaticGridMap(StaticMapParams & const params);

    /**
     * @brief 
     * Destroy the Static Grid Map:: Static Grid Map object
     */
    ~StaticGridMap();

    /**
     * @brief Initialize map with first frame data.First frame input is required to initialize
     * @param initial_measurement First frame measurement data
     * @param initial_pose First frame odometer data
     */
    void initialize_map(vector<StaticMeasurementCell*>& const initial_measurement, nav_msgs::Odometry& const initial_pose);

    /**
     * @brief Update grid map based on measurements
     * @param current_measurement Measured value of the current frame
     * @param moving_vector Vehicle translation and rotation vectors
     */
    void update_occupancy(map<int,StaticMeasurementCell*>& const current_measurement);

    /**
     * @brief Update the subscript index of the grid
     * @param moving_vector  Translation vector of coordinate center
     */
    void update_subscript_index(MovingMatrix moving_vector);

    /**
     * @brief Update the position and yaw of the map origin in the world coordinate system
     * @param current_position Current odometer information
     * @return MovingMatrix
     */
    MovingMatrix update_origin(nav_msgs::Odometry & const current_position);

    void get_measurement_array(vector<Obstacle<float>>& const obstacle, vector<Point2d>& const freespace);


public:

    //Static Grid Cell array
    vector<StaticGridCell*> master_grid_cell;

    //Measurement Cell array
    vector<StaticMeasurementCell*> measurement_grid_cell;

    //Origin position and yaw
    OriginPosition origin_position;

    //Storing static raster map parameters
    StaticMapParams params;

    //Is it initialized
    bool is_initialized;
};

}
/* namespace dogm_holo */