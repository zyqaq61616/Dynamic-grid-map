#include "kernel/static_layer/static_grid_map_parameters.h"
#include "kernel/transform.h"


namespace dogm_holo
{

                                                                                                                                                                                        //还有一个浅拷贝的问题
    /**
     * @brief 
     * Construct a new Static Grid Map:: Static Grid Map object
     * @param params Static grid map parameters
     */
        StaticGridMap::StaticGridMap(StaticMapParams & const params)
    {

        //set static grid map parameter
        this->params = params; //这是浅拷贝
        this->is_initialized = false;
        this->origin_position.origin_x = 0;
        this->origin_position.origin_y = 0;
        this->origin_position.origin_yaw = 0;
        //Initialize map state
    }

    /**
     * @brief 
     * Destroy the Static Grid Map:: Static Grid Map object
     */
        StaticGridMap::~StaticGridMap()
    {
        for(auto it = this->master_grid_cell.begin(); it != this->master_grid_cell.end(); it++)
        {
            StaticGridCell* temp = *it;
            delete temp;
            temp = nullptr;
        }
    }

                                                                                                                                                                                        //这块还需要用里程计初始化位置 和new空间的问题
    /**
     * @brief Initialize map with first frame data.First frame input is required to initialize
     * @param initial_measurement First frame measurement data
     * @param initial_pose First frame odometer data
     */
    void StaticGridMap::initialize_map(vector<StaticMeasurementCell*>& const initial_measurement, nav_msgs::Odometry& const initial_pose)
    {
        for(int i = 0; i < this->params.height*this->params.width ; i++)
        {
            //这块有一个关于指针和new空间的问题
            StaticGridCell * temp = new  StaticGridCell;
            temp->occ_ratio = this->params.initial_value;
            temp->class_index = -1;
            this->master_grid_cell.emplace_back(temp);
        }
        //Traversal measurement array
        for(int i =  0; i < initial_measurement.size(); i++)
        {
            int index = (initial_measurement[i]->point.x - 1) * this->params.width + initial_measurement[i]->point.y;
            this->master_grid_cell[index]->occ_ratio + initial_measurement[i]->value;
            this->master_grid_cell[index]->class_index = initial_measurement[i]->class_index;
        }
        this->origin_position.origin_x = 0;
        this->origin_position.origin_y = 0;
        this->origin_position.origin_yaw = 0;
        this->is_initialized = true;

    }
                                                                                                                                                                                            //需要根据Odomtrey计算平移和旋转向量
    /**
     * @brief Update the position and yaw of the map origin in the world coordinate system
     * @param current_position Current odometer information
     * @return MovingMatrix
     */
        MovingMatrix StaticGridMap::update_origin(nav_msgs::Odometry & const current_position)
    {

    }

    /**
     * @brief Update the subscript index of the grid
     * @param moving_vector  Translation vector of coordinate center
     */
    void StaticGridMap::update_subscript_index(MovingMatrix moving_vector)
    {
        //Create a temporary map
        vector<pair<int,StaticGridCell>> temp_map;
        for(int i = 0; i < this->master_grid_cell.size(); i++)
        {

            if(this->master_grid_cell[i]->occ_ratio ==this->params.initial_value) continue;
            StaticGridCell temp_cell = *master_grid_cell[i];
            temp_map.emplace_back(make_pair(i,temp_cell));
            this->master_grid_cell[i]->class_index = -1;
            this->master_grid_cell[i]->occ_ratio = this->params.initial_value;
        }

        //Traverse all points of the temporary map
        for(int i=0; i <= temp_map.size(); i++)
        {
            //Subscript in new map
            int new_index = update_grid_coordinates(temp_map[i].first, moving_vector, this->params.resolution, this->params.width, this->params.height);
            if(new_index == -1) 
            {
                continue;
            }

            //If a grid has been occupied
            if(this->master_grid_cell[new_index]->occ_ratio != this->params.initial_value)
            {
                if(this->master_grid_cell[new_index]->occ_ratio > temp_map[i].second.occ_ratio) 
                {
                    continue;
                }
                else
                {
                    this->master_grid_cell[new_index]->occ_ratio = temp_map[i].second.occ_ratio;
                    this->master_grid_cell[new_index]->class_index = temp_map[i].second.class_index;
                }
            }
            else
            {
            this->master_grid_cell[new_index]->occ_ratio = temp_map[i].second.occ_ratio;
            this->master_grid_cell[new_index]->class_index = temp_map[i].second.class_index;
            }

        }
    }

    /**
     * @brief Get the measurement array object
     * 
     */
    void StaticGridMap::get_measurement_array(vector<Obstacle<float>>& const obstacle, vector<Point2d>& const freespace )
    {
            vector<Point2d_grid> freespace_index = freespace2grid(freespace,this->params);
            int obstacle_number = obstacle.size();
            vector<StaticMeasurementCell> obstacle_cell;
            for(int i = 0; i < obstacle.size();i++)
            {
                vector<Point2d> obstacle_conor = turn_3Dbounxingbox22Dgrid(obstacle[i],this->params);
                vector<StaticMeasurementCell> temp_obstacle_cell = boundingbox2grid(obstacle_conor,obstacle[i].classification_,this->params);
                obstacle_cell.insert(obstacle_cell.end(),temp_obstacle_cell.begin(),temp_obstacle_cell.end());
            }

            map<int,StaticMeasurementCell> measurement_grid_cell = merge_cell_array(freespace_index,obstacle_cell,this->params);
                                                                                                                                                                                                            //这块赋值怎么办?
            //for(int i=0 ; i<)
    }
                                                                                                                                                                                                            //需要生成一个map存放测量值
    /**
     * @brief Update grid map based on measurements
     * @param current_measurement Measured value of the current frame
     * @param moving_vector Vehicle translation and rotation vectors
     */
    void StaticGridMap::update_occupancy(map<int,StaticMeasurementCell*>& const current_measurement)
    {
        //Update the occupancy of the main map
        for(int i =  0; i < current_measurement.size(); i++)
        {
            Point2d_grid current_measurement_point;
            current_measurement_point.x = current_measurement[i]->point.x;
            current_measurement_point.y = current_measurement[i]->point.y;
            int index = grid_coordinate2sequence_number(current_measurement_point ,this->params.width);
            this->master_grid_cell[index]->occ_ratio + current_measurement[i]->value;
            this->master_grid_cell[index]->class_index + current_measurement[i]->class_index;
        }
    }
    
}
/* namespace dogm_holo */