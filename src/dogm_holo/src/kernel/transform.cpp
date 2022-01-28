#include "kernel/transform.h"

namespace dogm_holo
{

    vector<Point2d_grid> find_the_middle_point(Point2d_grid top_left_corner,Point2d_grid lower_right_corner, int width,int height)
    {
        //Judge whether the boundary is crossed
        if(top_left_corner.x >  height)
        {
            top_left_corner.x = height;
        }
        if(top_left_corner.y >  width)
        {
            top_left_corner.y = width;
        }
        if(lower_right_corner.x < 1)
        {
            lower_right_corner.x = 1;
        }
        if(lower_right_corner.y < 1)
        {
             lower_right_corner.y = 1;
        }
        vector<Point2d_grid> result;
        for(int i = lower_right_corner.x; i <=top_left_corner.x; i++)
        {
            for(int j = lower_right_corner.y; j<= top_left_corner.y; j++)
            {
                Point2d_grid temp;
                temp.x = i;
                temp.y = j;
                 result.emplace_back(temp);
            }
        }
        return result;
    }


/**
     * @brief Map freespace to a 2D grid map
     * @return vector<Point2d_grid>
     */
    vector<Point2d_grid> freespace2grid(vector<Point2d>& point_vector , StaticMapParams& param)
    {
        //Find boundary points
        if(point_vector.size() <= 2) return vector<Point2d_grid> ();
        float x_max = point_vector[0].x;
        float x_min =  point_vector[0].x;
        float y_max =  point_vector[0].y;
        float y_min = point_vector[0].y;
        for(int i = 0; i < point_vector.size(); i++)
        {
            if(x_max <  point_vector[i].x)
            {
                x_max = point_vector[i].x;
            }
            if(x_min > point_vector[i].x)
            {
                x_min = point_vector[i].x;
            }
            if(y_max <  point_vector[i].y)
            {
                y_max = point_vector[i].y;
            }
            if(y_min > point_vector[i].y)
            {
                y_min = point_vector[i].y;
            }
        }
        //Calculate the vertex of the freespace
        //top_left_corner
        Point2d top_left_corner;
        top_left_corner.x = x_max;
        top_left_corner.y = y_max;
        Point2d_grid top_left_corner_grid = real_coordinate2grid_coordinate(top_left_corner, param.resolution,param.width,param.height);

        //lower_right_corner
        Point2d lower_right_corner;
        lower_right_corner.x = x_max;
        lower_right_corner.y = y_max;
        Point2d_grid lower_right_corner_grid = real_coordinate2grid_coordinate(lower_right_corner,param.resolution,param.width,param.height);

        //vector<int> top_left_corner;
        vector<Point2d_grid> points_to_be_tested = find_the_middle_point(top_left_corner_grid,lower_right_corner_grid,param.width,param.height);
        vector<Point2d_grid> result;
        for(int j = 0; j < points_to_be_tested.size(); j++)
        {
            Point2d temp = grid_coordinate2real_coordinate(points_to_be_tested[j],param.resolution,param.width,param.height);
            if(is_interior_point(temp,point_vector))
            {
                
                result.push_back(points_to_be_tested[j]);
            }
        }
        return result;

    }
    
    /**
     * @brief 
     * 
     * @param obstacle 
     * @param param 
     * @return vector<Point2d_grid> 
     */
    vector<Point2d> turn_3Dbounxingbox22Dgrid(Obstacle<float>& obstacle,StaticMapParams& param)
    {
        Point3d centre_3d = obstacle.box_3d_.center_point_;
        Point2d  centre_2d;
        centre_2d.x =  centre_3d.x;
        centre_2d.y =  centre_3d.y;
    }
                                                                                                                                                            // boundingbox to grid 还未开发
        /**
     * @brief Calculate the occupied grid according to the four corners of box on the grid map
     * @param point_vector  Corner array
     * @param class_index Class_index
     * @param param Static map parameter
     * @return StaticMeasurementCell* 
     */
     vector<StaticMeasurementCell> boundingbox2grid(vector<Point2d>& point_vector ,int class_index, StaticMapParams& param)
    {
        //Find boundary points
        if(point_vector.size() <= 2) return vector<StaticMeasurementCell>();
        float x_max = point_vector[0].x;
        float x_min =  point_vector[0].x;
        float y_max =  point_vector[0].y;
        float y_min = point_vector[0].y;
        for(int i = 0; i < point_vector.size(); i++)
        {
            if(x_max <  point_vector[i].x)
            {
                x_max = point_vector[i].x;
            }
            if(x_min > point_vector[i].x)
            {
                x_min = point_vector[i].x;
            }
            if(y_max <  point_vector[i].y)
            {
                y_max = point_vector[i].y;
            }
            if(y_min > point_vector[i].y)
            {
                y_min = point_vector[i].y;
            }
        }
        //Calculate the vertex of the freespace
        //top_left_corner
        Point2d top_left_corner;
        top_left_corner.x = x_max;
        top_left_corner.y = y_max;
        Point2d_grid top_left_corner_grid = real_coordinate2grid_coordinate(top_left_corner, param.resolution,param.width,param.height);

        //lower_right_corner
        Point2d lower_right_corner;
        lower_right_corner.x = x_max;
        lower_right_corner.y = y_max;
        Point2d_grid lower_right_corner_grid = real_coordinate2grid_coordinate(lower_right_corner,param.resolution,param.width,param.height);

        //vector<int> top_left_corner;
        vector<Point2d_grid> points_to_be_tested = find_the_middle_point(top_left_corner_grid,lower_right_corner_grid,param.width,param.height);
        vector<StaticMeasurementCell> result;
        for(int j = 0; j < points_to_be_tested.size(); j++)
        {
            Point2d temp = grid_coordinate2real_coordinate(points_to_be_tested[j],param.resolution,param.width,param.height);
            if(is_interior_point(temp,point_vector))
            {
                StaticMeasurementCell result_cell;
                result_cell.point = points_to_be_tested[j];
                result_cell.class_index = class_index;
                result_cell.value = param.log_occ_increment;
                result.push_back(result_cell);
            }
        }
        return result;

    }

                                                                                                                                                                //freespace和boundingbox的融合之间还需要开发 主要是内存的问题
        /**
     * @brief Merge freespace and boundingbox
     * @return StaticMeasurementCell* 
     */
     map<int, StaticMeasurementCell> merge_cell_array(vector<Point2d_grid>& freespace_grid,vector<StaticMeasurementCell>& boundingbox_grid,StaticMapParams & const param)
    {
        map<int,StaticMeasurementCell> measurement_map;
        //Put freespace cell array into a map
        for(int i =0; i< freespace_grid.size(); i++)
        {
            StaticMeasurementCell freespace_cell;
            freespace_cell.point =  freespace_grid[i];
            freespace_cell.value = param.log_free_increment;
            int index_freespace = grid_coordinate2sequence_number(freespace_cell.point,param.width);
            measurement_map.insert(make_pair(index_freespace,freespace_cell));
        }

        //update occupied cell array
        for(int j = 0;j<boundingbox_grid.size(); j++)
        {
            int index_boundingbox = grid_coordinate2sequence_number(boundingbox_grid[j].point,param.width);
            measurement_map[index_boundingbox] = boundingbox_grid[j];
        }
        return measurement_map;

    }

                                                                                                                                                                    //通过Odometry计算旋转和平移向量还没开发
        /**
     * @brief Calculate the rotation and translation matrix from the odometer information
     * @return  MovingMatrix 
     */
    MovingMatrix calculate_movement_matrix(nav_msgs::Odometry& current_data)
    {

    }

    /**
     * @brief Convert grid coordinate system to array sequence number
     * @param point Grid coordinate system point
     * @param width Horizontal grid number of map
     * @return int  Sequence number in array

     */
    int  grid_coordinate2sequence_number(Point2d_grid point,int width)
    {
        int result = (point.x - 1) * width+point.y - 1; 
        return result ; 
    }

    /**
     * @brief Convert grid array sequence number to grid coordinate system
     * @param index Sequence number in array
     * @param width Horizontal grid number of map
     * @return vector<int> : [0]:Grid coordinate system line number ; [1] : Grid coordinate system column number
     */
    Point2d_grid sequence_number2grid_coordinate(int index,int width)
    {
            Point2d_grid result;
            index ++;
            int remainder = index % width;
            if(remainder ==0)
            {
                result.x = index /width;
                result.y = width;
            }
            else
            {
                result.x = index /width +1;
                result.y = remainder;
            }
            return result;
    }

    /**
     * @brief Grid map coordinate system to real coordinate system
     * @param point Grid coordinate system point
     * @param resolution Map resolution
     * @param width Number of horizontal grids
     * @param height Number of vertical grids
     * @return vector<float> [0]: Real coordinate system system line number ; [1] :  Real coordinate system column number
     */
    Point2d grid_coordinate2real_coordinate(Point2d_grid point,float resolution,int width,int height)
    {
        Point2d result;
        float temp_x = (point.x - 1) * resolution + resolution / 2;
        float temp_y = (point.y - 1) * resolution + resolution / 2;
        result.x = (temp_x - (height * resolution) / 2);
        result.y = (temp_y - (width * resolution) / 2);
        return result;
    }   

    /**
     * @brief Real coordinate system to grid map coordinate
     * @param point Real coordinate system point
     * @param resolution Map resolution
     * @param width Number of horizontal grids
     * @param height Number of vertical grids
     * @return vector<int> [0]:Grid coordinate system line number ; [1] : Grid coordinate system column number
     */
    Point2d_grid real_coordinate2grid_coordinate(Point2d point,float resolution,int width,int height)
    {
        Point2d_grid result;
        float temp_x = point.x +  (height * resolution) / 2;
        float temp_y = point.y + (width * resolution) / 2;
        //Each grid does not include the upper and left boundaries
        result.x = (int(temp_x / resolution)+1);
        result.y = (int(temp_y / resolution)+1);
        return result;
    }

    /**
     * @brief Grid coordinate system conversion
     * @param primal  Primal grid coordinate system point
     * @param moving_vector Translation rotation vector in real coordinate system
     * @param resolution Map resolution
     * @param width Number of horizontal grids
     * @param height Number of vertical grids
     * @return Point2d_grid [x]Converted grid coordinate system line number ; [y] : Converted grid coordinate system column number
     */
    Point2d_grid grid_coordinate_system_conversion(Point2d_grid primal, MovingMatrix moving_vector, float resolution, int width, int height)
    {

        //First convert the grid coordinate system to the real coordinate system
        Point2d temp = grid_coordinate2real_coordinate(primal, resolution, width,height);
        float real_primal_x = temp.x;
        float real_primal_y = temp.y;

        // Coordinate system conversion
        Point2d real_point;
        real_point.x = (real_primal_x - moving_vector.x_movement) * cos(moving_vector.rotation_angle) - (real_primal_y - moving_vector.y_movement) * sin(moving_vector.rotation_angle);
         real_point.y = (real_primal_y - moving_vector.y_movement) * cos(moving_vector.rotation_angle) + (real_primal_x - moving_vector.x_movement) * sin(moving_vector.rotation_angle);

        //Convert to grid coordinates
        Point2d_grid result = real_coordinate2grid_coordinate(real_point, resolution, width,height);
        return result;
    }

    /**
     * @brief Update the grid map array sequence number according to the change of coordinate system
     * @param index Sequence number in the original array
     * @param moving_vector Translation rotation vector in real coordinate system
     * @param resolution Map resolution
     * @param width Number of horizontal grids
     * @param height Number of vertical grids
     * @return int Subscript in the updated grid map array , Cross border return - 1
     */
    int update_grid_coordinates(int index,MovingMatrix moving_vector, float resolution, int width, int height)
    {
            //Calculate primal coordinates in grid coordinate system
            index ++;
            int primal_grid_x;
            int primal_grid_y;
            int remainder = index % width;
            if(remainder == 0)
            {
                primal_grid_x = index /width;
                primal_grid_y = width;
            }
            else
            {
                primal_grid_x = index /width +1;
                primal_grid_y = remainder;
            }

            //Calculate primal coordinates in real coordinate system
            float primal_real_x = (primal_grid_x - 1) * resolution + resolution / 2;
            float primal_real_y = (primal_grid_y - 1) * resolution + resolution / 2;
            float after_real_x = (primal_real_x - moving_vector.x_movement) * cos(moving_vector.rotation_angle) - (primal_real_y - moving_vector.y_movement) * sin(moving_vector.rotation_angle);
            float after_real_y = (primal_real_y - moving_vector.y_movement) * cos(moving_vector.rotation_angle) + (primal_real_x - moving_vector.x_movement) * sin(moving_vector.rotation_angle);

            //Judge whether the boundary is crossed
            if((0 <= after_real_x < height * resolution) && (0 <= after_real_y < width * resolution))
            {
                float after_grid_x =after_real_x+  (height * resolution) / 2;
                float after_grid_y =after_real_y + (width * resolution) / 2;
                int result = (after_grid_x - 1) * width + after_real_y - 1; 
                return result;
            }
            else
            {
                return -1;
            }
    }

    /**
     * @brief Judge whether a point is an inner point of a polygon
     * @param point Points to be tested
     * @param corner_vector Polygon corner
     * @return true The point to be detected is the inner point of the polygon
     * @return false  The point to be detected is not the inner point of the polygon
     */
    bool is_interior_point(Point2d & const point, vector<Point2d>& const corner_vector )
    {
        int count = corner_vector.size();
	    if (count < 2)
        {
		return false;
        }
	    bool inside = false;
	    for (int i = 1; i <= count; ++i)
	    {
		    Point2d& A = corner_vector[i - 1];
		    Point2d& B =  corner_vector[i%count];
		    if ((B.y <= point.y && point.y < A.y) || (A.y <= point.y && point.y < B.y))
		    {
			    float t = (point.x - B.x)*(A.y - B.y) - (A.x - B.x)*(point.y - B.y);
			    if (A.y < B.y)
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
 
} // namespace dogm_holo
