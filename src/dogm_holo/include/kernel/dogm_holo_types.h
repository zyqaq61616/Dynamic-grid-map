/*
This document is used to define various structures
*/
#pragma once

#include<vector>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <queue>
#include <map>
#include <iostream>
#include <math.h>

#define PI 3.1415926
using namespace std;
namespace dogm_holo
{
    struct Point3d
    {
        float x = 0;
        float y = 0;
        float z = 0;
    };

    struct Timestamp
    {

    };
    template <typename T>
    struct Box3DT
    {
    using Scalar       = T;
    using Box3Type     = Box3DT<Scalar>;
    Point3d center_point_;
    Scalar     x_dim_;
    Scalar     y_dim_;
    Scalar     z_dim_;
    Scalar     rx_;  ///< rotates around x-axis, in radian
    Scalar     ry_;  ///< rotates around y-axis, in radian
    Scalar     rz_;  ///< rotates around z-axis, in radian
    };

    template <typename T>
    struct Obstacle
    {
        using Scalar       = T;
        using Point3Type = Point3d;
        using Box3Type     = Box3DT<Scalar>;
        Timestamp      timestamp_;
       int classification_;
       int  obstacle_id_;
       float obstacle_exist_score_;
       Point3Type     position;
       Box3Type        box_3d_;
    };
    /**
     * @brief Point in real coordinate system
     */
    struct Point2d
    {
        float x=0;
        float y=0;
    };
    

    /**
     * @brief Point in grid coordinate system
     */
    struct Point2d_grid
    {
        int x=0;
        int y=0;
    };
                                                                                                                                        //是否需要开发类别检查功能(防止误报)的功能
    /**
     * @brief Parameters of static grid cell
     * @param occ_ratio Occupancy value
     * @param class_index Category queue
     */
    struct StaticGridCell
    {
    float occ_ratio;
    int class_index; 
    //看情况决定是否开发类别检查功能(防止误报)
    // StaticGridCell()
    // {
    //     for(int i = 0; i < 3; i++)
    //     {
    //     class_index.push(-1);
    //     }
    // }

    // //input class index
    // void insert_class(int index)
    // {
    //     class_index.pop();
    //     class_index.push(index);
    // }
    // //get calss index
    // int get_class()
    // {
    //     vector<int> temp;
    //     for(int i = 0; i<3; i++)
    //     {
    //         int category = class_index.front();
    //         class_index.pop();
    //         class_index.push(category);
    //         temp.push_back(category);
    //     }
    //     sort(temp.begin(),temp.end());
    //     return temp[1];
    // }
    // void operator=(StaticGridCell temp)
    // {
    //     this->occ_ratio = temp.occ_ratio;
    //     this->class_index = temp.class_index;
    // }
    };

  /**
   * @brief Parameters of measurement static grid cell
   * @param index_x Number of corresponding horizontal grids
   * @param index_y Number of corresponding vertical grids 
   * @param value Measured value 

   */
    struct StaticMeasurementCell
    {
    Point2d_grid point;
    int class_index;
    float value;
    };

/**
 * @brief  Parameters of static grid map
 * @param resolution Each grid cell size [m]
 * @param width Number of horizontal grids
 * @param height Number of vertical grids
 * @param initial_value Initial occupancy value
 * @param threshold Judge whether it is occupied
 * @param log_free_increment The increment of measurement as free cell
 * @param log_occ_increment The increment of measurement as occupied cell
 * @param origin_x Horizontal ordinate of the cell where the origin of the automobile coordinate system is located
 * @param origin_y Ordinate ordinate of the cell where the origin of the automobile coordinate system is located  
 */
struct StaticMapParams
{
    float resolution;
    int width;
    int height;
    float  initial_value;
    float threshold;
    float log_free_increment;
    float log_occ_increment;
    int origin_x;
    int origin_y;
 };

/**
 * @brief Change matrix of coordinate system
 * @param x_movement Distance of vertical movement of coordinate system
    @param y_movement Distance of horizontal movement of coordinate system
    @param rotation_angle clockwise rotation angle of the coordinate system
 */
struct MovingMatrix
{
        float x_movement;
        float y_movement;
        float rotation_angle;
};
    //dynamic_grid_cell
        struct DynamicGridCell
    {
    int start_idx;
    int end_idx;
    float new_born_occ_mass;
    float pers_occ_mass;
    float free_mass;
    float occ_mass;
    float pred_occ_mass;
    float mu_A;
    float mu_UA;

    float w_A;
    float w_UA;

    float mean_x_vel;
    float mean_y_vel;
    float var_x_vel;
    float var_y_vel;
    float covar_xy_vel;
    };

 //dynamic_Measurement_cell
    struct DynamicMeasurementCell
{
    float free_mass;
    float occ_mass;
    float likelihood;
    float p_A;
};


 //Dynamic Map Params
    struct DynamicMapParams
{
        // Grid size [m]
        float size;

        // Grid cell size [m/cell]
        float resolution;

        // Number of persistent particles
        int particle_count;

        // Number of birth particles
        int new_born_particle_count;

        // Probability of persistence
        float persistence_prob;

        // Process noise position
        float stddev_process_noise_position;

        // Process noise velocity
        float stddev_process_noise_velocity;

        // Probability of birth
        float birth_prob;

        // Velocity to sample birth particles from (normal distribution) [m/s]
        float stddev_velocity;

        // Velocity to sample the initial particles from (uniform distribution) [m/s]
        float init_max_velocity;
};
//Origin position and yaw
    struct OriginPosition
    {
        float origin_x;
        float origin_y;
        float origin_yaw;    
    };
}
/* namespace dogm */