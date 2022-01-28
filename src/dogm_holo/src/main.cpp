#include "../include/main.h"


    test::test(ros::NodeHandle &n)
{
    //set static grid map params
    para.resolution = 20.0;
    para.height = 200;
    para.width = 50;
    para.threshold = 55.0;
    para.initial_value = 50.0;
    para.log_free_increment = -1.0;
    para.log_occ_increment = 2.0;
    para.origin_x = 0;
    para.origin_x = 0;
    dogm_holo::StaticGridMap sgp(para);
}


int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "dogm_holo");
    ros::NodeHandle nh;
    test tes(nh);
    ros::spin();
    return 0;
}