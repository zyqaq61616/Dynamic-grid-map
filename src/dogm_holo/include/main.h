#include <iostream>
#include <sstream>
//ros相关
#include <ros/ros.h>
#include "kernel/static_layer/static_grid_map_parameters.h"
using namespace std;
using namespace ros;
class test
{
public:
    ros::Subscriber sub;
    ros::Publisher pub_object;
    dogm_holo::StaticMapParams para;
public:
    test(ros::NodeHandle &n);
    ~test();
    void creat_map_cb();
};