#ifndef WRIST_HELPERS_H
#define WRIST_HELPERS_H

#include <ros/ros.h>

template <typename T>
void getParam(ros::NodeHandle *nh, T *param, const std::string param_name)
{
    // wait for param_name on parameter server
    while (ros::ok())
    {
        if (nh->getParam(param_name, *param))
        {
            ROS_INFO_STREAM("Obtained " << param_name << ": " << *param << " from parameter server.");
            return;
        }
        else
        {
            ROS_WARN("Could not find parameter '%s' on parameter server.", param_name.c_str());
            ros::Duration(1.0).sleep();
        }
    }
}

#endif 