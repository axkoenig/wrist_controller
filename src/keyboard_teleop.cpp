#include <math.h>
#include <iostream>
#include <map>
#include <termios.h>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "helpers.hpp"

std::string node_name = "wrist_teleop_node";
std::string source_frame = "world";

float trans_scaling = 0.01;
float rot_scaling = 0.2;

// format: {x, y, z, r, p ,y} in "reflex" frame
std::array<float, 6> init_pose = {0, 0, 0.1, -M_PI / 2, 0, 0};

// keys for wrist teleoperation (note this is for my german keyboard)
std::map<char, std::vector<float>> wrist_bindings{

    // format: {x, y, z, r, p ,y}
    {'u', {1, 0, 0, 0, 0, 0}},
    {'i', {0, 1, 0, 0, 0, 0}},
    {'o', {0, 0, 1, 0, 0, 0}},
    {'j', {0, 0, 0, 1, 0, 0}},
    {'k', {0, 0, 0, 0, 1, 0}},
    {'l', {0, 0, 0, 0, 0, 1}},
    {'U', {-1, 0, 0, 0, 0, 0}},
    {'I', {0, -1, 0, 0, 0, 0}},
    {'O', {0, 0, -1, 0, 0, 0}},
    {'J', {0, 0, 0, -1, 0, 0}},
    {'K', {0, 0, 0, 0, -1, 0}},
    {'L', {0, 0, 0, 0, 0, -1}},
};

// BEGIN CODE FROM https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp

// for non-blocking keyboard inputs
int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // get the current character
    ch = getchar();

    // reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

// END CODE FROM https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp

tf2::Transform calcTransformFromEuler(std::array<float, 6> pose)
{
    tf2::Vector3 t = {pose[0], pose[1], pose[2]};
    tf2::Quaternion q;
    q.setRPY(pose[3], pose[4], pose[5]);
    tf2::Transform transform(q, t);

    return transform;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Rate rate(10);
    ROS_INFO("Launched %s.", node_name.c_str());

    std::string target_frame;
    getParam(&nh, &target_frame, "robot_name");

    // init transform broadcaster
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped ts;
    tf2::Transform transform;

    // populate initial wrist transform
    std::array<float, 6> cur_pose = init_pose; 
    transform = calcTransformFromEuler(init_pose);
    ts.header.frame_id = source_frame;
    ts.child_frame_id = target_frame;
    ts.transform = tf2::toMsg(transform);

    // TODO find another, more elegant solution for this
    // wait before publishing first transform to fix warning from wrist_controller_node
    // ""reflex" passed to lookupTransform argument target_frame does not exist."
    ros::Duration(1).sleep();

    // send initial wrist transform
    ts.header.stamp = ros::Time::now();
    br.sendTransform(ts);

    ROS_INFO("Listening to keyboard input...");
    char key(' ');

    while (ros::ok())
    {
        key = getch();

        if (wrist_bindings.count(key) == 1)
        {
            // x, y, z
            cur_pose[0] += trans_scaling * wrist_bindings[key][0];
            cur_pose[1] += trans_scaling * wrist_bindings[key][1];
            cur_pose[2] += trans_scaling * wrist_bindings[key][2];

            // r, p, y
            cur_pose[3] += trans_scaling * wrist_bindings[key][3];
            cur_pose[4] += trans_scaling * wrist_bindings[key][4];
            cur_pose[5] += trans_scaling * wrist_bindings[key][5];

            ROS_INFO("Wrist transform updated.");
        }
        else if (key == 'm')
        {
            // reset wrist to init frame
            cur_pose = init_pose;
            ROS_INFO("Wrist transform reset.");
        }

        transform = calcTransformFromEuler(cur_pose);
        ts.header.stamp = ros::Time::now();
        ts.transform = tf2::toMsg(transform);
        br.sendTransform(ts);

        // EXIT ------------------------------------------
        if (key == '\x03')
        {
            ROS_INFO("Pressed Ctrl-C. Shutting down...");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
}