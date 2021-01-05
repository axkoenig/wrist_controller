#include <string.h>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>

using namespace std;

const string NODE_NAME = "wrist_control_node";
const string NS = "gazebo/";
const string source_frame = "world";

class WristAxisController
{
private:
    ros::Publisher pub_;
    std_msgs::Float64 msg_;

public:
    WristAxisController(ros::NodeHandle *nh, string axis)
    {
        pub_ = nh->advertise<std_msgs::Float64>(NS + axis + "_position_controller/command", 1);
    }

    void sendCommand(float command)
    {
        msg_.data = command;
        pub_.publish(msg_);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;
    ROS_INFO("Launched %s.", NODE_NAME.c_str());

    ros::Rate rate(1000);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped ts;

    WristAxisController px_controller = WristAxisController(&nh, "px");
    WristAxisController py_controller = WristAxisController(&nh, "py");
    WristAxisController pz_controller = WristAxisController(&nh, "pz");
    WristAxisController rr_controller = WristAxisController(&nh, "rr");
    WristAxisController rp_controller = WristAxisController(&nh, "rp");
    WristAxisController ry_controller = WristAxisController(&nh, "ry");

    string target_frame;
    string desired_param = "robot_name";

    // wait for robot_name on parameter server
    while (ros::ok())
    {
        if (nh.hasParam(desired_param))
        {
            nh.getParam(desired_param, target_frame);
            ROS_INFO("Obtained %s: '%s' from parameter server.", desired_param.c_str(), target_frame.c_str());
            break;
        }
        else
        {
            ROS_WARN("Could not find parameter '%s' on parameter server.", desired_param.c_str());
            ros::Duration(1.0).sleep();
        }
    }

    while (ros::ok())
    {
        try
        {
            ts = tfBuffer.lookupTransform(source_frame, target_frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // convert geometry_msgs::Quaternion to tf2::Quaterion
        tf2::Quaternion q = tf2::Quaternion(ts.transform.rotation.x,
                                            ts.transform.rotation.y,
                                            ts.transform.rotation.z,
                                            ts.transform.rotation.w);

        // form rotation matrix and obtain r, p, y
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);

        // update commands
        px_controller.sendCommand(ts.transform.translation.x);
        py_controller.sendCommand(ts.transform.translation.y);
        pz_controller.sendCommand(ts.transform.translation.z);
        rr_controller.sendCommand(r);
        rp_controller.sendCommand(p);
        ry_controller.sendCommand(y);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
