#include <algorithm>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3
#include <map>
#include "boost/bind.hpp"
#include "utils.h"
#include "gantry_control.h"

class FaultyGripper{
public:
    // Subsribing to logical cameras
    void init(ros::NodeHandle & node);

    // Takes care of faulty gripper scenario
    void faultyGripperCheck(GantryControl &gantry, std::string agvId);

    // For getting the part pose in world frame
    void logical_camera_callback(
    const nist_gear::LogicalCameraImage::ConstPtr &msg, int index);

    // Getting the actual pose of the part in agv tray
    geometry_msgs::Pose getTargetPose(GantryControl &gantry);

    bool comparePose(geometry_msgs::Pose placedPose, geometry_msgs::Pose requiredPose);

    part getActualPose();

    part getFaultyPose();

private:
    ros::Subscriber logical_camera_subscriber[2];
    std::vector<std::string> cameraNames[2];
    geometry_msgs::Pose partPose;
    geometry_msgs::Pose targetPose;
    part actualPose, faultyPose;
};