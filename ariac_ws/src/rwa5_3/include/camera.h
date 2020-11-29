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
#include <cmath>
#include "utils.h"
#include "competition.h"


const int NUM_LOGICAL_CAMERAS = 17;
const int NUM_SHELF_BREAKBEAM = 12;
extern Competition *comp_ref;

class Camera{

public:
    void logical_camera_callback(
    const nist_gear::LogicalCameraImage::ConstPtr &msg, int index);

    void break_beam_callback(const nist_gear::Proximity::ConstPtr &msg);

    void shelf_breakbeam_callback(
    const nist_gear::Proximity::ConstPtr &msg, int index);

    void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg);

    void quality_control_sensor_callback1(const nist_gear::LogicalCameraImage &msg);

    void quality_control_sensor_callback2(const nist_gear::LogicalCameraImage &msg);

    void init(ros::NodeHandle & node);

    //std::map<std::string,std::vector<part>> get_detected_parts();
    //std::map<std::string,part> get_detected_parts();
    std::map<std::string, std::map<std::string, part>> get_detected_parts();

    std::map<std::string, part> get_conveyor_detected_parts();

    std::map<std::string, std::map<std::string, part>> get_agv_detected_parts();

    Camera();

    //void remove_part(std::string logical_camera, int index);

    geometry_msgs::Pose get_faulty_pose(std::string agv);
    void removefaultyPose(std::string agv_name, std::string faulty_pose);

    bool get_is_faulty(std::string agv);
    bool get_break_beam();
    std::vector<bool> get_shelf_breakbeams();
    

    void reset_is_faulty();
    void reset_break_beam();
    void reset_shelf_breakbeams();
    void reset_agv_logical_camera(std::string);
    void reset_conveyor_logical_camera();
    

    std::map<int,std::vector<nist_gear::Proximity::ConstPtr>> get_aisle_breakbeam_msgs(); 
    
    void removeElement(std::string prod_type, std::string prod);
    void removeAllElements(std::string prod_type);


    std::map<std::string,std::map<std::string,geometry_msgs::Pose>> get_faulty_poses();

    bool isSensorBlackout();


private:
    ros::Subscriber logical_camera_subscriber[NUM_LOGICAL_CAMERAS];
    //std::map<std::string,std::vector<geometry_msgs::PoseStamped>> detected_parts;
    //std::map<std::string,std::vector<part>> detected_parts;
    //std::map<std::string,part> detected_parts;
    
    std::map<std::string, std::map<std::string, part>> detected_parts;
    std::map<std::string, std::map<std::string, part>> agv_detected_parts;
    std::map<std::string, part> conveyor_detected_parts;

    // vector of parts for agv logical cameras

    ros::Subscriber quality_sensor_subscriber_1;
    ros::Subscriber quality_sensor_subscriber_2;
    ros::Subscriber breakbeam_1_sensor_subscriber;
    ros::Subscriber shelf_breakbeam_sensor_subscriber[NUM_SHELF_BREAKBEAM];

    bool is_faulty1, is_faulty2;
    bool break_beam_triggered;
    bool sensor_blackout;
    std::vector<bool> triggered_shelf_breakbeams;
    std::map<int,std::vector<nist_gear::Proximity::ConstPtr>> aisle_breakbeam_msgs;  
    



    geometry_msgs::Pose faulty_pose1, faulty_pose2;
    std::map<std::string,std::map<std::string,geometry_msgs::Pose>> faulty_poses;
};







