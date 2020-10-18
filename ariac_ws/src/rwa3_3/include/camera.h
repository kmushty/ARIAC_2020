#include <algorithm>
#include <vector>
#include <string>
#include <ros/ros.h>

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



const int NUM_LOGICAL_CAMERAS = 17;

class Camera{

  public:
    void logical_camera_callback(
    const nist_gear::LogicalCameraImage::ConstPtr &msg, int index);

    void break_beam_callback(const nist_gear::Proximity::ConstPtr &msg);

    void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg);
    
    void init(ros::NodeHandle & node);

    std::map<std::string,std::vector<part>> get_detected_parts();

    Camera();

    void remove_part(std::string logical_camera, int index);

 


            

  private:
    ros::Subscriber logical_camera_subscriber[NUM_LOGICAL_CAMERAS];
    //std::map<std::string,std::vector<geometry_msgs::PoseStamped>> detected_parts;
    std::map<std::string,std::vector<part>> detected_parts;
    

};






