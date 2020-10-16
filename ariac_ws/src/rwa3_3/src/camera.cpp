#include "camera.h"


void Camera::logical_camera_callback(
const nist_gear::LogicalCameraImage::ConstPtr &msg, int index) {
    if (msg->models.size() > 0) {
        //ROS_INFO_STREAM_THROTTLE(1,
                                 //"Logical camera " + std::to_string(index) + " detected '" << msg->models.size()
                                                                                           //<< "' objects.");
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Duration timeout(5.0);
        geometry_msgs::TransformStamped T_w_l;
        geometry_msgs::PoseStamped p_w, p_l;


        std::string name = "logical_camera_" + std::to_string(index) + "_frame";

        try {
            T_w_l = tfBuffer.lookupTransform("world", name,
                                             ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        for (int i = 0; i < msg->models.size(); i++) {
            p_l.header.frame_id =
            "logical_camera_" + std::to_string(index) + "_frame"; //+ std::to_string(index) + "_frame";
            p_l.pose = msg->models[i].pose;
            tf2::doTransform(p_l, p_w, T_w_l);
//            ROS_INFO_STREAM(p_w);
//
            part mypart;
            mypart.type = msg->models[i].type;
            mypart.pose = p_w.pose;
            mypart.frame = "world";

            //ROS_INFO("%s in world frame:\n\n"
                     //"Position: [x,y,z] = [%f,%f,%f]\n"
                     //"Orientation: [x,y,z,w] = [%f,%f,%f,%f]\n",
                     //msg->models[i].type.c_str(),
                     //p_w.pose.position.x,
                     //p_w.pose.position.y,
                     //p_w.pose.position.z,
                     //p_w.pose.orientation.x,
                     //p_w.pose.orientation.y,
                     //p_w.pose.orientation.z,
                     //p_w.pose.orientation.w);
            
           std::string key = "logical_camera_" + std::to_string(index); 
           detected_parts[key].push_back(mypart);
        }
    }
}



void Camera::break_beam_callback(const nist_gear::Proximity::ConstPtr &msg) {
    if (msg->object_detected)  // If there is an object in proximity.
        ROS_WARN("Break beam triggered.");
}




void Camera::laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
  size_t number_of_valid_ranges = std::count_if(
    msg->ranges.begin(), msg->ranges.end(), [](const float f) {return std::isfinite(f);});
  if (number_of_valid_ranges > 0) {
    ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
  }
}


Camera::Camera(){

}


void Camera::init(ros::NodeHandle & node){
  std::ostringstream otopic;
  std::string topic;

  for(int index = 0; index < NUM_LOGICAL_CAMERAS; index++) {
      otopic.str(""); otopic.clear();
      otopic << "/ariac/logical_camera_" << (index);
      topic = otopic.str();

      logical_camera_subscriber[index]= node.subscribe<nist_gear::LogicalCameraImage>(
        topic, 10, boost::bind(&Camera::logical_camera_callback, this, _1, index));
  }
}


std::map<std::string,std::vector<part>>  Camera::get_detected_parts(){
    return detected_parts;
}

