#include "camera.h"

Competition *comp_ref; 
bool stopAdding;


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
            mypart.logicalCameraName = key;
            //std::string temp = mypart.type + std::to_string((int)round(mypart.pose.position.x)) +
            //std::to_string((int)round(mypart.pose.position.y)) +
            //std::to_string((int)round(mypart.pose.position.z)) +
            //std::to_string((int)round(mypart.pose.orientation.x)) +
            //std::to_string((int)round(mypart.pose.orientation.y)) +
            //std::to_string((int)round(mypart.pose.orientation.z)) +
            //std::to_string((int)round(mypart.pose.orientation.w));

            std::string temp = mypart.type + std::to_string((int) std::round(mypart.pose.position.x * 10));
//                                             std::to_string((int)std::round(mypart.pose.position.y*10));
//             std::to_string((int)std::round(mypart.pose.position.z*10));

            if (key == "logical_camera_9") {                           // conveyor belt logical camera
                mypart.conveyor_time = (*comp_ref).getClock();
                conveyor_detected_parts[key] = mypart;
            } else if (key == "logical_camera_8" || key == "logical_camera_10") {  // agv_logical camera
                if (!std::isnan(mypart.pose.position.x) && !std::isnan(mypart.pose.position.y) &&
                    !std::isnan(mypart.pose.position.z)) {
                    if (agv_detected_parts.find(key) == agv_detected_parts.end()) {
                        mypart.count = agv_detected_parts[key].size() + 1;
                        agv_detected_parts[key][temp] = mypart;
                    } else {
                        bool exist = false;
                        auto vec = agv_detected_parts[mypart.type];
                        if (vec.size() < 4) {
                            for (auto part:vec) {
                                if ((std::max(part.second.pose.position.x, mypart.pose.position.x) -
                                     std::max(part.second.pose.position.x, mypart.pose.position.x)) < 0.2 &&
                                    (std::min(part.second.pose.position.y, mypart.pose.position.y) -
                                     std::min(part.second.pose.position.y, mypart.pose.position.y)) < 0.2)
                                    exist = true;
                            }
                            if (!exist) {
                                mypart.count = agv_detected_parts[key].size() + 1;
                                agv_detected_parts[key][temp] = mypart;
                            }
                        }
                    }
                }
            } else if(!stopAdding) {                                               //logical cameras for shelfs and bins
                if (!std::isnan(mypart.pose.position.x) && !std::isnan(mypart.pose.position.y) &&
                    !std::isnan(mypart.pose.position.z)) {
                    if (detected_parts.find(mypart.type) == detected_parts.end()) {
                        detected_parts[mypart.type][temp] = mypart;
                    }
                    else {
                        bool exist = false;
                        auto vec = detected_parts[mypart.type];
                        if (vec.size() < 4) {
                            for (auto part:vec) {
                                if (std::max(part.second.pose.position.x, mypart.pose.position.x) -
                                    std::min(part.second.pose.position.x, mypart.pose.position.x) < 0.2)
                                    exist = true;
                            }
                            if (!exist)
                                detected_parts[mypart.type][temp] = mypart;
                        }

                    }
                }
            }
        }
    }
}


void Camera::break_beam_callback(const nist_gear::Proximity::ConstPtr &msg) {
    //ROS_INFO_STREAM("in break beam callback");
    if (msg->object_detected) {  // If there is an object in proximity.
        break_beam_triggered = true;
        ROS_WARN_ONCE("Break beam triggered.");
    }
    else
        break_beam_triggered = false;
    //ROS_INFO_STREAM("exited break beam callback");
}


void Camera::shelf_breakbeam_callback(
const nist_gear::Proximity::ConstPtr &msg, int index) {
//    triggered_shelf_breakbeams = std::vector<bool> (NUM_SHELF_BREAKBEAM,false);
   
  //ROS_INFO_STREAM("in shelf callback");
    if (msg->object_detected) {  // If there is an object in proximity.
        ROS_INFO_STREAM("Break beam " + std::to_string(index) +"  triggered.");
        triggered_shelf_breakbeams[index] = true;
    }



   if(index < 3){
       aisle_breakbeam_msgs[0].push_back(msg);
   }else if(index >=3 && index < 6)
       aisle_breakbeam_msgs[1].push_back(msg);
   else if(index >=6 && index < 9)
       aisle_breakbeam_msgs[2].push_back(msg);
   else if(index >=9 && index < 12)
       aisle_breakbeam_msgs[3].push_back(msg);

   //ROS_INFO_STREAM("exited  shelf callback");
}



void Camera::laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
    size_t number_of_valid_ranges = std::count_if(
    msg->ranges.begin(), msg->ranges.end(), [](const float f) {return std::isfinite(f);});
    if (number_of_valid_ranges > 0) {
        ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
        sensor_blackout = false;
    }else{
      sensor_blackout = true;
    }
}

bool Camera::isSensorBlackout() {
  return sensor_blackout;
}

Camera::Camera(){

}


void Camera::init(ros::NodeHandle & node){
    ROS_INFO_STREAM("in init camera");
    std::ostringstream otopic;
    std::string topic;

    for(int index = 0; index < NUM_LOGICAL_CAMERAS; index++) {
        otopic.str(""); otopic.clear();
        otopic << "/ariac/logical_camera_" << (index);
        topic = otopic.str();

        logical_camera_subscriber[index]= node.subscribe<nist_gear::LogicalCameraImage>(
        topic, 1, boost::bind(&Camera::logical_camera_callback, this, _1, index));
    }

    ROS_INFO_STREAM("debug1");
    for(int index = 0; index < NUM_SHELF_BREAKBEAM; index++) {
        otopic.str(""); otopic.clear();
        otopic << "/ariac/shelf_breakbeam_" << (index) << "_change";
        topic = otopic.str();

        shelf_breakbeam_sensor_subscriber[index]= node.subscribe<nist_gear::Proximity>(
        topic, 1, boost::bind(&Camera::shelf_breakbeam_callback, this, _1, index));
    }



    ROS_INFO_STREAM("debug2");
    quality_sensor_subscriber_1 = node.subscribe(
    "/ariac/quality_control_sensor_1", 1, &Camera::quality_control_sensor_callback1,this
    );

    ROS_INFO_STREAM("debug4");
    quality_sensor_subscriber_2 = node.subscribe(
    "/ariac/quality_control_sensor_2", 1, &Camera::quality_control_sensor_callback2,this
    );

    ROS_INFO_STREAM("debug5");
    breakbeam_1_sensor_subscriber = node.subscribe("/ariac/breakbeam_1",1, &Camera::break_beam_callback, this);
    ROS_INFO_STREAM("debug7");
    
    break_beam_triggered = false;
    is_faulty1 = false;
    is_faulty2 = false;

    ROS_INFO_STREAM("debug6");
    triggered_shelf_breakbeams = std::vector<bool> (NUM_SHELF_BREAKBEAM,false);
   
    ROS_INFO_STREAM("debug3");
    for(int i = 0; i<4; i++){
       aisle_breakbeam_msgs[i] = {};
    }
    ROS_INFO_STREAM("exited out of init camera");
    sensor_blackout = false; 
}



//std::map<std::string,std::vector<part>>  Camera::get_detected_parts(){
//return detected_parts;
//}

std::map<std::string,std::map<std::string,part>>  Camera::get_detected_parts(){
    return detected_parts;
}


//void Camera::remove_part(std::string logical_camera,  int index){
//auto vec =  detected_parts[logical_camera];
//vec.erase(vec.begin()+index);
//}


void Camera::quality_control_sensor_callback1(const nist_gear::LogicalCameraImage &msg) {
    if (msg.models.size() > 0) {
        //ROS_INFO_STREAM("msg obtainted" << msg.models[8].pose);

        is_faulty1 = true;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Duration timeout(5.0);

        geometry_msgs::TransformStamped T_w_l;
        geometry_msgs::PoseStamped p_w, p_l;

        p_l.pose = msg.models[0].pose;
        T_w_l = tfBuffer.lookupTransform("world", "quality_control_sensor_1_frame", ros::Time(0), timeout);

        //ROS_INFO_STREAM("Pose of part in camera frame" << p_l.pose);
        tf2::doTransform(p_l, p_w, T_w_l);
        //ROS_INFO_STREAM("Pose of part in world frame" << p_w.pose);

        faulty_pose1 = p_w.pose;
        std::string key = std::to_string((int) round(p_w.pose.position.x)) +
                          std::to_string((int) round(p_w.pose.position.y));


        if (faulty_poses.find("agv1") == faulty_poses.end()) {
            faulty_poses["agv1"][key] = p_w.pose;
        } else {
            bool exist = false;
            auto vec = faulty_poses["agv1"];
            if (vec.size() < 4) {
                for (auto part:vec) {
                    if ((std::max(part.second.position.x, p_w.pose.position.x) -
                         std::max(part.second.position.x, p_w.pose.position.x)) < 0.2 &&
                        (std::min(part.second.position.y, p_w.pose.position.y) -
                         std::min(part.second.position.y, p_w.pose.position.y)) < 0.2)
                        exist = true;
                }
                if (!exist) {
                    faulty_poses["agv1"][key] = p_w.pose;
                }
            }
        }
    }
}


void Camera::quality_control_sensor_callback2(const nist_gear::LogicalCameraImage &msg){
    if(msg.models.size() > 0) {
        //ROS_INFO_STREAM("msg obtainted" << msg.models[8].pose);

        is_faulty2 = true;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Duration timeout(5.0);

        geometry_msgs::TransformStamped T_w_l;
        geometry_msgs::PoseStamped p_w, p_l;

        p_l.pose = msg.models[0].pose;
        T_w_l = tfBuffer.lookupTransform("world", "quality_control_sensor_2_frame", ros::Time(0), timeout);

        //ROS_INFO_STREAM("Pose of part in camera frame" << p_l.pose);
        tf2::doTransform(p_l, p_w, T_w_l);
        //ROS_INFO_STREAM("Pose of part in world frame" << p_w.pose);

        faulty_pose2 = p_w.pose;
        std::string key = std::to_string((int)round(p_w.pose.position.x)) +
                          std::to_string((int)round(p_w.pose.position.y)) +
                          std::to_string((int)round(p_w.pose.position.z)) +
                          std::to_string((int)round(p_w.pose.orientation.x)) +
                          std::to_string((int)round(p_w.pose.orientation.y)) +
                          std::to_string((int)round(p_w.pose.orientation.z)) +
                          std::to_string((int)round(p_w.pose.orientation.w));

       if(faulty_poses.find("agv2") == faulty_poses.end()) {
            faulty_poses["agv2"][key] = p_w.pose;
       }
       else{
           bool exist = false;
           auto vec = faulty_poses["agv2"];
           if(vec.size() < 4) {
               for(auto part:vec){
                 if((std::max(part.second.position.x,p_w.pose.position.x)  -
                     std::max(part.second.position.x,p_w.pose.position.x)) < 0.2 && 
                    (std::min(part.second.position.y,p_w.pose.position.y)  -
                     std::min(part.second.position.y,p_w.pose.position.y)) < 0.2 )
                     exist = true; 
               } 
               if(!exist){
                   faulty_poses["agv2"][key] = p_w.pose;
               }
           }
       }
    }
}


void Camera::reset_is_faulty(){
    is_faulty1 = false;
    is_faulty2 = false;
}


bool Camera::get_is_faulty(std::string agv) {
    if(agv == "agv2")
        return is_faulty1;
    return is_faulty2;
}


geometry_msgs::Pose Camera::get_faulty_pose(std::string agv) {
    if(agv == "agv2")
        return faulty_pose1;
    return faulty_pose2;
}

void Camera::reset_break_beam(){
    break_beam_triggered = false;
}

bool Camera::get_break_beam(){
    return break_beam_triggered;
}


void Camera::reset_shelf_breakbeams() {
  triggered_shelf_breakbeams = std::vector<bool>(NUM_SHELF_BREAKBEAM,false);
}


std::vector<bool> Camera::get_shelf_breakbeams() {
  return triggered_shelf_breakbeams;
}


std::map<int,std::vector<nist_gear::Proximity::ConstPtr>> Camera::get_aisle_breakbeam_msgs() {
  return aisle_breakbeam_msgs;
}
    
void Camera::removeElement(std::string prod_type, std::string prod){
  detected_parts[prod_type].erase(prod);
}

void Camera::removeAllElements(std::string prod_type){
    detected_parts[prod_type].clear();
}
    

std::map<std::string, part> Camera::get_conveyor_detected_parts() {
    return conveyor_detected_parts;
}

std::map<std::string, std::map<std::string, part>> Camera::get_agv_detected_parts() {
  return agv_detected_parts;
}


void Camera::reset_agv_logical_camera(std::string logical_camera_name) {
   agv_detected_parts[logical_camera_name].clear();
}

void Camera::reset_conveyor_logical_camera() {
  conveyor_detected_parts.clear();
}


std::map<std::string,std::map<std::string,geometry_msgs::Pose>> Camera::get_faulty_poses() {
  return faulty_poses;
}


void Camera::removefaultyPose(std::string agv_name, std::string faulty_pose) {
    faulty_poses[agv_name].erase(faulty_pose);
}


void Camera::removeAllFaultyElements(std::string agv_name){
    faulty_poses[agv_name].clear();
}


