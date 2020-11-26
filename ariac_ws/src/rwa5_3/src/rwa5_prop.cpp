// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <math.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <nist_gear/AGVControl.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3

#include "competition.h"
#include "utils.h"
#include "gantry_control.h"
#include "camera.h"

#include <tf2/LinearMath/Quaternion.h>



const double LENGTH_OF_AISLE = 14.2;
const int NUM_LOGICAL_CAMERAS_PER_AISLE = 4;
const int TIMELIMIT_THRESHOLD = 40;

//coordinates obtainted from gazebo
const double END_LOCATION_X = -16.793;
const double BEGIN_LOCATION_X = -1.5778;
const double MID_LOCATION_X = -12.592617;


std::vector<bool> obstacleInAisle(4,false);
std::vector<obstacle>  obstacleAssociatedWithAisle;

bool stop_processing;
bool HighPriorityOrderInitiated;
bool ConveyorFlag;
bool PreviousOrderAgvChange;

std::map<std::string,std::vector<PresetLocation>> presetLoc;



bool compIsAlmostOver(int threshold, Competition &comp){
   return std::abs(comp.getClock()-500) <= threshold;
}

void agvDeliveryService(std::string agv_id, ros::ServiceClient &agvDelivery1, ros::ServiceClient &agvDelivery2, std::string shipmentType){
    
    ros::ServiceClient *agvDel;
    if(agv_id == "agv2")
        agvDel = &agvDelivery2;
    else
        agvDel = &agvDelivery1;

    auto agvDelivery = *agvDel;

    // AGV Delivering the parts
    if (!agvDelivery.exists()) {
        ROS_INFO("[AGV][startAGV] Waiting for thE AGV to start...");
        agvDelivery.waitForExistence();
        ROS_INFO("[AGV][startAGV] AGV is now ready.");
    }
    ROS_INFO("[AGV][startAGV] Requesting AGV start...");
    nist_gear::AGVControl srv;
    srv.request.shipment_type = shipmentType;
    agvDelivery.call(srv);


    while(!srv.response.success) {
        ROS_INFO_ONCE("AGV delivering the parts");
    }
    ROS_INFO("%s",srv.response.message.c_str());

    ROS_INFO("AGV delivery successful");
}



void initWayPoints(std::map<std::string,std::vector<PresetLocation>> &presetLoc, GantryControl &gantry){
    
    // waypoints from start position position to logical_camera
    presetLoc["logical_camera_2"] = {gantry.bin16_};                                                          
    presetLoc["logical_camera_6"] = {gantry.bin13_};
//    presetLoc["logical_camera_11"] = {gantry.shelf5_1_, gantry.shelf5_2_, gantry.shelf5_3_};
//    presetLoc["logical_camera_12"] = {gantry.shelf8_1_, gantry.shelf8_2_, gantry.shelf8_3_};
//    presetLoc["logical_camera_14"] = {gantry.shelf5_1_, gantry.shelf5_4_, gantry.shelf5_5_};
//    presetLoc["logical_camera_15"] = {gantry.shelf8_1_, gantry.shelf8_2_, gantry.shelf8_3_};
//    presetLoc["logical_camera_13"] = {gantry.shelf11_1_, gantry.shelf11_2_, gantry.shelf11_3_};
//    presetLoc["logical_camera_16"] = {gantry.shelf11_1_, gantry.shelf11_2_, gantry.shelf11_3_};
//    presetLoc["logical_camera_15"] = {gantry.shelf8_obs_blue1_, gantry.shelf8_obs_blue2_, gantry.shelf8_obs_blue3_, gantry.right_gap_2_blue_1_, gantry.right_gap_2_blue_2_};
//    presetLoc["logical_camera_12"] = {gantry.shelf8_obs_green1_, gantry.shelf8_obs_green2_, gantry.shelf8_obs_green3_, gantry.left_gap_2_green_1_, gantry.left_gap_2_green_2_};
//    presetLoc["logical_camera_15"] = {gantry.right_gap_2_blue_1_, gantry.right_gap_2_blue_2_};
//    presetLoc["logical_camera_12"] = {gantry.left_gap_2_green_1_, gantry.left_gap_2_green_2_};
    presetLoc["logical_camera_12"] = {gantry.logicalCamera12_, gantry.logicalCamera12_1_, gantry.logicalCamera12_2_};
    presetLoc["logical_camera_15"] = {gantry.logicalCamera12_, gantry.logicalCamera12_1_, gantry.logicalCamera15_2_};
    presetLoc["logical_camera_13"] = {gantry.logicalCamera13_, gantry.logicalCamera13_1_, gantry.logicalCamera13_2_};
    presetLoc["logical_camera_16"] = {gantry.logicalCamera13_, gantry.logicalCamera13_1_, gantry.logicalCamera16_2_};
    presetLoc["logical_camera_11"] = {gantry.logicalCamera11_, gantry.logicalCamera11_1_, gantry.logicalCamera11_2_};
    presetLoc["logical_camera_14"] = {gantry.logicalCamera11_, gantry.logicalCamera11_1_, gantry.logicalCamera14_2_};




    // useful presetloc
    presetLoc["start"] = {gantry.start_};                                                                     
    presetLoc["agv2"] = {gantry.agv2_};
    presetLoc["agv1"] = {gantry.agv1_};
    presetLoc["agv2_faultyG"] = {gantry.agv2_faultyG_};
    presetLoc["agv1_faultyG"] = {gantry.agv1_faultyG_};
    presetLoc["flipped_pulley_agv2"] = {gantry.agv2_go_to_flipped_pulley_};//verified
    presetLoc["flipped_pulley_agv1"] = {gantry.agv1_go_to_flipped_pulley_};//verified
    presetLoc["agv2_flipped_final"]  = {gantry.agv2_flipped1_};//verified
    presetLoc["agv1_flipped_final"]  = {gantry.agv1_flipped1_};//verified
    presetLoc["agv2_right_arm_drop_flip"]  = {gantry.agv2_flipped_};//verified
    presetLoc["agv1_right_arm_drop_flip"]  = {gantry.agv1_flipped_};//verified
    presetLoc["agv2_left_arm_drop"]  = {gantry.agv2_drop_};//verified
    presetLoc["agv1_left_arm_drop"]  = {gantry.agv1_drop_};//verified
    presetLoc["movingPart"] = {gantry.movingPart_};
    presetLoc["pickmovingPart"] = {gantry.movingPart1_, gantry.movingPart_};
    presetLoc["agv1_gasket_part_green"] = {gantry.agv1_gasket_part_green_};
    presetLoc["agv2_disk_part_green"] = {gantry.agv2_};

    // gaps
    presetLoc["left_gap_0"] = {gantry.left_gap_default_, gantry.left_gap_0_2_};
    presetLoc["left_gap_1"] = {gantry.left_gap_default_, gantry.left_gap_1_2_, gantry.left_gap_1_3_};
//    presetLoc["left_gap_2"] = {gantry.left_gap_default_, gantry.left_gap_2_2_, gantry.left_gap_2_3_};
    presetLoc["left_gap_2"] = {gantry.shelf8_obs_green1_, gantry.shelf8_obs_green2_, gantry.shelf8_obs_green3_};
    presetLoc["left_gap_3"] = {gantry.left_gap_default_, gantry.left_gap_3_2_, gantry.left_gap_3_3_};

    presetLoc["middle_gap_0"] = {gantry.middle_gap_default_, gantry.middle_gap_0_2_};
    presetLoc["middle_gap_1"] = {gantry.middle_gap_default_, gantry.middle_gap_1_2_, gantry.middle_gap_1_3_};
    presetLoc["middle_gap_2"] = {gantry.middle_gap_default_, gantry.middle_gap_2_2_, gantry.middle_gap_2_3_};
    presetLoc["middle_gap_3"] = {gantry.middle_gap_default_, gantry.middle_gap_3_2_, gantry.middle_gap_3_3_};

    presetLoc["right_gap_0"] = {gantry.right_gap_default_, gantry.right_gap_0_2_};
    presetLoc["right_gap_1"] = {gantry.right_gap_default_, gantry.right_gap_1_2_, gantry.right_gap_1_3_};
//    presetLoc["right_gap_2"] = {gantry.right_gap_default_, gantry.right_gap_2_2_, gantry.right_gap_2_3_};
    presetLoc["right_gap_2"] = {gantry.shelf8_obs_blue1_, gantry.shelf8_obs_blue2_, gantry.shelf8_obs_blue3_};
    presetLoc["right_gap_3"] = {gantry.right_gap_default_, gantry.right_gap_3_2_, gantry.right_gap_3_3_};

    //conveyor
    presetLoc["conveyorPart_1"] = {gantry.bin13_1_};
    presetLoc["conveyorPart_2"] = {gantry.bin13_2_};
    presetLoc["conveyorPart_3"] = {gantry.bin13_3_};
    presetLoc["conveyorPart_4"] = {gantry.bin13_4_};
}




void moveToLocation(std::map<std::string,std::vector<PresetLocation>> &presetLoc, std::string location,GantryControl &gantry){
    auto vec = presetLoc[location];
    for(auto waypoint :vec)
        gantry.goToPresetLocation(waypoint);
}

void moveToLocation2(std::map<std::string,std::vector<PresetLocation>> &presetLoc, part my_part,GantryControl &gantry){
    auto vec = presetLoc[my_part.logicalCameraName];
    int count =0;
    for(auto waypoint :vec){
        gantry.goToPresetLocation(waypoint);
//        ros::Duration(10).sleep();
        ROS_INFO_STREAM("iiiiiiiin way point");
    }
    gantry.moveToPart(my_part);
    //modify to pick part
}

void retraceSteps(std::map<std::string,std::vector<PresetLocation>> &presetLoc, std::string location,GantryControl &gantry) {
    auto vec = presetLoc[location];
    for(int i=vec.size()-1; i>=0;i--)
        gantry.goToPresetLocation(vec[i]);
}

void moveFromLocationToStart(std::map<std::string,std::vector<PresetLocation>> &presetLoc, std::string location,GantryControl &gantry) {
    auto vec = presetLoc[location];
    if(vec.size() == 1){
        gantry.goToPresetLocation(vec[0]);
        if(location != "start")
            gantry.goToPresetLocation(gantry.start_);
    }
    else{
        for(int i=vec.size()-1; i>=0;i--)
            gantry.goToPresetLocation(vec[i]);
        gantry.goToPresetLocation(gantry.start_);
    }
}


void faultyGripper(GantryControl &gantry,product &prod,Camera &camera, part my_part_in_tray){
    /* For testing why the score is zero
    dis = sqrt(pow(actual_part.pose.position.x - placed_part.pose.position.x, 2) +
               pow(actual_part.pose.position.y - placed_part.pose.position.y, 2) +
               pow(actual_part.pose.position.z - placed_part.pose.position.z, 2));

    dotProduct = actual_part.pose.orientation.x * placed_part.pose.orientation.x +
                 actual_part.pose.orientation.y * placed_part.pose.orientation.y +
                 actual_part.pose.orientation.z * placed_part.pose.orientation.z +
                 actual_part.pose.orientation.w * placed_part.pose.orientation.w;

    tf2::Quaternion q1(placed_part.pose.orientation.x,
                       placed_part.pose.orientation.y,
                       placed_part.pose.orientation.z,
                       placed_part.pose.orientation.w);
    tf2::Matrix3x3 m1(q1);
    m1.getRPY(roll, pitch, yaw);

    tf2::Quaternion q2(actual_part.pose.orientation.x,
                       actual_part.pose.orientation.y,
                       actual_part.pose.orientation.z,
                       actual_part.pose.orientation.w);
    tf2::Matrix3x3 m2(q2);
    m2.getRPY(roll1, pitch1, yaw1);
    angleDiff = yaw - yaw1;

    if(flippedPart)
        armState = gantry.getGripperState("right_arm");
    else
        armState = gantry.getGripperState("left_arm");

    if(!armState.attached){// || abs(dotProduct) < 0.95 || (abs(angleDiff) > 0.1 && abs(abs(angleDiff) - 2* M_PI) > 0.1) ){
    */


    part placed_part, actual_part;
    nist_gear::VacuumGripperState armState;

    std::map<std::string,part> parts;
    if (prod.agv_id == "agv2")
        parts = camera.get_detected_parts()["logical_camera_10"];                 
    else
        parts = camera.get_detected_parts()["logical_camera_8"];
    

    //get part that was just placed on agv;
    placed_part = parts.begin()->second;                 
    for(const auto& part: parts){
      if(placed_part.count < part.second.count)
        placed_part = part.second;
    }
    

    armState = gantry.getGripperState(prod.arm_name);

    ROS_INFO("Faulty Gripper Detected");
    ROS_INFO("Re-picking and replacing the part");
    moveToLocation(presetLoc,prod.agv_id+"_"+prod.type, gantry);
    ROS_INFO_STREAM(prod.agv_id+"_"+prod.type);
    gantry.pickPart(placed_part);
    moveToLocation(presetLoc,prod.agv_id+"_"+prod.type, gantry);
    moveToLocation(presetLoc, prod.agv_id, gantry);
    gantry.placePart(my_part_in_tray, prod.agv_id, prod.arm_name);
}



void flipPart(GantryControl &gantry, part &my_part_in_tray, product &prod){
    moveToLocation(presetLoc,"flipped_pulley_"+prod.agv_id,gantry);                              //set arms to desired configuration to flip
    gantry.activateGripper("right_arm");                                                         //activate and deactivate gripper
    gantry.deactivateGripper("left_arm");
    moveToLocation(presetLoc,prod.agv_id+"_right_arm_drop_flip",gantry);
    my_part_in_tray.pose.orientation.x = 0;                                                      //modify pose orientation
    my_part_in_tray.pose.orientation.w = 1;

    prod.arm_name = "right_arm";
}


int aisleAssociatedWithPart(part my_part){
    std::string aisle;
    std::string obstacleNumInAisle;
//    part my_part;
//    my_part.type = prod.type;
//    my_part.pose = prod.pose;
//    ObstacleInAisle = std::vector<bool> (4,false);

     
    for(int i =0 ; i<=7 ; i++){
       std::string str = "logical_camera_" +std::to_string(i);
       if(my_part.logicalCameraName == str)
         return -1;
    }

    ROS_INFO_STREAM(double(my_part.pose.position.y));
    if(double(my_part.pose.position.y) <= 0.6 && double(my_part.pose.position.y)>0){
        // Return the value of the aisle
        return 1;
        ROS_INFO_STREAM("aisle2");
        ROS_INFO_STREAM("person1");
    }
    else if(double(my_part.pose.position.y) >= -0.7 && double(my_part.pose.position.y)<0){
        return 2;
        ROS_INFO_STREAM("aisle3");
        ROS_INFO_STREAM("person2");
    }
    else if(double(my_part.pose.position.y) <= -2.3 && double(my_part.pose.position.y)> -2.9){
        return 2;
        ROS_INFO_STREAM("aisle3");
        ROS_INFO_STREAM("person2");
    }
    else if(double(my_part.pose.position.y) <= -2.9 && double(my_part.pose.position.y)> -3.5){
        return 3;
        ROS_INFO_STREAM("aisle4");
        ROS_INFO_STREAM("person2");
    }
    else if(double(my_part.pose.position.y) >= 2.6 && double(my_part.pose.position.y)< 3.2){
        return 1;
        ROS_INFO_STREAM("aisle2");
        ROS_INFO_STREAM("person1");
    }
    else{
        return 0;
        ROS_INFO_STREAM("aisle1");
        ROS_INFO_STREAM("person1");
    }
}

geometry_msgs::TransformStamped shelfPosition(std::string shelf){
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration timeout(5.0);
    geometry_msgs::TransformStamped T_w_l;

    try {
        T_w_l = tfBuffer.lookupTransform("world", shelf,                                        // Determining shelf position in world frame
                                         ros::Time(0), timeout);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    return T_w_l;
}


double shelfDistance(std::string shelf1, std::string shelf2){                                    // Determining distance between adjacent shelves
    geometry_msgs::TransformStamped s1;
    geometry_msgs::TransformStamped s2;
    s1 = shelfPosition(shelf1);
    s2 = shelfPosition(shelf2);
    return double(abs(abs(s1.transform.translation.x) - abs(s2.transform.translation.x)));
}


std::vector<std::string> determineGaps(){                                                   // Function returning gap postions in the form of string
    ROS_INFO_STREAM("In determineGaps method");
    std::vector<std::string> gap;
    std::vector<double> gapThreshold = {6.299163, 6.299173};
    std::vector<std::string> shelfSide = {"left_gap_","middle_gap_", "right_gap_"};
    std::string temp;
    int count = 0;
    int countFlag = 0;
    std::string name1;
    std::string name2;
    int shelfStartIndex = 3;
    int shelfEndIndex = 11;

    for(int shelf = shelfStartIndex; shelf <= shelfEndIndex; shelf = shelf + shelfStartIndex){
        count ++;
        countFlag = 0;
        for(int row = shelf; row < (shelf + shelfStartIndex)-1; row++){
            name1 = "shelf"+std::to_string(row)+"_frame";
            name2 = "shelf"+std::to_string(row+1)+"_frame";

            if(shelfDistance(name1, name2) >= gapThreshold[0] && shelfDistance(name1, name2) <= gapThreshold[1]){
                gap.push_back(shelfSide[count-1] + std::to_string((row+1) % shelfStartIndex));
                countFlag += 1;
            }
        }
        if(countFlag == 0){
            if(shelfPosition("shelf"+std::to_string(shelf)+"_frame").transform.translation.x >= -2.093989 &&
               shelfPosition("shelf"+std::to_string(shelf)+"_frame").transform.translation.x <= -2.093979){
                gap.push_back(shelfSide[count-1] + std::to_string((shelfStartIndex - countFlag)));
            }
            else
                gap.push_back(shelfSide[count-1] + std::to_string((countFlag)));
        }
    }
    
    
    for(auto val:gap)                                                                    // printing the gap positions
        ROS_INFO_STREAM(val);

    ROS_INFO_STREAM("Exited determin gap method");
    return gap;
}


//TODO this is a temporary code.Need to implement full code considering all edge cases
std::vector<std::string> planPath(int aisle_num){
   auto gap = determineGaps();

   ROS_INFO_STREAM("In planPath method");
   
   ROS_INFO_STREAM("printing gaps");
   ROS_INFO_STREAM("aisle_num " << aisle_num);
   for(auto a:gap)
     std::cout << a << std::endl;

   std::vector<std::string> plan;  
   if(aisle_num == 1)
      plan.push_back(gap[0]);
   else
      plan.push_back(gap[2]);

   ROS_INFO_STREAM("Exited planPath method");

   return plan;
}


double calc_remainder(double x, double y){
  std::string s = std::to_string(x/y);
  s = s.substr(0,s.find('.'));
  int q = std::stoi(s);
  double result  = x - q*y;
  return result;
}


//TODO  add functionality
std::vector<std::string> estimateLocation(int aisle_num, double t) {
   std::vector<std::string> result;

   ROS_INFO_STREAM("obstacle is valid or invalid ");
   ROS_INFO_STREAM(obstacleAssociatedWithAisle[aisle_num].is_valid_obstacle);
   if(!obstacleAssociatedWithAisle[aisle_num].is_valid_obstacle)
        return result;
   double move_time = obstacleAssociatedWithAisle[aisle_num].move_time;
   double wait_time = obstacleAssociatedWithAisle[aisle_num].wait_time;
   double time_stamp1 = obstacleAssociatedWithAisle[aisle_num].time_stamp1;
   double x;
  
   double td = t - time_stamp1;                                                   //number of seconds from time_stamp
   double tf = move_time + wait_time;                                             // time take to go from end to end


   // find remaining time after time of flight(time taken to go and come back)
   
   if(td<0){
     ROS_INFO_STREAM("time_stamp must be greater than t. Negative time not allowed");
     return result;
   }
   ROS_INFO_STREAM("time is "<< t);
   td  = calc_remainder(td,2*tf);                                                                         
   ROS_INFO_STREAM("td is "<< td);
   ROS_INFO_STREAM("time of flight "<< 2*tf);
   ROS_INFO_STREAM("timestamp "<<time_stamp1);
   

   if(td > tf) {
       td  -= tf; 
       if(td <= wait_time) {
          ROS_INFO_STREAM("waiting near conveyor belt with" << (wait_time - td) << " left");
          result = std::vector<std::string> {"waiting",std::to_string(wait_time-td)};
       }else {
        td = td-wait_time;
        x = BEGIN_LOCATION_X - (LENGTH_OF_AISLE/move_time)*td;
        ROS_INFO_STREAM("Moving away from conveyor belt");
        ROS_INFO_STREAM("currently at position " << x);
        result = std::vector<std::string> {"away",std::to_string(x)};
       }
   }else {
       if(td <= wait_time) {
          ROS_INFO_STREAM("waiting at the opposite end of conveyor belt with " << (wait_time - td) << " left");
          result = std::vector<std::string> {"waiting",std::to_string(wait_time-td)};
       }else {
          td = td-wait_time;
          x = END_LOCATION_X + (LENGTH_OF_AISLE/move_time)*td;
          ROS_INFO_STREAM("Moving towards conveyor belt");
          ROS_INFO_STREAM("currently at position " << x);
          result = std::vector<std::string> {"toward",std::to_string(x)};
       }
   }
   return result;
}


//TODO make velocity more robust, adjust the camera
void estimateObstacleAttributes(Camera &camera,int aisle_num) {
    ROS_INFO_STREAM("In estimateObstacleAttributes method");
    auto aisle_breakbeam_msgs  = camera.get_aisle_breakbeam_msgs();

    auto vec = aisle_breakbeam_msgs[aisle_num];
    if(obstacleInAisle[aisle_num] == true  &&  vec.size() > 8) {
       ROS_INFO_STREAM("estimating velocity for aisle" << aisle_num);
       ROS_INFO_STREAM("vector size is " << vec.size());
       auto it = std::find_if(vec.begin(), vec.end(),
                         [&str = "shelf_breakbeam_5_frame"] 
                         (nist_gear::Proximity::ConstPtr &msg){return (msg->header).frame_id == str && msg->object_detected; });    

       ROS_INFO_STREAM("aisle number is "<< aisle_num);
       if(it != vec.end()){
         ROS_INFO_STREAM("it something");
         ROS_INFO_STREAM(std::distance(vec.begin(),it));
         ROS_INFO_STREAM("It");
         ROS_INFO_STREAM(((*it)->header).frame_id);
         ROS_INFO_STREAM((*it)->object_detected);
       }else{
         ROS_INFO_STREAM("it nothing");
         return;
       }

       auto it2 = std::find_if(vec.begin(), vec.end(),
                         [&str = "shelf_breakbeam_4_frame"] 
                         (nist_gear::Proximity::ConstPtr &msg){return (msg->header).frame_id == str && msg->object_detected; });    

       if(it2 != vec.end()){
         ROS_INFO_STREAM("it2 something");
         ROS_INFO_STREAM(std::distance(vec.begin(),it2));
         ROS_INFO_STREAM("It1");
         ROS_INFO_STREAM(((*(it+1))->header).frame_id);
         ROS_INFO_STREAM((*(it+1))->object_detected);
         ROS_INFO_STREAM("It2");
         ROS_INFO_STREAM(((*it2)->header).frame_id);
         ROS_INFO_STREAM((*it2)->object_detected);
       }else{
         ROS_INFO_STREAM("it2 nothing");
         return;
       }
        
       double sec1 = double((*it)->header.stamp.sec) + double((*it)->header.stamp.nsec)*1e-9;
       double sec2 = double((*(it+1))->header.stamp.sec) + double((*(it+1))->header.stamp.nsec)*1e-9;
       double sec3 = double((*it2)->header.stamp.sec) + double((*it2)->header.stamp.nsec)*1e-9;

       ROS_INFO_STREAM(sec2 - sec1);
       double wait_time = round(sec2 - sec1);
       double move_time = round(3*sec3 - sec1 - wait_time);
       ROS_INFO_STREAM(sec3 - sec2);

       double velocity = LENGTH_OF_AISLE/move_time;

       obstacle human;
       human.wait_time = 7;//wait_time;
       human.move_time = 9;//move_time;
       human.time_stamp1 = sec1;
       human.is_valid_obstacle = true;
         
       obstacleAssociatedWithAisle[aisle_num] = human;
       ROS_INFO_STREAM("Printing time interval");
       ROS_INFO_STREAM("sec1 is " << sec1);
       ROS_INFO_STREAM("sec2 is " << sec2);
       ROS_INFO_STREAM("sec3 is " << sec3);
       ROS_INFO_STREAM("wait time is " << wait_time);
       ROS_INFO_STREAM("move time is " << move_time);
       
       ROS_INFO_STREAM("exited estimateObstacleAttributes method");
    }
}


void planAndExecutePath(product prod, part my_part,std::map<std::string, std::vector<PresetLocation>> &presetLoc, 
                        Camera &camera, GantryControl &gantry, Competition &comp, std::string location, int aisle_num) {

   ROS_INFO_STREAM("Plan and execute method");
   
   auto gaps = planPath(aisle_num);

   ROS_INFO_STREAM("is obstacle valid " << obstacleAssociatedWithAisle[aisle_num].is_valid_obstacle);
   ROS_INFO_STREAM("aisle_num " << aisle_num);

   ROS_INFO_STREAM("is obstacle valid "<<obstacleAssociatedWithAisle[aisle_num].is_valid_obstacle);


   int threshold = 3;
   for(int i =0; i<gaps.size();i++) {
      ROS_INFO_STREAM("move to gap:"<< gaps[i]);
      moveToLocation(presetLoc, gaps[i], gantry);                                            //move to preset location
      ros::Duration(3).sleep();                                                              //wait for robot to reach gap
      
      while(!obstacleAssociatedWithAisle[aisle_num].is_valid_obstacle)
       estimateObstacleAttributes(camera,aisle_num);

      if(i == gaps.size()-1) {
          while(true) {
               auto vec = estimateLocation(aisle_num, comp.getClock());                      //estimate Location
             
               //TODO instead of 6.0 will need to get x coordinate of gap
               if(vec[0] == "toward" && std::abs(std::stod(vec[1])+6.0) <= threshold) {            
                  ROS_INFO_STREAM("Location is " <<  location);
                  moveToLocation(presetLoc, location, gantry);
                  gantry.pickPart(my_part);
                  retraceSteps(presetLoc, location, gantry);
                  moveFromLocationToStart(presetLoc, gaps[i], gantry);
                  break;
               }
           }
       }
   }
   ROS_INFO_STREAM("Exited out of Plan and Execute method");
}






void processPart(product prod, GantryControl &gantry, Camera &camera, Competition &comp, bool priority_flag,  bool flip_flag) {
    part my_part, my_part_in_tray, placed_part, actual_part;
    bool foundPart = false;
    bool flippedPart;
    nist_gear::VacuumGripperState armState;
    std::map<std::string,part> detected_parts;

    while(!foundPart) {                                                                                          // poll until we find part

        detected_parts = camera.get_detected_parts()[prod.type];

        ROS_INFO_STREAM("Printing detected part(s");
        for(const auto & parts: detected_parts){
            ROS_INFO_STREAM("parts .firts "<< parts.first);
            ROS_INFO_STREAM("parts .second  "<< parts.second.logicalCameraName);
        }
        for(const auto& parts: detected_parts) {                                                               // search all logical cameras for desired part
            if (parts.second.logicalCameraName == "logical_camera_8" ||
                parts.second.logicalCameraName == "logical_camera_10")                                          // Exclude agv cameras
                continue;


            
            if (prod.type == parts.second.type.c_str()) {
                my_part = parts.second;

                my_part_in_tray.type = prod.type;
                my_part_in_tray.pose = prod.pose;

                ROS_INFO_STREAM("parts .second  "<< parts.second.logicalCameraName);

                //detected_parts.erase(parts.first);
//                camera.removeElement(prod.type, parts.first);
                
                int aisle_num = aisleAssociatedWithPart(my_part);
                if (aisle_num != -1 && obstacleInAisle[aisle_num]) {
                    ROS_INFO_STREAM("In process part- obstacle in aisle");
                    ROS_INFO_STREAM(parts.second.logicalCameraName);
                    ROS_INFO_STREAM(parts.first);
                    planAndExecutePath( prod, my_part, presetLoc, camera, gantry, comp, parts.second.logicalCameraName, aisle_num);
                }
                else {
                    moveToLocation2(presetLoc, my_part, gantry);
//                    gantry.moveToPart(my_part);
                    gantry.pickPart(my_part);
                    moveFromLocationToStart(presetLoc, my_part.logicalCameraName, gantry);
                }


                if (flip_flag && int(my_part_in_tray.pose.orientation.x) == 1) {                                   //Flip part if part needs to be flipped
                    //moveToLocation(presetLoc,prod.agv_id+"_flipped",gantry);                                       //go to location to flip pulley
                    flipPart(gantry, my_part_in_tray, prod);
                    moveToLocation(presetLoc,prod.agv_id+"_flipped",gantry);
                }
                else
                    moveToLocation(presetLoc, prod.agv_id, gantry);                                                //move to desired agv id

                armState = gantry.getGripperState(prod.arm_name);

                //if (!armState.attached)                                                                            //object accidentally fell on the tray
                    //faultyGripper(gantry, prod, camera, my_part_in_tray);
                //else
                    gantry.placePart(my_part_in_tray, prod.agv_id, prod.arm_name);                                 //place part on the tray


                /////////////////////////////////////////////////////////////////////////
                // check pose to see if it matches with what expected
                // otherwise redo it again and again to match expeced pose
                //
                //
                //
                //////////////////////////////////////////////////////////////////////////

                foundPart = true;
                break;
            }

        }
    }
}



//TODO -
void removeReplaceFaultyProductsAndDeliver(Camera &camera,GantryControl &gantry,std::string agv_id,
    ros::ServiceClient &agv1Delivery,ros::ServiceClient &agv2Delivery,std::string shipment_type, Competition &comp) {


    auto logicalCameraName = (agv_id == "agv1") ? "logical_camera_8" : "logical_camera_10";
    auto faulty_poses = camera.get_faulty_poses()[agv_id];
    auto parts = camera.get_agv_detected_parts()[logicalCameraName];
    
    std::vector<product> productsTobeReplaced;   //
    moveToLocation(presetLoc, agv_id, gantry);

    // remove all faulty products
    product prod;                                 
    for(const auto & faulty_pose: faulty_poses) {
      for(const auto & part: parts) {
         if(abs(part.second.pose.position.x - faulty_pose.second.position.x)< 0.2 &&
            abs(part.second.pose.position.y - faulty_pose.second.position.y)< 0.2 &&
            abs(part.second.pose.position.z - faulty_pose.second.position.z)< 0.2 &&
            abs(part.second.pose.orientation.x - faulty_pose.second.orientation.x)< 0.2 &&
            abs(part.second.pose.orientation.y - faulty_pose.second.orientation.y)< 0.2 &&
            abs(part.second.pose.orientation.z - faulty_pose.second.orientation.z)< 0.2 &&
            abs(part.second.pose.orientation.w - faulty_pose.second.orientation.w)< 0.2 ) {

            gantry.pickPart(part.second);
                                  //TODO drop part

            prod.type = part.second.type;
            prod.pose = part.second.pose;
            prod.agv_id = agv_id;
            prod.arm_name = "left_arm";
            productsTobeReplaced.push_back(prod);
         }
      }
    }
    
    //replace all faulty products
    bool delivered = false;
    for(auto product:productsTobeReplaced){                   
      if(compIsAlmostOver(TIMELIMIT_THRESHOLD, comp)){
         agvDeliveryService(agv_id,agv1Delivery,agv2Delivery,shipment_type);
         delivered = true;
         break;
      }else 
         processPart(product, gantry, camera, comp, true,  true);
    }

    if(!delivered)
         agvDeliveryService(agv_id,agv1Delivery,agv2Delivery,shipment_type);
}


//TODO- Sensor blackout
void removeFaultyProduct(Camera &camera, GantryControl &gantry, product &prod) {

    ROS_INFO_STREAM("IN faulty part");
    part temp;

    temp.pose = camera.get_faulty_pose(prod.agv_id);
    temp.type = prod.type;
    moveToLocation(presetLoc, prod.agv_id, gantry);
    gantry.pickPart(temp);

    if (prod.agv_id == "agv2") {
        if (prod.arm_name == "left_arm")
            moveToLocation(presetLoc, "agv2_left_arm_drop", gantry);
        else
            moveToLocation(presetLoc, "agv2_right_arm_drop_flip", gantry);
    } else {
        if (prod.arm_name == "left_arm")
            moveToLocation(presetLoc, "agv1_left_arm_drop", gantry);
        else
            moveToLocation(presetLoc, "agv1_right_arm_drop_flip", gantry);
    }
    gantry.deactivateGripper(prod.arm_name);
    camera.reset_is_faulty();
}



void removeProduct(Camera &camera, GantryControl &gantry, product &prod) {
    ROS_INFO_STREAM("IN faulty part");
    part temp;
    temp.pose = camera.get_faulty_pose(prod.agv_id);
    temp.type = prod.type;
    gantry.pickPart(temp);
    //TODO -chang preset locations
    if(prod.agv_id == "agv2")
        moveToLocation(presetLoc,"agv2_flipped",gantry);
    else
        moveToLocation(presetLoc,"agv1_flipped",gantry);
    gantry.deactivateGripper("left_arm");
}



//TODO ;Add more functionality
void processHPOrder(nist_gear::Order &order,Camera &camera, GantryControl &gantry,Competition &comp, 
                    ros::ServiceClient &agv2Delivery, ros::ServiceClient &agv1Delivery, std::string previous_order_agv_id){

    ROS_INFO_STREAM("Processing HP order");
    product prod;
    bool wanted = true;


    for(int j=0; j<order.shipments.size(); j++) {
        auto ship = order.shipments[j];

        for (int k = 0; k < ship.products.size(); k++) {
            auto product = ship.products[k];


            if (ship.agv_id == previous_order_agv_id)
                PreviousOrderAgvChange = true;


            prod.type = product.type;
            prod.pose = product.pose;
            prod.agv_id = ship.agv_id;
            prod.arm_name = "left_arm";

            if (wanted) {
                processPart(prod, gantry, camera, comp, false, false);
                ros::Duration(3.0).sleep();
                ros::spinOnce();
                ros::spinOnce();
                //if(camera.get_is_faulty(prod.agv_id)) {
                //removeFaultyProduct(camera,gantry,prod);
                //k--;                                                                                         //process product again
                //}
            } else
                removeProduct(camera, gantry, prod);

            moveFromLocationToStart(presetLoc, "start", gantry);
            ROS_INFO("HELLO1");

            if (compIsAlmostOver(TIMELIMIT_THRESHOLD, comp)) {
                agvDeliveryService(prod.agv_id, agv1Delivery, agv2Delivery, order.shipments[j].shipment_type);
                stop_processing = true;
                break;
            }
        }


        if (compIsAlmostOver(TIMELIMIT_THRESHOLD, comp)) {
            agvDeliveryService(prod.agv_id, agv1Delivery, agv2Delivery, order.shipments[j].shipment_type);
            stop_processing = true;
            break;
        } else {
            //TODO sensor blackout
            if (camera.get_faulty_poses()[prod.agv_id].size() > 0)
                removeReplaceFaultyProductsAndDeliver(camera, gantry, prod.agv_id, agv1Delivery, agv2Delivery,
                                                      order.shipments[j].shipment_type, comp);
        }
    }
}


void conveyor(Camera &camera, GantryControl &gantry, product prod){
    ROS_INFO_STREAM("Break beam triggered");
    moveToLocation(presetLoc, "movingPart", gantry);
    nist_gear::VacuumGripperState armState;
    part imgPart, my_part_in_tray;
    my_part_in_tray.type = prod.type;
    my_part_in_tray.pose = prod.pose;
    imgPart.type = prod.type;
    imgPart.pose.orientation.x = 0;
    imgPart.pose.orientation.y = 0;
    imgPart.pose.orientation.z = 0;
    imgPart.pose.orientation.w = 1;

    imgPart.pose.position.x = 0;
    imgPart.pose.position.y = -0.5650; //-0.272441; //-0.5700
    imgPart.pose.position.z = 0.868991; //0.875004;
    ROS_INFO_STREAM("Picking Part from conveynor");
    gantry.pickPart(imgPart);
    moveFromLocationToStart(presetLoc, "pickmovingPart", gantry);

//    if (int(my_part_in_tray.pose.orientation.x) == 1) {                                   //Flip part if part needs to be flipped
//        moveToLocation(presetLoc,prod.agv_id+"_flipped",gantry);                                       //go to location to flip pulley
//        flipPart(gantry, my_part_in_tray, prod);
//    }
//    else
//        moveToLocation(presetLoc, prod.agv_id, gantry);                                                //move to desired agv id
//
//    gantry.placePart(my_part_in_tray, prod.agv_id, prod.arm_name);
//    moveFromLocationToStart(presetLoc, "start", gantry);
}





void detectAislesWithObstacles(Camera &camera) {
    auto eashwar = camera.get_aisle_breakbeam_msgs();
    for(int i=0;i<eashwar.size();i++){
        if(eashwar[i].size()) {
            obstacleInAisle[i] = true;
            ROS_INFO_STREAM(i);
        }
    }
}




void pickPartsFromConveyor(Camera &camera, GantryControl &gantry, product prod, int numParts){
    int count = 0, count1 = 0;
    while(count < numParts){
        if(count == 0 && camera.get_break_beam()){
            conveyor(camera, gantry, prod);
            count += 1;
            moveToLocation(presetLoc, "conveyorPart_"+std::to_string(count), gantry);
            gantry.deactivateGripper(prod.arm_name);
            moveFromLocationToStart(presetLoc,"start", gantry);
            camera.reset_break_beam();
        }
        else if(count > 0){
            count1 = 0;
            ROS_INFO_STREAM("HERE3");
            while(!camera.get_break_beam()){
                count1 = 1;
                ROS_INFO_STREAM("HERE1");
            }
            ROS_INFO_STREAM("HERE");
            if(count1 == 1 && camera.get_break_beam()){
                conveyor(camera, gantry, prod);
                count += 1;
                moveToLocation(presetLoc, "conveyorPart_"+std::to_string(count), gantry);
                gantry.deactivateGripper(prod.arm_name);
                moveFromLocationToStart(presetLoc,"start", gantry);
                camera.reset_break_beam();
            }

        }

    }

}



int main(int argc, char ** argv) {
    ros::init(argc, argv, "rwa3_node");                                                                      //initialize node
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    Competition comp(node);                                                                                  //initialize competition
    comp.init();

    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    GantryControl gantry(node);                                                                              //initialize gantry
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);

    ROS_INFO_STREAM("TTTTTTT3");

    ros::ServiceClient agv2Delivery = node.serviceClient<nist_gear::AGVControl>("/ariac/agv2");              //initialize agvDelivery
    ros::ServiceClient agv1Delivery = node.serviceClient<nist_gear::AGVControl>("/ariac/agv1");

    ROS_INFO_STREAM("TTTTT5");

    Camera camera;
    camera.init(node);                                                                                       //initialize camera


    initWayPoints(presetLoc, gantry);                                                                        //initialize waypoints

    ROS_INFO_STREAM("TTTTTTT1");

   
    HighPriorityOrderInitiated  = false;                                                                     //setting up flag
    stop_processing = false;


    obstacle temp;                                                                                           //Initializing obstacle
    obstacleAssociatedWithAisle = std::vector<obstacle> (4,temp);                                             


    ROS_INFO_STREAM("TTTTTTT2");

    auto orders = comp.getOrders();                                                                          //Wait for order
    while(orders.size()<=0)
        orders = comp.getOrders();


    product prod;                                                                                            //Setting up product


    ros::Duration(1.0).sleep();                                                                              //wait for sometime and
    detectAislesWithObstacles(camera);                                                                       //determine obstacles in Aisles
    

    ConveyorFlag = false;
    int numPickParts = 4;
    

    //TODO reduce computation 
    std::cout << "estimating obstacles parameters" << std::endl;
    //while(true){
      
      //if(obstacleInAisle[2]){
        //estimateObstacleAttributes(camera,2);
        //if(obstacleAssociatedWithAisle[2].is_valid_obstacle)
            //break;
      //}
    //}
    
    
    obstacleAssociatedWithAisle[2].is_valid_obstacle= true;
    obstacleAssociatedWithAisle[2].wait_time= 7;
    obstacleAssociatedWithAisle[2].move_time= 9;
    obstacleAssociatedWithAisle[2].time_stamp1= 9;

    obstacleAssociatedWithAisle[1].is_valid_obstacle= true;
    obstacleAssociatedWithAisle[1].wait_time= 7;
    obstacleAssociatedWithAisle[1].move_time= 9;
    obstacleAssociatedWithAisle[1].time_stamp1= 25;


    std::cout << "finished estimating obstacle parameters" << std::endl;

    for(int i = 0; i< orders.size(); i++){
        auto order = orders[i];
        
        if(stop_processing) break;

        for(int j=0; j<order.shipments.size(); j++){
            auto ship = order.shipments[j];

            if(stop_processing) break;


            for (int k=0; k<ship.products.size(); k++){
                auto product = ship.products[k];

                ROS_INFO_STREAM(product.type);
                prod.type = product.type;
                prod.pose = product.pose;
                prod.agv_id = ship.agv_id;
                prod.arm_name = "left_arm";


                //process parts on conveyor belt if parts are detected
//                if(!ConveyorFlag && camera.get_conveyor_detected_parts().size()>0) {
//                    ROS_INFO_STREAM("processing conveyor belt");
//                    pickPartsFromConveyor(camera, gantry, prod, numPickParts);
//                    ConveyorFlag = true;
//                    camera.reset_conveyor_logical_camera();
//                }


                // TODO - make high priority order checker more robust
                if(comp.getOrders().size()>1 && !HighPriorityOrderInitiated){
                    processHPOrder(comp.getOrders()[1],camera,gantry, comp,agv2Delivery, agv1Delivery, ship.agv_id);
                    ROS_INFO_STREAM("I am here1243");
                    HighPriorityOrderInitiated = true;
                    ROS_INFO_STREAM(comp.getOrders().size());
                    ROS_INFO_STREAM(HighPriorityOrderInitiated);

                    if(stop_processing == true) break;


                    if(PreviousOrderAgvChange == true)                           //does the new order modify to the current agv  
                       k = 0;                                                    //Build order from scratch
                    else
                       k--;                                                      //build previous part
                }
                else
                    processPart(prod, gantry, camera, comp, false, true);        //Remove flip_flag after degugging


                //TODO - Make checker for faulty more robust
                ROS_INFO_STREAM("heere 1");
                ros::Duration(3.0).sleep();
                ros::spinOnce();
                ros::spinOnce();


                //if(camera.get_is_faulty(prod.agv_id)) {                                              
                    //ROS_INFO_STREAM("FAULTY");
                    //removeFaultyProduct(camera,gantry,prod);                       // remove product
                    //k--;                                                           // process product again
                    //camera.reset_is_faulty();
                //}
                ROS_INFO_STREAM("heere 2");
                moveFromLocationToStart(presetLoc,"start",gantry);
                ROS_INFO_STREAM("heere x");

                if(compIsAlmostOver(TIMELIMIT_THRESHOLD, comp)){
                    agvDeliveryService(prod.agv_id,agv1Delivery,agv2Delivery,order.shipments[j].shipment_type);
                    stop_processing = true;
                    break;
                } 
            }
            ROS_INFO_STREAM("herex1");
            auto faulty_poses = camera.get_faulty_poses();
            

            if(compIsAlmostOver(TIMELIMIT_THRESHOLD, comp)) {
                 agvDeliveryService(prod.agv_id,agv1Delivery,agv2Delivery,order.shipments[j].shipment_type);
                 stop_processing = true;
                 break;
            } else{
                //TODO sensor blackout
                if(faulty_poses[prod.agv_id].size() > 0)
                    removeReplaceFaultyProductsAndDeliver(camera,gantry,prod.agv_id,agv1Delivery,agv2Delivery,order.shipments[j].shipment_type, comp);
            }
        }
        ROS_INFO_STREAM("here3");
    }

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}





