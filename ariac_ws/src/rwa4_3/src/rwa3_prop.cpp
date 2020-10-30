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
#include <tf2/transform_datatypes.h>

#include "competition.h"
#include "utils.h"
#include "gantry_control.h"
#include "camera.h"

#include <tf2/LinearMath/Quaternion.h>


void set_preset_loc(std::map<std::string,std::vector<PresetLocation>> &presetLoc, GantryControl &gantry){
    presetLoc["logical_camera_2"] = {gantry.bin16_};
    presetLoc["logical_camera_6"] = {gantry.bin13_};
    presetLoc["logical_camera_11"] = {gantry.shelf5_1_, gantry.shelf5_2_, gantry.shelf5_3_};
    presetLoc["logical_camera_14"] = {gantry.shelf5_1_, gantry.shelf5_4_, gantry.shelf5_5_};
    // faulty gripper
    presetLoc["logical_camera_12"] = {gantry.shelf8_1_, gantry.shelf8_2_, gantry.shelf8_3_};
    presetLoc["logical_camera_15"] = {gantry.shelf8_1_, gantry.shelf8_2_, gantry.shelf8_3_};

    presetLoc["start"] = {gantry.start_};
    presetLoc["agv2"] = {gantry.agv2_};
    presetLoc["agv2_faultyG"] = {gantry.agv2_faultyG_};
    // presetLoc["Drop"] = {gantry.drop_};
}


// @TODO may be modified in future
void moveToPresetLocation(std::map<std::string,std::vector<PresetLocation>> &presetLoc, std::string location,GantryControl &gantry){
  auto vec = presetLoc[location];
  if(vec.size() == 1){
        gantry.goToPresetLocation(vec[0]);
  }
  for(auto waypoint :vec){
      gantry.goToPresetLocation(waypoint);
  }
}

void moveToStartLocation(std::map<std::string,std::vector<PresetLocation>> &presetLoc, std::string location,GantryControl &gantry){
    auto vec = presetLoc[location];
    if(vec.size() == 1){
        gantry.goToPresetLocation(vec[0]);
        if(location != "start")
            gantry.goToPresetLocation(gantry.start_);
    }
    else{
        for(int i=vec.size()-1; i>=0;i--){
            gantry.goToPresetLocation(vec[i]);
        }
        gantry.goToPresetLocation(gantry.start_);
    }

}

void agvDeliveryService(ros::ServiceClient &agvDelivery){
    // AGV Delivering the parts
    if (!agvDelivery.exists()) {
        ROS_INFO("[AGV][startAGV] Waiting for thE AGV to start...");
        agvDelivery.waitForExistence();
        ROS_INFO("[AGV][startAGV] AGV is now ready.");
    }
    ROS_INFO("[AGV][startAGV] Requesting AGV start...");
    nist_gear::AGVControl srv;
    srv.request.shipment_type =  "order_0_shipment_0";
    agvDelivery.call(srv);
   
    
    while(!srv.response.success) {
        ROS_INFO_ONCE("AGV delivering the parts");
    }
    ROS_INFO("%s",srv.response.message.c_str());

    ROS_INFO("AGV delivery successful");
}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "rwa3_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    Competition comp(node);
    comp.init();

    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    GantryControl gantry(node);
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);
   
     
    /***************************************
     ************New Modifications *********
    ****************************************/
    //--2-Look for parts in this order
    //--We go to this bin because a camera above
    //--this bin found one of the parts in the order

    //--1-Read order
    
    ros::ServiceClient agvDelivery = node.serviceClient<nist_gear::AGVControl>("/ariac/agv2");

    auto orders = comp.getOrders();   //Polling for order
    while(orders.size()<=0)
           orders = comp.getOrders();


    Camera camera;
    camera.init(node);



    // Parameters
    std::string agvId;
    bool foundPart = false;
    int index = 0;
    part my_part, my_part_in_tray, placed_part, actual_part;
    std::map<std::string,part> detected_parts;
    float dis, dotProduct;
    double yaw, pitch, roll;
    double yaw1, pitch1, roll1;
    double angleDiff;
    bool flippedPart;
    nist_gear::VacuumGripperState armState;
    std::map<std::string,std::vector<PresetLocation>> presetLoc;
    set_preset_loc(presetLoc, gantry);


    for(auto order: orders){  

        for(auto ship : order.shipments){
            agvId = ship.agv_id;

            for (auto product : ship.products){

                foundPart = false; 

                while(!foundPart){                                                                       // poll until we find part

                    detected_parts = camera.get_detected_parts();

                    for(auto const& parts: detected_parts) {
                        if (parts.first == "logical_camera_8" || parts.first == "logical_camera_10")    // agv cameras
                            continue;

                        index = 0;
                        flippedPart = false;
                        if (product.type == parts.second.type.c_str()) {
                            my_part = parts.second;
                            foundPart = true;

                            my_part_in_tray.type = product.type;
                            my_part_in_tray.pose = product.pose;
                            ROS_INFO_STREAM(parts.first);
                            moveToPresetLocation(presetLoc, parts.first, gantry);
                            ROS_INFO("here");
                            gantry.pickPart(my_part);
                            ROS_INFO("here1");
                            moveToStartLocation(presetLoc, parts.first, gantry);
                            ROS_INFO("here2");

                            if (int(my_part_in_tray.pose.orientation.x) == 1) {
                                gantry.goToPresetLocation(gantry.go_to_flipped_pulley_);
                                gantry.activateGripper("right_arm");
                                gantry.deactivateGripper("left_arm");
                                gantry.goToPresetLocation(gantry.agv2_flipped_);
                                my_part_in_tray.pose.orientation.x = 0;
                                my_part_in_tray.pose.orientation.w = 1;
                                flippedPart = true;

//                                gantry.placeFlippedPart(my_part_in_tray, agvId, "right_arm");
                            }
//                            else
//                                gantry.placePart(my_part_in_tray, agvId, "left_arm");

//                            moveToStartLocation(presetLoc, "start", gantry);

//
                            moveToPresetLocation(presetLoc, agvId, gantry);
                            //TODO Determining preset location for agv1
                            // Faulty Gripper Implementation
                            if (agvId == "agv2") {
                                placed_part = camera.get_detected_parts()["logical_camera_10"];
                                ROS_INFO_STREAM(placed_part.pose);
                            }
                            else {
                                placed_part = camera.get_detected_parts()["logical_camera_8"];
                                ROS_INFO_STREAM(placed_part.pose);
                            }

                            actual_part.type = placed_part.type;
                            actual_part.pose = gantry.getTargetWorldPose(my_part_in_tray.pose, agvId, "left_arm");
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
                                ROS_INFO_STREAM(!armState.attached);
                                moveToPresetLocation(presetLoc, agvId + "_faultyG", gantry);
                                ros::Duration(5).sleep();
                                ROS_INFO("Faulty Gripper Detected");
                                ROS_INFO("Re-picking and replacing the part");
                                gantry.pickPart(placed_part);
                                moveToPresetLocation(presetLoc, agvId, gantry);
                                gantry.placePart(actual_part, agvId, "left_arm");
                            }
                            else{
                                if(flippedPart)
                                    gantry.placeFlippedPart(my_part_in_tray, agvId, "right_arm");
                                else
                                    gantry.placePart(my_part_in_tray, agvId, "left_arm");
                            }

                            // Faulty gripper implemetation complete - Testing pending


                            ros::spinOnce();
                            ros::spinOnce();
                            if (camera.get_is_faulty()) {
                                ROS_INFO_STREAM("removing faulty parts");

                                part temp;

                                temp.pose = camera.get_faulty_pose();
                                // temp.type = "pulley_part_red";
                                temp.type = my_part.type;
                                ROS_INFO_STREAM(temp.pose);

                                moveToPresetLocation(presetLoc, agvId, gantry);
                                ROS_INFO_STREAM("no nononononononoons");
                                gantry.pickPart(temp);
                                ROS_INFO_STREAM("Hahahahahahaha");
                                moveToStartLocation(presetLoc, agvId, gantry);
                                foundPart = false;
                                camera.reset_is_faulty();
                                gantry.deactivateGripper("left_arm");
                            }
                            moveToStartLocation(presetLoc, "start", gantry);
                            break;
                        }
                    }
                    ros::spinOnce();
                }

             }
         }
     }
    
    agvDeliveryService(agvDelivery);
    

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}


