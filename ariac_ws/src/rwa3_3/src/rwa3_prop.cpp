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

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
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


// @TODO may be modified in future
void set_preset_loc(std::map<std::string,std::vector<PresetLocation>> &presetLoc, GantryControl &gantry){
    presetLoc["logical_camera_2"] = {gantry.bin16_};
    presetLoc["logical_camera_6"] = {gantry.bin13_};
    presetLoc["logical_camera_11"] = {gantry.shelf5_1_, gantry.shelf5_2_};
    presetLoc["logical_camera_14"] = {gantry.shelf5_1_, gantry.shelf5_2_};
}


// @TODO may be modified in future
void moveToPresetLocation(std::map<std::string,std::vector<PresetLocation>> &presetLoc, std::string location,GantryControl &gantry){
  auto vec = presetLoc[location];
  for(auto waypoint :vec){
      gantry.goToPresetLocation(waypoint);
  }
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
    auto orders = comp.getOrders();   //Polling for order
    while(orders.size()<=0)  
           orders = comp.getOrders();
     
    
    Camera camera;
    camera.init(node);

    // Parameters
    std::string agvId;
    bool foundPart = false;
    int index = 0;
    part my_part, my_part_in_tray;
    std::map<std::string,std::vector<part>> detected_parts;
    std::map<std::string,std::vector<PresetLocation>> presetLoc;
    set_preset_loc(presetLoc, gantry);

    
    for(auto order: orders){  // @TODO May need to modify here

        for(auto ship : order.shipments){
            agvId = ship.agv_id;

            ROS_INFO_STREAM(agvId);

            for (auto product : ship.products){

                foundPart = false;
                while(foundPart != true){

                    detected_parts = camera.get_detected_parts();
                    for(auto const& parts: detected_parts) {

                        ROS_INFO_STREAM(parts.first);
                        
                        index = 0;
                        for(auto part: parts.second){
                            ROS_INFO_STREAM("I am heree");
                            if(product.type == part.type.c_str()){
                                my_part = part;
                                foundPart = true;

                                ROS_INFO_STREAM(product.type);
                                ROS_INFO_STREAM("I am heree2");
                                ROS_INFO_STREAM(parts.first);

                                my_part_in_tray.type = product.type;
                                my_part_in_tray.pose = product.pose;

                                moveToPresetLocation(presetLoc,parts.first,gantry);

                                gantry.pickPart(my_part);
                                gantry.placePart(my_part_in_tray, "agv2");

                                camera.remove_part(parts.first, index);
                                index++;

                                break;
                            }
                          
                        }
                        
                    }
                    ros::spinOnce();
                }

             }
         }
     }


    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}





    //gantry.goToPresetLocation(gantry.bin3_);

//                auto detected_parts = camera.get_detected_parts();
//                while(detected_parts.size()<=0)                    //Polling for detected_part
//                    detected_parts = camera.get_detected_parts();




// @TODO May need to modify here

//part_in_tray.type = sm.products[0].type;
//part_in_tray.pose.position.x = sm.products[0].pose.position.x;
//part_in_tray.pose.position.x = sm.products[0].pose.position.y;
//part_in_tray.pose.position.x = sm.products[0].pose.position.z;
//part_in_tray.pose.orientation.x = sm.products[0].pose.orientation.x;
//part_in_tray.pose.orientation.y = sm.products[0].pose.orientation.y;
//part_in_tray.pose.orientation.z = sm.products[0].pose.orientation.z;
//part_in_tray.pose.orientation.w = sm.products[0].pose.orientation.w;


//auto agv = sm.agvid;

//--get pose of part in tray from /ariac/orders

//part_in_tray.type = sm.products[0].type;
//part_in_tray.pose.position.x = sm.products[0].pose.position.x;
//part_in_tray.pose.position.x = sm.products[0].pose.position.y;
//part_in_tray.pose.position.x = sm.products[0].pose.position.z;
//part_in_tray.pose.orientation.x = sm.products[0].pose.orientation.x;
//part_in_tray.pose.orientation.y = sm.products[0].pose.orientation.y;
//part_in_tray.pose.orientation.z = sm.products[0].pose.orientation.z;
//part_in_tray.pose.orientation.w = sm.products[0].pose.orientation.w;

//gantry.goToPresetLocation(gantry.shelf5_1_);
//gantry.goToPresetLocation(gantry.shelf5_2_);

//--Go pick the part
//gantry.pickPart(my_part);

//--Go place the part
//gantry.placePart(part_in_tray, "agv2");

