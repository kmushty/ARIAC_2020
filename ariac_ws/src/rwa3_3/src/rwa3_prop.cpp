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


void set_preset_loc(std::map<std::string,std::vector<PresetLocation>> &presetLoc, GantryControl &gantry){
    presetLoc["logical_camera_2"] = {gantry.bin16_};
    presetLoc["logical_camera_6"] = {gantry.bin13_};
    presetLoc["logical_camera_11"] = {gantry.shelf5_1_, gantry.shelf5_2_, gantry.shelf5_3_};
    presetLoc["logical_camera_14"] = {gantry.shelf5_1_, gantry.shelf5_2_, gantry.shelf5_4_};
    presetLoc["start"] = {gantry.start_};
    presetLoc["agv2"] = {gantry.agv2_};
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
        gantry.goToPresetLocation(gantry.start_);
    }
    else{
        for(int i=vec.size()-2; i>=0;i--){
            gantry.goToPresetLocation(vec[i]);
        }
        gantry.goToPresetLocation(gantry.start_);
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
    //std::map<std::string,std::vector<part>> detected_parts;
    std::map<std::string,part> detected_parts;
    
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
                        if (parts.first == "logical_camera_8" || parts.first == "logical_camera_10" )    // agv cameras
                            continue;
                        
                        index = 0;
                        if(product.type == parts.second.type.c_str()){
                              my_part = parts.second;
                              foundPart = true;

                              my_part_in_tray.type = product.type;
                              my_part_in_tray.pose = product.pose;

                              moveToPresetLocation(presetLoc,parts.first,gantry);

                              gantry.pickPart(my_part);
                              moveToStartLocation(presetLoc,parts.first,gantry);

                              if(int(my_part_in_tray.pose.orientation.x) == 1){
                                  gantry.goToPresetLocation(gantry.go_to_flipped_pulley);
                                  gantry.activateGripper("right_arm");
                                  gantry.deactivateGripper("left_arm");
                                  gantry.goToPresetLocation(gantry.agv2_flipped_);
                                  my_part_in_tray.pose.orientation.x = 0;
                                  my_part_in_tray.pose.orientation.w = 1;
                                
                                  gantry.placeFlippedPart(my_part_in_tray,"agv2","right_arm");
                              }else
                                  gantry.placePart(my_part_in_tray, "agv2");
                            
                              moveToStartLocation(presetLoc,"start",gantry);


                              ros::spinOnce();
                              ros::spinOnce();
                              if(camera.get_is_faulty()) {
                                  ROS_INFO_STREAM("removing faulty parts");
                                  
                                  part temp;

                                  temp.pose = camera.get_faulty_pose();
                                 // temp.type = "pulley_part_red";
                                  temp.type = my_part.type;
                                  ROS_INFO_STREAM(temp.pose);

                                  moveToPresetLocation(presetLoc,"agv2",gantry);
                                  ROS_INFO_STREAM("no nononononononoons");
                                  gantry.pickPart(temp);
                                  ROS_INFO_STREAM("Hahahahahahaha");
                                  moveToStartLocation(presetLoc,"start",gantry);

                                  //gantry.placePart(temp);
                                  //gantry.dropPart(temp);
                                  //gantry.presetArmLocation(gantry.start_);
                                  foundPart = false;
                                  camera.reset_is_faulty();
                                  gantry.deactivateGripper("left_arm");
                              }

                              //camera.remove_part(parts.first, index);
                              //index++;

                              break;
                          }

                        }

                    ros::spinOnce();
                }

             }
         }
     }
    
     //ROS_INFO_STREAM("pRINT I AM HERE");
       
     //ros::spinOnce();
     //while(camera.get_is_faulty()) {
        //ROS_INFO_STREAM("removing faulty parts");
        
        //part temp;

        //temp.pose = camera.get_faulty_pose();
        //temp.type = "pulley_part_red";
        //temp.type = "disk_part_blue";
        //ROS_INFO_STREAM(temp.pose);

        //moveToPresetLocation(presetLoc,"agv2",gantry);
        //ROS_INFO_STREAM("no nononononononoons");
        //gantry.pickPart(temp);
        //ROS_INFO_STREAM("Hahahahahahaha");
        //moveToStartLocation(presetLoc,"start",gantry);

        //gantry.placePart(temp);
        //gantry.dropPart(temp);
        //gantry.presetArmLocation(gantry.start_);
        //camera.reset_is_faulty();
        //gantry.deactivateGripper("left_arm");
        //ros::spinOnce();
    //}

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




                //while(!foundPart){                                                                      // poll until we find part

                    //detected_parts = camera.get_detected_parts();
                    //for(auto const& parts: detected_parts) {
                        //if (parts.first == "logical_camera_8" || parts.first == "logical_camera_10")   // agv cameras
                            //continue;
                        
                        //index = 0;
                        //for(auto pt: parts.second){
                            //if(product.type == pt.type.c_str()){
                                //my_part = pt;
                                //foundPart = true;

                                //my_part_in_tray.type = product.type;
                                //my_part_in_tray.pose = product.pose;

                                //moveToPresetLocation(presetLoc,parts.first,gantry);

                                //gantry.pickPart(my_part);
                                //moveToStartLocation(presetLoc,parts.first,gantry);
                                //gantry.placePart(my_part_in_tray, "agv2");
                                
                                //ROS_INFO_STREAM(camera.get_is_faulty());
                                //while(!camera.get_is_faulty()); //only exits if camera is faulty
                                //if(camera.get_is_faulty()) {
                                     //ros::Duration(3.0).sleep();
                                    //ROS_INFO_STREAM("I hate coding");
                                    //camera.reset_is_faulty();
                                    //ROS_INFO_STREAM("Part is faulty");
                                    //gantry.presetArmLocation(gantry.start_);
                                    
                                    //part temp; 
                                    //temp.pose = camera.get_faulty_pose();
                                    //ROS_INFO_STREAM(temp.pose);
                                    //gantry.pickPart(temp);
                                    
                                //}

                                //moveToStartLocation(presetLoc,"start",gantry);

                                //camera.remove_part(parts.first, index);
                                //index++;

                                //break;
                            //}

                        //}

                    //}
                    //ros::spinOnce();
                //}
