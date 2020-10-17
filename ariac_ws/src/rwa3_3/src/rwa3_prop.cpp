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
    std::string productType;
    geometry_msgs::Pose productPose;
    bool foundPart = false;
    part my_part;
    std::map<std::string,PresetLocation> presetLoc;
    presetLoc["logical_camera_2"] = gantry.bin16_;
    presetLoc["logical_camera_6"] = gantry.bin13_;
    part my_part_in_tray;
//    presetLoc["logical_camera_11"] = gantry.bin16_;
//    presetLoc["logical_camera_14"] = gantry.shelf5_2_;

    
    //--You should receive the following information from a camera
    
    //gantry.goToPresetLocation(gantry.bin3_);

    for(auto order: orders){  // @TODO May need to modify here

        for(auto ship : order.shipments){
            agvId = ship.agv_id;

            for (auto product : ship.products){
                productType = product.type;
                productPose = product.pose;

//                auto detected_parts = camera.get_detected_parts();
//                while(detected_parts.size()<=0)                    //Polling for detected_part
//                    detected_parts = camera.get_detected_parts();
                foundPart = false;
                while(foundPart != true){
                    for(auto const& parts: camera.get_detected_parts()){

                        for(auto part: parts.second)
                            if(productType == part.type.c_str()){
                                my_part = part;
                                foundPart = true;
                                gantry.goToPresetLocation(presetLoc[productType]);
                                gantry.pickPart(my_part);
                                gantry.goToPresetLocation(gantry.start_);
//                                if(agvId == "any" || agvId == "agv2") {
//                                    gantry.placePart(part_in_tray, "agv2");
//                                }
//                                else {
//                                    gantry.placePart(part_in_tray, "agv1");
//                                }
                                my_part_in_tray.type = productType;
                                my_part_in_tray.pose = product.pose;
                                gantry.placePart(my_part_in_tray, "agv2");
                            }
                    }
                }
            }

        }

        



            
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

        gantry.goToPresetLocation(gantry.shelf5_1_);
        gantry.goToPresetLocation(gantry.shelf5_2_);

        //--Go pick the part
        //gantry.pickPart(my_part);

        //--Go place the part
        //gantry.placePart(part_in_tray, "agv2");
    }

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}


