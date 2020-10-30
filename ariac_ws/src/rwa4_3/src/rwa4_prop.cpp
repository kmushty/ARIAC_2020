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


bool HighPriorityOrderInitiated;
std::map<std::string,std::vector<PresetLocation>> presetLoc;



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


void initWayPoints(std::map<std::string,std::vector<PresetLocation>> &presetLoc, GantryControl &gantry){

    presetLoc["logical_camera_2"] = {gantry.bin16_};                                                          // waypoints from start position position to logical_camera
    presetLoc["logical_camera_6"] = {gantry.bin13_};
    presetLoc["logical_camera_11"] = {gantry.shelf5_1_, gantry.shelf5_2_, gantry.shelf5_3_};
    // faulty gripper
    presetLoc["logical_camera_12"] = {gantry.shelf8_1_, gantry.shelf8_2_, gantry.shelf8_3_};
    presetLoc["logical_camera_15"] = {gantry.shelf8_1_, gantry.shelf8_2_, gantry.shelf8_3_};


    presetLoc["start"] = {gantry.start_};                                                                     // useful presetloc
    presetLoc["agv2"] = {gantry.agv2_};
    presetLoc["agv2_faultyG"] = {gantry.agv2_faultyG_};
}



void moveFromStartToLocation(std::map<std::string,std::vector<PresetLocation>> &presetLoc, std::string location,GantryControl &gantry){
    auto vec = presetLoc[location];
    for(auto waypoint :vec)
        gantry.goToPresetLocation(waypoint);
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

void faultyGripper(part placed_part, part actual_part,GantryControl &gantry){
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
    moveFromStartToLocation(presetLoc, "agv2_faultyG", gantry);
    ros::Duration(5).sleep();
    ROS_INFO("Faulty Gripper Detected");
    ROS_INFO("Re-picking and replacing the part");
    gantry.pickPart(placed_part);
    moveFromStartToLocation(presetLoc, "agv2", gantry);
    gantry.placePart(actual_part, "agv2", "left_arm");
}




void processPart(product prod, GantryControl &gantry, Camera &camera, bool priority_flag,  bool flip_flag) {
    part my_part, my_part_in_tray, placed_part, actual_part;
    bool foundPart = false;
    bool flippedPart;
    nist_gear::VacuumGripperState armState;
    std::map<std::string,part> detected_parts;

    while(!foundPart) {                                                                                                      // poll until we find part
        detected_parts = camera.get_detected_parts();

        for(auto const& parts: detected_parts) {                                                                          // search all logical cameras for desired part
            if (parts.first == "logical_camera_8" ||
                parts.first == "logical_camera_10")                                // Exclude agv cameras
                continue;

            flippedPart = false;
            if (prod.type == parts.second.type.c_str()) {
                my_part = parts.second;

                my_part_in_tray.type = prod.type;
                my_part_in_tray.pose = prod.pose;

                moveFromStartToLocation(presetLoc, parts.first, gantry);
                gantry.pickPart(my_part);
                moveFromLocationToStart(presetLoc, parts.first, gantry);


                if (flip_flag && int(my_part_in_tray.pose.orientation.x) == 1) {
                    gantry.goToPresetLocation(gantry.go_to_flipped_pulley_);
                    gantry.activateGripper("right_arm");
                    gantry.deactivateGripper("left_arm");
                    gantry.goToPresetLocation(gantry.agv2_flipped_);
                    my_part_in_tray.pose.orientation.x = 0;
                    my_part_in_tray.pose.orientation.w = 1;
                    flippedPart = true;

//                    gantry.placeFlippedPart(my_part_in_tray,"agv2","right_arm");
                }
//                else
//                    gantry.placePart(my_part_in_tray, "agv2", "left_arm");
                moveFromStartToLocation(presetLoc, "agv2", gantry);

                /* If both the agvs are considered
//                if (agvId == "agv2") {
//                    placed_part = camera.get_detected_parts()["logical_camera_10"];
//                    ROS_INFO_STREAM(placed_part.pose);
//                }
//                else {
//                    placed_part = camera.get_detected_parts()["logical_camera_8"];
//                    ROS_INFO_STREAM(placed_part.pose);
//                }
                 */
                placed_part = camera.get_detected_parts()["logical_camera_10"];
                actual_part.type = placed_part.type;
                actual_part.pose = gantry.getTargetWorldPose(my_part_in_tray.pose, "agv2", "left_arm");
                if (flippedPart)
                    armState = gantry.getGripperState("right_arm");
                else
                    armState = gantry.getGripperState("left_arm");

                if (!armState.attached) {
                    faultyGripper(placed_part, actual_part, gantry);
                } else {
                    if (flippedPart)
                        gantry.placeFlippedPart(my_part_in_tray, "agv2", "right_arm");
                    else
                        gantry.placePart(my_part_in_tray, "agv2", "left_arm");
                }


                foundPart = true;
                break;

            }

        }
    }
}



void removeFaultyProduct(Camera &camera, GantryControl &gantry, product &prod) {
    ROS_INFO_STREAM("IN faulty part");
    part temp;

    temp.pose = camera.get_faulty_pose();
    temp.type = prod.type;
    gantry.pickPart(temp);
    gantry.deactivateGripper("left_arm");
}



void removeProduct(Camera &camera, GantryControl &gantry, product &prod) {
    ROS_INFO_STREAM("IN faulty part");
    part temp;
    temp.pose = camera.get_faulty_pose();
    temp.type = prod.type;
    gantry.pickPart(temp);
    gantry.deactivateGripper("left_arm");
}



void processHPOrder(nist_gear::Order &order,Camera &camera, GantryControl &gantry){
    ROS_INFO_STREAM("Processing HP order");
    product prod;
    bool wanted = true;

    for(int j=0; j<=order.shipments.size(); j++){
        auto ship = order.shipments[j];

        for (int k=1; j<ship.products.size(); k++){
            auto product = ship.products[k];


            prod.type = product.type;
            prod.pose = product.pose;

            if(wanted){
                processPart(prod, gantry, camera, false, false);
                ros::spinOnce();
                ros::spinOnce();
                if(camera.get_is_faulty()) {
                    removeFaultyProduct(camera,gantry,prod);
                    k--;                                                                                         //process product again
                }
            }else
                removeProduct(camera,gantry,prod);

            moveFromLocationToStart(presetLoc,"start",gantry);
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


    ros::ServiceClient agvDelivery = node.serviceClient<nist_gear::AGVControl>("/ariac/agv2");               //initialize agvDelivery


    Camera camera;
    camera.init(node);                                                                                       //initialize camera


    initWayPoints(presetLoc, gantry);                                                                        //initialize waypoints


    HighPriorityOrderInitiated  = false;                                                                     //setting up flag


    auto orders = comp.getOrders();                                                                          //Wait for order
    while(orders.size()<=0)
        orders = comp.getOrders();


    product prod;

    for(int i = 0; i< orders.size(); i++){
        auto order = orders[i];


        for(int j=0; j<order.shipments.size(); j++){
            auto ship = order.shipments[j];


            for (int k=0; j<ship.products.size(); k++){
                auto product = ship.products[k];

                ROS_INFO_STREAM(product.type);
                prod.type = product.type;
                prod.pose = product.pose;

                //High priority Order
                if(comp.getOrders().size()<=1)
                    processPart(prod, gantry, camera, false, false);
                else
                    processHPOrder(comp.getOrders()[1],camera,gantry);

                ros::spinOnce();
                ros::spinOnce();
                if(camera.get_is_faulty()) {
                    removeFaultyProduct(camera,gantry,prod);
                    k--;                                                                                                 //process product again
                }
                moveFromLocationToStart(presetLoc,"start",gantry);

            }
        }
        agvDeliveryService(agvDelivery);
    }

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}





