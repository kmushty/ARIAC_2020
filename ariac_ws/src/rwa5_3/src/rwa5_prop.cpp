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



void agvDeliveryService(ros::ServiceClient &agvDelivery, std::string shipmentType){
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

    presetLoc["logical_camera_2"] = {gantry.bin16_};                                                          // waypoints from start position position to logical_camera
    presetLoc["logical_camera_6"] = {gantry.bin13_};
    presetLoc["logical_camera_11"] = {gantry.shelf5_1_, gantry.shelf5_2_, gantry.shelf5_3_};
    presetLoc["logical_camera_12"] = {gantry.shelf8_1_, gantry.shelf8_2_, gantry.shelf8_3_};
    presetLoc["logical_camera_14"] = {gantry.shelf5_1_, gantry.shelf5_4_, gantry.shelf5_5_};
    presetLoc["logical_camera_15"] = {gantry.shelf8_1_, gantry.shelf8_2_, gantry.shelf8_3_};
    presetLoc["logical_camera_13"] = {gantry.shelf11_1_, gantry.shelf11_2_, gantry.shelf11_3_};
    presetLoc["logical_camera_16"] = {gantry.shelf11_1_, gantry.shelf11_2_, gantry.shelf11_3_};

    presetLoc["start"] = {gantry.start_};                                                                     // useful presetloc
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
}



void moveToLocation(std::map<std::string,std::vector<PresetLocation>> &presetLoc, std::string location,GantryControl &gantry){
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

    // TODO: Looping through the logical cameras
    if (prod.agv_id == "agv2")
        placed_part = camera.get_detected_parts()["logical_camera_10"];                 // loop through to get placed part
    else
        placed_part = camera.get_detected_parts()["logical_camera_8"];

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
    moveToLocation(presetLoc,"flipped_pulley_"+prod.agv_id,gantry);                                                    //set arms to desired configuration to flip
    gantry.activateGripper("right_arm");                                                                   //activate and deactivate gripper
    gantry.deactivateGripper("left_arm");
    moveToLocation(presetLoc,prod.agv_id+"_right_arm_drop_flip",gantry);
    my_part_in_tray.pose.orientation.x = 0;                                                                //modify pose orientation
    my_part_in_tray.pose.orientation.w = 1;

    prod.arm_name = "right_arm";
}

void processPart(product prod, GantryControl &gantry, Camera &camera, bool priority_flag,  bool flip_flag) {
    part my_part, my_part_in_tray, placed_part, actual_part;
    bool foundPart = false;
    bool flippedPart;
    nist_gear::VacuumGripperState armState;
    std::map<std::string,part> detected_parts;

    while(!foundPart) {                                                                                          // poll until we find part
        detected_parts = camera.get_detected_parts();

        for(auto const& parts: detected_parts) {                                                                 // search all logical cameras for desired part
            if (parts.first == "logical_camera_8" ||
                parts.first == "logical_camera_10")                                                              // Exclude agv cameras
                continue;

            if (prod.type == parts.second.type.c_str()) {
                my_part = parts.second;

                my_part_in_tray.type = prod.type;
                my_part_in_tray.pose = prod.pose;

                moveToLocation(presetLoc, parts.first, gantry);
                gantry.pickPart(my_part);
                moveFromLocationToStart(presetLoc, parts.first, gantry);


                if (flip_flag && int(my_part_in_tray.pose.orientation.x) == 1) {                                   //Flip part if part needs to be flipped
                    //moveToLocation(presetLoc,prod.agv_id+"_flipped",gantry);                                       //go to location to flip pulley
                    flipPart(gantry, my_part_in_tray, prod);
                    moveToLocation(presetLoc,prod.agv_id+"_flipped",gantry);
                }
                else
                    moveToLocation(presetLoc, prod.agv_id, gantry);                                                //move to desired agv id

                armState = gantry.getGripperState(prod.arm_name);
                if (!armState.attached)                                                                            //object accidentally fell on the tray
                    faultyGripper(gantry, prod, camera, my_part_in_tray);
                else
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

void removeFaultyProduct(Camera &camera, GantryControl &gantry, product &prod) {
    ROS_INFO_STREAM("IN faulty part");
    part temp;

    temp.pose = camera.get_faulty_pose(prod.agv_id);
    temp.type = prod.type;
    moveToLocation(presetLoc, prod.agv_id, gantry);
    gantry.pickPart(temp);
    //TODO-Change preset locations
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


//TODO:Make it more robust
void processHPOrder(nist_gear::Order &order,Camera &camera, GantryControl &gantry, ros::ServiceClient &agv2Delivery, ros::ServiceClient &agv1Delivery){
    ROS_INFO_STREAM("Processing HP order");
    product prod;
    bool wanted = true;

    for(int j=0; j<order.shipments.size(); j++){
        auto ship = order.shipments[j];

        for (int k=0; k<ship.products.size(); k++){
            auto product = ship.products[k];


            prod.type = product.type;
            prod.pose = product.pose;
            prod.agv_id = ship.agv_id;
            prod.arm_name = "left_arm";

            if(wanted){
                processPart(prod, gantry, camera, false, false);
                ros::Duration(3.0).sleep();
                ros::spinOnce();
                ros::spinOnce();
                if(camera.get_is_faulty(prod.agv_id)) {
                    removeFaultyProduct(camera,gantry,prod);
                    k--;                                                                                         //process product again
                }
            }else
                removeProduct(camera,gantry,prod);

            moveFromLocationToStart(presetLoc,"start",gantry);
            ROS_INFO("HELLO1");
        }
        ROS_INFO("HELLO2");
        if(prod.agv_id == "agv2")
            agvDeliveryService(agv2Delivery, order.shipments[j].shipment_type);
        else
            agvDeliveryService(agv1Delivery, order.shipments[j].shipment_type);
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
    imgPart.pose.position.y = -0.272441;
    imgPart.pose.position.z = 0.875004;
    ROS_INFO_STREAM("Picking Part from conveynor");
    gantry.pickPart(imgPart);
    moveFromLocationToStart(presetLoc, "pickmovingPart", gantry);

    if (int(my_part_in_tray.pose.orientation.x) == 1) {                                   //Flip part if part needs to be flipped
        moveToLocation(presetLoc,prod.agv_id+"_flipped",gantry);                                       //go to location to flip pulley
        flipPart(gantry, my_part_in_tray, prod);
    }
    else
        moveToLocation(presetLoc, prod.agv_id, gantry);                                                //move to desired agv id

    gantry.placePart(my_part_in_tray, prod.agv_id, prod.arm_name);

    moveFromLocationToStart(presetLoc, "start", gantry);

}

geometry_msgs::TransformStamped shelfPosition(std::string shelf){
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration timeout(5.0);
    geometry_msgs::TransformStamped T_w_l;

    try {
        T_w_l = tfBuffer.lookupTransform("world", shelf,            // Determining shelf position in world frame
                                         ros::Time(0), timeout);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    return T_w_l;
}

double shelfDistance(std::string shelf1, std::string shelf2){                   // Determining distance between adjacent shelves
    geometry_msgs::TransformStamped s1;
    geometry_msgs::TransformStamped s2;
    s1 = shelfPosition(shelf1);
    s2 = shelfPosition(shelf2);
    return double(abs(abs(s1.transform.translation.x) - abs(s2.transform.translation.x)));
}

std::vector<std::string> determineGaps(){                                       // Function returning gap postions in the form of string
    std::vector<std::string> gap;
    std::vector<double> gapThreshold = {6.299163, 6.299173};
    std::vector<std::string> shelfSide = {"left_side_gap_", "right_side_gap_"};
    std::string temp;
    int count = 0;
    std::string name1;
    std::string name2;
    int shelfStartIndex = 3;
    int shelfEndIndex = 11;

    for(int shelf = shelfStartIndex; shelf <= shelfEndIndex; shelf = shelf + 2*shelfStartIndex){
        count ++;
        for(int row = shelf; row < (shelf + shelfStartIndex)-1; row++){
            name1 = "shelf"+std::to_string(row)+"_frame";
            name2 = "shelf"+std::to_string(row+1)+"_frame";

            if(shelfDistance(name1, name2) >= gapThreshold[0] && shelfDistance(name1, name2) <= gapThreshold[1]){
                gap.push_back(shelfSide[count-1] + std::to_string(row % shelfStartIndex));
            }
        }
    }
    return gap;
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


    ros::ServiceClient agv2Delivery = node.serviceClient<nist_gear::AGVControl>("/ariac/agv2");               //initialize agvDelivery
    ros::ServiceClient agv1Delivery = node.serviceClient<nist_gear::AGVControl>("/ariac/agv1");

    Camera camera;
    camera.init(node);                                                                                       //initialize camera


    initWayPoints(presetLoc, gantry);                                                                        //initialize waypoints


    HighPriorityOrderInitiated  = false;                                                                     //setting up flag


    auto orders = comp.getOrders();                                                                          //Wait for order
    while(orders.size()<=0)
        orders = comp.getOrders();


    product prod;


    std::vector<std::string> gap;
    gap = determineGaps();
    for(auto val:gap)                                                               // printing the gap positions
        ROS_INFO_STREAM(val);

    for(int i = 0; i< orders.size(); i++){
        auto order = orders[i];


        for(int j=0; j<order.shipments.size(); j++){
            auto ship = order.shipments[j];


            for (int k=0; k<ship.products.size(); k++){
                auto product = ship.products[k];

                ROS_INFO_STREAM(product.type);
                prod.type = product.type;
                prod.pose = product.pose;
                prod.agv_id = ship.agv_id;
                prod.arm_name = "left_arm";

                                                                                    //Conveyor Impementation
                if(k == 0) {
                    while (true) {
                        if (camera.get_break_beam()) {
                            conveyor(camera, gantry, prod);
                            break;
                        }
                    }
                    if(camera.get_break_beam()) {
                        camera.reset_break_beam();
                        continue;
                    }
                }

                ROS_INFO_STREAM(comp.getOrders().size());
                ROS_INFO_STREAM(HighPriorityOrderInitiated);
                if(comp.getOrders().size()>1 && !HighPriorityOrderInitiated){
                    processHPOrder(comp.getOrders()[1],camera,gantry, agv2Delivery, agv1Delivery);
                    ROS_INFO_STREAM("I am here1243");
                    HighPriorityOrderInitiated = true;
                    ROS_INFO_STREAM(comp.getOrders().size());
                    ROS_INFO_STREAM(HighPriorityOrderInitiated);
                    k--;
                }
                else
                    processPart(prod, gantry, camera, false, true);             // Remove flip_flag after degugging


                //TODO - Make checker for faulty more robust
                ROS_INFO_STREAM("heere 1");
                ros::Duration(3.0).sleep();
                ros::spinOnce();
                ros::spinOnce();
                if(camera.get_is_faulty(prod.agv_id)) {
                    ROS_INFO_STREAM("fAULTY");
                    removeFaultyProduct(camera,gantry,prod);
                    k--;                                                                    //process product again
                }
                ROS_INFO_STREAM("heere 2");
                moveFromLocationToStart(presetLoc,"start",gantry);
                ROS_INFO_STREAM("heere x");
            }
            ROS_INFO_STREAM("herex1");
            if(prod.agv_id == "agv2")
                agvDeliveryService(agv2Delivery, order.shipments[j].shipment_type);
            else
                agvDeliveryService(agv1Delivery, order.shipments[j].shipment_type);
        }
        ROS_INFO_STREAM("here3");

    }


    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    determineGaps();
    return 0;
}





