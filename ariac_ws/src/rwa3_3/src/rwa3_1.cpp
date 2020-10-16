//
// Created by eashwar on 10/15/20.
//

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

#include <tf2/LinearMath/Quaternion.h>


class OrderSubscriberClass {
public:
    explicit OrderSubscriberClass(ros::NodeHandle &node){

    }
    nist_gear::Order::ConstPtr orderDetails;
    void ordersCallback(const nist_gear::Order::ConstPtr &msg){
        // Parameters for subscribing logical cameras
        orderDetails = msg;
        ROS_INFO("%s", orderDetails->order_id.c_str());
        ROS_INFO("Information regarding order collected.");

    }

    nist_gear::Order::ConstPtr returnOrderMessage(){
        return orderDetails;
    }
};

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

    // Parameter list
    nist_gear::Order::ConstPtr orderDetails;
    int maxLogicalCamera = 17;
    std::ostringstream otopic;
    std::string topic;
    std::string agvId;
    part requiredPart;
    part part_in_tray;
    std::string quaSenName;

    //--1-Read order
    // Pre-kitting -
    //   1. Starting the competition
    //   2. Creating a subsrciber to ariac/orders

    ROS_INFO("Checkpoint - 1");
    OrderSubscriberClass orderObject(node);
    ros::Subscriber orderSub = node.subscribe<nist_gear::Order>
    ("/ariac/orders", 10, boost::bind(&OrderSubscriberClass::ordersCallback, &orderObject, _1));
    orderDetails = orderObject.returnOrderMessage();
    ROS_INFO("Checkpoint - 2");


    //--1-Read order
    //--2-Look for parts in this order
    //--We go to this bin because a camera above
    //--this bin found one of the parts in the order
    gantry.goToPresetLocation(gantry.bin3_);


    //--You should receive the following information from a camera
    part my_part;
    my_part.type = "pulley_part_red";
    my_part.pose.position.x = 4.365789;
    my_part.pose.position.y = 1.173381;
    my_part.pose.position.z = 0.728011;
    my_part.pose.orientation.x = 0.012;
    my_part.pose.orientation.y = -0.004;
    my_part.pose.orientation.z = 0.002;
    my_part.pose.orientation.w = 1.000;

    //--get pose of part in tray from /ariac/orders
    part_in_tray.type = "pulley_part_red";
    part_in_tray.pose.position.x = -0.12;
    part_in_tray.pose.position.x = -0.2;
    part_in_tray.pose.position.x = 0.0;
    part_in_tray.pose.orientation.x = 0.0;
    part_in_tray.pose.orientation.y = 0.0;
    part_in_tray.pose.orientation.z = 0.0;
    part_in_tray.pose.orientation.w = 1.0;

    //--Go pick the part
    gantry.pickPart(my_part);
    //--Go place the part
    gantry.placePart(part_in_tray, "agv2");

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}