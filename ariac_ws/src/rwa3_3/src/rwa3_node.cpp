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


//###########################################################
//TODO : Complete this class implementation
//###########################################################
//class FaultyPartDetectReplaceClass {
//public:
//    explicit FaultyPartDetectReplaceClass(ros::NodeHandle &node) {
//
//    }
//
//    void qualitySensorCallback(const nist_gear::LogicalCameraImage::ConstPtr &msg){
//        if(msg->models.size() != 0){
//            ROS_WARN("The placed part is faulty");
//
//        }
//        else
//            ROS_INFO("The placed part is not faulty");
//
//    }
//};
//
//
class MyCompetitionClass {
public:
    explicit MyCompetitionClass(ros::NodeHandle &node) {
//    : current_score_(0) {
    }
    part my_part;
    /**
     * @brief Called when a new Message is received on the Topic /ariac/logical_camera_x
     *
     * This function reports the number of objects detected by a logical camera.
     *
     * @param msg Message containing information on objects detected by the camera.
     */
    void logical_camera_callback(
    const nist_gear::LogicalCameraImage::ConstPtr &msg, int index, std::string productType) {
        if (msg->models.size() > 0) {
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
                if(msg->models[i].type == productType){
                    std::string quaSenName;
                    p_l.header.frame_id = "logical_camera_" + std::to_string(index) + "_frame";
                    p_l.pose = msg->models[i].pose;
                    tf2::doTransform(p_l, p_w, T_w_l);

                    //--You should receive the following information from a camera
                    my_part.type = msg->models[i].type;
                    my_part.pose.position.x = p_w.pose.position.x;
                    my_part.pose.position.y = p_w.pose.position.y;
                    my_part.pose.position.z = p_w.pose.position.z;
                    my_part.pose.orientation.x = p_w.pose.orientation.x;
                    my_part.pose.orientation.y = p_w.pose.orientation.y;
                    my_part.pose.orientation.z = p_w.pose.orientation.z;
                    my_part.pose.orientation.w = p_w.pose.orientation.w;

//                    ROS_INFO("Target Part details collected successfully");
//                    ROS_INFO("%s in world frame:\n\n"
//                             "Position: [x,y,z] = [%f,%f,%f]\n"
//                             "Orientation: [x,y,z,w] = [%f,%f,%f,%f]\n",
//                             msg->models[i].type.c_str(),
//                             p_w.pose.position.x,
//                             p_w.pose.position.y,
//                             p_w.pose.position.z,
//                             p_w.pose.orientation.x,
//                             p_w.pose.orientation.y,
//                             p_w.pose.orientation.z,
//                             p_w.pose.orientation.w);
                    break;
                }
            }
        }
    }
    part partDetails(){
        return my_part;
    }


private:
    std::string competition_state_;
    double current_score_;
    std::vector<nist_gear::Order> received_orders_;
};

class OrderSubscriberClass {
public:
    explicit OrderSubscriberClass(ros::NodeHandle &node){

    }
    nist_gear::Order::ConstPtr orderDetails;
    void ordersCallback(const nist_gear::Order::ConstPtr &msg){
        // Parameters for subscribing logical cameras
        orderDetails = msg;

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

    OrderSubscriberClass orderObject(node);
    ros::Subscriber orderSub = node.subscribe<nist_gear::Order>
    ("/ariac/orders", 10, boost::bind(&OrderSubscriberClass::ordersCallback, &orderObject, _1));
    orderDetails = orderObject.returnOrderMessage();

    // Testing
//     ROS_INFO("%f",orderDetails->shipments[0].products[0].pose.position.x);


    //--2-Look for parts in this order
    // Creating subscribers for logical cameras
    ros::Subscriber logical_camera_subscriber[maxLogicalCamera];
    MyCompetitionClass comp_class(node);

    //--2-Look for parts in this order
    for(auto sm : orderDetails->shipments){
        agvId = sm.agv_id;
        for(auto product : sm.products){
            for(int index = 0; index < maxLogicalCamera; index++) {
                otopic.str(""); otopic.clear();
                otopic << "/ariac/logical_camera_" << (index);
                topic = otopic.str();

                logical_camera_subscriber[index]= node.subscribe<nist_gear::LogicalCameraImage>(
                topic, 10, boost::bind(&MyCompetitionClass::logical_camera_callback, &comp_class, _1, index, product.type));

                // Getting target part details
                requiredPart = comp_class.partDetails();
                //--get pose of part in tray from /ariac/orders
                part_in_tray.type = product.type;
                part_in_tray.pose.position.x = product.pose.position.x;
                part_in_tray.pose.position.x = product.pose.position.y;
                part_in_tray.pose.position.x = product.pose.position.z;
                part_in_tray.pose.orientation.x = product.pose.orientation.x;
                part_in_tray.pose.orientation.y = product.pose.orientation.y;
                part_in_tray.pose.orientation.z = product.pose.orientation.z;
                part_in_tray.pose.orientation.w = product.pose.orientation.w;

                //--Go pick the part
                //#############################################
                // TODO
                //--We go to this bin because a camera above
                // --this bin found one of the parts in the order
                // gantry.goToPresetLocation(gantry.bin3_);
                //##############################################
                gantry.goToPresetLocation(gantry.bin3_);
                gantry.pickPart(requiredPart);
                //--Go place the part
                if(agvId == "any" || agvId == "agv2") {
                    gantry.placePart(part_in_tray, "agv2");
                    quaSenName = "/ariac/quality_control_sensor_1";
                }
                else {
                    gantry.placePart(part_in_tray, "agv1");
                    quaSenName = "/ariac/quality_control_sensor_2";
                }

//              FaultyPartDetectReplaceClass qualityCheck(node);
//              ros::Subscriber qualitySensorsSub;
//
//              qualitySensorsSub = node.subscribe<nist_gear::LogicalCameraImage>
//              (quaSenName, 10, boost::bind(&FaultyPartDetectReplaceClass::qualitySensorCallback, &qualityCheck));

            }
        }
    }
    //--We go to this bin because a camera above
    //--this bin found one of the parts in the order
//    gantry.goToPresetLocation(gantry.bin3_);


    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}