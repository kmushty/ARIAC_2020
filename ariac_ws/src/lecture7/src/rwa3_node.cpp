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
#include <std_msgs/String.h>

#include "competition.h"
#include "gantry_control.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char ** argv) {
    ros::init(argc, argv, "rwa3_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    //see https://github.com/Danfoa/invite-robotics/blob/kinetic-devel/invite_beginner_tutorials/src/pick_and_place.cpp
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
        Competition comp(node);
        comp.init();

    while (ros::ok()) {
//        ros::Publisher planning_scene_diff_publisher = node.advertise<moveit_msgs::PlanningScene>(
//                "/ariac/gantry/planning_scene", 1);
//        ros::WallDuration sleep_t(0.5);
//        while (planning_scene_diff_publisher.getNumSubscribers() < 1) {
//            sleep_t.sleep();
//        }
//        moveit_msgs::AttachedCollisionObject attached_object;
//        attached_object.link_name = "world";
///* The header must contain a valid TF frame*/
//        attached_object.object.header.frame_id = "world";
///* The id of the object */
//        attached_object.object.id = "shelf";
//
///* A default pose */
//        geometry_msgs::Pose pose;
//        pose.position.x = 4.209346;
//        pose.position.y = 3.640037;
//        pose.position.z = 1;
//        pose.orientation.x = 0.0;
//        pose.orientation.y = 0.0;
//        pose.orientation.z = 1.0;
//        pose.orientation.w = 0.0;
//
///* Define a box to be attached */
//        shape_msgs::SolidPrimitive primitive;
//        primitive.type = primitive.BOX;
//        primitive.dimensions.resize(3);
////        primitive.dimensions[0] = 3.788270;
////        primitive.dimensions[1] = 1.092530;
////        primitive.dimensions[2] = 1;
//
//        primitive.dimensions[0] = 4.025;
//        primitive.dimensions[1] = 1.265;
//        primitive.dimensions[2] = 3.021;
//
//        attached_object.object.primitives.push_back(primitive);
//        attached_object.object.primitive_poses.push_back(pose);
//
//        attached_object.object.operation = attached_object.object.ADD;
//
//        ROS_INFO("Adding the object into the world.");
//        moveit_msgs::PlanningScene planning_scene;
//        planning_scene.world.collision_objects.push_back(attached_object.object);
//        planning_scene.is_diff = true;
//        planning_scene_diff_publisher.publish(planning_scene);
//
//        ros::ServiceClient planning_scene_diff_client =
//                node.serviceClient<moveit_msgs::ApplyPlanningScene>("/ariac/gantry/apply_planning_scene");
//        planning_scene_diff_client.waitForExistence();
//// and send the diffs to the planning scene via a service call:
//        moveit_msgs::ApplyPlanningScene srv;
//        srv.request.scene = planning_scene;
//        planning_scene_diff_client.call(srv);
//
//
//        //--ariac stuff
        std::string c_state = comp.getCompetitionState();
        comp.getClock();

        GantryControl gantry(node);
        gantry.init();

        gantry.goToPresetLocation(gantry.start_);
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
        part part_in_tray;
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

//        gantry.goToPresetLocation(gantry.start_);
//        gantry.goToPresetLocation(gantry.bin3_);
//        gantry.goToPresetLocation(gantry.start_);
//        gantry.goToPresetLocation(gantry.bin3_);
//        gantry.goToPresetLocation(gantry.start_);
//        gantry.goToPresetLocation(gantry.bin3_);
    }
    comp.endCompetition();



    //Vector to scale 3D file units (to convert from mm to meters for example)
//    Eigen::Vector3d vectorScale(1, 1, 1);
    // Define a collision object ROS message.
//    moveit_msgs::CollisionObject shelf;
//    //set the relative frame
//    shelf.header.frame_id = "world";
//    // The id of the object is used to identify it.
//    shelf.id = "shelf1";
//
//    //Path where the .dae or .stl object is located
//    shapes::Mesh* m = shapes::createMeshFromResource("package://home/zeid/Desktop/ariac_2020_ws/src/ARIAC/nist_gear/models/workcell_shelf_ghost_base/meshes/shelf_ghost_base.dae", vectorScale);
//    ROS_WARN("Your mesh was loaded");
//
//    shape_msgs::Mesh mesh;
//    shapes::ShapeMsg mesh_msg;
//    shapes::constructMsgFromShape(m, mesh_msg);
//    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
//
//    //Define a pose for your mesh (specified relative to frame_id)
//    geometry_msgs::Pose obj_pose;
//    obj_pose.position.x = 6.044626;
//    obj_pose.position.y = 4.255148;
//    obj_pose.position.z = 0.0;
//    obj_pose.orientation.x=0;
//    obj_pose.orientation.y=0;
//    obj_pose.orientation.z=1;
//    obj_pose.orientation.w=0;
//
//    // Add the mesh to the Collision object message
//    shelf.meshes.push_back(mesh);
//    shelf.mesh_poses.push_back(obj_pose);
//    shelf.operation = shelf.ADD;
//
//    //--Publish object in monitored planning scene
//    // Create vector of collision objects to add
//    std::vector<moveit_msgs::CollisionObject> object;
//    object.push_back(shelf);
//
//    // Add the collision object into the world
//    ros::Duration(2).sleep();
//    planning_scene_interface.addCollisionObjects(object);
    spinner.stop();
    ros::shutdown();
    return 0;
}