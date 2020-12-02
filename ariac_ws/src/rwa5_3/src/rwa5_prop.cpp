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




const double LENGTH_OF_AISLE = 15;
const int NUM_LOGICAL_CAMERAS_PER_AISLE = 4;
const int TIMELIMIT_THRESHOLD = 40;

//coordinates obtainted from gazebo
const double END_LOCATION_X = -16.793;
const double BEGIN_LOCATION_X = -1.793;
const double MID_LOCATION_X = -12.592617;
const std::vector<double> AISLE_Y = {5, 1.54, -1.54, -5};


//keep track of env
std::vector<bool> obstacleInAisle(4,false);
std::vector<obstacle>  obstacleAssociatedWithAisle;
std::vector<part> processedParts;

//useful flags
bool stop_processing;
bool HighPriorityOrderInitiated;
bool ConveyorFlag;
bool PreviousOrderAgvChange;
bool flipped;

//store all preset locations
std::map<std::string,std::vector<PresetLocation>> presetLoc;






void initWayPoints(std::map<std::string,std::vector<PresetLocation>> &presetLoc, GantryControl &gantry){
    /******************************************************************************
     *     SHELF LOGICAL CAMERAS
     ******************************************************************************/

    ///////////////////////////////////////////////////////////////////////////////
    // shelf logical_cameras waypoints associated with aisles
    // ///////////////////////////////////////////////////////////////////////////
    
    // shelf logical_cameras without gaps
    presetLoc["logical_camera_11_aisle1_long"] = {gantry.logical_11_14_aisle_1_short_, gantry.logical_11_14_aisle_1_short_1_, gantry.logical_11_14_aisle_1_long_2_};
    presetLoc["logical_camera_11_aisle1_short"] = {gantry.logical_11_14_aisle_1_short_, gantry.logical_11_14_aisle_1_short_1_, gantry.logical_11_14_aisle_1_short_2_};
    presetLoc["logical_camera_14_aisle1_long"] = {gantry.logical_11_14_aisle_1_short_, gantry.logical_11_14_aisle_1_short_1_, gantry.logical_11_14_aisle_1_long_2_};
    presetLoc["logical_camera_14_aisle1_short"] = {gantry.logical_11_14_aisle_1_short_, gantry.logical_11_14_aisle_1_short_1_, gantry.logical_11_14_aisle_1_short_2_};

    presetLoc["logical_camera_11_aisle0_long"] =  {gantry.logical_11_14_aisle_0_short_, gantry.logical_11_14_aisle_0_short_1_, gantry.logical_11_14_aisle_0_long_2_};
    presetLoc["logical_camera_11_aisle0_short"] = {gantry.logical_11_14_aisle_0_short_, gantry.logical_11_14_aisle_0_short_1_, gantry.logical_11_14_aisle_0_short_2_};
    presetLoc["logical_camera_14_aisle0_long"] =  {gantry.logical_11_14_aisle_0_short_, gantry.logical_11_14_aisle_0_short_1_, gantry.logical_11_14_aisle_0_long_2_};
    presetLoc["logical_camera_14_aisle0_short"] = {gantry.logical_11_14_aisle_0_short_, gantry.logical_11_14_aisle_0_short_1_, gantry.logical_11_14_aisle_0_short_2_};


    presetLoc["logical_camera_12_aisle2_long"] =  {gantry.logical_12_15_aisle_2_short_, gantry.logical_12_15_aisle_2_short_1_, gantry.logical_12_15_aisle_2_long_2_};
    presetLoc["logical_camera_12_aisle2_short"] = {gantry.logical_12_15_aisle_2_short_, gantry.logical_12_15_aisle_2_short_1_, gantry.logical_12_15_aisle_2_short_2_};
    presetLoc["logical_camera_15_aisle2_long"] =  {gantry.logical_12_15_aisle_2_short_, gantry.logical_12_15_aisle_2_short_1_, gantry.logical_12_15_aisle_2_long_2_};
    presetLoc["logical_camera_15_aisle2_short"] = {gantry.logical_12_15_aisle_2_short_, gantry.logical_12_15_aisle_2_short_1_, gantry.logical_12_15_aisle_2_short_2_};

    presetLoc["logical_camera_12_aisle1_long"] =  {gantry.logical_12_15_aisle_1_short_, gantry.logical_12_15_aisle_1_short_1_, gantry.logical_12_15_aisle_1_long_2_};
    presetLoc["logical_camera_12_aisle1_short"] = {gantry.logical_12_15_aisle_1_short_, gantry.logical_12_15_aisle_1_short_1_, gantry.logical_12_15_aisle_1_short_2_};
    presetLoc["logical_camera_15_aisle1_long"] =  {gantry.logical_12_15_aisle_1_short_, gantry.logical_12_15_aisle_1_short_1_, gantry.logical_12_15_aisle_1_long_2_};
    presetLoc["logical_camera_15_aisle1_short"] = {gantry.logical_12_15_aisle_1_short_, gantry.logical_12_15_aisle_1_short_1_, gantry.logical_12_15_aisle_1_short_2_};


    presetLoc["logical_camera_13_aisle3_long"] =  {gantry.logical_13_16_aisle_3_short_, gantry.logical_13_16_aisle_3_short_1_, gantry.logical_13_16_aisle_3_long_2_};
    presetLoc["logical_camera_13_aisle3_short"] = {gantry.logical_13_16_aisle_3_short_, gantry.logical_13_16_aisle_3_short_1_, gantry.logical_13_16_aisle_3_short_2_};
    presetLoc["logical_camera_16_aisle3_long"] =  {gantry.logical_13_16_aisle_3_short_, gantry.logical_13_16_aisle_3_short_1_, gantry.logical_13_16_aisle_3_long_2_};
    presetLoc["logical_camera_16_aisle3_short"] = {gantry.logical_13_16_aisle_3_short_, gantry.logical_13_16_aisle_3_short_1_, gantry.logical_13_16_aisle_3_short_2_};

    presetLoc["logical_camera_13_aisle2_long"] =  {gantry.logical_13_16_aisle_2_short_, gantry.logical_13_16_aisle_2_short_1_, gantry.logical_13_16_aisle_2_long_2_};
    presetLoc["logical_camera_13_aisle2_short"] = {gantry.logical_13_16_aisle_2_short_, gantry.logical_13_16_aisle_2_short_1_, gantry.logical_13_16_aisle_2_short_2_};
    presetLoc["logical_camera_16_aisle2_long"] =  {gantry.logical_13_16_aisle_2_short_, gantry.logical_13_16_aisle_2_short_1_, gantry.logical_13_16_aisle_2_long_2_};
    presetLoc["logical_camera_16_aisle2_short"] = {gantry.logical_13_16_aisle_2_short_, gantry.logical_13_16_aisle_2_short_1_, gantry.logical_13_16_aisle_2_short_2_};
      

    // shelf logical_cameras with gaps 
    
    //right gaps
    presetLoc["right_gap_0_logical_camera_12_long"] =  {gantry.right_gap_default_, gantry.right_gap_0_2_, gantry.right_gap_aisle_3to2_0_, /*gantry.logical_12_15_aisle_2_short_1_,*/ gantry.logical_12_15_aisle_2_long_2_gap_};
    presetLoc["right_gap_0_logical_camera_12_short"] = {gantry.right_gap_default_, gantry.right_gap_0_2_, gantry.right_gap_aisle_3to2_0_, /*gantry.logical_12_15_aisle_2_short_1_,*/ gantry.logical_12_15_aisle_2_short_2_gap_};
    presetLoc["right_gap_0_logical_camera_15_long"] =  {gantry.right_gap_default_, gantry.right_gap_0_2_, gantry.right_gap_aisle_3to2_0_, /*gantry.logical_12_15_aisle_2_short_1_,*/ gantry.logical_12_15_aisle_2_long_2_gap_};
    presetLoc["right_gap_0_logical_camera_15_short"] = {gantry.right_gap_default_, gantry.right_gap_0_2_, gantry.right_gap_aisle_3to2_0_, /*gantry.logical_12_15_aisle_2_short_1_,*/ gantry.logical_12_15_aisle_2_short_2_gap_};

    presetLoc["right_gap_1_logical_camera_12_long"] =  {gantry.right_gap_default_, gantry.right_gap_1_2_, gantry.right_gap_1_3_, gantry.right_gap_aisle_3to2_1_, /*gantry.logical_12_15_aisle_2_short_1_,*/ gantry.logical_12_15_aisle_2_long_2_gap_};
    presetLoc["right_gap_1_logical_camera_12_short"] = {gantry.right_gap_default_, gantry.right_gap_1_2_, gantry.right_gap_1_3_, gantry.right_gap_aisle_3to2_1_, /*gantry.logical_12_15_aisle_2_short_1_,*/ gantry.logical_12_15_aisle_2_short_2_gap_};
    presetLoc["right_gap_1_logical_camera_15_long"] =  {gantry.right_gap_default_, gantry.right_gap_1_2_, gantry.right_gap_1_3_, gantry.right_gap_aisle_3to2_1_, /*gantry.logical_12_15_aisle_2_short_1_,*/ gantry.logical_12_15_aisle_2_long_2_gap_};
    presetLoc["right_gap_1_logical_camera_15_short"] = {gantry.right_gap_default_, gantry.right_gap_1_2_, gantry.right_gap_1_3_, gantry.right_gap_aisle_3to2_1_, /*gantry.logical_12_15_aisle_2_short_1_,*/ gantry.logical_12_15_aisle_2_short_2_gap_};

    presetLoc["right_gap_2_logical_camera_12_long"] =  {gantry.right_gap_default_, gantry.right_gap_2_2_, gantry.right_gap_2_3_, gantry.right_gap_aisle_3to2_2_, /*gantry.logical_12_15_aisle_2_short_1_,*/ gantry.logical_12_15_aisle_2_long_2_gap_};
    presetLoc["right_gap_2_logical_camera_12_short"] = {gantry.right_gap_default_, gantry.right_gap_2_2_, gantry.right_gap_2_3_, gantry.right_gap_aisle_3to2_2_, /*gantry.logical_12_15_aisle_2_short_1_,*/ gantry.logical_12_15_aisle_2_short_2_gap_};
    presetLoc["right_gap_2_logical_camera_15_long"] =  {gantry.right_gap_default_, gantry.right_gap_2_2_, gantry.right_gap_2_3_, gantry.right_gap_aisle_3to2_2_, /*gantry.logical_12_15_aisle_2_short_1_,*/ gantry.logical_12_15_aisle_2_long_2_gap_};
    presetLoc["right_gap_2_logical_camera_15_short"] = {gantry.right_gap_default_, gantry.right_gap_2_2_, gantry.right_gap_2_3_, gantry.right_gap_aisle_3to2_2_, /*gantry.logical_12_15_aisle_2_short_1_,*/ gantry.logical_12_15_aisle_2_short_2_gap_};

    presetLoc["right_gap_3_logical_camera_12_long"] =  {gantry.right_gap_default_, gantry.right_gap_3_2_, gantry.right_gap_3_3_, gantry.right_gap_aisle_3to2_3_, /*gantry.logical_12_15_aisle_2_short_1_,*/ gantry.logical_12_15_aisle_2_long_2_gap_};
    presetLoc["right_gap_3_logical_camera_12_short"] = {gantry.right_gap_default_, gantry.right_gap_3_2_, gantry.right_gap_3_3_, gantry.right_gap_aisle_3to2_3_, /*gantry.logical_12_15_aisle_2_short_1_,*/ gantry.logical_12_15_aisle_2_short_2_gap_};
    presetLoc["right_gap_3_logical_camera_15_long"] =  {gantry.right_gap_default_, gantry.right_gap_3_2_, gantry.right_gap_3_3_, gantry.right_gap_aisle_3to2_3_, /*gantry.logical_12_15_aisle_2_short_1_,*/ gantry.logical_12_15_aisle_2_long_2_gap_};
    presetLoc["right_gap_3_logical_camera_15_short"] = {gantry.right_gap_default_, gantry.right_gap_3_2_, gantry.right_gap_3_3_, gantry.right_gap_aisle_3to2_3_, /*gantry.logical_12_15_aisle_2_short_1_,*/ gantry.logical_12_15_aisle_2_short_2_gap_};

    //left gaps
    presetLoc["left_gap_0_logical_camera_12_long"] =  {gantry.left_gap_default_, gantry.left_gap_0_2_, gantry.left_gap_aisle_0to1_0_, /*gantry.logical_12_15_aisle_1_short_1_,*/ gantry.logical_12_15_aisle_1_long_2_gap_};
    presetLoc["left_gap_0_logical_camera_12_short"] = {gantry.left_gap_default_, gantry.left_gap_0_2_, gantry.left_gap_aisle_0to1_0_, /*gantry.logical_12_15_aisle_1_short_1_,*/ gantry.logical_12_15_aisle_1_short_2_gap_};
    presetLoc["left_gap_0_logical_camera_15_long"] =  {gantry.left_gap_default_, gantry.left_gap_0_2_, gantry.left_gap_aisle_0to1_0_, /*gantry.logical_12_15_aisle_1_short_1_,*/ gantry.logical_12_15_aisle_1_long_2_gap_};
    presetLoc["left_gap_0_logical_camera_15_short"] = {gantry.left_gap_default_, gantry.left_gap_0_2_, gantry.left_gap_aisle_0to1_0_, /*gantry.logical_12_15_aisle_1_short_1_,*/ gantry.logical_12_15_aisle_1_short_2_gap_};

    presetLoc["left_gap_1_logical_camera_12_long"] =  {gantry.left_gap_default_, gantry.left_gap_1_2_, gantry.left_gap_1_3_, gantry.left_gap_aisle_0to1_1_, /*gantry.logical_12_15_aisle_1_short_1_,*/ gantry.logical_12_15_aisle_1_long_2_gap_};
    presetLoc["left_gap_1_logical_camera_12_short"] = {gantry.left_gap_default_, gantry.left_gap_1_2_, gantry.left_gap_1_3_, gantry.left_gap_aisle_0to1_1_, /*gantry.logical_12_15_aisle_1_short_1_,*/ gantry.logical_12_15_aisle_1_short_2_gap_};
    presetLoc["left_gap_1_logical_camera_15_long"] =  {gantry.left_gap_default_, gantry.left_gap_1_2_, gantry.left_gap_1_3_, gantry.left_gap_aisle_0to1_1_, /*gantry.logical_12_15_aisle_1_short_1_,*/ gantry.logical_12_15_aisle_1_long_2_gap_};
    presetLoc["left_gap_1_logical_camera_15_short"] = {gantry.left_gap_default_, gantry.left_gap_1_2_, gantry.left_gap_1_3_, gantry.left_gap_aisle_0to1_1_, /*gantry.logical_12_15_aisle_1_short_1_,*/ gantry.logical_12_15_aisle_1_short_2_gap_};

    presetLoc["left_gap_2_logical_camera_12_long"] =  {gantry.left_gap_default_, gantry.left_gap_2_2_, gantry.left_gap_2_3_, gantry.left_gap_aisle_0to1_2_, /*gantry.logical_12_15_aisle_1_short_1_,*/ gantry.logical_12_15_aisle_1_long_2_gap_};
    presetLoc["left_gap_2_logical_camera_12_short"] = {gantry.left_gap_default_, gantry.left_gap_2_2_, gantry.left_gap_2_3_, gantry.left_gap_aisle_0to1_2_, /*gantry.logical_12_15_aisle_1_short_1_,*/ gantry.logical_12_15_aisle_1_short_2_gap_};
    presetLoc["left_gap_2_logical_camera_15_long"] =  {gantry.left_gap_default_, gantry.left_gap_2_2_, gantry.left_gap_2_3_, gantry.left_gap_aisle_0to1_2_, /*gantry.logical_12_15_aisle_1_short_1_,*/ gantry.logical_12_15_aisle_1_long_2_gap_};
    presetLoc["left_gap_2_logical_camera_15_short"] = {gantry.left_gap_default_, gantry.left_gap_2_2_, gantry.left_gap_2_3_, gantry.left_gap_aisle_0to1_2_, /*gantry.logical_12_15_aisle_1_short_1_,*/ gantry.logical_12_15_aisle_1_short_2_gap_};

    presetLoc["left_gap_3_logical_camera_12_long"] =  {gantry.left_gap_default_, gantry.left_gap_3_2_, gantry.left_gap_3_3_, gantry.left_gap_aisle_0to1_3_, /*gantry.logical_12_15_aisle_1_short_1_,*/ gantry.logical_12_15_aisle_1_long_2_gap_};
    presetLoc["left_gap_3_logical_camera_12_short"] = {gantry.left_gap_default_, gantry.left_gap_3_2_, gantry.left_gap_3_3_, gantry.left_gap_aisle_0to1_3_, /*gantry.logical_12_15_aisle_1_short_1_,*/ gantry.logical_12_15_aisle_1_short_2_gap_};
    presetLoc["left_gap_3_logical_camera_15_long"] =  {gantry.left_gap_default_, gantry.left_gap_3_2_, gantry.left_gap_3_3_, gantry.left_gap_aisle_0to1_3_, /*gantry.logical_12_15_aisle_1_short_1_,*/ gantry.logical_12_15_aisle_1_long_2_gap_};
    presetLoc["left_gap_3_logical_camera_15_short"] = {gantry.left_gap_default_, gantry.left_gap_3_2_, gantry.left_gap_3_3_, gantry.left_gap_aisle_0to1_3_, /*gantry.logical_12_15_aisle_1_short_1_,*/ gantry.logical_12_15_aisle_1_short_2_gap_};


    //middle gaps
    presetLoc["middle_gap_0_logical_camera_13_long"] =  {gantry.middle_gap_default_aisle1_, gantry.middle_gap_0_2_aisle1_, gantry.middle_gap_aisle_1to2_0_, gantry.logical_13_16_aisle_2_short_1_, gantry.logical_13_16_aisle_2_long_2_gap_};
    presetLoc["middle_gap_0_logical_camera_13_short"] = {gantry.middle_gap_default_aisle1_, gantry.middle_gap_0_2_aisle1_, gantry.middle_gap_aisle_1to2_0_, gantry.logical_13_16_aisle_2_short_1_, gantry.logical_13_16_aisle_2_short_2_gap_};
    presetLoc["middle_gap_0_logical_camera_16_long"] =  {gantry.middle_gap_default_aisle1_, gantry.middle_gap_0_2_aisle1_, gantry.middle_gap_aisle_1to2_0_, gantry.logical_13_16_aisle_2_short_1_, gantry.logical_13_16_aisle_2_long_2_gap_};
    presetLoc["middle_gap_0_logical_camera_16_short"] = {gantry.middle_gap_default_aisle1_, gantry.middle_gap_0_2_aisle1_, gantry.middle_gap_aisle_1to2_0_, gantry.logical_13_16_aisle_2_short_1_, gantry.logical_13_16_aisle_2_short_2_gap_};

    presetLoc["middle_gap_1_logical_camera_13_long"] =  {gantry.middle_gap_default_aisle1_, gantry.middle_gap_1_2_aisle1_, gantry.middle_gap_1_3_aisle1_, gantry.middle_gap_aisle_1to2_1_, gantry.logical_13_16_aisle_2_short_1_, gantry.logical_13_16_aisle_2_long_2_gap_};
    presetLoc["middle_gap_1_logical_camera_13_short"] = {gantry.middle_gap_default_aisle1_, gantry.middle_gap_1_2_aisle1_, gantry.middle_gap_1_3_aisle1_, gantry.middle_gap_aisle_1to2_1_, gantry.logical_13_16_aisle_2_short_1_, gantry.logical_13_16_aisle_2_short_2_gap_};
    presetLoc["middle_gap_1_logical_camera_16_long"] =  {gantry.middle_gap_default_aisle1_, gantry.middle_gap_1_2_aisle1_, gantry.middle_gap_1_3_aisle1_, gantry.middle_gap_aisle_1to2_1_, gantry.logical_13_16_aisle_2_short_1_, gantry.logical_13_16_aisle_2_long_2_gap_};
    presetLoc["middle_gap_1_logical_camera_16_short"] = {gantry.middle_gap_default_aisle1_, gantry.middle_gap_1_2_aisle1_, gantry.middle_gap_1_3_aisle1_, gantry.middle_gap_aisle_1to2_1_, gantry.logical_13_16_aisle_2_short_1_, gantry.logical_13_16_aisle_2_short_2_gap_};

    presetLoc["middle_gap_2_logical_camera_13_long"] =  {gantry.middle_gap_default_aisle1_, gantry.middle_gap_2_2_aisle1_, gantry.middle_gap_2_3_aisle1_, gantry.middle_gap_aisle_1to2_2_, /*gantry.logical_13_16_aisle_2_short_1_,*/ gantry.logical_13_16_aisle_2_long_2_gap_};
    presetLoc["middle_gap_2_logical_camera_13_short"] = {gantry.middle_gap_default_aisle1_, gantry.middle_gap_2_2_aisle1_, gantry.middle_gap_2_3_aisle1_, gantry.middle_gap_aisle_1to2_2_, /*gantry.logical_13_16_aisle_2_short_1_,*/ gantry.logical_13_16_aisle_2_short_2_gap_};
    presetLoc["middle_gap_2_logical_camera_16_long"] =  {gantry.middle_gap_default_aisle1_, gantry.middle_gap_2_2_aisle1_, gantry.middle_gap_2_3_aisle1_, gantry.middle_gap_aisle_1to2_2_, /*gantry.logical_13_16_aisle_2_short_1_,*/ gantry.logical_13_16_aisle_2_long_2_gap_};
    presetLoc["middle_gap_2_logical_camera_16_short"] = {gantry.middle_gap_default_aisle1_, gantry.middle_gap_2_2_aisle1_, gantry.middle_gap_2_3_aisle1_, gantry.middle_gap_aisle_1to2_2_, /*gantry.logical_13_16_aisle_2_short_1_,*/ gantry.logical_13_16_aisle_2_short_2_gap_};

    presetLoc["middle_gap_3_logical_camera_13_long"] =  {gantry.middle_gap_default_aisle1_, gantry.middle_gap_3_2_aisle1_, gantry.middle_gap_3_3_aisle1_, gantry.middle_gap_aisle_1to2_3_, gantry.logical_13_16_aisle_2_short_1_, gantry.logical_13_16_aisle_2_long_2_gap_};
    presetLoc["middle_gap_3_logical_camera_13_short"] = {gantry.middle_gap_default_aisle1_, gantry.middle_gap_3_2_aisle1_, gantry.middle_gap_3_3_aisle1_, gantry.middle_gap_aisle_1to2_3_, gantry.logical_13_16_aisle_2_short_1_, gantry.logical_13_16_aisle_2_short_2_gap_};
    presetLoc["middle_gap_3_logical_camera_16_long"] =  {gantry.middle_gap_default_aisle1_, gantry.middle_gap_3_2_aisle1_, gantry.middle_gap_3_3_aisle1_, gantry.middle_gap_aisle_1to2_3_, gantry.logical_13_16_aisle_2_short_1_, gantry.logical_13_16_aisle_2_long_2_gap_};
    presetLoc["middle_gap_3_logical_camera_16_short"] = {gantry.middle_gap_default_aisle1_, gantry.middle_gap_3_2_aisle1_, gantry.middle_gap_3_3_aisle1_, gantry.middle_gap_aisle_1to2_3_, gantry.logical_13_16_aisle_2_short_1_, gantry.logical_13_16_aisle_2_short_2_gap_};



    presetLoc["middle_gap_0_logical_camera_11_long"] =  {gantry.middle_gap_default_aisle2_, gantry.middle_gap_0_2_aisle2_, gantry.middle_gap_aisle_2to1_0_, gantry.logical_11_14_aisle_1_short_1_, gantry.logical_11_14_aisle_1_long_2_gap_};
    presetLoc["middle_gap_0_logical_camera_11_short"] = {gantry.middle_gap_default_aisle2_, gantry.middle_gap_0_2_aisle2_, gantry.middle_gap_aisle_2to1_0_, gantry.logical_11_14_aisle_1_short_1_, gantry.logical_11_14_aisle_1_short_2_gap_};
    presetLoc["middle_gap_0_logical_camera_14_long"] =  {gantry.middle_gap_default_aisle2_, gantry.middle_gap_0_2_aisle2_, gantry.middle_gap_aisle_2to1_0_, gantry.logical_11_14_aisle_1_short_1_, gantry.logical_11_14_aisle_1_long_2_gap_};
    presetLoc["middle_gap_0_logical_camera_14_short"] = {gantry.middle_gap_default_aisle2_, gantry.middle_gap_0_2_aisle2_, gantry.middle_gap_aisle_2to1_0_, gantry.logical_11_14_aisle_1_short_1_, gantry.logical_11_14_aisle_1_short_2_gap_};

    presetLoc["middle_gap_1_logical_camera_11_long"] =  {gantry.middle_gap_default_aisle2_, gantry.middle_gap_1_2_aisle2_, gantry.middle_gap_1_3_aisle2_, gantry.middle_gap_aisle_2to1_1_, gantry.logical_11_14_aisle_1_short_1_, gantry.logical_11_14_aisle_1_long_2_gap_};
    presetLoc["middle_gap_1_logical_camera_11_short"] = {gantry.middle_gap_default_aisle2_, gantry.middle_gap_1_2_aisle2_, gantry.middle_gap_1_3_aisle2_, gantry.middle_gap_aisle_2to1_1_, gantry.logical_11_14_aisle_1_short_1_, gantry.logical_11_14_aisle_1_short_2_gap_};
    presetLoc["middle_gap_1_logical_camera_14_long"] =  {gantry.middle_gap_default_aisle2_, gantry.middle_gap_1_2_aisle2_, gantry.middle_gap_1_3_aisle2_, gantry.middle_gap_aisle_2to1_1_, gantry.logical_11_14_aisle_1_short_1_, gantry.logical_11_14_aisle_1_long_2_gap_};
    presetLoc["middle_gap_1_logical_camera_14_short"] = {gantry.middle_gap_default_aisle2_, gantry.middle_gap_1_2_aisle2_, gantry.middle_gap_1_3_aisle2_, gantry.middle_gap_aisle_2to1_1_, gantry.logical_11_14_aisle_1_short_1_, gantry.logical_11_14_aisle_1_short_2_gap_};

    presetLoc["middle_gap_2_logical_camera_11_long"] =  {gantry.middle_gap_default_aisle2_, gantry.middle_gap_2_2_aisle2_, gantry.middle_gap_2_3_aisle2_, gantry.middle_gap_aisle_2to1_2_, /*gantry.logical_11_14_aisle_1_short_1_,*/ gantry.logical_11_14_aisle_1_long_2_gap_};
    presetLoc["middle_gap_2_logical_camera_11_short"] = {gantry.middle_gap_default_aisle2_, gantry.middle_gap_2_2_aisle2_, gantry.middle_gap_2_3_aisle2_, gantry.middle_gap_aisle_2to1_2_, /*gantry.logical_11_14_aisle_1_short_1_,*/ gantry.logical_11_14_aisle_1_short_2_gap_};
    presetLoc["middle_gap_2_logical_camera_14_long"] =  {gantry.middle_gap_default_aisle2_, gantry.middle_gap_2_2_aisle2_, gantry.middle_gap_2_3_aisle2_, gantry.middle_gap_aisle_2to1_2_, /*gantry.logical_11_14_aisle_1_short_1_,*/ gantry.logical_11_14_aisle_1_long_2_gap_};
    presetLoc["middle_gap_2_logical_camera_14_short"] = {gantry.middle_gap_default_aisle2_, gantry.middle_gap_2_2_aisle2_, gantry.middle_gap_2_3_aisle2_, gantry.middle_gap_aisle_2to1_2_, /*gantry.logical_11_14_aisle_1_short_1_,*/ gantry.logical_11_14_aisle_1_short_2_gap_};

    presetLoc["middle_gap_3_logical_camera_11_long"] =  {gantry.middle_gap_default_aisle2_, gantry.middle_gap_3_2_aisle2_, gantry.middle_gap_3_3_aisle2_, gantry.middle_gap_aisle_2to1_3_, gantry.logical_11_14_aisle_1_short_1_, gantry.logical_11_14_aisle_1_long_2_gap_};
    presetLoc["middle_gap_3_logical_camera_11_short"] = {gantry.middle_gap_default_aisle2_, gantry.middle_gap_3_2_aisle2_, gantry.middle_gap_3_3_aisle2_, gantry.middle_gap_aisle_2to1_3_, gantry.logical_11_14_aisle_1_short_1_, gantry.logical_11_14_aisle_1_short_2_gap_};
    presetLoc["middle_gap_3_logical_camera_14_long"] =  {gantry.middle_gap_default_aisle2_, gantry.middle_gap_3_2_aisle2_, gantry.middle_gap_3_3_aisle2_, gantry.middle_gap_aisle_2to1_3_, gantry.logical_11_14_aisle_1_short_1_, gantry.logical_11_14_aisle_1_long_2_gap_};
    presetLoc["middle_gap_3_logical_camera_14_short"] = {gantry.middle_gap_default_aisle2_, gantry.middle_gap_3_2_aisle2_, gantry.middle_gap_3_3_aisle2_, gantry.middle_gap_aisle_2to1_3_, gantry.logical_11_14_aisle_1_short_1_, gantry.logical_11_14_aisle_1_short_2_gap_};


    ///////////////////////////////////////////////////////////////////////////////
    // shelf logical_cameras without aisles
    // ///////////////////////////////////////////////////////////////////////////
    presetLoc["logical_camera_0_long"] = {gantry.logical_0_4_short_, gantry.logical_0_4_long_1_};
    presetLoc["logical_camera_0_short"] = {gantry.logical_0_4_short_, gantry.logical_0_4_short_1_};
    presetLoc["logical_camera_4_long"] = {gantry.logical_0_4_short_, gantry.logical_0_4_long_1_};
    presetLoc["logical_camera_4_short"] = {gantry.logical_0_4_short_, gantry.logical_0_4_short_1_};


    presetLoc["logical_camera_3_short"] = {gantry.logical_3_7_short_, gantry.logical_3_7_short_1_};
    presetLoc["logical_camera_3_long"] = {gantry.logical_3_7_short_, gantry.logical_3_7_long_1_};
    presetLoc["logical_camera_7_short"] = {gantry.logical_3_7_short_, gantry.logical_3_7_short_1_};
    presetLoc["logical_camera_7_long"] = {gantry.logical_3_7_short_, gantry.logical_3_7_long_1_};



    /***********************************************************************
     *   BIN LOGICAL CAMERAS
     **********************************************************************/
     presetLoc["logical_camera_1"] = {gantry.start_};
     presetLoc["logical_camera_2"] = {gantry.start_};
     presetLoc["logical_camera_5"] = {gantry.start_};
     presetLoc["logical_camera_6"] = {gantry.start_};



    /*******************************************************************
     *   USEFUL PRESET LOCATIONS
     ******************************************************************/
    presetLoc["start"] = {gantry.start_};                                                                     
    presetLoc["agv2"] = {gantry.agv2_};
    presetLoc["agv1"] = {gantry.agv1_};
    presetLoc["agv2_faultyG"] = {gantry.agv2_faultyG_};
    presetLoc["agv1_faultyG"] = {gantry.agv1_faultyG_};
    presetLoc["agv2_faultyP"] = {gantry.agv2_faultyP_};
    presetLoc["agv1_faultyP"] = {gantry.agv1_faultyP_};
    presetLoc["flipped_pulley_agv2"] = {gantry.agv2_go_to_flipped_pulley_};//verified
    presetLoc["flipped_pulley_agv1"] = {gantry.agv1_go_to_flipped_pulley_};//verified
    presetLoc["agv2_flipped_final"]  = {gantry.agv2_flipped1_};//verified
    presetLoc["agv1_flipped_final"]  = {gantry.agv1_flipped1_};//verified
    presetLoc["agv2_right_arm_drop_flip"]  = {gantry.agv2_flipped_, gantry.agv2_flipped1_};//verified
    presetLoc["agv1_right_arm_drop_flip"]  = {gantry.agv1_flipped_, gantry.agv1_flipped1_};//verified
    presetLoc["agv2_left_arm_drop"]  = {gantry.agv2_drop_};//verified
    presetLoc["agv1_left_arm_drop"]  = {gantry.agv1_drop_};//verified
    presetLoc["movingPart"] = {gantry.movingPart_};
    presetLoc["movingPartDisk"] = {gantry.movingPartDisk_};
    presetLoc["movingPartGear"] = {gantry.movingPartGear_};
    presetLoc["pickmovingPart"] = {gantry.movingPart1_, gantry.movingPart_};
    presetLoc["agv1_gasket_part_green"] = {gantry.agv1_gasket_part_green_};
    presetLoc["agv2_disk_part_green"] = {gantry.agv2_};

    
    /**********************************************************************
     *    CONVEYOR PRESET LOCATIONS
     ********************************************************************/
    presetLoc["conveyorPart_1"] = {gantry.bin13_1_};
    presetLoc["conveyorPart_2"] = {gantry.bin13_2_};
    presetLoc["conveyorPart_3"] = {gantry.bin13_3_};
    presetLoc["conveyorPart_4"] = {gantry.bin13_4_};
}


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


void keepTrackOfProcessedParts(part my_part, product prod,GantryControl &gantry,Camera &camera){
  part processed_part = my_part;
  ROS_INFO_STREAM("Entered Keep track");
  if(camera.isSensorBlackout()){
      ROS_INFO_STREAM("Entered sensor blackout");
      processed_part.placed_part_pose = gantry.getTargetWorldPose(processed_part.pose,prod.agv_id,prod.arm_name);
      processedParts.push_back(processed_part);

  }else {
    ROS_INFO_STREAM("dID NOT ENTER SENSOR BLACKOUT");
    std::map<std::string,part> parts;
    if (prod.agv_id == "agv2")
        parts = camera.get_agv_detected_parts()["logical_camera_10"];
    else
        parts = camera.get_agv_detected_parts()["logical_camera_8"];
    

    part placed_part = parts.begin()->second;                 
    ROS_INFO_STREAM("Placed part pose" << placed_part.pose);
    processedParts.push_back(placed_part);
    for(const auto& part: parts){
      ROS_INFO_STREAM("PLACED PART COUNT " << placed_part.count);
        ROS_INFO_STREAM("part second count " << part.second.count);
      if(placed_part.count < part.second.count){
        placed_part = part.second;
        processed_part.pose = placed_part.pose;
        ROS_INFO_STREAM("HERE");
        processedParts[processedParts.size()-1]=processed_part;
        ROS_INFO_STREAM("HERE1");
      }
    }
  }
}


void moveToLocation(std::map<std::string,std::vector<PresetLocation>> &presetLoc, std::string location,GantryControl &gantry){
    auto vec = presetLoc[location];
    for(auto waypoint :vec)
        gantry.goToPresetLocation(waypoint);
}

void moveToLocation2(std::map<std::string,std::vector<PresetLocation>> &presetLoc, part my_part,GantryControl &gantry,std::string location){
    auto vec = presetLoc[location];
    int count =0;
//    if(my_part.logicalCameraName == "logical_camera_0" || my_part.logicalCameraName == "logical_camera_4" ||
//       my_part.logicalCameraName == "logical_camera_3" || my_part.logicalCameraName == "logical_camera_7"){
//        for(int i=0; i<vec.size() - 1; i++){
//            gantry.goToPresetLocation(vec[i]);
//            //        ros::Duration(10).sleep();
//            ROS_INFO_STREAM("iiiiiiiin way point");
//        }
//    }
//    else {
        for (int i = 0; i < vec.size(); i++) {
            gantry.goToPresetLocation(vec[i]);
            //        ros::Duration(10).sleep();
            ROS_INFO_STREAM("iiiiiiiin way point");
        }
//    }
     gantry.moveToPart(my_part, vec[vec.size()-1]);
    //modify to pick part
}

void moveToGap(std::map<std::string,std::vector<PresetLocation>> &presetLoc, part my_part,GantryControl &gantry,std::string location){
    auto vec = presetLoc[location];
    for(int i=0; i< (vec.size() - 2); i++)
        gantry.goToPresetLocation(vec[i]);
}


void moveFromGapToLocation(std::map<std::string,std::vector<PresetLocation>> &presetLoc, part my_part,GantryControl &gantry,std::string location){
    auto vec = presetLoc[location];
    ROS_INFO_STREAM("bEFORE");
    ROS_INFO_STREAM("Vec size" << vec.size());
    for(int i=(vec.size() - 2); i < vec.size() - 1; i++) {
        ROS_INFO_STREAM(i);
        gantry.goToPresetLocation(vec[i]);
    }
    ROS_INFO_STREAM("aFTER");
     gantry.moveToPart(my_part, vec[vec.size() - 1]);
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

void moveFromLocationToGoal(product prod, part my_part, std::map<std::string,std::vector<PresetLocation>> &presetLoc, std::string location,GantryControl &gantry) {
    auto vec = presetLoc[location];
    if(my_part.logicalCameraName == "logical_camera_1" || my_part.logicalCameraName == "logical_camera_5" ||
       my_part.logicalCameraName == "logical_camera_2" || my_part.logicalCameraName == "logical_camera_6"){
        gantry.goToPresetLocation(gantry.start_);
        gantry.goToPresetLocation(presetLoc[prod.agv_id][0]);
    }
    else if(my_part.logicalCameraName == "logical_camera_0" || my_part.logicalCameraName == "logical_camera_4" ||
            my_part.logicalCameraName == "logical_camera_7" || my_part.logicalCameraName == "logical_camera_3"){
        for(int i=vec.size()-1; i>=0;i--)
            gantry.goToPresetLocation(vec[i]);
        gantry.goToPresetLocation(gantry.start_);
        gantry.goToPresetLocation(presetLoc[prod.agv_id][0]);
    }
    else{
        for(int i=vec.size()-1; i>=0;i--)
            gantry.goToPresetLocation(vec[i]);
        gantry.goToPresetLocation(presetLoc[prod.agv_id][0]);
    }
}

void moveFromLocationToGoalAvoidingObstacles(product prod, std::map<std::string,std::vector<PresetLocation>> &presetLoc, std::string location,GantryControl &gantry) {
    auto vec = presetLoc[location];
    for(int i=vec.size()-2; i>=0;i--)
        gantry.goToPresetLocation(vec[i]);
    gantry.goToPresetLocation(presetLoc[prod.agv_id][0]);

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
    if (prod.agv_id == "agv2") {
        parts = camera.get_agv_detected_parts()["logical_camera_10"];
    }
    else
        parts = camera.get_agv_detected_parts()["logical_camera_8"];

    ROS_INFO_STREAM(prod.agv_id);
    ROS_INFO_STREAM("fg1");
    ROS_INFO_STREAM("Parts size" << parts.size());
    //get part that was just placed on agv;
    placed_part = parts.begin()->second;                 
    ROS_INFO_STREAM(placed_part.pose.position.x);
    ROS_INFO_STREAM(placed_part.pose.position.y);
    ROS_INFO_STREAM(placed_part.pose.position.z);
    for(const auto& part: parts){
      if(placed_part.count < part.second.count)
        placed_part = part.second;
      ROS_INFO_STREAM("fg2");
    }
    

    armState = gantry.getGripperState(prod.arm_name);

    ROS_INFO("Faulty Gripper Detected");
    ROS_INFO("Re-picking and replacing the part");
    moveToLocation(presetLoc,prod.agv_id+"_faultyG", gantry);
    ROS_INFO_STREAM(prod.agv_id+"_"+prod.type);
    gantry.pickPart(placed_part);
    moveToLocation(presetLoc,prod.agv_id+"_faultyG", gantry);
    moveToLocation(presetLoc, prod.agv_id, gantry);
    gantry.placePart(my_part_in_tray, prod.agv_id, prod.arm_name);
}



void flipPart(GantryControl &gantry, Camera camera, part &my_part_in_tray, product &prod){
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

                double gap_position = shelfPosition(name1).transform.translation.x - 4 - (shelfDistance(name1,name2) - 4)/2;
                gap.push_back(std::to_string(gap_position));
            }
        }
        if(countFlag == 0) {
            if (shelfPosition("shelf" + std::to_string(shelf) + "_frame").transform.translation.x >= -2.093989 &&
                shelfPosition("shelf" + std::to_string(shelf) + "_frame").transform.translation.x <= -2.093979) {
                gap.push_back(shelfSide[count - 1] + std::to_string((shelfStartIndex - countFlag)));

                double gap_position = shelfPosition(name1).transform.translation.x - 1.0550175;
                gap.push_back(std::to_string(gap_position));
            } else {
                double gap_position = shelfPosition(name1).transform.translation.x + 1.0550175;
                gap.push_back(shelfSide[count - 1] + std::to_string((countFlag)));

            }
        }
    }
    
    
    for(auto val:gap)                                                                    // printing the gap positions
        ROS_INFO_STREAM(val);

    ROS_INFO_STREAM("Exited determin gap method");
    return gap;
}




std::vector<int> closestAislesWithoutObstacles(int aisle_num) {

    std::vector<int> result;
    int i = aisle_num , j = aisle_num ,closest = 10, index=aisle_num;
    bool toggle = true;
    
    while(i!=-1 || j!= 4) {
      if(index >= 0 && index < 4 && !obstacleInAisle[index] && index != aisle_num && abs(index-aisle_num)<= closest){
             result.push_back(index);
             closest = abs(index-aisle_num);
      }
      if(toggle){
          index = i;
          if(i!= -1) i--;
          toggle = false;
       }else{
         index = j;
         if(j!=4) j++;
         toggle = true;
       }
    }
    return result;
}


std::string getGapName(std::string gapType){

   auto gaps = determineGaps();
   for(auto gap: gaps) {
     std::string token = gap.substr(0, gap.size()-2);
     ROS_INFO_STREAM("tOKEN" << token);
       ROS_INFO_STREAM("Gap Type" << gapType);
     if(token == gapType)
        return gap;
   }
   return "None";
}

std::string getGapCoorX(std::string gapName) {
   auto gaps = determineGaps();
   auto it = std::find(gaps.begin(),gaps.end(), gapName); 
   int index = it - gaps.begin() + 1;
   return gaps[index];
}

std::vector<std::string> planPath(int aisle_num, part my_part){
   std::vector<std::string> plan;
   
   std::vector<std::string> gapType{"left_gap", "middle_gap", "right_gap"};
   auto closest_aisles_wo  = closestAislesWithoutObstacles(aisle_num);
   ROS_INFO_STREAM("In planPath");

   for(int i=0;i <closest_aisles_wo.size(); i++)
       ROS_INFO_STREAM("CLOSEST AISLE NUM" << closest_aisles_wo[i]);

   if(closest_aisles_wo.size() == 1) {
      if(aisle_num == 0){
           if(closest_aisles_wo[0] == 1)
               return std::vector<std::string> {"no_gap_needed","-","1","long"};
           else
               return std::vector<std::string> {"gap_needed", getGapName("middle_gap"),"2","long", getGapCoorX(getGapName("middle_gap"))};
      }else if(aisle_num == 3) {
           if(closest_aisles_wo[0] == 2)
               return std::vector<std::string> {"no_gap_needed","-","2","long"};
           else
               return std::vector<std::string> {"gap_needed", getGapName("middle_gap"),"2","long", getGapCoorX(getGapName("middle_gap"))};
      }else if(aisle_num == 1) {
            if(closest_aisles_wo[0] == 0){
               if(std::max(my_part.pose.position.y, AISLE_Y[0]) - std::min(my_part.pose.position.y, AISLE_Y[0]) < 3)      // select closest aisle to part
                   return std::vector<std::string> {"no_gap_needed","-","0","long"};
               else
                   return std::vector<std::string> {"gap_needed",getGapName("left_gap"),"0","short", getGapCoorX(getGapName("left_gap"))};
            }
            else{   //closest aisle is 2
               if(std::max(my_part.pose.position.y, AISLE_Y[2]) - std::min(my_part.pose.position.y, AISLE_Y[2]) < 3)      // select closest aisle to part
                   return std::vector<std::string> {"no_gap_needed","-","2","long"};
               else
                   return std::vector<std::string> {"gap_needed",getGapName("middle_gap"),"2","short",getGapCoorX(getGapName("middle_gap"))};
            }
      }else{ 
        if(closest_aisles_wo[0] == 1) {
            if(std::max(my_part.pose.position.y, AISLE_Y[1]) - std::min(my_part.pose.position.y, AISLE_Y[1]) < 3)      // select closest aisle to part
                   return std::vector<std::string> {"no_gap_needed","-","1","long"};
            else
                   return std::vector<std::string> {"gap_needed",getGapName("middle_gap"),"1","short", getGapCoorX(getGapName("middle_gap"))};
        }
        else{
            if(std::max(my_part.pose.position.y, AISLE_Y[3]) - std::min(my_part.pose.position.y, AISLE_Y[3]) < 3)      // select closest aisle to part
                return std::vector<std::string> {"no_gap_needed","-","3","long"};
            else
                return std::vector<std::string> {"gap_needed",getGapName("right_gap"),"3","short", getGapCoorX(getGapName("right_gap"))};
         }
      }
   }else{
     for(auto aisle:closest_aisles_wo){
        if(std::max(my_part.pose.position.y, AISLE_Y[aisle]) - std::min(my_part.pose.position.y, AISLE_Y[aisle]) < 3)      // select closest aisle to part
          return std::vector<std::string> {"no_gap_needed","-",std::to_string(aisle),"long"};
     }
   }

   ROS_INFO_STREAM("Exited planPath");
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
   

   if(td >= tf) {
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
    if(obstacleInAisle[aisle_num] == true  &&  obstacleAssociatedWithAisle[aisle_num].is_valid_obstacle == false && vec.size() > 8) {
       std::string str1 =  "shelf_breakbeam_" + std::to_string(aisle_num*3 + 2) + "_frame";
       std::string str2 =  "shelf_breakbeam_" + std::to_string(aisle_num*3 + 1) + "_frame";

       ROS_INFO_STREAM("estimating velocity for aisle" << aisle_num);
       ROS_INFO_STREAM("vector size is " << vec.size());
       auto it = std::find_if(vec.begin(), vec.end(),
                         [&str = str1] 
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

       auto it2 = std::find_if(it, vec.end(),
                         [&str = str2]
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
       double wait_time = sec2 - sec1;
       double move_time = sec3 - sec1 - wait_time;
       ROS_INFO_STREAM(sec3 - sec2);

       double velocity = LENGTH_OF_AISLE/move_time;
       double distanceBtnLastBreakBeams   = 4.462133;

       obstacle human;
       human.wait_time = round(human.wait_time);
       human.move_time = round(((BEGIN_LOCATION_X - END_LOCATION_X)*move_time)/distanceBtnLastBreakBeams);
       human.time_stamp1 = sec1;
       human.is_valid_obstacle = true;
         
       obstacleAssociatedWithAisle[aisle_num] = human;
       ROS_INFO_STREAM("Printing time interval");
       ROS_INFO_STREAM("sec1 is " << sec1);
       ROS_INFO_STREAM("sec2 is " << sec2);
       ROS_INFO_STREAM("sec3 is " << sec3);
       ROS_INFO_STREAM("wait time is " << human.wait_time);
       ROS_INFO_STREAM("move time is " << human.move_time);
       
    }
      ROS_INFO_STREAM("exited estimateObstacleAttributes method");
}



void planAndExecutePath(product prod, part my_part,std::map<std::string, std::vector<PresetLocation>> &presetLoc, 
                        Camera &camera, GantryControl &gantry, Competition &comp, int aisle_num) {

   ROS_INFO_STREAM("Plan and execute method");
   auto plan = planPath(aisle_num,my_part);
   ROS_INFO_STREAM(plan.size());
   for(int i=0;i < plan.size(); i++)
       ROS_INFO_STREAM(plan[i]);
   std::string location;
   float threshold = 0.7; 

   if(plan[0] == "no_gap_needed") {
       location = my_part.logicalCameraName + "_aisle" + plan[2] + "_" + plan[3];
       moveToLocation2(presetLoc, my_part, gantry, location);
       gantry.pickPart(my_part);

//       retraceSteps(presetLoc, location, gantry);
       moveFromLocationToGoal(prod, my_part,presetLoc, location, gantry);
   }
   else{
       location = plan[1] + "_" + my_part.logicalCameraName + "_" + plan[3];



       // Move to the gap
       moveToGap(presetLoc, my_part, gantry, location);
       while(!obstacleAssociatedWithAisle[aisle_num].is_valid_obstacle)
           estimateObstacleAttributes(camera,aisle_num);
       while(true) {
               auto vec = estimateLocation(aisle_num, comp.getClock());                      //estimate Location
             
               ROS_INFO_STREAM("Gap Coordinate" << std::stod(plan[4]));
               ROS_INFO_STREAM("VEC1" << vec[1]);
               if(vec[0] == "toward" && std::stod(vec[1]) - std::stod(plan[4]) > threshold) {
                  ROS_INFO_STREAM("Location is " <<  location);
                  //Move from gap to location
                  moveFromGapToLocation(presetLoc, my_part, gantry, location);
//                  moveToLocation(presetLoc, location, gantry);
                  gantry.pickPart(my_part);
//                  retraceSteps(presetLoc, location, gantry);
                  moveFromLocationToGoalAvoidingObstacles(prod, presetLoc, location, gantry);
                  break;
               }
         }
    }
}




std::string getLocationName(part my_part, int aisle_num){
    std::string location;
    if(my_part.logicalCameraName == "logical_camera_3" || my_part.logicalCameraName == "logical_camera_7") {
        if(my_part.pose.position.y > -3.55)
            location = my_part.logicalCameraName + "_short";
        else
            location = my_part.logicalCameraName + "_long";
    }else if(my_part.logicalCameraName == "logical_camera_0" || my_part.logicalCameraName == "logical_camera_4"){
        if(my_part.pose.position.y > 3.5)
            location = my_part.logicalCameraName + "_long";
        else
            location = my_part.logicalCameraName + "_short";
    }else if(my_part.logicalCameraName =="logical_camera_1" || my_part.logicalCameraName =="logical_camera_2" ||
             my_part.logicalCameraName =="logical_camera_5" || my_part.logicalCameraName =="logical_camera_6"){
        location = my_part.logicalCameraName;
    }else
        location = my_part.logicalCameraName + "_aisle" + std::to_string(aisle_num) + "_short";
    return location;
}

void processPart(product prod, GantryControl &gantry, Camera &camera, Competition &comp, bool priority_flag,  bool flip_flag) {
    part my_part, my_part_in_tray, placed_part, actual_part;
    bool foundPart = false;
    bool flippedPart;
    nist_gear::VacuumGripperState armState;
    std::map<std::string,part> detected_parts;

    while(!foundPart) {                                                                                          // poll until we find part
        if(!camera.isSensorBlackout()) {
            camera.removeAllElements(prod.type);
            camera.reset_conveyor_logical_camera();
            camera.reset_agv_logical_camera("logical_camera_10");
            camera.reset_agv_logical_camera("logical_camera_8");
            ROS_INFO_STREAM("rEMOVING ALL ELEMENTS");
        }
        detected_parts = camera.get_detected_parts()[prod.type];

        ROS_INFO_STREAM("Printing detected part(s");
        for(const auto & parts: detected_parts){
            ROS_INFO_STREAM("parts .firts "<< parts.first);
            ROS_INFO_STREAM("parts .second  "<< parts.second.logicalCameraName);
            ROS_INFO_STREAM("Part x position " << parts.second.pose.position.x);
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

//                detected_parts.erase(parts.first);

                int aisle_num = aisleAssociatedWithPart(my_part);

                if (aisle_num != -1 && obstacleInAisle[aisle_num]) {
                    ROS_INFO_STREAM("In process part- obstacle in aisle");
                    ROS_INFO_STREAM(parts.second.logicalCameraName);
                    ROS_INFO_STREAM(parts.first);
                    planAndExecutePath( prod, my_part, presetLoc, camera, gantry, comp, aisle_num);
                }
                else {
                    ROS_INFO_STREAM("bEFORE" << getLocationName(my_part,aisle_num));
                    moveToLocation2(presetLoc, my_part, gantry, getLocationName(my_part,aisle_num));
                    if( gantry.pickPart(my_part)){
                       ROS_INFO_STREAM("successfully picked part");
                       moveFromLocationToGoal(prod, my_part, presetLoc, getLocationName(my_part,aisle_num), gantry);
                    }else{
                       ROS_INFO_STREAM("Failed to pick part up");
                       moveFromLocationToStart(presetLoc, getLocationName(my_part,aisle_num), gantry);
                       continue;
                    }
                    ROS_INFO_STREAM("AFTER" << getLocationName(my_part,aisle_num));
                    ROS_INFO_STREAM("FG0");
                }

                ROS_INFO_STREAM("FG1");
                if (flip_flag && int(my_part_in_tray.pose.orientation.x) == 1) {                                   //Flip part if part needs to be flipped
                    //moveToLocation(presetLoc,prod.agv_id+"_flipped",gantry);                                       //go to location to flip pulley
                    flipped = true;
                    flipPart(gantry, camera, my_part_in_tray, prod);
//                    moveToLocation(presetLoc,prod.agv_id+"_flipped",gantry);
                }
                else
                    flipped = false;
//                else
//                    moveToLocation(presetLoc, prod.agv_id, gantry);                                                //move to desired agv id
                ROS_INFO_STREAM("FG2");
                armState = gantry.getGripperState(prod.arm_name);
                ROS_INFO_STREAM("aRM STATE" << armState.attached);
                if (!armState.attached) {                                                                            //object accidentally fell on the tray
                    ROS_INFO_STREAM("Inside faulty gripper");
                    prod.arm_name = "left_arm";
                    faultyGripper(gantry, prod, camera, my_part_in_tray);
                    ROS_INFO_STREAM("Outside faulty gripper");
                }


                if(!camera.isSensorBlackout()) {
                    camera.removeAllElements(prod.type);
                    ROS_INFO_STREAM("rEMOVING ALL ELEMENTS");
                }
                else {
                    camera.removeElement(prod.type, parts.first);
                    ROS_INFO_STREAM("rEMOVING ELEMENTS");
                }
                gantry.placePart(my_part_in_tray, prod.agv_id, prod.arm_name);                                 //place part on the tray
                if(flipped)
                    moveToLocation(presetLoc,prod.agv_id+"_flipped",gantry);
                ROS_INFO_STREAM("bEFORE PROCESSED PART");
                keepTrackOfProcessedParts(my_part, prod,gantry,camera);


                auto hx = camera.get_detected_parts()[prod.type];
                ROS_INFO_STREAM("Printing detected part(s before");
                for(const auto & parts: hx){
                    ROS_INFO_STREAM("parts .firts "<< parts.first);
                    ROS_INFO_STREAM("parts .second  "<< parts.second.logicalCameraName);
                }

                ROS_INFO_STREAM("part to key remove is " << parts.first);
                ROS_INFO_STREAM("part to type remove is " << parts.first);


                hx = camera.get_detected_parts()[prod.type];

                ROS_INFO_STREAM("Printing detected after");
                for(const auto & parts: hx){
                    ROS_INFO_STREAM("parts .firts "<< parts.first);
                    ROS_INFO_STREAM("parts .second  "<< parts.second.logicalCameraName);
                }

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
         if(std::max(part.second.pose.position.x,faulty_pose.second.position.x) - std::min(part.second.pose.position.x,faulty_pose.second.position.x)< 0.2 &&
            std::max(part.second.pose.position.y,faulty_pose.second.position.y)- std::min(part.second.pose.position.y,faulty_pose.second.position.y)< 0.2){
            //abs(part.second.pose.orientation.x - faulty_pose.second.orientation.x)< 0.2 &&
            //abs(part.second.pose.orientation.y - faulty_pose.second.orientation.y)< 0.2 &&
            //abs(part.second.pose.orientation.z - faulty_pose.second.orientation.z)< 0.2 &&
            //abs(part.second.pose.orientation.w - faulty_pose.second.orientation.w)< 0.2 ) {

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

    camera.removeAllFaultyElements(agv_id);

    if(!delivered)
         agvDeliveryService(agv_id,agv1Delivery,agv2Delivery,shipment_type);
}


//TODO- Sensor blackout
void removeFaultyProduct(Camera &camera, GantryControl &gantry, product &prod, bool flipped) {

    ROS_INFO_STREAM("IN faulty part");
    part temp;

    temp.pose = camera.get_faulty_pose(prod.agv_id);
    temp.type = prod.type;
    if(prod.agv_id == "agv2")
        temp.logicalCameraName = "logical_camera_10";
    else
        temp.logicalCameraName = "logical_camera_8";
    moveToLocation(presetLoc, prod.agv_id + "_faultyP", gantry);
    gantry.moveToPart(temp, gantry.start_);

    if(!flipped)
        gantry.pickPart(temp);
    else
        gantry.pickPulleyPart(temp);

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


void processShipment(nist_gear::Shipment &ship,Camera &camera, GantryControl &gantry,Competition &comp, 
                    ros::ServiceClient &agv2Delivery, ros::ServiceClient &agv1Delivery){

      product prod;
      for (int k = 0; k < ship.products.size(); k++) {
          auto product = ship.products[k];

          prod.type = product.type;
          prod.pose = product.pose;
          prod.agv_id = ship.agv_id;
          prod.arm_name = "left_arm";
            
          processPart(prod, gantry, camera, comp, false, false);

          moveFromLocationToStart(presetLoc, "start", gantry);
          ROS_INFO("HELLO1");

          if (compIsAlmostOver(TIMELIMIT_THRESHOLD, comp)) {
              agvDeliveryService(prod.agv_id, agv1Delivery, agv2Delivery, ship.shipment_type);
              stop_processing = true;
              break;
          }
      }

      if (compIsAlmostOver(TIMELIMIT_THRESHOLD, comp)) {
          agvDeliveryService(prod.agv_id, agv1Delivery, agv2Delivery, ship.shipment_type);
          stop_processing = true;
      } else {
          //TODO sensor blackout
          if (camera.get_faulty_poses()[prod.agv_id].size() > 0)
              removeReplaceFaultyProductsAndDeliver(camera, gantry, prod.agv_id, agv1Delivery, agv2Delivery,
                                                    ship.shipment_type, comp);
          else
              agvDeliveryService(prod.agv_id, agv1Delivery, agv2Delivery, ship.shipment_type);
     }
}


void addWantedParts(std::vector<product> &wanted_products, GantryControl &gantry, Camera &camera, Competition &comp) {
  for(int i=0; i< wanted_products.size(); i++) {
     processPart(wanted_products[i], gantry, camera, comp, false, true);
  }
}


void repositionParts(std::vector<part> reposition_parts,std::string agv_id, GantryControl &gantry, Camera &camera, Competition &comp) {
    moveToLocation(presetLoc, agv_id,gantry);
    for(int i=0 ; i < reposition_parts.size(); i++) {
       gantry.pickPart(reposition_parts[i]);
       //need preset locations for placing part
       //need preset locations for picking and placing part back;
       //place part at reposition_pose
    }
}

void removeUnwatedParts(std::vector<part> &unwanted_parts,std::string agv_id, GantryControl &gantry, Camera &camera, Competition &comp) {
    moveToLocation(presetLoc, agv_id,gantry);
    for(int i=0 ; i < unwanted_parts.size(); i++) {
       gantry.pickPart(unwanted_parts[i]);
       //pick part placed_part
       //and throw away or store in a bin
    }
}


void processHPOrder(nist_gear::Order &order,Camera &camera, GantryControl &gantry,Competition &comp, 
                    ros::ServiceClient &agv2Delivery, ros::ServiceClient &agv1Delivery, std::string previous_order_agv_id){

    ROS_INFO_STREAM("Processing HP order");
    product prod;
    std::vector<part> unwanted_parts;
    std::vector<product> wanted_products;
    std::vector<part> reposition_parts;

    for(int j=0; j<order.shipments.size(); j++){
        auto ship = order.shipments[j];
        
        if (ship.agv_id == previous_order_agv_id) {
            PreviousOrderAgvChange = true;
       
              for(int k=0; k< ship.products.size(); k++){
                  auto product = ship.products[k];

                  prod.type = product.type;
                  prod.pose = product.pose;
                  prod.agv_id = ship.agv_id;
                  prod.arm_name = "left_arm";

                  bool found = false;
                  for(int i=0; i< processedParts.size(); i++) {
                      if(prod.type == processedParts[i].type){
                         part new_part = processedParts[i];
                         new_part.reposition_pose = prod.pose;
                         reposition_parts.push_back(new_part);
                         processedParts.erase(processedParts.begin()+i);
                         found = true;
                         break;
                      }
                  }

                  if(found == false){
                     wanted_products.push_back(prod);
                  }
              }
              unwanted_parts = processedParts; 
              processedParts = std::vector<part>{};

              removeUnwatedParts(unwanted_parts,ship.agv_id,gantry,camera,comp);
              repositionParts(reposition_parts,ship.agv_id,gantry,camera,comp);
              addWantedParts(wanted_products, gantry, camera, comp);
        }else 
             processShipment(ship,camera, gantry,comp, agv2Delivery, agv1Delivery);
   }
}


void conveyor(Camera &camera, GantryControl &gantry, product prod){
    ROS_INFO_STREAM("Break beam triggered");
//    moveToLocation(presetLoc, "movingPart", gantry);
    nist_gear::VacuumGripperState armState;
    part imgPart, my_part_in_tray;
    my_part_in_tray.type = prod.type;
    my_part_in_tray.pose = prod.pose;
    imgPart.type = prod.type;
    imgPart.pose.orientation.x = 0;
    imgPart.pose.orientation.y = 0;
    imgPart.pose.orientation.z = 0;
    imgPart.pose.orientation.w = 1;

    if(prod.type.find("pulley_part") != -1){
        moveToLocation(presetLoc, "movingPart", gantry);
        imgPart.pose.position.x = 0;
        imgPart.pose.position.y = -0.46;//-0.5650; //-0.272441; //-0.5700
//    imgPart.pose.position.z = 0.874991;//0.864004; //0.875004;0.874988
        imgPart.pose.position.z = 0.888;// 0.879 (kinda works)//0.884991
    }
    else if(prod.type.find("gasket_part") != -1){
        moveToLocation(presetLoc, "movingPart", gantry);
        imgPart.pose.position.x = 0;
        imgPart.pose.position.y = -0.464;//-0.5650; //-0.272441; //-0.5700
//    imgPart.pose.position.z = 0.874991;//0.864004; //0.875004;0.874988
        imgPart.pose.position.z = 0.88;// 0.879 (kinda works)//0.884991
    }
    else if(prod.type.find("disk_part") != -1){
        moveToLocation(presetLoc, "movingPartDisk", gantry);
        imgPart.pose.position.x = 0;
        imgPart.pose.position.y = -0.484;//-0.5650; //-0.272441; //-0.5700
//    imgPart.pose.position.z = 0.874991;//0.864004; //0.875004;0.874988
        imgPart.pose.position.z = 0.88;// 0.879 (kinda works)//0.884991
    }
    else if(prod.type.find("gear_part") != -1){
        moveToLocation(presetLoc, "movingPartGear", gantry);
        imgPart.pose.position.x = 0;
        imgPart.pose.position.y = -0.64;//-0.5650; //-0.272441; //-0.5700
//    imgPart.pose.position.z = 0.874991;//0.864004; //0.875004;0.874988
        imgPart.pose.position.z = 0.88;// 0.879 (kinda works)//0.884991
    }
    else{
        moveToLocation(presetLoc, "movingPartGear", gantry);
        imgPart.pose.position.x = 0;
        imgPart.pose.position.y = -0.64;//-0.5650; //-0.272441; //-0.5700
//    imgPart.pose.position.z = 0.874991;//0.864004; //0.875004;0.874988
        imgPart.pose.position.z = 0.88;// 0.879 (kinda works)//0.884991
    }

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



void pickPartsFromConveyor2(Camera &camera, Competition &comp, GantryControl &gantry, product prod, int numParts){
     part conveyor_part = camera.get_conveyor_detected_parts()["logical_camera_9"];
     
     double conveyor_speed = 0.2;
     double location_y0 = conveyor_part.pose.position.y;

     double location_y = (comp.getClock() - conveyor_part.conveyor_time)*conveyor_speed + location_y0;
     if(location_y > -10000) {
       conveyor_part.pose.position.y = location_y;
       gantry.moveToPart(conveyor_part, gantry.start_);
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
//            camera.reset_break_beam();
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
//                ros::Duration(1).sleep();
//                while(camera.get_break_beam()){}
//                camera.reset_break_beam();
                ROS_INFO_STREAM("HERE2");
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
    comp_ref = &comp;

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


    ros::Duration(3.0).sleep();                                                                              //wait for sometime and
    detectAislesWithObstacles(camera);                                                                       //determine obstacles in Aisles
    

    ConveyorFlag = false;
    int numPickParts = 4;



    // for debugging
    //for(int i = 0; i<4; i++) {
      //if(obstacleInAisle[i] == true){
         //while(!obstacleAssociatedWithAisle[i].is_valid_obstacle)
              //estimateObstacleAttributes(camera,i);
       //}
    //}


    //}
    //obstacleAssociatedWithAisle[2].is_valid_obstacle= true;
    //obstacleAssociatedWithAisle[2].wait_time= 7;
    //obstacleAssociatedWithAisle[2].move_time= 9;
    //obstacleAssociatedWithAisle[2].time_stamp1= 9;

    //obstacleAssociatedWithAisle[3].is_valid_obstacle= true;
    //obstacleAssociatedWithAisle[3].wait_time= 7;
    //obstacleAssociatedWithAisle[3].move_time= 9;
    //obstacleAssociatedWithAisle[3].time_stamp1= 9;



//    auto  vec = estimateLocation(2,15);
//    ROS_INFO_STREAM("Time at: " << 15 << " action:" << vec[0] << " location: " <<vec[1]);
//    vec= estimateLocation(2,20);
//    ROS_INFO_STREAM("Time at: " << 20 << " action:" << vec[0] << " location: " <<vec[1]);
//
//    vec = estimateLocation(2,30);
//    ROS_INFO_STREAM("Time at: " << 25 << " action:" << vec[0] << " location: " <<vec[1]);
//
//    vec = estimateLocation(2,40);
//    ROS_INFO_STREAM("Time at: " << 30 << " action:" << vec[0] << " location: " <<vec[1]);

    std::cout << "finished estimating obstacle parameters" << std::endl;
    int n = orders.size();
    for(int i = 0; i< n; i++){
        auto order = comp.getOrders()[i];
        
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
                ros::Duration(18.0).sleep();
                if(!ConveyorFlag && camera.get_conveyor_detected_parts().size()>0) {
                    ROS_INFO_STREAM("processing conveyor belt");
                    pickPartsFromConveyor(camera, gantry, prod, numPickParts);
                    ConveyorFlag = true;
                    camera.reset_conveyor_logical_camera();
                }


                // TODO - make high priority order checker more robust
                if(comp.getOrders().size() > n && !HighPriorityOrderInitiated){
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
                else {
                    processPart(prod, gantry, camera, comp, false, true);        //Remove flip_flag after degugging
                }


                //TODO - Make checker for faulty more robust
                ROS_INFO_STREAM("heere 1");
                ros::Duration(3.0).sleep();
                ros::spinOnce();
                ros::spinOnce();


                if(camera.get_is_faulty(prod.agv_id)) {
                    ROS_INFO_STREAM("FAULTY");
                    removeFaultyProduct(camera,gantry,prod, flipped);                       // remove product
                    k--;                                                           // process product again
                    flipped = false;
                    camera.reset_is_faulty();
                    camera.removeAllFaultyElements(prod.agv_id);
                }
                ROS_INFO_STREAM("heere 2");
                moveFromLocationToStart(presetLoc,"start",gantry);
                ROS_INFO_STREAM("heere x");

                if(compIsAlmostOver(TIMELIMIT_THRESHOLD, comp)){
                    agvDeliveryService(prod.agv_id,agv1Delivery,agv2Delivery,order.shipments[j].shipment_type);
                    stop_processing = true;
                    break;
                } 
            }
            ROS_INFO_STREAM("ORDER SIZE" << comp.getOrders().size());
            ROS_INFO_STREAM("ORDERS LENGTH" << n);
            if(comp.getOrders().size() > n && !HighPriorityOrderInitiated){
                n = comp.getOrders().size();
                ROS_INFO_STREAM("eNTERED Here");
            }
            ROS_INFO_STREAM("herex1");
            auto faulty_poses = camera.get_faulty_poses();
            

            if(compIsAlmostOver(TIMELIMIT_THRESHOLD, comp)) {
                 agvDeliveryService(prod.agv_id,agv1Delivery,agv2Delivery,order.shipments[j].shipment_type);
                 stop_processing = true;
                 break;
            }
            else{
                //TODO sensor blackout
                if(faulty_poses[prod.agv_id].size() > 0)
                    removeReplaceFaultyProductsAndDeliver(camera,gantry,prod.agv_id,agv1Delivery,agv2Delivery,order.shipments[j].shipment_type, comp);
                else
                    agvDeliveryService(prod.agv_id,agv1Delivery,agv2Delivery,order.shipments[j].shipment_type);
            }
        }
        ROS_INFO_STREAM("here3");
    }

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}





