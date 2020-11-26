#include "gantry_control.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

GantryControl::GantryControl(ros::NodeHandle & node):
        node_("/ariac/gantry"),
        planning_group_ ("/ariac/gantry/robot_description"),
        full_robot_options_("Full_Robot",planning_group_,node_),
        left_arm_options_("Left_Arm",planning_group_,node_),
        right_arm_options_("Right_Arm",planning_group_,node_),
        left_ee_link_options_("Left_Endeffector",planning_group_,node_),
        right_ee_link_options_("Right_Endeffector",planning_group_,node_),
        full_robot_group_(full_robot_options_),
        left_arm_group_(left_arm_options_),
        right_arm_group_(right_arm_options_),
        left_ee_link_group_(left_ee_link_options_),
        right_ee_link_group_(right_ee_link_options_)
{
    ROS_INFO_STREAM("[GantryControl::GantryControl] constructor called... ");
}


void GantryControl::init() {
    ROS_INFO_STREAM("[GantryControl::init] init... ");
    double time_called = ros::Time::now().toSec();


    ROS_INFO_NAMED("init", "Planning frame: %s", left_arm_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", right_arm_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", full_robot_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", right_ee_link_group_.getPlanningFrame().c_str());
    ROS_INFO_NAMED("init", "Planning frame: %s", left_ee_link_group_.getPlanningFrame().c_str());

    ROS_INFO_NAMED("init", "End effector link: %s", left_arm_group_.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("init", "End effector link: %s", right_arm_group_.getEndEffectorLink().c_str());

    left_arm_group_.setPoseReferenceFrame("world");

    //--start location
    start_.gantry = {0,0,0};
    start_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    start_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //--bin3
    //--bin13
    // bin13_.gantry = {3.32, 1.12, 3.77};//, 2.10};
    // bin13_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    // bin13_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


    bin13_.gantry = {3.10, 1.68, 3.77};//, 2.10};
    bin13_.left_arm = {0, -0.63, 1.26, -0.78, PI/2, 0};
    bin13_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    // ---------------------------------------------------- //
    // conveyor part deactivate gripper location - 1
//    bin13_1_.gantry = {2.30, 2.30, 0.28};//, 2.10};
//    bin13_1_.left_arm = {0, -0.63, 1.26, -0.78, PI/2, 0};
//    bin13_1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
//    // conveyor part deactivate gripper location - 2
//    bin13_2_.gantry = {2.00, 2.30, 0.28};//, 2.10};
//    bin13_2_.left_arm = {0, -0.63, 1.26, -0.78, PI/2, 0};
//    bin13_2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
//    // conveyor part deactivate gripper location -3
//    bin13_3_.gantry = {1.94, 2.52, 0.15};//, 2.10};
//    bin13_3_.left_arm = {0, -0.63, 1.26, -0.78, PI/2, 0};
//    bin13_3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
//    // conveyor part deactivate gripper location -4
//    bin13_4_.gantry = {2.28, 2.52, 0.15};//, 2.10};
//    bin13_4_.left_arm = {0, -0.63, 1.26, -0.78, PI/2, 0};
//    bin13_4_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    // conveyor part deactivate gripper location - 1
    bin13_1_.gantry = {2.20, 2.30, 0.20};//, 2.10};
    bin13_1_.left_arm = {0, -0.63, 1.26, -0.65, PI/2, 0};
    bin13_1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    // conveyor part deactivate gripper location - 2
    bin13_2_.gantry = {2.05, 2.30, 0.20};//, 2.10};
    bin13_2_.left_arm = {0, -0.63, 1.26, -0.65, PI/2, 0};
    bin13_2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    // conveyor part deactivate gripper location -3
    bin13_3_.gantry = {2.05, 2.50, 0.15};//, 2.10};
    bin13_3_.left_arm = {0, -0.63, 1.26, -0.65, PI/2, 0};
    bin13_3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    // conveyor part deactivate gripper location -4
    bin13_4_.gantry = {2.20, 2.52, 0.15};//, 2.10};
    bin13_4_.left_arm = {0, -0.63, 1.26, -0.65, PI/2, 0};
    bin13_4_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    // ---------------------------------------------------- //

    //--agv2
    agv2_.gantry = {0.6, 6.9, PI};
    agv2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //--agv1
    agv1_.gantry = {-0.6, -6.9, 0.0};
    agv1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //--agv2_1 for faulty gripper
//    agv2_faultyG_.gantry = {0.6, 6.9, PI};
//    agv2_faultyG_.left_arm = {0.0, -1.01, 1.76, -PI/4, PI/2, 0};
//    agv2_faultyG_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv2_faultyG_.gantry = {0.7, 6.9, PI};
    agv2_faultyG_.left_arm = {0.0, -1.38, 2.14, -PI/4, PI/2, 0};
    agv2_faultyG_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv1_faultyG_.gantry = {-0.7, -6.9, 0.0};
    agv1_faultyG_.left_arm = {0.0, -1.38, 2.14, -PI/4, PI/2, 0};
    agv1_faultyG_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


//    agv2_.gantry = {0, 6.30, 0};
//    agv2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    agv2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

//    //--bin13
//     bin13_.gantry = {3.10, 1.68, 3.77};//, 2.10};
//     bin13_.left_arm = {0, -0.63, 1.26, -0.78, PI/2, 0};
//     bin13_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

//    //--bin13
//    bin13_.gantry = {1.85, 2.14,0};//, 2.10};
//    // bin13_.left_arm = {0, -1.45, 1.58, -0.13, PI/2, -0.88};
//    bin13_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    bin13_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //--bin13
    bin13_.gantry = {2.10, 2.25,0};//, 2.10};
    // bin13_.left_arm = {0, -1.45, 1.58, -0.13, PI/2, -0.88};
    bin13_.left_arm = {0.0, -PI/4, PI/2, -0.80, PI/2, 0};
    bin13_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};


//   //--bin16
//    bin16_.gantry = {6.25, 1.96, -3.14};
//    bin16_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    bin16_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //--bin16
    bin16_.gantry = {4.60, 1.95, 0};
    bin16_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    bin16_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //--shelf5 - waypoint 1
    shelf5_1_.gantry = {0.0, -4.76, 0};
    shelf5_1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf5_1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //--shelf5 - waypoint 2
//    shelf5_2_.gantry = {-15.13, -5.46, 0};
//    shelf5_2_.left_arm = {-1.25, -PI/2, PI/2, 0, 0.25, 1.38};
//    shelf5_2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
//
//    //--shelf5 - waypoint 3
//    shelf5_3_.gantry = {-14.23, -4.25, 0};
//    shelf5_3_.left_arm = {-1.39, -0.75, 1.26, 0, 0.28, 1.38};
//    shelf5_3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf5_2_.gantry = {-15.13, -5.46, 0};
    shelf5_2_.left_arm = {-1.88, -0.73, 1.46, -0.76, 0.25, -1.88};
    shelf5_2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    //--shelf5 - waypoint 3
//    shelf5_3_.gantry = {-14.23, -4.25, 0};
//    shelf5_3_.left_arm = {-1.39, -0.75, 1.26, 0, 0.28, 1.38};
//    shelf5_3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf5_3_.gantry = {-14.68, -4.34, 0};
    shelf5_3_.left_arm = {-1.88, -0.73, 1.46, -0.76, -0.25, 0};
    shelf5_3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //--shelf5 - waypoint 4
    shelf5_4_.gantry = {-15.19, -5.46, 0};
    shelf5_4_.left_arm = {-1.39, -0.75, 1.26, 0, 0.28, 1.38};
    shelf5_4_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf5_5_.gantry = {-15.19, -4.25, 0};
    shelf5_5_.left_arm = {-1.39, -0.75, 1.26, 0, 0.28, 1.38};
    shelf5_5_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //--shelf8 - waypoint 1
    shelf8_1_.gantry = {0.18, -1.50, 0};
    shelf8_1_.left_arm = {-2.39, -PI/4, PI/2, -PI/4, 0, 0};
    shelf8_1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf8_2_.gantry = {-14.68, -1.50, 0};
    shelf8_2_.left_arm = {-2.39, -PI/4, PI/2, -PI/4, 0, 0};
    shelf8_2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    // shelf8_3_.gantry = {-14.07, -1.14, 0};
    // shelf8_3_.left_arm = {-1.24, -PI/4, PI/2, -PI/4, 0.30, 0};
    // shelf8_3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    // shelf8_3_.gantry = {-14.55, -1.14, 0};
    // shelf8_3_.left_arm = {-1.57, -PI/4, PI/2, -PI/4, 0.0, 0};
    // shelf8_3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf8_3_.gantry = {-14.55, -1.2, 0};
    shelf8_3_.left_arm = {-1.78, -PI/4, PI/2, -PI/4, -0.20, 0};
    shelf8_3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf11_1_.gantry = {0.18, 1.50, 0};
    shelf11_1_.left_arm = {-2.39, -PI/4, PI/2, -PI/4, 0, 0};
    shelf11_1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf11_2_.gantry = {-14.68, 1.50, 0};
    shelf11_2_.left_arm = {-2.39, -PI/4, PI/2, -PI/4, 0, 0};
    shelf11_2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    shelf11_3_.gantry = {-14.55, 1.8, 0};
    shelf11_3_.left_arm = {-1.78, -PI/4, PI/2, -PI/4, -0.20, 0};
    shelf11_3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv2_go_to_flipped_pulley_.gantry = {0, 6.2, 0};
    agv2_go_to_flipped_pulley_.left_arm = {1.84, -2.73, -1.88, -0.2, 1.63, 0};
    agv2_go_to_flipped_pulley_.right_arm = {1.75, -3.35, -1.4, 0.13, 1.51, 0};

    agv1_go_to_flipped_pulley_.gantry = {0, -6.2, 3.14};
    agv1_go_to_flipped_pulley_.left_arm = {1.84, -2.73, -1.88, -0.2, 1.63, 0};
    agv1_go_to_flipped_pulley_.right_arm = {1.75, -3.35, -1.4, 0.13, 1.51, 0};

    agv2_flipped1_.gantry = {0.6, 6.9, 0};
    agv2_flipped1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    // agv2_flipped1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2_flipped1_.right_arm =  {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv2_flipped_.gantry = {0.6, 6.30, 0};
    agv2_flipped_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2_flipped_.right_arm = {1.57, -1.57, -2.26, 0.13, 0, 0.13};

    agv1_flipped1_.gantry = {-0.6, -6.9, 3.14};
    agv1_flipped1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    // agv2_flipped1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1_flipped1_.right_arm =  {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv1_flipped_.gantry = {-0.6, -6.45, PI};
    agv1_flipped_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1_flipped_.right_arm = {1.57, -1.57, -2.26, 0.13, 0, 0.13};

    agv1_drop_.gantry = {-0.6, -6.30, 0};
    agv1_drop_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv1_drop_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    agv2_drop_.gantry = {0.6, 6.30, 0};
    agv2_drop_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2_drop_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

   movingPart_.gantry = {-0.16, -0.22, -1.57};
   movingPart_.left_arm = {0.0, -0.63, 1.38, -0.73, PI/2, 0};
   movingPart_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

   movingPart1_.gantry = {-0.16, -0.22, -1.57};
   movingPart1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
   movingPart1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

   agv1_gasket_part_green_.gantry = {-0.7, -6.65, 0.01};
   agv1_gasket_part_green_.left_arm = {0.0, -PI/4, PI/2, -0.78, 1.63, 0};
   agv1_gasket_part_green_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

// ------------------------------------------------------------------------ //


// // shel8_obs_green - waypoint 1
//    shelf8_obs_green1_.gantry = {0.0, -5.18, 0};
//    shelf8_obs_green1_.left_arm = {-0.17, -0.13, 0.25, -0.77, 1.34, 0};
//    shelf8_obs_green1_.right_arm = {3.14, 0, 0.25, -0.78, 1.76, 1.88};
//
// // shel8_obs_green - waypoint 2
//    shelf8_obs_green2_.gantry = {-11.3, -5.18, 0};
//    shelf8_obs_green2_.left_arm = {-0.17, -0.13, 0.25, -0.77, 1.34, 0};
//    shelf8_obs_green2_.right_arm = {3.14, 0, 0.25, -0.78, 1.76, 1.88};
//
// // shel8_obs_green - waypoint 3
//    shelf8_obs_green3_.gantry = {-11.3, -3.5, 0};
//    shelf8_obs_green3_.left_arm = {-0.17, -0.13, 0.25, -0.77, 1.34, 0};
//    shelf8_obs_green3_.right_arm = {3.14, 0, 0.25, -0.78, 1.76, 1.88};

// shel8_obs_green - waypoint 1
    shelf8_obs_green1_.gantry = {0.0, -5.18, 0};
    shelf8_obs_green1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf8_obs_green1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

// shel8_obs_green - waypoint 2
    shelf8_obs_green2_.gantry = {-11.3, -5.18, 0};
    shelf8_obs_green2_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    shelf8_obs_green2_.right_arm = {0.13, -0.13, 0.00, 0.1, 0, 0};

// shel8_obs_green - waypoint 3
    shelf8_obs_green3_.gantry = {-11.3, -3.15, 0};
    shelf8_obs_green3_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    shelf8_obs_green3_.right_arm = {0.13, -0.13, 0.00, 0.1, 0, 0};

// shel8_obs_green - waypoint 4
    shelf8_obs_green4_.gantry = {-11.3, -1.50, 0};
    shelf8_obs_green4_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf8_obs_green4_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

// shel8_obs_green - waypoint 5
    shelf8_obs_green5_.gantry = {-14.55, -1.2, 0};
    shelf8_obs_green5_.left_arm = {-1.78, -PI/4, PI/2, -PI/4, -0.20, 0};
    shelf8_obs_green5_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

//    // shel8_obs_blue - waypoint 1
//    shelf8_obs_blue1_.gantry = {0.0, 5.18, 3.14};
//    shelf8_obs_blue1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    shelf8_obs_blue1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
//
//    // shel8_obs_blue - waypoint 2
//    shelf8_obs_blue2_.gantry = {-11.3, 5.18, 3.14};
//    shelf8_obs_blue2_.left_arm = {-0.17, -0.13, 0.25, -0.77, 1.34, 0};
//    shelf8_obs_blue2_.right_arm = {3.14, 0, 0.25, -0.78, 1.76, 1.88};
//
//    // shel8_obs_blue - waypoint 3
//    shelf8_obs_blue3_.gantry = {-11.3, 3.5, 3.14};
//    shelf8_obs_blue3_.left_arm = {-0.17, -0.13, 0.25, -0.77, 1.34, 0};
//    shelf8_obs_blue3_.right_arm = {3.14, 0, 0.25, -0.78, 1.76, 1.88};


// test config modified right arm

    // shel8_obs_blue - waypoint 1
    shelf8_obs_blue1_.gantry = {0.0, 5.18, 3.14};
    shelf8_obs_blue1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf8_obs_blue1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    // shel8_obs_blue - waypoint 2
    shelf8_obs_blue2_.gantry = {-11.3, 5.18, 3.14};
    shelf8_obs_blue2_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    shelf8_obs_blue2_.right_arm = {0.13, -0.13, 0.00, 0.1, 0, 0};

    // shel8_obs_blue - waypoint 3
    shelf8_obs_blue3_.gantry = {-11.3, 3.15, 3.14};
    shelf8_obs_blue3_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    shelf8_obs_blue3_.right_arm = {0.13, -0.13, 0.00, 0.1, 0, 0};


    // shel8_obs_blue - waypoint 4
    shelf8_obs_blue4_.gantry = {-11.3, 1.50, 3.14};
    shelf8_obs_blue4_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf8_obs_blue4_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    // shel8_obs_blue - waypoint 5
    shelf8_obs_blue5_.gantry = {-15.00, 1.5, 3.14};
    shelf8_obs_blue5_.left_arm = {-1.78, -PI/4, PI/2, -PI/4, -0.20, 0};
    shelf8_obs_blue5_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    // shel8_obs_blue - waypoint 6
    shelf8_obs_blue6_.gantry = {-15.00, 1.3, 3.14};
    shelf8_obs_blue6_.left_arm = {-1.78, -PI/4, PI/2, -PI/4, -0.20, 0};
    shelf8_obs_blue6_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

// ----------------------------------------------------------------------------- //

    // middleGap_default
    middle_gap_default_.gantry = {-0.80, -1.54, -1.57};
    middle_gap_default_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    middle_gap_default_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};


    //LeftGap_default

    left_gap_default_.gantry = {-0.8, -5.18, -1.57};
    left_gap_default_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    left_gap_default_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};

    //RightGap_default

    right_gap_default_.gantry = {-0.8, 5.18, -1.57};
    right_gap_default_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    right_gap_default_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};


    // middleGap_0
    middle_gap_0_2_.gantry = {-3.00, 0, -1.57};
    middle_gap_0_2_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    middle_gap_0_2_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};

    // middleGap_1
    middle_gap_1_2_.gantry = {-7.25, -1.54, -1.57};
    middle_gap_1_2_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    middle_gap_1_2_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};

    middle_gap_1_3_.gantry = {-7.25, 0.05, -1.57};
    middle_gap_1_3_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    middle_gap_1_3_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};

    // middleGap_2
    middle_gap_2_2_.gantry = {-11.3, -1.54, -1.57};
    middle_gap_2_2_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    middle_gap_2_2_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};

    middle_gap_2_3_.gantry = {-11.3, 0.05, -1.57};
    middle_gap_2_3_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    middle_gap_2_3_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};

    // middleGap_3
    middle_gap_3_2_.gantry = {-15.20, -1.54, -1.57};
    middle_gap_3_2_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    middle_gap_3_2_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};

    middle_gap_3_3_.gantry = {-15.20, 0.05, -1.57};
    middle_gap_3_3_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    middle_gap_3_3_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};


    //leftGap_0
    left_gap_0_2_.gantry = {-3.00, -3.08, -1.57};
    left_gap_0_2_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    left_gap_0_2_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};


    //LeftGap_1
    left_gap_1_2_.gantry = {-7.25, -5.18, -1.57};
    left_gap_1_2_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    left_gap_1_2_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};


    left_gap_1_3_.gantry = {-7.25, -3.08, -1.57};
    left_gap_1_3_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    left_gap_1_3_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};

    //LeftGap_2
    left_gap_2_2_.gantry = {-11.3, -5.18, -1.57};
    left_gap_2_2_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    left_gap_2_2_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};


    left_gap_2_3_.gantry = {-11.3, -3.08, -1.57};
    left_gap_2_3_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    left_gap_2_3_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};

    //LeftGap_3
    left_gap_3_2_.gantry = {-15.20, -5.18, -1.57};
    left_gap_3_2_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    left_gap_3_2_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};


    left_gap_3_3_.gantry = {-15.20, -3.08, -1.57};
    left_gap_3_3_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    left_gap_3_3_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};


    //rightGap_0
    right_gap_0_2_.gantry = {-3.00, 3.08, -1.57};
    right_gap_0_2_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    right_gap_0_2_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};


    // rightGap_1
    right_gap_1_2_.gantry = {-7.25, 5.18, -1.57};
    right_gap_1_2_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    right_gap_1_2_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};


    right_gap_1_3_.gantry = {-7.25, 3.08, -1.57};
    right_gap_1_3_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    right_gap_1_3_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};

    // rightGap_2
    right_gap_2_2_.gantry = {-11.3, 5.18, -1.57};
    right_gap_2_2_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    right_gap_2_2_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};


    right_gap_2_3_.gantry = {-11.30, 3.08, -1.57};
    right_gap_2_3_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    right_gap_2_3_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};

    // rightGap_3
    right_gap_3_2_.gantry = {-15.20, 5.18, -1.57};
    right_gap_3_2_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    right_gap_3_2_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};


    right_gap_3_3_.gantry = {-15.20, 3.08, -1.57};
    right_gap_3_3_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
    right_gap_3_3_.right_arm = {PI, 0.00, 1.00, -PI/4, PI/2, 0};


    // ------------------------------------------------------- //

//    // leftgap_2 green part pickup waypoints
//    left_gap_2_green_1_.gantry = {-11.30, -1.90, 0};
//    left_gap_2_green_1_.left_arm = {-0.17, -0.13, 0.25, -0.77, 1.34, 0};
//    left_gap_2_green_1_.right_arm = {3.14, 0, 0.25, -0.78, 1.76, 1.88};
//
//    left_gap_2_green_2_.gantry = {-13.70, -1.90, 0};
//    left_gap_2_green_2_.left_arm = {-2.39, -PI/4, PI/2, -PI/4, 0, 0};
//    left_gap_2_green_2_.right_arm = {3.14, 0, 0.25, -0.78, 1.76, 1.88};
//
//    left_gap_2_green_3_.gantry = {-14.13, -1.25, 0};
//    left_gap_2_green_3_.left_arm = {-1.78, -PI/4, PI/2, -PI/4, -0.20, 0};
//    left_gap_2_green_3_.right_arm = {3.14, 0, 0.25, -0.78, 1.76, 1.88};

    // test config left gap 2

    left_gap_2_green_1_.gantry = {-11.30, -1.70, 0};
    left_gap_2_green_1_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    left_gap_2_green_1_.right_arm = {0.13, -0.13, 0.00, 0.1, 0, 0};

    left_gap_2_green_2_.gantry = {-13.75, -1.32, 0};
    left_gap_2_green_2_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    left_gap_2_green_2_.right_arm = {0.13, -0.13, 0.00, 0.1, 0, 0};


    // rightgap_2 green part pickup waypoints

//    right_gap_2_blue_1_.gantry = {-11.30, 1.90, 3.14};
//    right_gap_2_blue_1_.left_arm = {-2.25, -0.15, PI/2, -1.5, 0, 0};
//    right_gap_2_blue_1_.right_arm = {PI, 0.00, 0.87, -PI/4, PI/2, 0};
//
//    right_gap_2_blue_2_.gantry = {-14.65, 1.90, 3.14};
//    right_gap_2_blue_2_.left_arm = {-2.39, -PI/4, PI/2, -PI/4, 0, 0};
//    right_gap_2_blue_2_.right_arm = {3.14, 0, 0.25, -0.78, 1.76, 1.88};

//    right_gap_2_blue_3_.gantry = {-14.92, 1.25, 3.14};
//    right_gap_2_blue_3_.left_arm = {-1.78, -PI/4, PI/2, -PI/4, -0.20, 0};
//    right_gap_2_blue_3_.right_arm = {PI, 0.00, 0.87, -PI/4, PI/2, 0};

// test config right gap 2 - working

    right_gap_2_blue_1_.gantry = {-11.30, 1.90, 3.14};
    right_gap_2_blue_1_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    right_gap_2_blue_1_.right_arm = {0.13, -0.13, 0.00, 0.1, 0, 0};

    right_gap_2_blue_2_.gantry = {-15.40, 1.43, 3.14};
    right_gap_2_blue_2_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
    right_gap_2_blue_2_.right_arm = {0.13, -0.13, 0.00, 0.1, 0, 0};

    // -Final
//    logicalCamera12_.gantry = {-0.80, -1.54, -1.57};;
//    logicalCamera12_.left_arm = {-1.82, -0.40, 1.82, -1.41, -0.25, 0};
//    logicalCamera12_.right_arm = {1.25, -3.27, -1.13, 0.88, -0.13, 0};

    logical_12_15_aisle_1_short_.gantry = {0, -1.48, 0};
    logical_12_15_aisle_1_short_.left_arm = {-1.48, -2.89, -1.74, -1.72, 0, 0};
    logical_12_15_aisle_1_short_.right_arm = {1.49, -0.34, 1.74, -1.53, 3.14, 0};

    logical_12_15_aisle_1_short_1_.gantry = {-14, -1.48, 0};
    logical_12_15_aisle_1_short_1_.left_arm = {-1.48, -2.89, -1.74, -1.72, 0, 0};
    logical_12_15_aisle_1_short_1_.right_arm = {1.49, -0.34, 1.74, -1.53, 3.14, 0};

    logical_12_15_aisle_1_short_2_.gantry = {-14.7, -1.80, -1.57};
    logical_12_15_aisle_1_short_2_.left_arm = {0, -2.05, 1.57, -2.65, -1.57, 0};
    logical_12_15_aisle_1_short_2_.right_arm = {1.51, -1.57, 2.8, -1.44, 3.14, 0};


    logical_12_15_aisle_1_long_2_.gantry = {-15.11, -1.10, -1.57};
    logical_12_15_aisle_1_long_2_.left_arm = {0, -2.05, 1.57, -2.65, -1.57, 0};
    logical_12_15_aisle_1_long_2_.right_arm = {1.51, -1.57, 2.8, -1.44, 3.14, 0};



    logical_13_16_aisle_2_short_.gantry = {0, 1.52, 0};
    logical_13_16_aisle_2_short_.left_arm = {-1.48, -2.89, -1.74, -1.72, 0, 0};
    logical_13_16_aisle_2_short_.right_arm = {1.49, -0.34, 1.74, -1.53, 3.14, 0};

    logical_13_16_aisle_2_short_1_.gantry = {-14, 1.52, 0};
    logical_13_16_aisle_2_short_1_.left_arm = {-1.48, -2.89, -1.74, -1.72, 0, 0};
    logical_13_16_aisle_2_short_1_.right_arm = {1.49, -0.34, 1.74, -1.53, 3.14, 0};

    logical_13_16_aisle_2_short_2_.gantry = {-14.7, 1.20, -1.57};
    logical_13_16_aisle_2_short_2_.left_arm = {0, -2.05, 1.57, -2.65, -1.57, 0};
    logical_13_16_aisle_2_short_2_.right_arm = {1.51, -1.57, 2.8, -1.44, 3.14, 0};

    logical_13_16_aisle_2_long_2_.gantry = {-15.11, 1.90, -1.57};
    logical_13_16_aisle_2_long_2_.left_arm = {0, -2.05, 1.57, -2.65, -1.57, 0};
    logical_13_16_aisle_2_long_2_.right_arm = {1.51, -1.57, 2.8, -1.44, 3.14, 0};


    logical_11_14_aisle_0_short_.gantry = {0, -4.48, 0};
    logical_11_14_aisle_0_short_.left_arm = {-1.48, -2.89, -1.74, -1.72, 0, 0};
    logical_11_14_aisle_0_short_.right_arm = {1.49, -0.34, 1.74, -1.53, 3.14, 0};

    logical_11_14_aisle_0_short_1_.gantry = {-14, -4.48, 0};
    logical_11_14_aisle_0_short_1_.left_arm = {-1.48, -2.89, -1.74, -1.72, 0, 0};
    logical_11_14_aisle_0_short_1_.right_arm = {1.49, -0.34, 1.74, -1.53, 3.14, 0};

    logical_11_14_aisle_0_short_2_.gantry = {-14.7, -4.80, -1.57};
    logical_11_14_aisle_0_short_2_.left_arm = {0, -2.05, 1.57, -2.65, -1.57, 0};
    logical_11_14_aisle_0_short_2_.right_arm = {1.51, -1.57, 2.8, -1.44, 3.14, 0};

    // --blue - logical camera 15
    logical_11_14_aisle_0_long_2_.gantry = {-15.11, -4.10, -1.57};
    logical_11_14_aisle_0_long_2_.left_arm = {0, -2.05, 1.57, -2.65, -1.57, 0};
    logical_11_14_aisle_0_long_2_.right_arm = {1.51, -1.57, 2.8, -1.44, 3.14, 0};


    logical_11_14_aisle_1_short_.gantry = {0, -1.46, 3.14};
    logical_11_14_aisle_1_short_.left_arm = {-1.48, -2.89, -1.74, -1.72, 0, 0};
    logical_11_14_aisle_1_short_.right_arm = {1.49, -0.34, 1.74, -1.53, 3.14, 0};

    logical_11_14_aisle_1_short_1_.gantry = {-14, -1.46, 3.14};
    logical_11_14_aisle_1_short_1_.left_arm = {-1.48, -2.89, -1.74, -1.72, 0, 0};
    logical_11_14_aisle_1_short_1_.right_arm = {1.49, -0.34, 1.74, -1.53, 3.14, 0};

    logical_11_14_aisle_1_short_2_.gantry = {-14.3, -1.2, 1.57};
    logical_11_14_aisle_1_short_2_.left_arm = {0, -2.05, 1.57, -2.65, -1.57, 0};
    logical_11_14_aisle_1_short_2_.right_arm = {1.51, -1.57, 2.8, -1.44, 3.14, 0};

    // --blue - logical camera 15
    logical_11_14_aisle_1_long_2_.gantry = {-14.3, -1.9, 1.57};
    logical_11_14_aisle_1_long_2_.left_arm = {0, -2.05, 1.57, -2.65, -1.57, 0};
    logical_11_14_aisle_1_long_2_.right_arm = {1.51, -1.57, 2.8, -1.44, 3.14, 0};


    logical_12_15_aisle_2_short_.gantry = {0, 1.54, 3.14};
    logical_12_15_aisle_2_short_.left_arm = {-1.48, -2.89, -1.74, -1.72, 0, 0};
    logical_12_15_aisle_2_short_.right_arm = {1.49, -0.34, 1.74, -1.53, 3.14, 0};

    logical_12_15_aisle_2_short_1_.gantry = {-14, 1.54, 3.14};
    logical_12_15_aisle_2_short_1_.left_arm = {-1.48, -2.89, -1.74, -1.72, 0, 0};
    logical_12_15_aisle_2_short_1_.right_arm = {1.49, -0.34, 1.74, -1.53, 3.14, 0};

    logical_12_15_aisle_2_short_2_.gantry = {-14.3, 1.8, 1.57};
    logical_12_15_aisle_2_short_2_.left_arm = {0, -2.05, 1.57, -2.65, -1.57, 0};
    logical_12_15_aisle_2_short_2_.right_arm = {1.51, -1.57, 2.8, -1.44, 3.14, 0};

    // --blue - logical camera 15
    logical_12_15_aisle_2_long_2_.gantry = {-14.3, 1.1, 1.57};
    logical_12_15_aisle_2_long_2_.left_arm = {0, -2.05, 1.57, -2.65, -1.57, 0};
    logical_12_15_aisle_2_long_2_.right_arm = {1.51, -1.57, 2.8, -1.44, 3.14, 0};


    logical_13_16_aisle_3_short_.gantry = {0, 4.54, 3.14};
    logical_13_16_aisle_3_short_.left_arm = {-1.48, -2.89, -1.74, -1.72, 0, 0};
    logical_13_16_aisle_3_short_.right_arm = {1.49, -0.34, 1.74, -1.53, 3.14, 0};

    logical_13_16_aisle_3_short_1_.gantry = {-14, 4.54, 3.14};
    logical_13_16_aisle_3_short_1_.left_arm = {-1.48, -2.89, -1.74, -1.72, 0, 0};
    logical_13_16_aisle_3_short_1_.right_arm = {1.49, -0.34, 1.74, -1.53, 3.14, 0};

    logical_13_16_aisle_3_short_2_.gantry = {-14.3, 4.8, 1.57};
    logical_13_16_aisle_3_short_2_.left_arm = {0, -2.05, 1.57, -2.65, -1.57, 0};
    logical_13_16_aisle_3_short_2_.right_arm = {1.51, -1.57, 2.8, -1.44, 3.14, 0};

    // --blue - logical camera 15
    logical_13_16_aisle_3_long_2_.gantry = {-14.3, 4.1, 1.57};
    logical_13_16_aisle_3_long_2_.left_arm = {0, -2.05, 1.57, -2.65, -1.57, 0};
    logical_13_16_aisle_3_long_2_.right_arm = {1.51, -1.57, 2.8, -1.44, 3.14, 0};



//    logicalCamera12_2_.gantry = {-13.78, -1.85, -1.57};;
//    logicalCamera12_2_.left_arm = {0, -2.01, 1.63, -2.76, -1.57, 0};
//    logicalCamera12_2_.right_arm = {1.25, -3.27, -1.13, 0.88, -0.13, 0};



    /*
     //--start location
    start_.gantry = {0,0,0};
    start_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    start_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //--bin3
    bin3_.gantry = {4.0, -1.1, 0.};
    bin3_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    bin3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //--agv2
    agv2_.gantry = {0.6, 6.9, PI};
    agv2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    agv2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

//    agv2_.gantry = {0, 6.30, 0};
//    agv2_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
//    agv2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

   //--bin13
    bin13_.gantry = {3.10, 1.68, 3.77};//, 2.10};
    bin13_.left_arm = {0, -0.63, 1.26, -0.78, PI/2, 0};
    bin13_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

   //--bin16
    bin16_.gantry = {6.25, 1.96, -3.14};
    bin16_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    bin16_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //--shelf5 - waypoint 1
    shelf5_1_.gantry = {0.0, -4.76, 0};
    shelf5_1_.left_arm = {0.0, -PI/4, PI/2, -PI/4, PI/2, 0};
    shelf5_1_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //--shelf5 - waypoint 2
    shelf5_2_.gantry = {-15.13, -5.46, 0};
    shelf5_2_.left_arm = {-1.25, -PI/2, PI/2, 0, 0.25, 1.38};
    shelf5_2_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //--shelf5 - waypoint 3
    shelf5_3_.gantry = {-14.23, -4.25, 0};
    shelf5_3_.left_arm = {-1.39, -0.75, 1.26, 0, 0.28, 1.38};
    shelf5_3_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};

    //--shelf5 - waypoint 4
    shelf5_4_.gantry = {-15.19, -4.25, 0};
    shelf5_4_.left_arm = {-1.39, -0.75, 1.26, 0, 0.28, 1.38};
    shelf5_4_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};     


  */   






     


//    //--shelf5 - waypoint 4
//    shelf5_5_.gantry = {-15.13, -4.62, -0.13};
//    shelf5_5_.left_arm = {-1.51, -0.5, 0.88, 4.65, 0, 0};
//    shelf5_5_.right_arm = {PI, -PI/4, PI/2, -PI/4, PI/2, 0};
//    tf2_ros::Buffer tfBuffer;
//    tf2_ros::TransformListener tfListener(tfBuffer);
//    ros::Rate rate(10);
//    ros::Duration timeout(5.0);
//
//
//    geometry_msgs::TransformStamped transformStamped;
//    for (int i=0; i< 10; i++) {
//        try {
//            transformStamped = tfBuffer.lookupTransform("world", "left_ee_link",
//                                                        ros::Time(0), timeout);
//        }
//        catch (tf2::TransformException &ex) {
//            ROS_WARN("%s", ex.what());
//            ros::Duration(1.0).sleep();
//            continue;
//        }
//    }


    //--converting quaternions to rpy
//        tf2::Quaternion q(
//                transformStamped.transform.rotation.x,
//                transformStamped.transform.rotation.y,
//                transformStamped.transform.rotation.z,
//                transformStamped.transform.rotation.w);

//    left_ee_quaternion_.at(0) = transformStamped.transform.rotation.x;
//    left_ee_quaternion_.at(1) = transformStamped.transform.rotation.y;
//    left_ee_quaternion_.at(2) = transformStamped.transform.rotation.z;
//    left_ee_quaternion_.at(3) = transformStamped.transform.rotation.w;



    //--Raw pointers are frequently used to refer to the planning group for improved performance.
    //--To start, we will create a pointer that references the current robot’s state.
    const moveit::core::JointModelGroup* joint_model_group =
            full_robot_group_.getCurrentState()->getJointModelGroup("Full_Robot");

    //--Let’s set a joint space goal and move towards it.
    moveit::core::RobotStatePtr current_state = full_robot_group_.getCurrentState();

    //--Next get the current set of joint values for the group.
//    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);



    gantry_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_controller/command", 10);

    left_arm_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/left_arm_controller/command", 10);

    right_arm_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/right_arm_controller/command", 10);

    joint_states_subscriber_ = node_.subscribe(
            "/ariac/gantry/joint_states", 10, &GantryControl::joint_states_callback, this);

    left_gripper_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/left_arm/gripper/state", 10, &GantryControl::left_gripper_state_callback, this);

    right_gripper_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/right_arm/gripper/state", 10, &GantryControl::right_gripper_state_callback, this);

    gantry_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/gantry_controller/state", 10, &GantryControl::gantry_controller_state_callback, this);

    left_arm_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/left_arm_controller/state", 10, &GantryControl::left_arm_controller_state_callback, this);

    right_arm_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/right_arm_controller/state", 10, &GantryControl::right_arm_controller_state_callback, this);


    while( (current_gantry_controller_state_.joint_names.size() == 0)
           || (current_left_arm_controller_state_.joint_names.size() == 0)
           || (current_right_arm_controller_state_.joint_names.size() == 0) ) {
        ROS_WARN("[GantryControl::init] Waiting for first controller_state callbacks...");
        ros::Duration(0.1).sleep();
    }

    left_gripper_control_client =
            node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/left_arm/gripper/control");
    left_gripper_control_client.waitForExistence();

    right_gripper_control_client =
            node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/right_arm/gripper/control");
    right_gripper_control_client.waitForExistence();

    // Move robot to init position
    ROS_INFO("[GantryControl::init] Init position ready)...");
}


stats GantryControl::getStats(std::string function) {
    if (function == "init") return init_;
    if (function == "moveJ") return moveJ_;
    if (function == "IK") return IK_;
    if (function == "moveGantry") return moveGantry_;
    if (function == "pickPart") return pickPart_;
    if (function == "placePart") return placePart_;
    if (function == "dropPart") return dropPart_;
    if (function == "gripFirmly") return gripFirmly_;
    if (function == "gripFromBelt") return gripFromBelt_;
    if (function == "grip") return grip_;
}

geometry_msgs::Pose GantryControl::getTargetWorldPose(geometry_msgs::Pose target,
                                                      std::string agv, std::string arm){
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    std::string kit_tray;
    if (agv.compare("agv1")==0)
        kit_tray = "kit_tray_1";
    else
        kit_tray = "kit_tray_2";
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = kit_tray;
    transformStamped.child_frame_id = "target_frame";
    transformStamped.transform.translation.x = target.position.x;
    transformStamped.transform.translation.y = target.position.y;
    transformStamped.transform.translation.z = target.position.z;
    transformStamped.transform.rotation.x = target.orientation.x;
    transformStamped.transform.rotation.y = target.orientation.y;
    transformStamped.transform.rotation.z = target.orientation.z;
    transformStamped.transform.rotation.w = target.orientation.w;


    for (int i{0}; i<15; ++i)
        br.sendTransform(transformStamped);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10);
    ros::Duration timeout(5.0);


    geometry_msgs::TransformStamped world_target_tf;
    geometry_msgs::TransformStamped ee_target_tf;
    for (int i=0; i< 10; i++) {
        try {
            world_target_tf = tfBuffer.lookupTransform("world", "target_frame",
                                                       ros::Time(0), timeout);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        if(arm == "right_arm") {
            try {
                ee_target_tf = tfBuffer.lookupTransform("target_frame", "right_ee_link",
                                                        ros::Time(0), timeout);
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }
        else{
            try {
                ee_target_tf = tfBuffer.lookupTransform("target_frame", "left_ee_link",
                                                        ros::Time(0), timeout);
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }
    }

    geometry_msgs::Pose world_target{target};
    world_target.position.x = world_target_tf.transform.translation.x;
    world_target.position.y = world_target_tf.transform.translation.y;
    world_target.position.z = world_target_tf.transform.translation.z;
//    world_target.orientation.x = world_target_tf.transform.rotation.x;
//    world_target.orientation.y = world_target_tf.transform.rotation.y;
//    world_target.orientation.z = world_target_tf.transform.rotation.z;
//    world_target.orientation.w = world_target_tf.transform.rotation.w;
    world_target.orientation.x = ee_target_tf.transform.rotation.x;
    world_target.orientation.y = ee_target_tf.transform.rotation.y;
    world_target.orientation.z = ee_target_tf.transform.rotation.z;
    world_target.orientation.w = ee_target_tf.transform.rotation.w;

    return world_target;
}
 
////TODO- make pick part more robust. Sometimes fails to pick part
//bool GantryControl::pickPart(part part){
//    //--Activate gripper
//    activateGripper("left_arm");
//
////    ros::AsyncSpinner spinner(1);
////    spinner.start();
//
////    left_arm_group_.setPoseReferenceFrame("world");
//    geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;
//
////    ROS_INFO_STREAM("[left_arm_group_]= " << currentPose.position.x << ", " << currentPose.position.y << "," << currentPose.position.z);
//
//    part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON;
//    part.pose.orientation.x = currentPose.orientation.x;
//    part.pose.orientation.y = currentPose.orientation.y;
//    part.pose.orientation.z = currentPose.orientation.z;
//    part.pose.orientation.w = currentPose.orientation.w;
////    ROS_INFO_STREAM("["<< part.type<<"]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);
//
//
//    auto state = getGripperState("left_arm");
//    if (state.enabled) {
//        ROS_INFO_STREAM("[Gripper] = enabled");
//        //--Move arm to part
//        left_arm_group_.setPoseTarget(part.pose);
//        left_arm_group_.move();
//        auto state = getGripperState("left_arm");
//        if (state.attached) {
//            ROS_INFO_STREAM("[Gripper] = object attached");
//            //--Move arm to previous position
//            left_arm_group_.setPoseTarget(currentPose);
//            left_arm_group_.move();
//            /*goToPresetLocation(start_);*/
//            return true;
//        }
//        else {
//            ROS_INFO_STREAM("[Gripper] = object not attached");
//            int max_attempts{5};
//            int current_attempt{0};
//            while(!state.attached) {
//                left_arm_group_.setPoseTarget(currentPose);
//                left_arm_group_.move();
//                ros::Duration(0.5).sleep();
//                left_arm_group_.setPoseTarget(part.pose);
//                left_arm_group_.move();
//                activateGripper("left_arm");
//                ROS_INFO_STREAM(state.attached);
//                state = getGripperState("left_arm");
//              // ros::spinOnce();
//        }
//     }
//    }
//    else {
//        ROS_INFO_STREAM("[Gripper] = not enabled");
//    }
//    return false;
//
//    /**
//     * We want the Cartesian path to be interpolated at a resolution of 1 cm which is why
//     * we will specify 0.01 as the max step in Cartesian translation.
//     * We will specify the jump threshold as 0.0, effectively disabling it.
//     */
//    //--define a set of waypoints
////    geometry_msgs::Pose near_pick_pose;
////    geometry_msgs::Pose pick_pose;
////    near_pick_pose = part.pose;
////    pick_pose = part.pose;
////
////    near_pick_pose.position.z += 0.1;
////    pick_pose.position.z += 0.015;
////
////    //--waypoints
////    ROS_INFO_STREAM("[near_pick_pose]= " << near_pick_pose.position.x << "," << near_pick_pose.position.y << "," << near_pick_pose.position.z << "," << near_pick_pose.orientation.x << "," << near_pick_pose.orientation.y << "," << near_pick_pose.orientation.z << "," << near_pick_pose.orientation.w);
////    ROS_INFO_STREAM("[pick_pose]= " << pick_pose.position.x << "," << pick_pose.position.y << "," << pick_pose.position.z << "," << pick_pose.orientation.x << "," << pick_pose.orientation.y << "," << pick_pose.orientation.z << "," << pick_pose.orientation.w);
////    std::vector<geometry_msgs::Pose> waypoints;
////    waypoints.push_back(near_pick_pose);
////    waypoints.push_back(pick_pose);
//
////    moveit_msgs::RobotTrajectory trajectory;
////    const double jump_threshold = 0.0;
////    const double eef_step = 0.001;
////    double fraction = left_arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
////
////    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
////    bool success = (left_arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
////    if (success)
////        left_arm_group_.move();
////    ros::waitForShutdown();
//}

//TODO- make pick part more robust. Sometimes fails to pick part
bool GantryControl::pickPart(part part){
    //--Activate gripper
    activateGripper("left_arm");

//    ros::AsyncSpinner spinner(1);
//    spinner.start();
//    left_arm_group_.setPoseReferenceFrame("world");
    geometry_msgs::Pose currentPose = left_arm_group_.getCurrentPose().pose;

//    ROS_INFO_STREAM("[left_arm_group_]= " << currentPose.position.x << ", " << currentPose.position.y << "," << currentPose.position.z);

    part.pose.position.z = part.pose.position.z + model_height.at(part.type) + GRIPPER_HEIGHT - EPSILON;
    part.pose.orientation.x = currentPose.orientation.x;
    part.pose.orientation.y = currentPose.orientation.y;
    part.pose.orientation.z = currentPose.orientation.z;
    part.pose.orientation.w = currentPose.orientation.w;
//    ROS_INFO_STREAM("["<< part.type<<"]= " << part.pose.position.x << ", " << part.pose.position.y << "," << part.pose.position.z << "," << part.pose.orientation.x << "," << part.pose.orientation.y << "," << part.pose.orientation.z << "," << part.pose.orientation.w);


    auto state = getGripperState("left_arm");
    while(!state.enabled){
        ROS_INFO_STREAM("Activating Gripper again");
        activateGripper("left_arm");
        state = getGripperState("left_arm");
    }
//    if (state.enabled) {
    ROS_INFO_STREAM("[Gripper] = enabled");
    //--Move arm to part
    left_arm_group_.setPoseTarget(part.pose);
    left_arm_group_.move();
    state = getGripperState("left_arm");
    if (state.attached) {
        ROS_INFO_STREAM("[Gripper] = object attached");
        //--Move arm to previous position
        left_arm_group_.setPoseTarget(currentPose);
        left_arm_group_.move();
        /*goToPresetLocation(start_);*/
        return true;
    }
    else {
        ROS_INFO_STREAM("[Gripper] = object not attached");
        int max_attempts{5};
        int current_attempt{0};
        while(!state.attached) {
            left_arm_group_.setPoseTarget(currentPose);
            left_arm_group_.move();
            ros::Duration(0.5).sleep();
            left_arm_group_.setPoseTarget(part.pose);
            left_arm_group_.move();
            activateGripper("left_arm");
            ROS_INFO_STREAM(state.attached);
            state = getGripperState("left_arm");
            // ros::spinOnce();
        }
    }
    return false;
}


void GantryControl::placePart(part part, std::string agv, std::string arm){
    auto target_pose_in_tray = getTargetWorldPose(part.pose, agv, arm);
    target_pose_in_tray.position.z += (ABOVE_TARGET + 1.5*model_height[part.type]);
    ROS_INFO_STREAM(target_pose_in_tray);
    //target_pose_in_tray.orientation.x = -0.710413;
    //target_pose_in_tray.orientation.y = 0.00131;
    //target_pose_in_tray.orientation.z = 0.7037;
    //target_pose_in_tray.orientation.w = -0.0018;
    
    if(arm == "left_arm"){
       left_arm_group_.setPoseTarget(target_pose_in_tray);
       left_arm_group_.move();
    }else {
       right_arm_group_.setPoseTarget(target_pose_in_tray);
       right_arm_group_.move();
    }
    deactivateGripper(arm);
    auto state = getGripperState(arm);
}





void GantryControl::presetArmLocation(PresetLocation location) {
    //--left arm
    joint_group_positions_.at(3) = location.left_arm.at(0);
    joint_group_positions_.at(4) = location.left_arm.at(1);
    joint_group_positions_.at(5) = location.left_arm.at(2);
    joint_group_positions_.at(6) = location.left_arm.at(3);
    joint_group_positions_.at(7) = location.left_arm.at(4);
    joint_group_positions_.at(8) = location.left_arm.at(5);

    //--right arm
    joint_group_positions_.at(9) = location.right_arm.at(0);
    joint_group_positions_.at(10) = location.right_arm.at(1);
    joint_group_positions_.at(11) = location.right_arm.at(2);
    joint_group_positions_.at(12) = location.right_arm.at(3);
    joint_group_positions_.at(13) = location.right_arm.at(4);
    joint_group_positions_.at(14) = location.right_arm.at(5);
}

void GantryControl::goToPresetLocation(PresetLocation location) {
    //--gantry
    joint_group_positions_.at(0) = location.gantry.at(0);
    joint_group_positions_.at(1) = location.gantry.at(1);
    joint_group_positions_.at(2) = location.gantry.at(2);
    //--left arm
    joint_group_positions_.at(3) = location.left_arm.at(0);
    joint_group_positions_.at(4) = location.left_arm.at(1);
    joint_group_positions_.at(5) = location.left_arm.at(2);
    joint_group_positions_.at(6) = location.left_arm.at(3);
    joint_group_positions_.at(7) = location.left_arm.at(4);
    joint_group_positions_.at(8) = location.left_arm.at(5);
    //--right arm
    joint_group_positions_.at(9) = location.right_arm.at(0);
    joint_group_positions_.at(10) = location.right_arm.at(1);
    joint_group_positions_.at(11) = location.right_arm.at(2);
    joint_group_positions_.at(12) = location.right_arm.at(3);
    joint_group_positions_.at(13) = location.right_arm.at(4);
    joint_group_positions_.at(14) = location.right_arm.at(5);

    full_robot_group_.setJointValueTarget(joint_group_positions_);


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (full_robot_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        full_robot_group_.move();
}

/// Turn on vacuum gripper
void GantryControl::activateGripper(std::string arm_name) {
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = true;

    if (arm_name == "left_arm") {
        left_gripper_control_client.call(srv);
    } else {
        right_gripper_control_client.call(srv);
    }
    ROS_INFO_STREAM("[GantryControl][activateGripper] DEBUG: srv.response =" << srv.response);
}

/// Turn off vacuum gripper
void GantryControl::deactivateGripper(std::string arm_name) {
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = false;

    if (arm_name == "left_arm") {
        left_gripper_control_client.call(srv);
    } else {
        right_gripper_control_client.call(srv);
    }
    ROS_INFO_STREAM("[GantryControl][deactivateGripper] DEBUG: srv.response =" << srv.response);
}

/// Retrieve gripper state
nist_gear::VacuumGripperState GantryControl::getGripperState(std::string arm_name) {
    if (arm_name == "left_arm") {
        return current_left_gripper_state_;
    } else {
        return current_right_gripper_state_;
    }
}

/// Called when a new VacuumGripperState message is received
void GantryControl::left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & gripper_state_msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
    current_left_gripper_state_ = *gripper_state_msg;
}

void GantryControl::right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & gripper_state_msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gripper States (throttled to 0.1 Hz):\n" << *gripper_state_msg);
    current_right_gripper_state_ = *gripper_state_msg;
}

/// Called when a new JointState message is received
void GantryControl::joint_states_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg) {
    if (joint_state_msg->position.size() == 0) {
        ROS_ERROR("[gantry_control][joint_states_callback] msg->position.size() == 0!");
    }
    current_joint_states_ = *joint_state_msg;
}


void GantryControl::gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Gantry controller states (throttled to 0.1 Hz):\n" << *msg);
    current_gantry_controller_state_ = *msg;
}

void GantryControl::left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Left arm controller states (throttled to 0.1 Hz):\n" << *msg);
    current_left_arm_controller_state_ = *msg;
}

void GantryControl::right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg) {
    // ROS_INFO_STREAM_THROTTLE(10,
    //   "Right arm controller states (throttled to 0.1 Hz):\n" << *msg);
    current_right_arm_controller_state_ = *msg;
}



bool GantryControl::send_command(trajectory_msgs::JointTrajectory command_msg) {
    // ROS_INFO_STREAM("[gantry_control][send_command] called.");

    if(command_msg.points.size() == 0) {
        ROS_WARN("[gantry_control][send_command] Trajectory is empty or NAN, returning.");
        return false;
    }
    else if ((command_msg.joint_names[0] == "small_long_joint") // command is for gantry
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0])) {

        gantry_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][send_command] gantry command published!");
        return true;
    }
    else if ((command_msg.joint_names[0].substr(0,4) == "left") // command is for left_arm
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0])) {

        left_arm_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][send_command] left_arm command published!");
        return true;
    }
    else if ((command_msg.joint_names[0].substr(0,5) == "right") // command is for right arm
             && (command_msg.points[0].positions[0] == command_msg.points[0].positions[0])) {

        right_arm_joint_trajectory_publisher_.publish(command_msg);
        // ROS_INFO_STREAM("[gantry_control][send_command] right_arm command published!");
        return true;
    }
    else {
        return false;
    }
}


void GantryControl::moveToPart(part my_part) {
  auto gantryConfiguration = full_robot_group_.getCurrentJointValues();
  ROS_INFO_STREAM("gNTRY" << gantryConfiguration[2]);
   
   //shelf logical cameras
   if(my_part.logicalCameraName != "logical_camera_1" &&   my_part.logicalCameraName != "logical_camera_2" &&
      my_part.logicalCameraName != "logical_camera_5" &&   my_part.logicalCameraName != "logical_camera_6") {
      if(gantryConfiguration[2] < -1.50 && gantryConfiguration[2] > -1.60) {
          gantryConfiguration[0] = my_part.pose.position.x - 0.2;
      }
      else if(gantryConfiguration[2] > 1.50 && gantryConfiguration[2] < 1.60) {
          gantryConfiguration[0] = my_part.pose.position.x + 0.2;
      } 
   } else{                        //bin_logical cameras
         gantryConfiguration[0] = my_part.pose.position.x - 0.2;
         gantryConfiguration[1] = -1*my_part.pose.position.y;
   }
  full_robot_group_.setJointValueTarget(gantryConfiguration);
  full_robot_group_.move();


//  goToPresetLocation(logicalCamera12_1_);
//  goToPresetLocation(logicalCamera12_2_);



  //if(gantryConfiguration[2] < 0.2 && gantryConfiguration[2] > -0.2) {
        //gantryConfiguration[0] = part.pose.position.x - 0.7;
    //}
    //else if(gantryConfiguration[2] > 3 && gantryConfiguration[2] < 3.2) {
        //gantryConfiguration[1] = -1 * (part.pose.position.y - 0.7);
        //gantryConfiguration[0] = part.pose.position.x + 0.7;
    //}
    //full_robot_group_.setJointValueTarget(gantryConfiguration);
    //full_robot_group_.move();
    
}




