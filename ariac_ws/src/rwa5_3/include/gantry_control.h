#ifndef GANTRYCONTROL_H
#define GANTRYCONTROL_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <array>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "utils.h"


class GantryControl {

  public:
    GantryControl(ros::NodeHandle & node);

    void init();
    stats getStats(std::string function);

//    bool moveGantry(std::string waypoints);

//    bool pickPart(part part, std::string arm_name);
    bool pickPart(part part);
    
    /// Send command message to robot controller
    bool send_command(trajectory_msgs::JointTrajectory command_msg);
    void goToPresetLocation(PresetLocation location);

    void activateGripper(std::string gripper_id);
    void deactivateGripper(std::string gripper_id);
    nist_gear::VacuumGripperState getGripperState(std::string arm_name);
    geometry_msgs::Pose getTargetWorldPose(geometry_msgs::Pose target, std::string agv, std::string arm);


    void presetArmLocation(PresetLocation location);
    void placePart(part part, std::string agv, std::string arm);
    void placeFlippedPart(part part, std::string agv, std::string arm);
    void movetoPart(part my_part);

    //--preset locations;
    start start_;
    bin3 bin3_;
    agv2 agv2_;
    agv1 agv1_;
    agv2_faultyG agv2_faultyG_;
    agv1_faultyG agv1_faultyG_;
    agv1_drop agv1_drop_;
    agv2_drop agv2_drop_;
    bin13 bin13_;
    bin16 bin16_;
    shelf5_1 shelf5_1_;
    shelf5_2 shelf5_2_;
    shelf5_3 shelf5_3_;
    // Added coordinates for shelf 8 to test faulty gripper functionality
    shelf8_1 shelf8_1_;
    shelf8_2 shelf8_2_;
    shelf8_3 shelf8_3_;

    shelf11_1 shelf11_1_;
    shelf11_2 shelf11_2_;
    shelf11_3 shelf11_3_;

    shelf8_obs_green1 shelf8_obs_green1_;
    shelf8_obs_green2 shelf8_obs_green2_;
    shelf8_obs_green3 shelf8_obs_green3_;
    shelf8_obs_green4 shelf8_obs_green4_;
    shelf8_obs_green5 shelf8_obs_green5_;

    shelf8_obs_blue1 shelf8_obs_blue1_;
    shelf8_obs_blue2 shelf8_obs_blue2_;
    shelf8_obs_blue3 shelf8_obs_blue3_;
    shelf8_obs_blue4 shelf8_obs_blue4_;
    shelf8_obs_blue5 shelf8_obs_blue5_;
    shelf8_obs_blue6 shelf8_obs_blue6_;


    
    // --------- //
    bin13_1 bin13_1_;
    bin13_2 bin13_2_;
    bin13_3 bin13_3_;
    bin13_4 bin13_4_;
    // --------- //


    /** -------------**/
    shelf5_4 shelf5_4_;
    shelf5_5 shelf5_5_;
    agv2_go_to_flipped_pulley agv2_go_to_flipped_pulley_;
    agv1_go_to_flipped_pulley agv1_go_to_flipped_pulley_;
    agv2_flipped agv2_flipped_;
    agv2_flipped1 agv2_flipped1_;
    agv1_flipped agv1_flipped_;
    agv1_flipped1 agv1_flipped1_;
    movingPart movingPart_;
    movingPart1 movingPart1_;
    agv1_gasket_part_green agv1_gasket_part_green_;


    // gaps
    left_gap_default left_gap_default_;
    left_gap_0_2 left_gap_0_2_;
    left_gap_1_2 left_gap_1_2_;
    left_gap_1_3 left_gap_1_3_;
    left_gap_2_2 left_gap_2_2_;
    left_gap_2_3 left_gap_2_3_;
    left_gap_3_2 left_gap_3_2_;
    left_gap_3_3 left_gap_3_3_;

    middle_gap_default middle_gap_default_;
    middle_gap_0_2 middle_gap_0_2_;
    middle_gap_1_2 middle_gap_1_2_;
    middle_gap_1_3 middle_gap_1_3_;
    middle_gap_2_2 middle_gap_2_2_;
    middle_gap_2_3 middle_gap_2_3_;
    middle_gap_3_2 middle_gap_3_2_;
    middle_gap_3_3 middle_gap_3_3_;

    right_gap_default right_gap_default_;
    right_gap_0_2 right_gap_0_2_;
    right_gap_1_2 right_gap_1_2_;
    right_gap_1_3 right_gap_1_3_;
    right_gap_2_2 right_gap_2_2_;
    right_gap_2_3 right_gap_2_3_;
    right_gap_3_2 right_gap_3_2_;
    right_gap_3_3 right_gap_3_3_;

    //gap to part
    left_gap_2_green_1 left_gap_2_green_1_;
    left_gap_2_green_2 left_gap_2_green_2_;
    left_gap_2_green_3 left_gap_2_green_3_;
    right_gap_2_blue_1 right_gap_2_blue_1_;
    right_gap_2_blue_2 right_gap_2_blue_2_;
    right_gap_2_blue_3 right_gap_2_blue_3_;


  private:
    std::vector<double> joint_group_positions_;
    ros::NodeHandle node_;
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface::Options full_robot_options_;
    moveit::planning_interface::MoveGroupInterface::Options left_arm_options_;
    moveit::planning_interface::MoveGroupInterface::Options right_arm_options_;
    moveit::planning_interface::MoveGroupInterface::Options left_ee_link_options_;
    moveit::planning_interface::MoveGroupInterface::Options right_ee_link_options_;
    moveit::planning_interface::MoveGroupInterface full_robot_group_;
    moveit::planning_interface::MoveGroupInterface left_arm_group_;
    moveit::planning_interface::MoveGroupInterface right_arm_group_;
    moveit::planning_interface::MoveGroupInterface left_ee_link_group_;
    moveit::planning_interface::MoveGroupInterface right_ee_link_group_;

    double left_ee_roll_;
    double left_ee_pitch_;
    double left_ee_yaw_;
    std::array<float,4> left_ee_quaternion_;

    sensor_msgs::JointState current_joint_states_;


    nist_gear::VacuumGripperState current_left_gripper_state_;
    nist_gear::VacuumGripperState current_right_gripper_state_;

    control_msgs::JointTrajectoryControllerState current_gantry_controller_state_;
    control_msgs::JointTrajectoryControllerState current_left_arm_controller_state_;
    control_msgs::JointTrajectoryControllerState current_right_arm_controller_state_;

    ros::Publisher gantry_joint_trajectory_publisher_;
    ros::Publisher left_arm_joint_trajectory_publisher_;
    ros::Publisher right_arm_joint_trajectory_publisher_;

    ros::Subscriber joint_states_subscriber_;
    ros::Subscriber left_gripper_state_subscriber_;
    ros::Subscriber right_gripper_state_subscriber_;
    ros::Subscriber gantry_controller_state_subscriber_;
    ros::Subscriber left_arm_controller_state_subscriber_;
    ros::Subscriber right_arm_controller_state_subscriber_;

    ros::ServiceClient left_gripper_control_client;
    ros::ServiceClient right_gripper_control_client;

    // ---------- Callbacks ----------
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg);
    void left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    void right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    void gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    void left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    void right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);




     // collect stats
    stats init_;
    stats moveJ_;
    stats IK_;
    stats moveGantry_;
    stats pickPart_;
    stats placePart_;
    stats dropPart_;
    stats gripFirmly_;
    stats gripFromBelt_;
    stats grip_;
};

#endif
