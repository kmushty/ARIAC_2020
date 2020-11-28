#ifndef UTILS_H
#define UTILS_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <unordered_map>
#include <string>

#include <ros/ros.h>

#include <nist_gear/VacuumGripperState.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef struct Shipment shipment; // forward declarations
typedef struct Order order;
typedef struct Product product;

const double PI = 3.141592; // TODO correct!

// Logical cameras
const int MAX_NUMBER_OF_CAMERAS = 17;



const int MAX_PICKING_ATTEMPTS = 3; // for pickup
const double ABOVE_TARGET = 0.2; // above target z pos when picking/placing part
const double PICK_TIMEOUT = 4.0;
const double RETRIEVE_TIMEOUT = 2.0;

const double BELT_SPEED = 0.2; // m/s

const double GRIPPER_HEIGHT = 0.01;
const double EPSILON = 0.008; // for the gripper to firmly touch

const double BIN_HEIGHT = 0.724;
const double TRAY_HEIGHT = 0.755;
const double RAIL_HEIGHT = 0.95;

const double PLANNING_TIME = 20; // for move_group
const int MAX_EXCHANGE_ATTEMPTS = 6; // Pulley flip

extern std::string action_state_name[];
extern std::unordered_map<std::string, double> model_height;

enum PartStates {FREE, BOOKED, UNREACHABLE, ON_TRAY, GRIPPED, GOING_HOME,
  REMOVE_FROM_TRAY, LOST};



typedef struct PresetLocation {
    std::vector<double> gantry;
    std::vector<double> left_arm;
    std::vector<double> right_arm;
} start, bin3, agv2, agv1, agv1_faultyG, agv2_faultyG, bin13, bin16, shelf5_1, shelf5_2, shelf5_3, shelf5_4, shelf5_5,shelf11_1, shelf11_2, shelf11_3,
shelf8_1, shelf8_2, shelf8_3, agv2_go_to_flipped_pulley, agv1_go_to_flipped_pulley,agv2_flipped, agv2_flipped1,agv1_flipped, agv1_flipped1, agv1_drop,
agv2_drop, movingPart, movingPart1, agv1_gasket_part_green,shelf8_obs_green1, shelf8_obs_green2, shelf8_obs_green3, shelf8_obs_green4, shelf8_obs_green5,
shelf8_obs_blue1, shelf8_obs_blue2, shelf8_obs_blue3, shelf8_obs_blue4, shelf8_obs_blue5, shelf8_obs_blue6, left_gap_default, middle_gap_default_aisle1, middle_gap_default_aisle2, right_gap_default,
left_gap_0_2, left_gap_1_2, left_gap_1_3, left_gap_2_2, left_gap_2_3, left_gap_3_2, left_gap_3_3, left_gap_aisle_0to1,
middle_gap_0_2_aisle1, middle_gap_1_2_aisle1, middle_gap_1_3_aisle1, middle_gap_2_2_aisle1, middle_gap_2_3_aisle1, middle_gap_3_2_aisle1, middle_gap_3_3_aisle1, middle_gap_aisle_1to2,
middle_gap_0_2_aisle2, middle_gap_1_2_aisle2, middle_gap_1_3_aisle2, middle_gap_2_2_aisle2, middle_gap_2_3_aisle2, middle_gap_3_2_aisle2, middle_gap_3_3_aisle2, middle_gap_aisle_2to1,
right_gap_0_2, right_gap_1_2, right_gap_1_3, right_gap_2_2, right_gap_2_3, right_gap_3_2, right_gap_3_3, right_gap_aisle_3to2,
left_gap_2_green_1, left_gap_2_green_2, left_gap_2_green_3, right_gap_2_blue_1, right_gap_2_blue_2, right_gap_2_blue_3,bin13_1, bin13_2, bin13_3, bin13_4,
logical_12_15_aisle_1_short, logical_12_15_aisle_1_long, logical_13_16_aisle_2_short, logical_13_16_aisle_2_long, logical_11_14_aisle_0_short, logical_11_14_aisle_0_long, logical_11_14_aisle_1_short, logical_11_14_aisle_1_long,
logical_12_15_aisle_1_short, logical_12_15_aisle_1_long, logical_13_16_aisle_2_short, logical_13_16_aisle_2_long, logical_11_14_aisle_0_short, logical_11_14_aisle_0_long, logical_11_14_aisle_1_short, logical_11_14_aisle_1_long,
logical_12_15_aisle_2_short, logical_12_15_aisle_2_long, logical_13_16_aisle_3_short, logical_13_16_aisle_3_long, logical_0_4_short, logical_0_4_long, logical_3_7_short, logical_3_7_long,
temp;


typedef struct Part {
  std::string type; // model type
  std::string logicalCameraName;
  geometry_msgs::Pose pose; // model pose (in frame)
  geometry_msgs::Pose save_pose;
  std::string frame; // model frame (e.g., "logical_camera_1_frame")
  ros::Time time_stamp;
  std::string id;
  PartStates state; // model state (enum PartStates)
  int count;
  bool faulty;
  double conveyor_time = -1;
} part;

typedef struct Position {
    std::vector<double> gantry;
    std::vector<double> left;
    std::vector<double> right;
} position;

typedef struct Shipment {
    std::string shipment_type;
    std::string agv_id;
    std::vector<Product> products;
    order* parent_order;
} shipment;

typedef struct Product {
    std::string type;
    geometry_msgs::Pose pose;
    part p; // NEW here!
    // std::string frame_of_origin;
    geometry_msgs::Pose actual_pose;
    std::string actual_pose_frame;
    std::string agv_id;
    std::string tray;
    std::string arm_name;
    std::string cache_id;
    shipment* parent_shipment{};
    bool high_priority{};
    int correction_attempts{};
    int service_attempts{};

    Product(); // contructor
} product;

typedef struct Order {
    std::string order_id;
    std::vector<Shipment> shipments;
} order;

typedef struct Stats {
  double total_time = 0.0;
  double fail_time = 0.0;
  int calls = 0;
  int fails = 0;
} stats;


typedef struct Obstacle{
   bool   is_valid_obstacle = false;              //checker for whether obstacle is valid
   double wait_time;                              //wait_time of obstacle
   double move_time;                              //move_time of obstacle
   double time_stamp1;                            //useful time stamps for estimating position
   double time_stamp2;      
   double time_stamp3;      
}obstacle;
#endif



