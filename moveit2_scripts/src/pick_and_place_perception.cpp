#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <unistd.h>
#define MOVE_TO_PREAPPROACH 1
#define FINETUNE 1
#define JUMP_PICK_OBJECT 1
#define GRIPPING 1
#define RETREAT 1
#define TURN180 1
#define RELEASE 1
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
  unsigned int microseconds = 1;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);
  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  std::vector<double> joint_group_positions_gripper;
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double target_x_from_perception = 0.327921;
  double target_y_from_perception = -0.010783;
  double offset_x = 0.012079, offset_y = -0.009217;
  double target_x = target_x_from_perception + offset_x;
  double target_y = target_y_from_perception + offset_y;
  double close_gripper_initial_angle = 0.642;//0.642 a good value
  double close_gripper_angle =
      0.6460; // 0.6460 completely liftup, and retreat for 20 secs!. 0.646675 lifted up, but slide out after retreat in 2 secs
  // TODO try to slow down close grip speed.
  // while(true) {
#if MOVE_TO_PREAPPROACH
  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");
  /*
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.x = -1.0;
    target_pose1.orientation.y = 0.00;
    target_pose1.orientation.z = 0.00;
    target_pose1.orientation.w = 0.00;
    target_pose1.position.x = 0.343;
    target_pose1.position.y = 0.132;
    target_pose1.position.z = 0.264;
    move_group_arm.setPoseTarget(target_pose1);
  */

  joint_group_positions_arm[0] = 3.434;
  joint_group_positions_arm[1] = -1.7171;
  joint_group_positions_arm[2] = -1.735;
  joint_group_positions_arm[3] = -1.226;
  joint_group_positions_arm[4] = 1.589;
  joint_group_positions_arm[5] = 0.244;
  joint_group_positions_arm[6] = 0.0; // Gripper

  move_group_arm.setJointValueTarget(joint_group_positions_arm);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

#endif

  // print current pose
  geometry_msgs::msg::Pose current_pose = move_group_arm.getCurrentPose().pose;

  // Print the current pose of the end effector
  RCLCPP_INFO(LOGGER, "Current pose: %f %f %f %f %f %f %f",
              current_pose.position.x, current_pose.position.y,
              current_pose.position.z, current_pose.orientation.x,
              current_pose.orientation.y, current_pose.orientation.z,
              current_pose.orientation.w);

  RCLCPP_INFO(LOGGER, "Approach to object!");
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.position.x = current_pose.position.x;//start with current, and waypont to real target
  target_pose1.position.y = current_pose.position.y;
  target_pose1.position.z = current_pose.position.z;
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.00;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;


  double xy_resolution = 0.000001, xy_goal_threshold = 0.000001;

  std::vector<geometry_msgs::msg::Pose> waypoints;

#if FINETUNE
  while (abs(target_pose1.position.x - target_x) > xy_goal_threshold) {
    target_pose1.position.x += xy_resolution *
                               (target_x - target_pose1.position.x) /
                               abs(target_pose1.position.x - target_x);
    waypoints.push_back(target_pose1);
  }
  while (abs(target_pose1.position.y - target_y) > xy_goal_threshold) {
    target_pose1.position.y += xy_resolution *
                               (target_y - target_pose1.position.y) /
                               abs(target_pose1.position.y - target_y);
    waypoints.push_back(target_pose1);
  }

  moveit_msgs::msg::RobotTrajectory trajectory;

  double fraction = move_group_arm.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory);

  move_group_arm.execute(trajectory);

  RCLCPP_INFO(LOGGER, "Current pose: %f %f %f %f %f %f %f",
              current_pose.position.x, current_pose.position.y,
              current_pose.position.z, current_pose.orientation.x,
              current_pose.orientation.y, current_pose.orientation.z,
              current_pose.orientation.w);
#endif
#if JUMP_PICK_OBJECT

  RCLCPP_INFO(LOGGER, "Open Gripper! Current Target: x:%f y:%f", target_x,
              target_y);
  joint_group_positions_gripper[2] = 0.55;
  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);
  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.z -= 0.08;
  approach_waypoints.push_back(target_pose1);

  // target_pose1.position.z -= 0.03;
  // approach_waypoints.push_back(target_pose1);

  // JUMP code
  moveit_msgs::msg::RobotTrajectory trajectory_approach;

  double fraction2 = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);

#endif

#if GRIPPING
  double step_size = 0.00001;

  // Sleep for some seconds
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  // Close Gripper
  // set name target.
  double gripper_iter = close_gripper_initial_angle;
  while (gripper_iter <= close_gripper_angle ) {
    joint_group_positions_gripper[2] = gripper_iter;// <= close_gripper_angle ? gripper_iter: close_gripper_angle;
    move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper2;
    success_gripper = (move_group_gripper.plan(my_plan_gripper2) ==
                       moveit::core::MoveItErrorCode::SUCCESS);

    move_group_gripper.execute(my_plan_gripper2);
    // sleep(1.0);

    RCLCPP_INFO(LOGGER, "Closing Gripper close_angle:%f", gripper_iter);
    //  if (gripper_iter < 0.64) step_size = 0.02;  
    // if (gripper_iter >= 0.64 && gripper_iter < 0.643) step_size = 0.0001;
    // if (gripper_iter >= 0.643 && gripper_iter < 0.645) step_size = 0.00005;
    // if (gripper_iter >= 0.645) step_size= 0.000005;
    gripper_iter += step_size;
    usleep(microseconds);
  }

#endif

  // Retreat
#if RETREAT
  geometry_msgs::msg::Pose current_pose2 = move_group_arm.getCurrentPose().pose;
  RCLCPP_INFO(LOGGER, "Retreat from object!");
  geometry_msgs::msg::Pose target_pose2;
  target_pose2.position.x = current_pose2.position.x;
  target_pose2.position.y = current_pose2.position.y;
  target_pose2.position.z = current_pose2.position.z;
  target_pose2.orientation.x = -1.0;
  target_pose2.orientation.y = 0.00;
  target_pose2.orientation.z = 0.00;
  target_pose2.orientation.w = 0.00;
  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose2.position.z += 0.08;
  retreat_waypoints.push_back(target_pose2);

  //   target_pose2.position.z += 0.03;
  //   retreat_waypoints.push_back(target_pose2);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;

  double fraction3 = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  move_group_arm.execute(trajectory_retreat);
#endif
  // close_gripper_angle += 0.00001;
  //}//end while
//   while (true) {
//     sleep(5);
//   }
#if TURN180

  joint_group_positions_arm[0] -= 3.14;

  move_group_arm.setJointValueTarget(joint_group_positions_arm);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm3;
  bool success_arm3 = (move_group_arm.plan(my_plan_arm3) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm3);


#endif

#if RELEASE
  RCLCPP_INFO(LOGGER, "Release Object!");
  move_group_gripper.setNamedTarget("gripper_open");
  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
  move_group_gripper.execute(my_plan_gripper);
#endif
  rclcpp::shutdown();
  return 0;
}