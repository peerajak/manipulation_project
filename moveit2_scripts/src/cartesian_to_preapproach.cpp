#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

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

  joint_group_positions_arm[0] =3.434;
  joint_group_positions_arm[1] =-1.8398;
  joint_group_positions_arm[2] =-1.735;
  joint_group_positions_arm[3] =-1.226;
  joint_group_positions_arm[4] =1.589;
  joint_group_positions_arm[5] =0.244;
  joint_group_positions_arm[6] = 0.0;//Gripper

  move_group_arm.setJointValueTarget(joint_group_positions_arm);	
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);
  
  move_group_arm.execute(my_plan_arm);
  RCLCPP_INFO(LOGGER, "Approach to object!");
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.00;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  target_pose1.position.x = 0.358;
  target_pose1.position.y = -0.027;
  target_pose1.position.z = 0.198;
  move_group_arm.setPoseTarget(target_pose1);



  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.z -= 0.03;
  approach_waypoints.push_back(target_pose1);

  //target_pose1.position.z -= 0.03;
  //approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);
  RCLCPP_INFO(LOGGER, "Open Gripper!");

  joint_group_positions_gripper[2] = 0.0;
  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  // Sleep for some seconds
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  // Close Gripper
  // set name target.
  move_group_gripper.setNamedTarget("gripper_close");
  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);
  /* X motion

  RCLCPP_INFO(LOGGER, "Draw an X!");

  std::vector<geometry_msgs::msg::Pose> waypoints;

  target_pose1.position.x += 0.05;
  target_pose1.position.y -= 0.05;
  waypoints.] =target_pose1);

  target_pose1.position.y += 0.05;
  waypoints.] =target_pose1);

  target_pose1.position.x -= 0.05;
  target_pose1.position.y -= 0.05;
  waypoints.] =target_pose1);

  target_pose1.position.y += 0.05;
  waypoints.] =target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory);

  move_group_arm.execute(trajectory);
  */
  rclcpp::shutdown();
  return 0;
}