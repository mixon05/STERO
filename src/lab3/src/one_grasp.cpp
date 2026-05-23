#include <memory>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <gazebo_msgs/msg/entity_state.hpp>


geometry_msgs::msg::Pose cubePose;
geometry_msgs::msg::Pose tablePose;

moveit_msgs::msg::CollisionObject createCollisionObjectBox(std::string frame_id, std::string id, const std::vector<double> &size,
                                                           const geometry_msgs::msg::Pose &pose){
  moveit_msgs::msg::CollisionObject collisionObject;
  shape_msgs::msg::SolidPrimitive primitive;

  collisionObject.header.frame_id = frame_id;
  collisionObject.id = id;

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = size[0];
  primitive.dimensions[primitive.BOX_Y] = size[1];
  primitive.dimensions[primitive.BOX_Z] = size[2];

  collisionObject.primitives.push_back(primitive);
  collisionObject.primitive_poses.push_back(pose);

  collisionObject.operation = collisionObject.ADD;

  return collisionObject;
}

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("one_grasp", 
  rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("one_grasp"); 
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);
  rclcpp::sleep_for(std::chrono::seconds(1)); 

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor](){ executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  auto moveGroupInterface = MoveGroupInterface(node, "arm_torso");
  moveGroupInterface.setPoseReferenceFrame("base_footprint");
  moveGroupInterface.setMaxAccelerationScalingFactor(1.0);
  moveGroupInterface.setMaxVelocityScalingFactor(1.0);

  auto gripperGroupInterface = MoveGroupInterface(node, "gripper");
  gripperGroupInterface.setMaxAccelerationScalingFactor(1.0);
  gripperGroupInterface.setMaxVelocityScalingFactor(1.0);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  auto moveitVisualTools =
      moveit_visual_tools::MoveItVisualTools{node, "base_footprint", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                             moveGroupInterface.getRobotModel()};
  moveitVisualTools.deleteAllMarkers();
  moveitVisualTools.loadRemoteControl();

  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client =
      node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");

  while (!client->wait_for_service(std::chrono::seconds(1))){
    RCLCPP_INFO(logger, "Waiting for /get_entity_state service to be available ");
  }

  auto requestNew = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
  requestNew->name = "green_cube_3";
  requestNew->reference_frame = "base_footprint";

  auto resultFuture = client->async_send_request(requestNew);

  resultFuture.wait();
  if (resultFuture.valid()){
    auto result = resultFuture.get();
    gazebo_msgs::msg::EntityState state = result->state;
    std::string textMessage = "Position of the cube: " + std::to_string(state.pose.position.x) + ", " +
                               std::to_string(state.pose.position.y) + ", " + std::to_string(state.pose.position.z) + "\n";
                               
    RCLCPP_INFO(logger, textMessage.c_str());

    cubePose.position = state.pose.position;
    cubePose.orientation = state.pose.orientation;
    moveitVisualTools.publishSphere(cubePose, rviz_visual_tools::GREEN, 0.05);
    moveitVisualTools.publishAxis(cubePose, 0.1, 0.01, "cube");

    geometry_msgs::msg::TransformStamped transform;
    try{
      auto now = node->get_clock()->now();
      transform = tf_buffer->lookupTransform("gripper_grasping_frame", "arm_tool_link", now);
    }
    catch (tf2::TransformException &ex){
      RCLCPP_ERROR(logger, "Error while transform: %s", ex.what());
    }

    geometry_msgs::msg::Pose graspAtmTl;
    graspAtmTl.position.x = cubePose.position.x;
    graspAtmTl.position.y = cubePose.position.y;
    graspAtmTl.position.z = cubePose.position.z - transform.transform.translation.x +
                                   0.08;
    graspAtmTl.orientation = cubePose.orientation;
    moveitVisualTools.publishSphere(graspAtmTl, rviz_visual_tools::BLUE, 0.05);
    moveitVisualTools.publishAxis(graspAtmTl, 0.1, 0.01, "grasp");

    geometry_msgs::msg::Pose pre_grasp_atm_tl_pose = graspAtmTl;
    pre_grasp_atm_tl_pose.position.z += 0.2 - 0.08;
    moveitVisualTools.publishSphere(pre_grasp_atm_tl_pose, rviz_visual_tools::RED, 0.05);
    moveitVisualTools.publishAxis(pre_grasp_atm_tl_pose, 0.1, 0.01, "pre_grasp");

    moveitVisualTools.trigger();
  }
  else{
    RCLCPP_ERROR(logger, "Failed to call service get_entity_state for cube");
  }

  auto requestNewTemp = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
  requestNewTemp->name = "table";
  requestNewTemp->reference_frame = "base_footprint";
  auto resultFutureTemp = client->async_send_request(requestNewTemp);

  resultFutureTemp.wait();
  if (resultFutureTemp.valid()){
    auto result = resultFutureTemp.get();
    gazebo_msgs::msg::EntityState state = result->state;

    tablePose.position = state.pose.position;
    tablePose.position.z += 0.51 / 2;
    tablePose.orientation = state.pose.orientation;
  }
  else{
    RCLCPP_ERROR(logger, "Failed to call service get_entity_state for table");
  }

  auto const collisionObjectCube =
      createCollisionObjectBox(moveGroupInterface.getPlanningFrame(), "box1", std::vector<double>({0.07, 0.07, 0.07}), cubePose);
  planning_scene_interface.applyCollisionObject(collisionObjectCube);

  auto const collisionObjectTable =
      createCollisionObjectBox(moveGroupInterface.getPlanningFrame(), "table", std::vector<double>({0.75, 0.75, 0.51}), tablePose);
  planning_scene_interface.applyCollisionObject(collisionObjectTable);

  moveGroupInterface.setJointValueTarget(std::vector<double>({0.35, M_PI / 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  moveit::planning_interface::MoveGroupInterface::Plan plan1;

  if ((moveGroupInterface.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS)){
    RCLCPP_INFO(logger, "Plan 1 succeeded, now in executing");
    moveGroupInterface.execute(plan1);
  }
  else{
    RCLCPP_WARN(logger, "Planning is failed");
  }

  geometry_msgs::msg::Pose preGraspPose = cubePose;
  preGraspPose.position.z += 0.2;

  tf2::Quaternion q;
  q.setRPY(0, M_PI / 2, 0);
  preGraspPose.orientation = tf2::toMsg(q);

  moveGroupInterface.setPoseTarget(preGraspPose, "gripper_grasping_frame");
  moveit::planning_interface::MoveGroupInterface::Plan plan2;

  if ((moveGroupInterface.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS)){
    RCLCPP_INFO(logger, "Plan 2 succeeded, now in executing");
    moveGroupInterface.execute(plan2);
  }
  else{
    RCLCPP_WARN(logger, "Planning failed");
  }

  gripperGroupInterface.setJointValueTarget(std::vector<double>({0.044, 0.044}));
  moveit::planning_interface::MoveGroupInterface::Plan plan3;

  if ((gripperGroupInterface.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS)){
    RCLCPP_INFO(logger, "Plan 3 succeeded, now in executing");
    gripperGroupInterface.execute(plan3);
  }
  else{
    RCLCPP_WARN(logger, "Planning failed");
  }

  planning_scene_interface.removeCollisionObjects({"box1"});

  geometry_msgs::msg::Pose graspPose = preGraspPose;
  graspPose.position.z -= 0.2;
  graspPose.position.z += 0.08;

  moveGroupInterface.setPoseTarget(graspPose, "gripper_grasping_frame");

  moveit::planning_interface::MoveGroupInterface::Plan plan4;

  if ((moveGroupInterface.plan(plan4) == moveit::core::MoveItErrorCode::SUCCESS)){
    RCLCPP_INFO(logger, "Plan 4 succeeded, now in executing");
    moveGroupInterface.execute(plan4);
  }
  else{
    RCLCPP_WARN(logger, "Planning failed");
  }

  gripperGroupInterface.setJointValueTarget(std::vector<double>({0.030, 0.030}));

  moveit::planning_interface::MoveGroupInterface::Plan plan5;

  if ((gripperGroupInterface.plan(plan5) == moveit::core::MoveItErrorCode::SUCCESS)){
    RCLCPP_INFO(logger, "Plan 5 succeeded, now in executing");
    gripperGroupInterface.execute(plan5);
  }
  else{
    RCLCPP_WARN(logger, "Planning failed");
  }

  std::vector<std::string> touchLinks = {"gripper_left_finger_link", "gripper_right_finger_link"};
  planning_scene_interface.applyCollisionObject(collisionObjectCube);
  moveGroupInterface.attachObject(collisionObjectCube.id, "arm_tool_link", touchLinks);

  geometry_msgs::msg::Pose aboveGraspPose = graspPose;
  aboveGraspPose.position.z += 0.05;

  moveGroupInterface.setPoseTarget(aboveGraspPose, "gripper_grasping_frame");

  moveit::planning_interface::MoveGroupInterface::Plan plan6;

  if ((moveGroupInterface.plan(plan6) == moveit::core::MoveItErrorCode::SUCCESS)){
    RCLCPP_INFO(logger, "Plan 6 succeeded, now in executing");
    moveGroupInterface.execute(plan6);
  }
  else{
    RCLCPP_WARN(logger, "Planning failed");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}