#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <vector>
#include <memory>
#include <tf2_eigen/tf2_eigen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <cmath>


using namespace std::literals::chrono_literals;

enum class TiagoInterfaceType {Arm,Gripper};

struct TargetPose {
    std::string name;
    TiagoInterfaceType tiago_interface_type;

    std::variant<Eigen::Isometry3d, std::vector<double>> data;

    TargetPose(const std::string& name, const Eigen::Isometry3d& iso)
        : name(name), tiago_interface_type(TiagoInterfaceType::Arm), data(iso) {}

    TargetPose(const std::string& name, const std::vector<double>& vec)
        : name(name), tiago_interface_type(TiagoInterfaceType::Gripper), data(vec) {}

    TargetPose(const TargetPose& other) : tiago_interface_type(other.tiago_interface_type), data(other.data) {}

    TargetPose& operator=(const TargetPose& other) {
        if (this != &other) {
            name = other.name;
            tiago_interface_type = other.tiago_interface_type;
            data = other.data;
        }
        return *this;
    }
};

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "pick_and_place", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("pick_and_place");

  bool initStartMotion = true;
  node->get_parameter_or("init_start_motion", initStartMotion, true);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });
  
  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client = node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");

  using moveit::planning_interface::MoveGroupInterface;
  auto moveGroupInterface = MoveGroupInterface(node, "arm");

  moveGroupInterface.setPoseReferenceFrame("base_footprint");
  moveGroupInterface.setMaxVelocityScalingFactor(1);
  moveGroupInterface.setMaxAccelerationScalingFactor(1);

  auto moveGroupInterface_gripper = MoveGroupInterface(node, "gripper");
  moveGroupInterface_gripper.setMaxVelocityScalingFactor(1);
  moveGroupInterface_gripper.setMaxAccelerationScalingFactor(1);

  auto moveitVisualTools = moveit_visual_tools::MoveItVisualTools{node, "base_footprint",  rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                                                  moveGroupInterface.getRobotModel() };
  moveitVisualTools.deleteAllMarkers();
  moveitVisualTools.loadRemoteControl();

  auto requestTable = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
  requestTable->name = "table";
  requestTable->reference_frame = "base_footprint";

  while (!client->wait_for_service(std::chrono::seconds(1s))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "error. Exiting.");
      return 0;
    }
    RCLCPP_INFO(logger, "Service not available, wait");
  }

  auto futureTableResult = client->async_send_request(requestTable);

  if (!(futureTableResult.wait_for(3s) == std::future_status::ready)) {
    RCLCPP_ERROR(logger, "Failed to call service /get_entity_state for table");
    return 0;
  };
    
  auto respTable = futureTableResult.get();
  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

  auto const tableSurface = [frame_id = moveGroupInterface.getPlanningFrame(), pose = respTable->state.pose] {
    moveit_msgs::msg::CollisionObject tableSurface;
    tableSurface.header.frame_id = frame_id;
    tableSurface.id = "table";
    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.80;
    primitive.dimensions[primitive.BOX_Y] = 0.80;
    primitive.dimensions[primitive.BOX_Z] = 0.05;

    geometry_msgs::msg::Pose boxPose;
    boxPose = pose;
    boxPose.position.z += 0.5;
    tableSurface.primitives.push_back(primitive);
    tableSurface.primitive_poses.push_back(boxPose);
    tableSurface.operation = tableSurface.ADD;

    return tableSurface;
  }();

  planningSceneInterface.applyCollisionObject(tableSurface);

  auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
  request->name = "green_cube_3";
  request->reference_frame = "base_footprint";

  while (!client->wait_for_service(std::chrono::seconds(1s))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "error. Exiting.");
      return 0;
    }
    RCLCPP_INFO(logger, "Service not available, wait");
  }
  auto future_result = client->async_send_request(request);

  if (future_result.wait_for(3s) == std::future_status::ready){
    auto resp = future_result.get();
    RCLCPP_INFO(logger, "The reference frame for green_cube_3 is base_footprint");
    RCLCPP_INFO(logger, "The position of green_cube_3 is: x = %f, y = %f, z = %f", resp->state.pose.position.x, resp->state.pose.position.y, resp->state.pose.position.z);
    RCLCPP_INFO(logger, "The orientation of green_cube_3 is: x = %f, y = %f, z = %f, w = %f", resp->state.pose.orientation.x, resp->state.pose.orientation.y, resp->state.pose.orientation.z, resp->state.pose.orientation.w);

    geometry_msgs::msg::Pose markerPose;
    markerPose.position = resp->state.pose.position;
    markerPose.orientation = resp->state.pose.orientation;

    geometry_msgs::msg::Pose markerFinalCubePose;
    markerFinalCubePose = markerPose;
    markerFinalCubePose.position.y = -markerPose.position.y;

    Eigen::Isometry3d cube_pose;
    tf2::fromMsg(markerPose, cube_pose);

    Eigen::Isometry3d final_cube_pose;
    tf2::fromMsg(markerFinalCubePose, final_cube_pose);

    moveitVisualTools.publishAxis(markerPose, 0.1, 0.01);

    Eigen::Isometry3d pose1 = Eigen::Translation3d(0.0, 0.0, 0.33) *  cube_pose * Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ())* Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())* Eigen::Isometry3d::Identity();
    moveitVisualTools.publishAxis(pose1, 0.1, 0.01);

    Eigen::Isometry3d pose2 =  Eigen::Translation3d(0.0, 0.0, 0.23) * cube_pose * Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ())* Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())* Eigen::Isometry3d::Identity();
    moveitVisualTools.publishAxis(pose2, 0.1, 0.01);

    Eigen::Isometry3d pose3 =  Eigen::Translation3d(0.0, 0.0, 0.23+0.05) * cube_pose * Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ())* Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())* Eigen::Isometry3d::Identity();

    Eigen::Isometry3d pose4 =  Eigen::Translation3d(0.0, 0.0, 0.23+0.05) * final_cube_pose * Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ())* Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())* Eigen::Isometry3d::Identity();

    Eigen::Isometry3d pose5 =  Eigen::Translation3d(0.0, 0.0, 0.27) * final_cube_pose * Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ())* Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())* Eigen::Isometry3d::Identity();

    Eigen::Isometry3d pose6 = Eigen::Translation3d(0.0, 0.0, 0.33) *  final_cube_pose * Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ())* Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())* Eigen::Isometry3d::Identity();

    moveitVisualTools.trigger();

    if (sqrt(pow(markerFinalCubePose.position.x, 2) + pow(markerFinalCubePose.position.y, 2)) > 0.8 ){
        RCLCPP_INFO(logger, "The cube can't be reach!");
        rclcpp::shutdown();
        spinner.join();
        return 0;
    }

    if(initStartMotion){
      std::vector<double> targetPose = {90*0.01745329, 0, 0, 0, 0 ,0, 0};
      moveGroupInterface.setJointValueTarget(targetPose);

      RCLCPP_INFO(logger, "Planning move from start");

      auto const [success, plan] = [&moveGroupInterface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(moveGroupInterface.plan(msg));
        return std::make_pair(ok, msg);
      }();

      if (success){
        std::stringstream ss_exec;
        RCLCPP_INFO(logger, "Executing move from start");
        RCLCPP_INFO(logger, "%s", ss_exec.str().c_str());
        moveGroupInterface.execute(plan);
      }
      else{
        RCLCPP_ERROR(logger, "Planning failed!");
        return 0;
      }
    }

    RCLCPP_INFO(logger, "Creating targets vector");
    const int targets_array_size = 10;

    TargetPose targets[targets_array_size] =    {
      TargetPose("pre-grasp pose", pose1),
      TargetPose("open gripper before grap", std::vector<double>({0.044, 0.044})),
      TargetPose("grasp pose", pose2),
      TargetPose("close gripper before grap", std::vector<double>({0.029, 0.029})),
      TargetPose("post-grasp pose", pose3),
      TargetPose("move before putting cube pose", pose4),
      TargetPose("put cube pose", pose5),
      TargetPose("open gripper after grap", std::vector<double>({0.044, 0.044})),
      TargetPose("go to final pose", pose6),
      TargetPose("close gripper after grap", std::vector<double>({0.030, 0.030})),
    };

    RCLCPP_INFO(logger, "Created targets vector");

    for(int i = 0; i<targets_array_size; i++){
      RCLCPP_INFO(logger, "NOW DOING %s", targets[i].name.c_str());

      if(targets[i].tiago_interface_type == TiagoInterfaceType::Gripper){
          const auto& gripper_data = std::get<std::vector<double>>(targets[i].data);
          moveGroupInterface_gripper.setJointValueTarget(gripper_data);
          RCLCPP_INFO(logger, "Planning route to the %s", targets[i].name.c_str());
          auto const [success, plan] = [&moveGroupInterface_gripper] {
              moveit::planning_interface::MoveGroupInterface::Plan msg;
              auto const ok = static_cast<bool>(moveGroupInterface_gripper.plan(msg));
              return std::make_pair(ok, msg);
          }();
          if (success){
              RCLCPP_INFO(logger, "Executing route to the %s", targets[i].name.c_str());
              moveGroupInterface_gripper.execute(plan);
          }
          else{
              RCLCPP_ERROR(logger, "Planning failed!");
              return 0;
          }
      }
      else if(targets[i].tiago_interface_type == TiagoInterfaceType::Arm){
        const auto& arm_data = std::get<Eigen::Isometry3d>(targets[i].data);
        moveGroupInterface.setPoseTarget(arm_data);
        RCLCPP_INFO(logger, "Planning route to the %s", targets[i].name.c_str());
        auto const [success, plan] = [&moveGroupInterface] {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(moveGroupInterface.plan(msg));
            return std::make_pair(ok, msg);
        }();
        if (success){
            RCLCPP_INFO(logger, "Executing route to the %s", targets[i].name.c_str());
            moveGroupInterface.execute(plan);
        }
        else{
            RCLCPP_ERROR(logger, "Planning failed!");
            return 0;
        }
      }
    }
  }
  else {
    RCLCPP_ERROR(logger, "Failed to call service /get_entity_state");
    return 0;
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
