#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <vector>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <gazebo_msgs/srv/get_entity_state.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::literals::chrono_literals;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto const node = std::make_shared<rclcpp::Node>(
        "show_side_grasps", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    auto const logger = rclcpp::get_logger("show_side_grasps");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]()
                               { executor.spin(); });

    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client = node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
    Eigen::Isometry3d cube_pose;

    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = "green_cube_3";
    request->reference_frame = "base_footprint";

    while (!client->wait_for_service(std::chrono::seconds(1s)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(logger, "Interrupted. Exiting.");
            return 0;
        }
        RCLCPP_INFO(logger, "Service not available, waiting...");
    }
    auto future_result = client->async_send_request(request);
    if (future_result.wait_for(3s) == std::future_status::ready)
    {
        auto resp = future_result.get();

        geometry_msgs::msg::Pose marker_pose;
        marker_pose.position = resp->state.pose.position;
        marker_pose.orientation = resp->state.pose.orientation;

        tf2::fromMsg(marker_pose, cube_pose);
    }

    else
    {
        RCLCPP_ERROR(logger, "Service /get_entity_state failed, no green_cube_3 pose!");
        return 0;
    }

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "arm");

    auto moveit_visual_tools =
        moveit_visual_tools::MoveItVisualTools{node, "base_footprint", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                               move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();
    moveit_visual_tools.trigger();

    auto const prompt = [&moveit_visual_tools](auto text)
    { moveit_visual_tools.prompt(text); };

    auto jmg = move_group_interface.getRobotModel()->getJointModelGroup("gripper");

    Eigen::Isometry3d T_Egr_B;
    Eigen::Isometry3d T_Epre_B;

    Eigen::Isometry3d T_rest = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()) * (Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY())) * Eigen::Translation3d(-0.23, 0.0, 0.0) * Eigen::Isometry3d::Identity();

    Eigen::Isometry3d T_Ogr_O;

    for (int i = 0; i < 12; i++)
    {

        if (i < 4)
        {
            T_Ogr_O = Eigen::AngleAxisd(M_PI / 2.0 * i, Eigen::Vector3d::UnitZ()) * Eigen::Isometry3d::Identity();
        }
        else if (i < 8)
        {
            T_Ogr_O = Eigen::AngleAxisd(M_PI / 2.0 * i, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitY()) * Eigen::Isometry3d::Identity();
        }
        else
        {
            T_Ogr_O = Eigen::AngleAxisd(M_PI / 2.0 * i, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI / 12.0, Eigen::Vector3d::UnitY()) * Eigen::Isometry3d::Identity();
        }

        T_Egr_B = cube_pose * T_Ogr_O * T_rest;
        T_Epre_B = T_Egr_B * Eigen::Translation3d(-0.1, 0.0, 0.0);

        moveit_msgs::msg::Grasp vis_gr;
        vis_gr.id = "grasp_" + std::to_string(i + 1);
        vis_gr.grasp_pose.header.frame_id = "arm_tool_link";
        vis_gr.grasp_pose.pose = tf2::toMsg(T_Egr_B);

        std::vector<moveit_msgs::msg::Grasp> vis_grasps({vis_gr});

        moveit_visual_tools.deleteAllMarkers();
        moveit_visual_tools.publishAxis(cube_pose, 0.1, 0.01);
        moveit_visual_tools.publishAxis(T_Egr_B, 0.1, 0.01);
        moveit_visual_tools.publishAxis(T_Epre_B, 0.1, 0.01);
        moveit_visual_tools.trigger();
        moveit_visual_tools.publishGrasps(vis_grasps, jmg);

        RCLCPP_INFO(logger, "This is grasp no. %d", (i + 1));

        prompt("Click 'Next' to show next grasp...");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}