#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <algorithm>
#include <cctype>
#include <string>
#include <memory>

class GoalNavigator : public rclcpp::Node{
public: 
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GoalNavigator() : Node("goal_navigator"){
        declare_parameter<std::string>("destination", "living_room");
        target_location_ = get_parameter("destination").as_string();
        std::transform(target_location_.begin(), target_location_.end(), target_location_.begin(),
                       [](unsigned char c) { return std::tolower(c); });

        declare_parameter<std::string>("difficulty", "easy");
        difficulty_level_ = get_parameter("difficulty").as_string();
        std::transform(difficulty_level_.begin(), difficulty_level_.end(), difficulty_level_.begin(),
                       [](unsigned char c) { return std::tolower(c); });

        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

        timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&GoalNavigator::dispatch_goal, this));
    }

private:
    void dispatch_goal(){
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "Action server unavailable after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = NavigateToPose::Goal();

        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = get_clock()->now();
        goal_msg.pose.pose.orientation.w = 1.0;

        configure_goal_position(goal_msg);

        RCLCPP_INFO(get_logger(), "Dispatching goal to: %s, difficulty: %s (x=%.2f, y=%.2f)",
                    target_location_.c_str(),
                    difficulty_level_.c_str(),
                    goal_msg.pose.pose.position.x, 
                    goal_msg.pose.pose.position.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&GoalNavigator::goal_result_callback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void configure_goal_position(NavigateToPose::Goal &goal_msg){
        if (target_location_ == "living_room") {
            assign_position(goal_msg, 1.0, 1.0, 5.0, 2.0, -4.0, -1.0);
        } else if (target_location_ == "bathroom") {
            assign_position(goal_msg, 0.0, 4.0, -1.0, 4.0, 2.2, 3.9);
        } else if (target_location_ == "hallway") {
            assign_position(goal_msg, -3.0, -3.0, -3.0, -3.9, -3.5, -3.5);
        } else if (target_location_ == "kitchen") {
            assign_position(goal_msg, -0.5, -4.0, 3.5, -3.0, 4.5, -0.5);
        } else if (target_location_ == "bedroom") {
            assign_position(goal_msg, -3.0, 3.0, -2.2, 4.0, -4.5, 4.0);
        } else if (target_location_ == "childroom") {
            assign_position(goal_msg, -3.0, -3.0, -2.2, -3.2, -4.0, -3.5);
        }
    }

    void assign_position(NavigateToPose::Goal &goal_msg,
                         double easy_x, double easy_y,
                         double medium_x, double medium_y,
                         double hard_x, double hard_y){
        if (difficulty_level_ == "easy") {
            goal_msg.pose.pose.position.x = easy_x;
            goal_msg.pose.pose.position.y = easy_y;
        } else if (difficulty_level_ == "medium") {
            goal_msg.pose.pose.position.x = medium_x;
            goal_msg.pose.pose.position.y = medium_y;
        } else if (difficulty_level_ == "hard") {
            goal_msg.pose.pose.position.x = hard_x;
            goal_msg.pose.pose.position.y = hard_y;
        }
    }

    void goal_result_callback(const GoalHandleNavigateToPose::WrappedResult &result){
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(get_logger(), "Goal reached successfully!");
        } else {
            RCLCPP_WARN(get_logger(), "Goal outcome unknown!");
        }
        rclcpp::shutdown();
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string target_location_;
    std::string difficulty_level_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
