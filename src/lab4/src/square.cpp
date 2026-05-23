#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>

class SquareMotionNode : public rclcpp::Node{
public:
    SquareMotionNode() : Node("square_motion_node"){
        this->declare_parameter<double>("side_length", 2.0);
        this->declare_parameter<int>("repetitions", 1);
        this->declare_parameter<bool>("clockwise", false);

        this->get_parameter("side_length", side_length_);
        this->get_parameter("repetitions", repetitions_);
        this->get_parameter("clockwise", clockwise_);

        move_duration_ = side_length_ / linear_speed_*1000;

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/mobile_base_controller/cmd_vel_unstamped", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SquareMotionNode::move_robot, this));
        last_time = this->now();

        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/mobile_base_controller/odom", 10,
            std::bind(&SquareMotionNode::odom_callback, this, std::placeholders::_1));

        odom2_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth_odom", 10,
            std::bind(&SquareMotionNode::odom2_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Square printing starting: loops=%d, direction=%d, side_length=%.2f",
                repetitions_, clockwise_, side_length_);

    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        cur_x_odom = msg->pose.pose.position.x;
        cur_y_odom = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_yaw2 = normalize_angle(yaw);

        record_errors();
    }

    void odom2_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        cur_x_odom2 = msg->pose.pose.position.x;
        cur_y_odom2 = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_yaw1 = normalize_angle(yaw);
    }

    void move_robot(){
        auto current_time = this->now();
        double delta_time = (current_time - last_time).seconds() * 1000;
        last_time = current_time;

        auto message = geometry_msgs::msg::Twist();

        if (current_state_ == MotionState::MOVE_FORWARD){
            message.linear.x = linear_speed_;
            message.angular.z = 0.0;

            elapsed_time_ += delta_time;
            if (elapsed_time_ >= move_duration_){
                start_yaw_ = current_yaw2;
                target_yaw_ = normalize_angle(start_yaw_ + clockwise_ * M_PI/2);
                elapsed_time_ = 0.0;
                current_state_ = MotionState::STOP;
                stop_next_turn_ = true;
                RCLCPP_INFO(this->get_logger(), "Stopping after moving forward.");
            }
        }
        else if (current_state_ == MotionState::TURN){
            message.linear.x = 0.0;
            message.angular.z = clockwise_ ? -angular_speed_ : angular_speed_;

            elapsed_time_ += delta_time;
            if (elapsed_time_ >= turn_duration_){
                elapsed_time_ = 0.0;
                current_state_ = MotionState::STOP;
                stop_next_turn_ = false;
                RCLCPP_INFO(this->get_logger(), "Stopping after turning.");

                if (++completed_sides_ >= 4){
                    sum_loop_errors();
                    completed_sides_ = 0;
                    if (++completed_laps_ >= repetitions_){
                        sum_all_errors();
                        RCLCPP_INFO(this->get_logger(), "Completed all laps.");
                        rclcpp::shutdown();
                        return;
                    }
                }
            }
        }
        else if (current_state_ == MotionState::STOP){
            message.linear.x = 0.0;
            message.angular.z = 0.0;

            elapsed_time_ += delta_time;
            if (elapsed_time_ >= stop_duration_){
                elapsed_time_ = 0.0;
                if (stop_next_turn_){
                    current_state_ = MotionState::TURN;
                    RCLCPP_INFO(this->get_logger(), "Turning robot.");
                }
                else{
                    current_state_ = MotionState::MOVE_FORWARD;
                    RCLCPP_INFO(this->get_logger(), "Moving robot forward.");
                }
            }
        }
        publisher_->publish(message);
    }

    double normalize_angle(double angle){
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle <= -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    double angle_error(double target, double current){
        double diff = normalize_angle(target - current);
        return diff;
    }

    void record_errors(){
        double pos_error = std::sqrt(std::pow((cur_x_odom - cur_x_odom2), 2) +
                                     std::pow((cur_y_odom - cur_y_odom2), 2));
        double yaw_error = std::fabs(angle_error(current_yaw1, current_yaw2));

        pos_errors_.push_back(pos_error);
        yaw_errors_.push_back(yaw_error);
    }

    void sum_loop_errors(){
        double pos_sum = 0.0;
        double yaw_sum = 0.0;
        for (auto e : pos_errors_) pos_sum += e*e;
        for (auto e : yaw_errors_) yaw_sum += e*e;

        double pos_rms = std::sqrt(pos_sum / pos_errors_.size());
        double yaw_rms = std::sqrt(yaw_sum / yaw_errors_.size());

        RCLCPP_INFO(this->get_logger(), "After loop %d: Position RMS=%.4f, Yaw RMS=%.4f",
                    completed_sides_, pos_rms, yaw_rms);
    }

    void sum_all_errors(){
        double pos_sum = 0.0;
        double yaw_sum = 0.0;
        for (auto e : pos_errors_) pos_sum += e*e;
        for (auto e : yaw_errors_) yaw_sum += e*e;

        double pos_rms = std::sqrt(pos_sum / pos_errors_.size());
        double yaw_rms = std::sqrt(yaw_sum / yaw_errors_.size());

        RCLCPP_INFO(this->get_logger(), "Total: Position RMS=%.4f, Yaw RMS=%.4f",
                    pos_rms, yaw_rms);

        std::ofstream ofs("errors_report.csv");
        ofs << "pos_error,yaw_error\n";
        for (size_t i=0; i<pos_errors_.size(); i++){
            ofs << pos_errors_[i] << "," << yaw_errors_[i] << "\n";
        }
        ofs.close();

        RCLCPP_INFO(this->get_logger(), "Errors saved to errors_report.csv");
    }

    enum class MotionState{
        MOVE_FORWARD,
        TURN,
        STOP
    };

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    MotionState current_state_ = MotionState::MOVE_FORWARD;
    double elapsed_time_ = 0.0;
    int completed_sides_ = 0;
    int completed_laps_ = 0;
    bool stop_next_turn_ = false;

    const double linear_speed_ = 0.45;
    const double angular_speed_ = 1;
    double move_duration_ = 7000;
    const double turn_duration_ = 3420;
    const double stop_duration_ = 1000;
    double side_length_ = 2.0;
    int repetitions_ = 1;
    bool clockwise_ = false;
    std::vector<double> pos_errors_;
    std::vector<double> yaw_errors_;
    double start_x_, start_y_, start_yaw_;
    double target_yaw_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom2_subscriber;
    double cur_x_odom, cur_y_odom, cur_x_odom2, cur_y_odom2, current_yaw1, current_yaw2;
    rclcpp::Time last_time;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareMotionNode>());
    rclcpp::shutdown();
    return 0;
}
