#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PentagonMotionNode : public rclcpp::Node
{
public:
    PentagonMotionNode() : Node("pentagon_motion_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/mobile_base_controller/cmd_vel_unstamped", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PentagonMotionNode::move_robot, this));
        last_time_ = this->now();
    }

private:
    void move_robot()
    {
        auto current_time = this->now();
        double delta_time = (current_time - last_time_).seconds() * 1000; // Oblicz rzeczywisty czas od ostatniego wywołania
        last_time_ = current_time;

        auto message = geometry_msgs::msg::Twist();

        if (current_state_ == MotionState::MOVE_FORWARD)
        {
            message.linear.x = linear_speed_;
            message.angular.z = 0.0;

            elapsed_time_ += delta_time;
            if (elapsed_time_ >= move_duration_)
            {
                elapsed_time_ = 0.0;
                current_state_ = MotionState::STOP;
                stop_next_turn_ = true; // Przejdź do STOP przed obrotem
                RCLCPP_INFO(this->get_logger(), "Stopping after moving forward.");
            }
        }
        else if (current_state_ == MotionState::TURN)
        {
            message.linear.x = 0.0;
            message.angular.z = angular_speed_;

            elapsed_time_ += delta_time;
            if (elapsed_time_ >= turn_duration_)
            {
                elapsed_time_ = 0.0;
                current_state_ = MotionState::STOP;
                stop_next_turn_ = false; // Przejdź do STOP po obrocie
                RCLCPP_INFO(this->get_logger(), "Stopping after turning.");

                ++completed_sides_;
            }
        }
        else if (current_state_ == MotionState::STOP)
        {
            message.linear.x = 0.0;
            message.angular.z = 0.0;

            elapsed_time_ += delta_time;
            if (elapsed_time_ >= stop_duration_)
            {
                elapsed_time_ = 0.0;
                if (stop_next_turn_)
                {
                    current_state_ = MotionState::TURN;
                    RCLCPP_INFO(this->get_logger(), "Turning robot.");
                }
                else
                {
                    current_state_ = MotionState::MOVE_FORWARD;
                    RCLCPP_INFO(this->get_logger(), "Moving robot forward.");
                }
            }
        }

        publisher_->publish(message);
    }

    enum class MotionState
    {
        MOVE_FORWARD,
        TURN,
        STOP
    };

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    MotionState current_state_ = MotionState::MOVE_FORWARD;
    double elapsed_time_ = 0.0;
    int completed_sides_ = 0;
    bool stop_next_turn_ = false;

    const double linear_speed_ = 0.45;        // m/s
    const double angular_speed_ = 2.2608;       // rad/s
    const double move_duration_ = 7000;      // ms (adjust for side length)
    const double turn_duration_ = 2390;      // ms for 72 degrees (2*pi/5)
    const double stop_duration_ = 1000;      // ms pause at each vertex

    rclcpp::Time last_time_;                 // Przechowuje ostatni znacznik czasu
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PentagonMotionNode>());
    rclcpp::shutdown();
    return 0;
}
