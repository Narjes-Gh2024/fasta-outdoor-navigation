#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>


class KinematicController : public rclcpp::Node
{

public:
    KinematicController()
    : Node("kinematic_controller_node")
    {
        // Parameters
        this->declare_parameter<double>("wheel_radius", 0.25);
        this->declare_parameter<double>("wheel_base", 1.116);
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_filtered");
        this->declare_parameter<std::string>("wheel_command_topic", "/wheel_controller/command");
        this->declare_parameter<double>("cmd_timeout", 0.5);

        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
        wheel_command_topic_ = this->get_parameter("wheel_command_topic").as_string();
        cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();

        // Publisher
        wheel_cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(wheel_command_topic_, 10);

        // Subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10,
                std::bind(&KinematicController::cmdVelCallback, this, std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Kinematic controller started");

            last_cmd_time_ = this->now();        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(
                &KinematicController::checkTimeout,
                this));
    }

private:
    void cmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_time_ = this->now();

        double v = msg->linear.x;
        double w = msg->angular.z;

        double left_speed =
            (v - w * wheel_base_ / 2.0)
            / wheel_radius_;

        double right_speed =
            (v + w * wheel_base_ / 2.0)
            / wheel_radius_;

        publishWheelCommand(
            left_speed,
            right_speed);
    }

    void publishWheelCommand(double left, double right)
    {
        trajectory_msgs::msg::JointTrajectory cmd;
        cmd.joint_names =
        {
            "W1_j",
            "W2_j",
            "W3_j",
            "W4_j"
        };

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.velocities =
        {
            left,
            left,
            right,
            right
        };

        cmd.points.push_back(point);
        wheel_cmd_pub_->publish(cmd);
    }

    void checkTimeout()
    {
        double elapsed =
            (this->now() - last_cmd_time_).seconds();


        if(elapsed > cmd_timeout_)
        {
            publishWheelCommand(
                0.0,
                0.0);
        }
    }

    // Parameters
    double wheel_radius_;
    double wheel_base_;
    double cmd_timeout_ = 0.5;

    std::string cmd_vel_topic_;
    std::string wheel_command_topic_;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr wheel_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_cmd_time_;

};

int main(
    int argc,
    char **argv)
{
    rclcpp::init(
        argc,
        argv);
    auto node =
        std::make_shared<KinematicController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}