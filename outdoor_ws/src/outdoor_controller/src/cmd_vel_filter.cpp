#include <cmath>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>


class CmdVelFilter : public rclcpp::Node
{

public:

    CmdVelFilter()
    : Node("cmd_vel_filter_node")
    {
        // Parameters
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_out");
        this->declare_parameter<std::string>("filtered_cmd_vel_topic", "/cmd_vel_filtered");
        this->declare_parameter<double>("linear_acceleration_limit", 0.5);
        this->declare_parameter<double>("linear_deceleration_limit", 1.0);
        this->declare_parameter<double>("angular_acceleration_limit", 1.0);
        this->declare_parameter<double>("angular_deceleration_limit", 2.0);
        this->declare_parameter<double>("update_rate", 50.0);
        this->declare_parameter<double>("cmd_timeout", 0.5);

        cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
        filtered_cmd_vel_topic_ = this->get_parameter("filtered_cmd_vel_topic").as_string();
        linear_acceleration_limit_ = this->get_parameter("linear_acceleration_limit").as_double();
        linear_deceleration_limit_ = this->get_parameter("linear_deceleration_limit").as_double();
        angular_acceleration_limit_ = this->get_parameter("angular_acceleration_limit").as_double();
        angular_deceleration_limit_ = this->get_parameter("angular_deceleration_limit").as_double();
        update_rate_ = this->get_parameter("update_rate").as_double();
        cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();

        if (update_rate_ <= 0.0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "update_rate must be greater than zero");

            rclcpp::shutdown();
            return;
        }

        if (cmd_timeout_ <= 0.0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "cmd_timeout must be greater than zero");

            rclcpp::shutdown();
            return;
        }
        
        last_update_time_ = this->now();
        last_cmd_time_ = this->now();

        // Publisher
        filtered_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(filtered_cmd_vel_topic_, 10);

        // Subscriber
        cmd_vel_sub_ =this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10, std::bind(&CmdVelFilter::cmdVelCallback, this, std::placeholders::_1));

        auto period = std::chrono::duration<double>(1.0 / update_rate_);

        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&CmdVelFilter::filterCallback, this));
        RCLCPP_INFO(this->get_logger(), "Cmd velocity filter started");
    }


private:

    void cmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_linear_ = msg->linear.x;
        target_angular_ = msg->angular.z;
        last_cmd_time_ = this->now();
    }

    void filterCallback()
    {
        auto now = this->now();

        if ((now - last_cmd_time_).seconds() > cmd_timeout_)
        {
            target_linear_ = 0.0;
            target_angular_ = 0.0;
        }

        double dt = (now - last_update_time_).seconds();
        last_update_time_ = now;

        current_linear_ = limitVelocity(target_linear_, current_linear_, linear_acceleration_limit_, linear_deceleration_limit_, dt);
        current_angular_ = limitVelocity(target_angular_, current_angular_, angular_acceleration_limit_, angular_deceleration_limit_, dt);

        geometry_msgs::msg::Twist filtered_msg;

        filtered_msg.linear.x = current_linear_;
        filtered_msg.angular.z = current_angular_;

        filtered_cmd_pub_->publish(filtered_msg);
    }

double limitVelocity(double target, double current, double acceleration_limit, double deceleration_limit, double dt)
{
    double difference = target - current;
    double limit = (std::abs(target) > std::abs(current)) ? acceleration_limit : deceleration_limit;
    double step = limit * dt;

    if (difference > step) return current + step;
    if (difference < -step) return current - step;

    return target;
}

    // Parameters
    std::string cmd_vel_topic_;
    std::string filtered_cmd_vel_topic_;

    double linear_acceleration_limit_;
    double linear_deceleration_limit_;
    double angular_acceleration_limit_;
    double angular_deceleration_limit_;
    double update_rate_;
    double cmd_timeout_;

    // State
    double target_linear_ = 0.0;
    double target_angular_ = 0.0;
    double current_linear_ = 0.0;
    double current_angular_ = 0.0;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr filtered_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_update_time_;
    rclcpp::Time last_cmd_time_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}