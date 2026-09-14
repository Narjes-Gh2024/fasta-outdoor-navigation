#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <string>
#include <memory>


class EncoderOdometry : public rclcpp::Node
{
    public:
        EncoderOdometry() : Node("encoder_odometry_node")
        {
            this->declare_parameter<std::string>("odom_frame_id", "odom");
            this->declare_parameter<std::string>("base_frame_id", "base_link");
            this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
            this->declare_parameter<std::string>("odom_topic", "/encoder_odom");
            this->declare_parameter<double>("wheel_base", 0.5);
            this->declare_parameter<double>("wheel_radius", 0.1);
            this->declare_parameter<bool>("simulation_mode", true);

            odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
            base_frame_id_ = this->get_parameter("base_frame_id").as_string();
            joint_states_topic_ = this->get_parameter("joint_states_topic").as_string();
            odom_topic_ = this->get_parameter("odom_topic").as_string();
            wheel_base_ = this->get_parameter("wheel_base").as_double();
            wheel_radius_ = this->get_parameter("wheel_radius").as_double();
            simulation_mode_ = this->get_parameter("simulation_mode").as_bool();

            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
            joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                joint_states_topic_,
                10,
                std::bind(&EncoderOdometry::jointStatesCallback, this, std::placeholders::_1)
            );
        }

    private:
        void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            // Guard against messages that don't have the 8 expected joints
            if (msg->position.size() < 8 || msg->velocity.size() < 8)
            {
                RCLCPP_WARN(this->get_logger(),
                    "Expected at least 8 joint positions/velocities, got %zu/%zu",
                    msg->position.size(), msg->velocity.size());
                return;
            }

            rclcpp::Time current_time = msg->header.stamp;

            if (first_callback_)
            {
                last_time_ = current_time;
                first_callback_ = false;
                return;  // no dt yet, skip this iteration
            }

            double dt = (current_time - last_time_).seconds();
            last_time_ = current_time;

            if (dt <= 0.0)
            {
                RCLCPP_WARN(this->get_logger(), "Non-positive dt (%.6f), skipping", dt);
                return;
            }

            double fl_q, bl_q, fl_qdot, bl_qdot;

            if (simulation_mode_)
            {
                fl_q = -msg->position[4];
                bl_q = -msg->position[6];
                fl_qdot = -msg->velocity[4];
                bl_qdot = -msg->velocity[6];
                dt = dt * 2;
            }
            else
            {
                fl_q = msg->position[4];
                bl_q = msg->position[6];
                fl_qdot = msg->velocity[4];
                bl_qdot = msg->velocity[6];
            }

            double fr_q = msg->position[5];
            double br_q = msg->position[7];
            double fr_qdot = msg->velocity[5];
            double br_qdot = msg->velocity[7];

            double left_q, right_q, left_qdot, right_qdot;

            if (fl_qdot != fr_qdot || bl_qdot != br_qdot)
            {
                left_q = (fl_q + bl_q) / 2.0;
                right_q = (fr_q + br_q) / 2.0;
                left_qdot = (fl_qdot + bl_qdot) / 2.0;
                right_qdot = (fr_qdot + br_qdot) / 2.0;
            }

            else
            {
                left_q = fl_q;
                right_q = fr_q;
                left_qdot = fl_qdot;
                right_qdot = fr_qdot;
            }

            // Convert wheel angular velocities (rad/s) to linear wheel speeds (m/s)
            double left_v = wheel_radius_ * left_qdot;
            double right_v = wheel_radius_ * right_qdot;

            double vx = (left_v + right_v) / 2.0;
            double vy = 0.0;
            double wz = (right_v - left_v) / wheel_base_;
            RCLCPP_INFO(this->get_logger(), "Value of dt is: (%.6f)", dt);

            // --- Persistent state: integrate into accumulated pose ---
            theta_ += wz * dt;
            x_ += vx * std::cos(theta_) * dt;
            y_ += vx * std::sin(theta_) * dt;

            tf2::Quaternion q;
            q.setRPY(0, 0, theta_);

            auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
            odom_msg->header.stamp = current_time;
            odom_msg->header.frame_id = odom_frame_id_;
            odom_msg->child_frame_id = base_frame_id_;

            odom_msg->pose.pose.position.x = x_;
            odom_msg->pose.pose.position.y = y_;
            odom_msg->pose.pose.position.z = 0.0;

            odom_msg->pose.pose.orientation.x = q.x();
            odom_msg->pose.pose.orientation.y = q.y();
            odom_msg->pose.pose.orientation.z = q.z();
            odom_msg->pose.pose.orientation.w = q.w();

            odom_msg->twist.twist.linear.x = vx;
            odom_msg->twist.twist.linear.y = vy;
            odom_msg->twist.twist.angular.z = wz;

            odom_pub_->publish(*odom_msg);
        }

        // Parameters
        std::string odom_frame_id_;
        std::string base_frame_id_;
        std::string joint_states_topic_;
        std::string odom_topic_;
        double wheel_base_;
        double wheel_radius_;

        // Persistent state (must survive between callbacks)
        rclcpp::Time last_time_;
        bool first_callback_ = true;
        double x_ = 0.0;
        double y_ = 0.0;
        double theta_ = 0.0;
        bool simulation_mode_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EncoderOdometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}