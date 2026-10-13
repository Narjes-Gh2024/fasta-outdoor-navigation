#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <stdexcept>

class EncoderOdometry : public rclcpp::Node
{
    public:
        EncoderOdometry() : Node("encoder_odometry_node")
        {
            this->declare_parameter<std::string>("odom_frame_id", "odom");
            this->declare_parameter<std::string>("base_frame_id", "base_link");
            this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
            this->declare_parameter<std::string>("odom_topic", "/encoder_odom");
            this->declare_parameter<double>("wheel_base", 1.116);
            this->declare_parameter<double>("wheel_radius", 0.25);
            this->declare_parameter<bool>("simulation_mode", true);
            this->declare_parameter<bool>("publish_tf", true);
            this->declare_parameter<std::vector<std::string>>("left_wheel_joints", {});
            this->declare_parameter<std::vector<std::string>>("right_wheel_joints", {});

            odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
            base_frame_id_ = this->get_parameter("base_frame_id").as_string();
            joint_states_topic_ = this->get_parameter("joint_states_topic").as_string();
            odom_topic_ = this->get_parameter("odom_topic").as_string();
            wheel_base_ = this->get_parameter("wheel_base").as_double();
            wheel_radius_ = this->get_parameter("wheel_radius").as_double();
            simulation_mode_ = this->get_parameter("simulation_mode").as_bool();
            publish_tf_ = this->get_parameter("publish_tf").as_bool();
            left_wheel_joints_ = this->get_parameter("left_wheel_joints").as_string_array();
            right_wheel_joints_ = this->get_parameter("right_wheel_joints").as_string_array();

            if(left_wheel_joints_.size() != 2 ||
            right_wheel_joints_.size() != 2)
            {
                throw std::runtime_error("Need exactly two left and two right wheel joints");
            }

            // Publisher
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            // Subscriber
            joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                joint_states_topic_,
                10,
                std::bind(&EncoderOdometry::jointStatesCallback, this, std::placeholders::_1)
            );
        }

    private:
        void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
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

            double left_qdot_1 = getJointVelocity(msg, left_wheel_joints_[0]);
            double left_qdot_2 = getJointVelocity(msg, left_wheel_joints_[1]);
            double right_qdot_1 = getJointVelocity(msg, right_wheel_joints_[0]);
            double right_qdot_2 = getJointVelocity(msg, right_wheel_joints_[1]);
            double left_qdot = (left_qdot_1 + left_qdot_2) / 2.0;
            double right_qdot = (right_qdot_1 + right_qdot_2) / 2.0;

            // Convert wheel angular velocities (rad/s) to linear wheel speeds (m/s)
            double left_v = wheel_radius_ * left_qdot;
            double right_v = wheel_radius_ * right_qdot;

            double vx = (left_v + right_v) / 2.0;
            double vy = 0.0;
            double wz = (right_v - left_v) / wheel_base_;
            RCLCPP_DEBUG(this->get_logger(), "Value of dt is: (%.6f)", dt);

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
            if(publish_tf_)
            {
                geometry_msgs::msg::TransformStamped transform;

                transform.header.stamp = current_time;
                transform.header.frame_id = odom_frame_id_;
                transform.child_frame_id = base_frame_id_;

                transform.transform.translation.x = x_;
                transform.transform.translation.y = y_;
                transform.transform.translation.z = 0.0;

                transform.transform.rotation.x = q.x();
                transform.transform.rotation.y = q.y();
                transform.transform.rotation.z = q.z();
                transform.transform.rotation.w = q.w();

                tf_broadcaster_->sendTransform(transform);
            }
        }
        double getJointVelocity(
            const sensor_msgs::msg::JointState::SharedPtr msg,
            const std::string &joint_name)
        {
            for(size_t i = 0; i < msg->name.size(); i++)
            {
                if(msg->name[i] == joint_name)
                {
                    if(i < msg->velocity.size())
                    {
                        return msg->velocity[i];
                    }
                    break;  // velocity array is empty/too short
                }
            }

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Joint %s not found (or no velocity)",
                joint_name.c_str());

            return 0.0;
        }

        // Parameters
        std::string odom_frame_id_;
        std::string base_frame_id_;
        std::string joint_states_topic_;
        std::string odom_topic_;
        std::vector<std::string> left_wheel_joints_;
        std::vector<std::string> right_wheel_joints_;
        double wheel_base_;
        double wheel_radius_;

        // Persistent state (must survive between callbacks)
        rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
        bool first_callback_ = true;
        double x_ = 0.0;
        double y_ = 0.0;
        double theta_ = 0.0;
        bool simulation_mode_;
        bool publish_tf_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<EncoderOdometry>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("encoder_odometry_node"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}