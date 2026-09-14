#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <memory>
#include <mutex>
#include <chrono>

using namespace std::chrono_literals;

class OdometryStateMachine : public rclcpp::Node
{
    public:
        OdometryStateMachine() : Node("odometry_state_machine_node")
        {
            this->declare_parameter<std::string>("odom_frame_id", "odom");
            this->declare_parameter<std::string>("base_frame_id", "base_link");
            this->declare_parameter<std::string>("lidar_odometry_topic", "/odom");
            this->declare_parameter<std::string>("encoder_odometry_topic", "/encoder_odom");
            this->declare_parameter<double>("timeout_seconds", 10.0);

            odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
            base_frame_id_ = this->get_parameter("base_frame_id").as_string();
            lidar_odometry_topic_ = this->get_parameter("lidar_odometry_topic").as_string();
            encoder_odometry_topic_ = this->get_parameter("encoder_odometry_topic").as_string();
            timeout_seconds_ = this->get_parameter("timeout_seconds").as_double();

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);
            lidar_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                lidar_odometry_topic_,
                10,
                std::bind(&OdometryStateMachine::lidarOdomCallback, this, std::placeholders::_1)
            );
            encoder_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                encoder_odometry_topic_,
                10,
                std::bind(&OdometryStateMachine::encoderOdomCallback, this, std::placeholders::_1)
            );

            // Initialize "last good" time to now so we don't immediately time out at startup
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                last_valid_lidar_time_ = this->now();
            }

            // Periodic watchdog: check whether we've gone longer than timeout_seconds_
            // since the last valid lidar odometry / TF, and flip source if so.
            watchdog_timer_ = this->create_wall_timer(
                200ms,
                std::bind(&OdometryStateMachine::checkTimeout, this)
            );
        }

    private:
        void lidarOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            // Verify the TF chain odom->base_link is actually resolvable right now,
            // not just that a message arrived on the topic.
            bool tf_ok = tf_buffer_->canTransform(
                odom_frame_id_, base_frame_id_, tf2::TimePointZero);

            if (!tf_ok)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Received /%s message but %s->%s TF is not available",
                    lidar_odometry_topic_.c_str(), odom_frame_id_.c_str(), base_frame_id_.c_str());
                return;  // don't count this as a valid reception
            }

            bool was_using_encoder;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                last_valid_lidar_time_ = this->now();
                was_using_encoder = using_encoder_;
                using_encoder_ = false;  // valid lidar data => prefer lidar again
            }

            if (was_using_encoder)
            {
                RCLCPP_INFO(this->get_logger(), "Lidar odometry recovered — switching back to lidar source");
            }

            odom_pub_->publish(*msg);
            publishTf(msg->header.stamp, msg->pose.pose);
        }

        void encoderOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            bool active;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                active = using_encoder_;
            }

            if (active)
            {
                odom_pub_->publish(*msg);
                publishTf(msg->header.stamp, msg->pose.pose);
            }
            // If lidar is currently the active source, silently drop encoder messages here.
        }

        void publishTf(const rclcpp::Time &stamp, const geometry_msgs::msg::Pose &pose)
        {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = stamp;
            tf_msg.header.frame_id = odom_frame_id_;
            tf_msg.child_frame_id = base_frame_id_;
            tf_msg.transform.translation.x = pose.position.x;
            tf_msg.transform.translation.y = pose.position.y;
            tf_msg.transform.translation.z = pose.position.z;
            tf_msg.transform.rotation = pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);
        }

        void checkTimeout()
        {
            rclcpp::Time now = this->now();
            bool should_switch = false;

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                double elapsed = (now - last_valid_lidar_time_).seconds();
                if (elapsed > timeout_seconds_ && !using_encoder_)
                {
                    using_encoder_ = true;
                    should_switch = true;
                }
            }

            if (should_switch)
            {
                RCLCPP_WARN(this->get_logger(),
                    "No valid lidar odometry/TF for over %.1f seconds — switching to /%s",
                    timeout_seconds_, encoder_odometry_topic_.c_str());
            }
        }

        // Parameters
        std::string odom_frame_id_;
        std::string base_frame_id_;
        std::string lidar_odometry_topic_;
        std::string encoder_odometry_topic_;
        std::string output_odom_topic_ = "/output_odom";
        double timeout_seconds_;

        // State
        std::mutex state_mutex_;
        rclcpp::Time last_valid_lidar_time_;
        bool using_encoder_ = false;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr encoder_odom_sub_;
        rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryStateMachine>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}