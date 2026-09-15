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
#include <string>

using namespace std::chrono_literals;

class OdometryStateMachine : public rclcpp::Node
{
    public:
        enum class OdomSource
        {
            LIDAR,
            FUSED,
            ENCODER
        };

        OdometryStateMachine() : Node("odometry_state_machine_node")
        {
            this->declare_parameter<std::string>("odom_frame_id", "odom");
            this->declare_parameter<std::string>("base_frame_id", "base_link");
            this->declare_parameter<std::string>("lidar_odometry_topic", "/odom");
            this->declare_parameter<std::string>("encoder_odometry_topic", "/encoder_odom");
            this->declare_parameter<std::string>("fused_odometry_topic", "/odometry/filtered");
            this->declare_parameter<std::string>("output_odom_topic", "/output_odom");
            this->declare_parameter<double>("lidar_timeout_seconds", 10.0);
            this->declare_parameter<double>("fused_timeout_seconds", 5.0);

            odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
            base_frame_id_ = this->get_parameter("base_frame_id").as_string();
            lidar_odometry_topic_ = this->get_parameter("lidar_odometry_topic").as_string();
            encoder_odometry_topic_ = this->get_parameter("encoder_odometry_topic").as_string();
            fused_odometry_topic_ = this->get_parameter("fused_odometry_topic").as_string();
            output_odom_topic_ = this->get_parameter("output_odom_topic").as_string();
            lidar_timeout_seconds_ = this->get_parameter("lidar_timeout_seconds").as_double();
            fused_timeout_seconds_ = this->get_parameter("fused_timeout_seconds").as_double();

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
            fused_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                fused_odometry_topic_,
                10,
                std::bind(&OdometryStateMachine::fusedOdomCallback, this, std::placeholders::_1)
            );

            // Initialize "last good" times to now so we don't immediately time out at startup
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                last_valid_lidar_time_ = this->now();
                last_valid_fused_time_ = this->now();
            }

            // Periodic watchdog: cascades LIDAR -> FUSED -> ENCODER on staleness.
            watchdog_timer_ = this->create_wall_timer(
                200ms,
                std::bind(&OdometryStateMachine::checkTimeout, this)
            );
        }

    private:
        static const char* sourceName(OdomSource s)
        {
            switch (s)
            {
                case OdomSource::LIDAR:   return "lidar";
                case OdomSource::FUSED:   return "fused";
                case OdomSource::ENCODER: return "encoder";
            }
            return "unknown";
        }

        void lidarOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            bool tf_ok = tf_buffer_->canTransform(
                odom_frame_id_, base_frame_id_, tf2::TimePointZero);

            if (!tf_ok)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Received /%s message but %s->%s TF is not available",
                    lidar_odometry_topic_.c_str(), odom_frame_id_.c_str(), base_frame_id_.c_str());
                return; 
            }

            OdomSource previous_source;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                last_valid_lidar_time_ = this->now();
                previous_source = current_source_;
                current_source_ = OdomSource::LIDAR;  // lidar is always top priority when valid
            }

            if (previous_source != OdomSource::LIDAR)
            {
                RCLCPP_INFO(this->get_logger(),
                    "Lidar odometry recovered — switching back to lidar source (was %s)",
                    sourceName(previous_source));
            }

            odom_pub_->publish(*msg);
            publishTf(msg->header.stamp, msg->pose.pose);
        }

        void fusedOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            bool active = false;
            bool promoted = false;
            OdomSource previous_source;

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                last_valid_fused_time_ = this->now();
                previous_source = current_source_;

                if (current_source_ == OdomSource::FUSED)
                {
                    active = true;
                }
                else if (current_source_ == OdomSource::ENCODER)
                {
                    // Fused data came back while we were on encoder -> promote.
                    current_source_ = OdomSource::FUSED;
                    active = true;
                    promoted = true;
                }
                // If current_source_ == LIDAR, lidar remains preferred; just record freshness.
            }

            if (promoted)
            {
                RCLCPP_INFO(this->get_logger(),
                    "Fused odometry recovered — switching from %s to fused source",
                    sourceName(previous_source));
            }

            if (active)
            {
                odom_pub_->publish(*msg);
                publishTf(msg->header.stamp, msg->pose.pose);
            }
        }

        void encoderOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            bool active;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                active = (current_source_ == OdomSource::ENCODER);
            }

            if (active)
            {
                odom_pub_->publish(*msg);
                publishTf(msg->header.stamp, msg->pose.pose);
            }
            // If lidar or fused is currently the active source, silently drop encoder messages.
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
            OdomSource previous_source;
            OdomSource new_source;

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                previous_source = current_source_;

                if (current_source_ == OdomSource::LIDAR)
                {
                    double elapsed = (now - last_valid_lidar_time_).seconds();
                    if (elapsed > lidar_timeout_seconds_)
                    {
                        current_source_ = OdomSource::FUSED;
                        should_switch = true;
                    }
                }
                else if (current_source_ == OdomSource::FUSED)
                {
                    double elapsed = (now - last_valid_fused_time_).seconds();
                    if (elapsed > fused_timeout_seconds_)
                    {
                        current_source_ = OdomSource::ENCODER;
                        should_switch = true;
                    }
                }
                // If already on ENCODER, there's nothing further to fall back to;
                // recovery back to FUSED/LIDAR is handled in their callbacks.

                new_source = current_source_;
            }

            if (should_switch)
            {
                RCLCPP_WARN(this->get_logger(),
                    "No valid %s odometry for too long — switching to %s",
                    sourceName(previous_source), sourceName(new_source));
            }
        }

        // Parameters
        std::string odom_frame_id_;
        std::string base_frame_id_;
        std::string lidar_odometry_topic_;
        std::string encoder_odometry_topic_;
        std::string fused_odometry_topic_;
        std::string output_odom_topic_;
        double lidar_timeout_seconds_;
        double fused_timeout_seconds_;

        // State
        std::mutex state_mutex_;
        rclcpp::Time last_valid_lidar_time_;
        rclcpp::Time last_valid_fused_time_;
        OdomSource current_source_ = OdomSource::LIDAR;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr encoder_odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fused_odom_sub_;
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