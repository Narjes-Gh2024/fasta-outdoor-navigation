#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

class FrameConverter : public rclcpp::Node
{
public:
    FrameConverter() : Node("frame_converter_node")
    {
        this->declare_parameter<std::string>("lidar_frame_id", "sim_lidar");
        this->declare_parameter<std::string>("base_frame_id", "base_link");
        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->declare_parameter<std::string>("lidar_odometry_odom_frame_id", "camera_init");
        this->declare_parameter<std::string>("lidar_odometry_base_frame_id", "body");
        this->declare_parameter<bool>("publish_tf", true);
        this->declare_parameter<std::string>("lidar_odometry_topic", "/Odometry");
        this->declare_parameter<std::string>("odom_topic", "/odom");

        lidar_frame_id_ = this->get_parameter("lidar_frame_id").as_string();
        base_frame_id_ = this->get_parameter("base_frame_id").as_string();
        odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
        lidar_odometry_odom_frame_id_ = this->get_parameter("lidar_odometry_odom_frame_id").as_string();
        lidar_odometry_base_frame_id_ = this->get_parameter("lidar_odometry_base_frame_id").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        lidar_odometry_topic_ = this->get_parameter("lidar_odometry_topic").as_string();
        odom_topic_ = this->get_parameter("odom_topic").as_string();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            lidar_odometry_topic_,
            10,
            std::bind(&FrameConverter::odomCallback, this, std::placeholders::_1)
        );
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Optional sanity check: warn (once) if incoming frames don't match what we expect
        if (msg->header.frame_id != lidar_odometry_odom_frame_id_ ||
            msg->child_frame_id != lidar_odometry_base_frame_id_)
        {
            RCLCPP_WARN_ONCE(
                this->get_logger(),
                "Incoming odometry frames (%s -> %s) do not match configured "
                "lidar_odometry frames (%s -> %s)",
                msg->header.frame_id.c_str(), msg->child_frame_id.c_str(),
                lidar_odometry_odom_frame_id_.c_str(), lidar_odometry_base_frame_id_.c_str());
        }

        // Static extrinsic calibration: pose of lidar frame expressed in base_link
        // (base_link -> lidar). This is fixed, so query the latest available value
        // rather than at the odometry's exact timestamp.
        tf2::Transform base_T_lidar;
        try
        {
            geometry_msgs::msg::TransformStamped extrinsic = tf_buffer_->lookupTransform(
                base_frame_id_,
                lidar_frame_id_,
                tf2::TimePointZero
            );
            tf2::fromMsg(extrinsic.transform, base_T_lidar);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Could not look up static transform %s -> %s: %s",
                base_frame_id_.c_str(), lidar_frame_id_.c_str(), ex.what());
            return;
        }

        // Pose reported by the lidar odometry: camera_init -> body (body == lidar frame)
        tf2::Transform odom_T_lidar;
        tf2::fromMsg(msg->pose.pose, odom_T_lidar);

        // We want camera_init -> base_link:
        //   odom_T_base = odom_T_lidar * lidar_T_base
        //               = odom_T_lidar * inverse(base_T_lidar)
        tf2::Transform odom_T_base = odom_T_lidar * base_T_lidar.inverse();

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = odom_frame_id_;
        odom_msg.child_frame_id = base_frame_id_;
        tf2::toMsg(odom_T_base, odom_msg.pose.pose);

        // Twist in the incoming message is expressed in the body/lidar frame.
        // Rotate (not translate) it into base_link using the extrinsic rotation only.
        tf2::Vector3 lin_vel(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        tf2::Vector3 ang_vel(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
        tf2::Matrix3x3 R_base_lidar(base_T_lidar.getRotation());
        tf2::Vector3 lin_vel_base = R_base_lidar * lin_vel;
        tf2::Vector3 ang_vel_base = R_base_lidar * ang_vel;

        odom_msg.twist.twist.linear.x = lin_vel_base.x();
        odom_msg.twist.twist.linear.y = lin_vel_base.y();
        odom_msg.twist.twist.linear.z = lin_vel_base.z();
        odom_msg.twist.twist.angular.x = ang_vel_base.x();
        odom_msg.twist.twist.angular.y = ang_vel_base.y();
        odom_msg.twist.twist.angular.z = ang_vel_base.z();

        // Covariances are left as-is; properly rotating them would require
        // transforming the 6x6 covariance blocks by R_base_lidar as well.
        odom_msg.pose.covariance = msg->pose.covariance;
        odom_msg.twist.covariance = msg->twist.covariance;

        odom_pub_->publish(odom_msg);

        if (publish_tf_)
        {
            geometry_msgs::msg::TransformStamped odom_tf;
            odom_tf.header.stamp = msg->header.stamp;
            odom_tf.header.frame_id = odom_frame_id_;
            odom_tf.child_frame_id = base_frame_id_;
            odom_tf.transform = tf2::toMsg(odom_T_base);
            tf_broadcaster_->sendTransform(odom_tf);
        }
    }

    std::string lidar_frame_id_;
    std::string base_frame_id_;
    std::string odom_frame_id_;
    std::string lidar_odometry_odom_frame_id_;
    std::string lidar_odometry_base_frame_id_;
    std::string lidar_odometry_topic_;
    std::string odom_topic_;
    bool publish_tf_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrameConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}