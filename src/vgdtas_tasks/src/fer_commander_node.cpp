#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::placeholders;

class FERCommander : public rclcpp::Node
{

    private:

        std::string base_frame, fer_eef_frame;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;

        geometry_msgs::msg::PoseStamped::SharedPtr latest_pose;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber;
        rclcpp::TimerBase::SharedPtr timer;

        void controlLoop()
        {

            if (!latest_pose) return;

            rclcpp::Time pose_time(latest_pose->header.stamp);
            rclcpp::Duration age = this->now() - pose_time;

            if (age.seconds() > 0.5)
            {
                geometry_msgs::msg::TransformStamped tf_curr;
                try
                {
                    tf_curr = tf_buffer->lookupTransform(base_frame, fer_eef_frame, tf2::TimePointZero);
                }
                catch (const tf2::TransformException & ex)
                {
                    return;
                }

                auto stop_pose = geometry_msgs::msg::PoseStamped();
                stop_pose.header.stamp = this->now();
                stop_pose.header.frame_id = base_frame;
                stop_pose.pose.position.x = tf_curr.transform.translation.x;
                stop_pose.pose.position.y = tf_curr.transform.translation.y;
                stop_pose.pose.position.z = tf_curr.transform.translation.z;
                stop_pose.pose.orientation = tf_curr.transform.rotation;
                
                pose_publisher->publish(stop_pose);
                
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Marker not visible. Holding current position."
                );
                return;
            }

            geometry_msgs::msg::PoseStamped target_pose_base;
            if (latest_pose->header.frame_id != base_frame && !latest_pose->header.frame_id.empty())
            {
                try {
                    target_pose_base = tf_buffer->transform(*latest_pose, base_frame, tf2::durationFromSec(0.0));
                } 
                catch (const tf2::TransformException & ex) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Target TF Error: %s", ex.what());
                    return;
                }
            } 
            else 
            {
                target_pose_base = *latest_pose;
            }
            target_pose_base.header.stamp = this->now();
            pose_publisher->publish(target_pose_base);

        }

        void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            latest_pose = msg;
        }

    public:

        FERCommander() :
        Node("fer_commander_node"),
        base_frame("world"), fer_eef_frame("fer_link8"),
        tf_buffer(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
        tf_listener(std::make_shared<tf2_ros::TransformListener>(*tf_buffer)),
        pose_publisher(this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/servo_node/pose_target_cmds",
            10
        )),
        pose_subscriber(this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/fer_target_pose",
            10,
            std::bind(&FERCommander::poseCallback, this, _1)
        ))
        {

            timer = this->create_wall_timer(
                std::chrono::milliseconds(10),
                std::bind(&FERCommander::controlLoop, this)
            );

        }

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FERCommander>());
    rclcpp::shutdown();
    return 0;
}