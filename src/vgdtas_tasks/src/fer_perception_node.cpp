#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::placeholders;

class FERPerceptionNode : public rclcpp::Node
{

private:
  bool offset_calibrated, filter_initialized;
  const double filter_alpha;
  float marker_size;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::vector<cv::Point3f> obj_points;

  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params;
  cv::Ptr<cv::aruco::Dictionary> dictionary;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;

  tf2::Quaternion offset_orientation, filtered_orientation;
  tf2::Vector3 offset_position, filtered_position;

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (camera_matrix.empty()) {
      camera_matrix = (cv::Mat_<double>(3, 3) <<
        msg->k[0], msg->k[1], msg->k[2],
        msg->k[3], msg->k[4], msg->k[5],
        msg->k[6], msg->k[7], msg->k[8]
      );
      dist_coeffs = cv::Mat(msg->d, true);
      RCLCPP_INFO(this->get_logger(), "Camera intrinsics loaded.");
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {

    if (camera_matrix.empty()) {return;}

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    cv::aruco::detectMarkers(gray, dictionary, corners, ids, detector_params, rejected);

    if (!ids.empty()) {
      cv::Mat rvec, tvec;
      bool success = cv::solvePnP(
                    obj_points, corners[0], camera_matrix, dist_coeffs,
                    rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE
      );

      if (success) {

        cv::Mat rot_mat;
        cv::Rodrigues(rvec, rot_mat);
        tf2::Matrix3x3 tf2_rot(
          rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2),
          rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
          rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2)
        );

        tf2::Quaternion q;
        tf2_rot.getRotation(q);

        auto pose_in_camera = geometry_msgs::msg::PoseStamped();
        pose_in_camera.header.stamp = msg->header.stamp;
        pose_in_camera.header.frame_id = msg->header.frame_id;
        pose_in_camera.pose.position.x = tvec.at<double>(0);
        pose_in_camera.pose.position.y = tvec.at<double>(1);
        pose_in_camera.pose.position.z = tvec.at<double>(2);
        pose_in_camera.pose.orientation.x = q.x();
        pose_in_camera.pose.orientation.y = q.y();
        pose_in_camera.pose.orientation.z = q.z();
        pose_in_camera.pose.orientation.w = q.w();

        try {
          geometry_msgs::msg::PoseStamped pose_in_world;

          geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer->lookupTransform(
                            "world", pose_in_camera.header.frame_id,
                            msg->header.stamp,
                            rclcpp::Duration::from_nanoseconds(50000000)
          );
          tf2::doTransform(pose_in_camera, pose_in_world, transform_stamped);

          tf2::Transform T_base_ur;
          tf2::fromMsg(pose_in_world.pose, T_base_ur);

          if (!offset_calibrated) {
            geometry_msgs::msg::TransformStamped tf_ee = tf_buffer->lookupTransform(
                                "world", "fer_link8",
                                msg->header.stamp,
                                rclcpp::Duration::from_nanoseconds(50000000)
            );
            tf2::Transform T_ee;
            tf2::fromMsg(tf_ee.transform, T_ee);
            tf2::Transform T_offset_calc = T_base_ur.inverse() * T_ee;
            offset_position = T_offset_calc.getOrigin();
            offset_orientation = T_offset_calc.getRotation();
            offset_calibrated = true;
            RCLCPP_INFO(this->get_logger(), "fer_perception_node initialized successfully.");
          }

          tf2::Transform T_des = T_base_ur * tf2::Transform(offset_orientation, offset_position);

          if (!filter_initialized) {
            filtered_position = T_des.getOrigin();
            filtered_orientation = T_des.getRotation();
            filter_initialized = true;
          } else {
            filtered_position = filtered_position.lerp(T_des.getOrigin(), filter_alpha);
            filtered_orientation = filtered_orientation.slerp(T_des.getRotation(), filter_alpha);
            filtered_orientation.normalize();
          }

          tf2::Transform T_des_filtered(filtered_orientation, filtered_position);

          geometry_msgs::msg::PoseStamped pose_msg;
          pose_msg.header.stamp = this->now();
          pose_msg.header.frame_id = "world";
          tf2::toMsg(T_des_filtered, pose_msg.pose);

          pose_publisher->publish(pose_msg);

        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            1000,
                            "TF lookup error: %s",
                            ex.what()
          );
        }

        cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
        cv::drawFrameAxes(cv_ptr->image, camera_matrix, dist_coeffs, rvec, tvec, 0.05);

      }
    } else {
      if (!offset_calibrated) {
        RCLCPP_FATAL(this->get_logger(), "fer_peception_node initialization failed.");
        rclcpp::shutdown();
        return;
      }
    }

    auto annotated_msg = cv_ptr->toImageMsg();
    image_publisher->publish(*annotated_msg);

  }

public:
  FERPerceptionNode()
  :Node("fer_perception_node"),
    offset_calibrated(false),
    filter_initialized(false),
    filter_alpha(0.09),
    marker_size(0.05),
    tf_buffer(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener(std::make_shared<tf2_ros::TransformListener>(*tf_buffer)),
    detector_params(cv::aruco::DetectorParameters::create()),
    dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250)),
    image_publisher(this->create_publisher<sensor_msgs::msg::Image>(
            "/fer_camera_link/image_annotated",
            10
      )),
    camera_info_subscriber(this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/fer_camera_link/camera_info",
            10,
            std::bind(&FERPerceptionNode::cameraInfoCallback, this, _1)
      )),
    image_subscriber(this->create_subscription<sensor_msgs::msg::Image>(
            "/fer_camera_link/image_raw",
            10,
            std::bind(&FERPerceptionNode::imageCallback, this, _1)
    ))
  {

    float half_size = marker_size / 2.0;
    obj_points = {
      cv::Point3f(-half_size, half_size, 0.0),
      cv::Point3f(half_size, half_size, 0.0),
      cv::Point3f(half_size, -half_size, 0.0),
      cv::Point3f(-half_size, -half_size, 0.0)
    };

    pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/servo_node/pose_target_cmds",
                10
    );

  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FERPerceptionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
