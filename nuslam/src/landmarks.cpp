#include <armadillo>
#include <iostream>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "nuturtle_interfaces/msg/circle.hpp"
#include "nuturtle_interfaces/msg/circles.hpp"

#include "turtlelib/detect.hpp"
#include "turtlelib/ekf_slam.hpp"

using rcl_interfaces::msg::ParameterDescriptor;
using sensor_msgs::msg::LaserScan;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using nuturtle_interfaces::msg::Circle;
using nuturtle_interfaces::msg::Circles;


class Landmark : public rclcpp::Node
{
private:
  /// @brief
  /// @param msg
  void sub_scan_callback_(LaserScan::SharedPtr msg)
  {
    scan_frame_id_ = msg->header.frame_id;
    const auto angle_min = msg->angle_min;
    const auto angle_inc = msg->angle_increment;
    const auto data = msg->ranges;
    const auto range_min = msg->range_min;
    const auto range_max = msg->range_max;

    bool detect_flag = false;
    bool on_start = true;

    std::vector<turtlelib::Point2D> data_points;
    std::vector<turtlelib::Point2D> start_points;
    data_points.clear();
    // std::vector<turtlelib::Point2D> data_detect;

    turtlelib::Point2D pre_data{1e10, 1e10};
    // size_t pre_index = 1e10;

    for (size_t i = 0; i < data.size(); ++i) {
      const auto theta = angle_min + i * angle_inc;
      const auto range = data.at(i);

      const auto x = range * cos(theta);
      const auto y = range * sin(theta);


      if (range > range_min && range < range_max) {

        const turtlelib::Point2D point{x, y};

        if ((turtlelib::almost_equal(pre_data.x, 1e10) &&
          turtlelib::almost_equal(pre_data.y, 1e10)))
        {
          // pre_index = i;
          pre_data.x = point.x;
          pre_data.y = point.y;
          continue;
        }

        if (on_start) {
          start_points.push_back(pre_data);
        } else {
          data_points.push_back(pre_data);
        }
        const auto dist = sqrt(pow(pre_data.x - point.x, 2.0) + pow(pre_data.y - point.y, 2.0));

        if (dist >= 0.5) {
          if (data_points.size() >= 4) {
            detect_flag = true;
          } else {
            data_points.clear();
          }
        }

        // pre_data = point;
        pre_data.x = point.x;
        pre_data.y = point.y;
        // pre_index = i;
        RCLCPP_DEBUG_STREAM(get_logger(), i << ": " << point << " dist: " << dist);
      } else if (data_points.size() >= 4) {
        RCLCPP_DEBUG_STREAM(get_logger(), data_points.size() << " " << i);
        // for (size_t j = 0; j < data_points.size(); ++i) {
        //   RCLCPP_DEBUG_STREAM(get_logger(), data_points.at(j));
        // }
        detect_flag = true;
        pre_data.x = 1e10;
        pre_data.y = 1e10;
      } else {
        on_start = false;
        if (data_points.size() > 0) {
          RCLCPP_WARN_STREAM(
            get_logger(),
            "Ignoring data with size of " << data_points.size() << " at index of " << i);
        }
        data_points.clear();
        detect_flag = false;
      }

      if (i == data_points.size() - 1 && data_points.size() > 0) {

      }

      if (detect_flag) {
        RCLCPP_DEBUG_STREAM(get_logger(), "Start detecting " << data_points.size() << " ...");
        detection_.update_data(data_points);

        const double x_hat = detection_.compute_x_mean();
        const double y_hat = detection_.compute_y_mean();

        detection_.shift_coordinate(x_hat, y_hat);
        const double z_bar = detection_.compute_z_mean();

        const arma::mat Z_mat = detection_.construct_Z_mat();
        const arma::mat M_mat = detection_.compute_M_mat(Z_mat);
        const arma::mat H_mat = detection_.construct_H_mat(z_bar);
        const arma::mat H_inv = H_mat.i();

        RCLCPP_DEBUG_STREAM(get_logger(), "Z_mat: " << std::endl << Z_mat);

        arma::mat U_mat;
        arma::vec sigma_vec;
        arma::mat V_mat;

        arma::svd(U_mat, sigma_vec, V_mat, Z_mat);
        const arma::mat Sigma_mat = arma::diagmat(sigma_vec);

        RCLCPP_DEBUG_STREAM(get_logger(), "sigma_vec: " << std::endl << sigma_vec);
        RCLCPP_DEBUG_STREAM(get_logger(), "Sigma_mat: " << std::endl << Sigma_mat);

        const double sigma_min = sigma_vec.min();
        RCLCPP_DEBUG_STREAM(get_logger(), "sigma_min: " << sigma_min);

        arma::mat A_vec;
        if (sigma_min < 1e-12) {
          A_vec = V_mat.col(3);
        } else {
          const arma::mat Y_mat = V_mat * Sigma_mat * V_mat.t();
          const arma::mat Q_mat = Y_mat * H_inv * Y_mat;

          arma::vec eig_val;
          arma::mat eig_vec;
          arma::eig_sym(eig_val, eig_vec, Q_mat);
          RCLCPP_DEBUG_STREAM(get_logger(), "Eigen value: " << std::endl << eig_vec << eig_val);

          const int min_eig_ind = eig_val.index_min();
          const arma::vec A_star_vec = eig_vec.col(min_eig_ind);

          A_vec = arma::solve(Y_mat, A_star_vec);
        }

        RCLCPP_DEBUG_STREAM(get_logger(), "A_vec: " << std::endl << A_vec);

        const double A1 = A_vec(0);
        const double A2 = A_vec(1);
        const double A3 = A_vec(2);
        const double A4 = A_vec(3);

        const double a = -A2 / (2.0 * A1);
        const double b = -A3 / (2.0 * A1);
        const double R2 = (pow(A2, 2.0) + pow(A3, 2.0) - 4.0 * A1 * A4) / (4.0 * pow(A1, 2.0));

        const double center_x = a + x_hat;
        const double center_y = b + y_hat;
        const double R = sqrt(R2);

        const turtlelib::Point2D center{center_x, center_y};
        RCLCPP_DEBUG_STREAM(get_logger(), "fit center: " << center);
        RCLCPP_DEBUG_STREAM(get_logger(), "fit radius: " << R);
        landmarks_.push_back({center_x, center_y, R});

        data_points.clear();
        detect_flag = false;
      }
    }

    if (landmarks_.size() > 0) {
      publish_markers_(msg->header.stamp);
      publish_measurements_();
      landmarks_.clear();
    }
  }

  /// @brief
  /// @param time_stamp
  void publish_markers_(rclcpp::Time time_stamp)
  {
    MarkerArray msg_markers;

    for (size_t i = 0; i < landmarks_.size(); ++i) {
      Marker m;

      m.header.stamp = time_stamp;
      m.header.frame_id = scan_frame_id_;
      m.id = 40 + i;
      m.type = Marker::CYLINDER;
      m.action = Marker::ADD;
      m.pose.position.x = landmarks_.at(i).x;
      m.pose.position.y = landmarks_.at(i).y;
      m.pose.position.z = marker_height_ / 2.0 - 0.182;
      m.scale.x = 2.0 * landmarks_.at(i).r;
      m.scale.y = 2.0 * landmarks_.at(i).r;
      m.scale.z = marker_height_;
      m.color.r = 0.0;
      m.color.g = 0.0;
      m.color.b = 1.0;
      m.color.a = 1.0;

      msg_markers.markers.push_back(m);
    }
    pub_detect_marker_->publish(msg_markers);
  }

  void publish_measurements_()
  {
    Circles msg_circles;

    for (size_t i = 0; i < landmarks_.size(); ++i) {
      turtlelib::Landmark landmark = landmarks_.at(i);

      const auto dx = landmark.x;
      const auto dy = landmark.y;

      const turtlelib::Point2D ps{dx, dy};
      const turtlelib::Point2D pb = Tbs_(ps);

      Circle msg_circle;
      msg_circle.x = pb.x;
      msg_circle.y = pb.y;
      msg_circle.r = landmark.r;

      msg_circles.circles.push_back(msg_circle);
    }

    pub_detect_circle_->publish(msg_circles);
  }

  rclcpp::QoS marker_qos_;

  rclcpp::Subscription<LaserScan>::SharedPtr sub_scan_;

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_detect_marker_;
  rclcpp::Publisher<Circles>::SharedPtr pub_detect_circle_;

  std::string scan_frame_id_;

  turtlelib::CircleDetect detection_;
  std::vector<turtlelib::Landmark> landmarks_;
  double marker_radius_;
  double marker_height_;
  turtlelib::Transform2D Tbs_;

public:
  Landmark()
  : rclcpp::Node("landmark"), marker_qos_(10), marker_radius_(0.038), marker_height_(0.25),
    Tbs_({-0.032, 0.0}, 0.0)
  {
    ParameterDescriptor scan_frame_id_des;

    scan_frame_id_des.description = "Frame id of the scan frame";

    declare_parameter("scan_frame_id", "scan", scan_frame_id_des);

    scan_frame_id_ = get_parameter("scan_frame_id").as_string();

    marker_qos_.transient_local();

    sub_scan_ =
      create_subscription<LaserScan>(
      "scan", 10,
      std::bind(&Landmark::sub_scan_callback_, this, std::placeholders::_1));

    pub_detect_marker_ = create_publisher<MarkerArray>("~/detect", marker_qos_);
    pub_detect_circle_ = create_publisher<Circles>("detect/circles", 10);
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_landmark = std::make_shared<Landmark>();
  rclcpp::spin(node_landmark);

  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}
