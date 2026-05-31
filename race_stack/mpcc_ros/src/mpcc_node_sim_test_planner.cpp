#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include <mpcc_ros/msg/track_points.hpp>

#include "MPC/mpc.h"
#include "Model/integrator.h"
#include "Params/track.h"
#include "Plotting/plotting.h"

#include <nlohmann/json.hpp>
#include <fstream>

#include <deque>
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>

using json = nlohmann::json;
using namespace mpcc;
using std::placeholders::_1;

class MPCCNode : public rclcpp::Node {
public:
  MPCCNode() : Node("mpcc_node") {
      std::ifstream iConfig("/home/misys/forza_ws/race_stack/MPCC/C++/Params/config_sim.json");
      json jsonConfig;
      iConfig >> jsonConfig;
      json_paths_ = PathToJson{
          jsonConfig["model_path"],
          jsonConfig["cost_path"],
          jsonConfig["bounds_path"],
          jsonConfig["track_path"],
          jsonConfig["normalization_path"]
      };
      Ts_ = jsonConfig["Ts"];
      v0_ = jsonConfig["v0"];

      mpc_ = std::make_shared<MPC>(jsonConfig["n_sqp"], jsonConfig["n_reset"], jsonConfig["sqp_mixing"], Ts_, json_paths_);
      integrator_ = std::make_shared<Integrator>(Ts_, json_paths_);
      plotter_ = std::make_shared<Plotting>(Ts_, json_paths_);
      Track track(json_paths_.track_path);
      visualize_track_xy_ = track.getTrack();

      x0_ = {0, 0, 0, v0_, 0, 0, 0, 0, 0, v0_};

      // Simulated pose pub
      sim_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("sim/pose", 10);
      pub_sim_pose(x0_);

      // pred_traj pub
      pred_traj_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mpcc_predicted_traj", 10);


      // track pub (시각화 목적)
      track_center_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_center", 1);
      track_inner_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_inner", 1);
      track_outer_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_outer", 1);

      // 시각화 목적
      publish_track_center(visualize_track_xy_.X, visualize_track_xy_.Y);
      publish_track_inner(visualize_track_xy_.X_inner, visualize_track_xy_.Y_inner);
      publish_track_outer(visualize_track_xy_.X_outer, visualize_track_xy_.Y_outer);
      
      sub_track_points_ = this->create_subscription<mpcc_ros::msg::TrackPoints>("/track_points",10,std::bind(&MPCCNode::track_points_callback, this, _1));
      
      // timer
      timer_ = this->create_wall_timer(std::chrono::duration<double>(Ts_),std::bind(&MPCCNode::timer_callback, this));

      // 디버깅 로그 기록
      rclcpp::on_shutdown([this]() 
      {
        RCLCPP_INFO(this->get_logger(), "Node shutting down, plotting results...");
        plotter_->plotRun(log_, visualize_track_xy_);
        // plotter_->plotSim(log_, track_xy_);
      });
  }

private:
  void timer_callback() {

    if (track_xy_.X.size() == 0) return;            

    auto mpc_sol = mpc_->runMPC(x0_);
    x0_ = integrator_->simTimeStep(x0_, mpc_sol.u0, Ts_);
    pub_sim_pose(x0_);

    pub_pred_traj(mpc_sol.mpc_horizon);
    log_.push_back(mpc_sol);
  }

  void track_points_callback(const mpcc_ros::msg::TrackPoints::SharedPtr msg) {
    track_xy_.X.resize(msg->center_points.size());
    track_xy_.Y.resize(msg->center_points.size());
    for (size_t i = 0; i < msg->center_points.size(); i++) {
        track_xy_.X[i] = msg->center_points[i].x;
        track_xy_.Y[i] = msg->center_points[i].y;
    }

    track_xy_.X_inner.resize(msg->inner_points.size());
    track_xy_.Y_inner.resize(msg->inner_points.size());
    for (size_t i = 0; i < msg->inner_points.size(); i++) {
        track_xy_.X_inner[i] = msg->inner_points[i].x;
        track_xy_.Y_inner[i] = msg->inner_points[i].y;
    }

    track_xy_.X_outer.resize(msg->outer_points.size());
    track_xy_.Y_outer.resize(msg->outer_points.size());
    for (size_t i = 0; i < msg->outer_points.size(); i++) {
        track_xy_.X_outer[i] = msg->outer_points[i].x;
        track_xy_.Y_outer[i] = msg->outer_points[i].y;
    }

    // MPCC에 트랙 갱신
    mpc_->setTrack(track_xy_.X, track_xy_.Y);
  }

  void pub_sim_pose(const State &state) {
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.frame_id = "sim";
    pose_msg.header.stamp = this->now();

    pose_msg.pose.position.x = state.X;
    pose_msg.pose.position.y = state.Y;
    pose_msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, state.phi);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    sim_pose_pub_->publish(pose_msg);
  }

  void pub_pred_traj(const std::array<OptVariables,N+1> mpc_horizon)
  {
    auto marker_array = visualization_msgs::msg::MarkerArray();
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "sim";
    marker.header.stamp = this->now();
    marker.ns = "mpcc_pred_traj";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS; 
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.12;
    marker.scale.y = 0.12;
    marker.scale.z = 0.12; 
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    for (const auto& opt : mpc_horizon)
    {
        geometry_msgs::msg::Point pt;
        pt.x = opt.xk.X;
        pt.y = opt.xk.Y;
        pt.z = 0.0;
        marker.points.push_back(pt);
    }

    marker_array.markers.push_back(marker);
    pred_traj_pub_->publish(marker_array);
  }

  void publish_track_center(const Eigen::VectorXd &X, const Eigen::VectorXd &Y) {
    auto marker_array = visualization_msgs::msg::MarkerArray();
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "sim";
    marker.header.stamp = this->now();
    marker.ns = "track_center";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS; 
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05; 
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;


    for (int i = 0; i < X.size(); ++i) {
        geometry_msgs::msg::Point pt;
        pt.x = X[i];
        pt.y = Y[i];
        pt.z = 0.0;
        marker.points.push_back(pt);
    }

    marker_array.markers.push_back(marker);
    track_center_pub_->publish(marker_array);
  };

  void publish_track_inner(const Eigen::VectorXd &X, const Eigen::VectorXd &Y) {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "sim";
    marker.header.stamp = this->now();
    marker.ns = "track_inner_wall_surface";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    double height = 0.2;

    for (int i = 0; i < X.size() - 1; ++i) {
        geometry_msgs::msg::Point p1, p2, p1_top, p2_top;

        p1.x = X[i];
        p1.y = Y[i];
        p1.z = 0.0;

        p2.x = X[i + 1];
        p2.y = Y[i + 1];
        p2.z = 0.0;

        p1_top = p1;
        p1_top.z = height;

        p2_top = p2;
        p2_top.z = height;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p1_top);

        marker.points.push_back(p1_top);
        marker.points.push_back(p2);
        marker.points.push_back(p2_top);
    }

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    track_outer_pub_->publish(marker_array);
  };

  void publish_track_outer(const Eigen::VectorXd &X, const Eigen::VectorXd &Y) {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "sim";
    marker.header.stamp = this->now();
    marker.ns = "track_outer_wall_surface";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    double height = 0.2;

    for (int i = 0; i < X.size() - 1; ++i) {
        geometry_msgs::msg::Point p1, p2, p1_top, p2_top;

        p1.x = X[i];
        p1.y = Y[i];
        p1.z = 0.0;

        p2.x = X[i + 1];
        p2.y = Y[i + 1];
        p2.z = 0.0;

        p1_top = p1;
        p1_top.z = height;

        p2_top = p2;
        p2_top.z = height;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p1_top);

        marker.points.push_back(p1_top);
        marker.points.push_back(p2);
        marker.points.push_back(p2_top);
    }

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    track_outer_pub_->publish(marker_array);
  }

  double Ts_, v0_;

  State x0_;

  TrackPos visualize_track_xy_, track_xy_;
  PathToJson json_paths_;

  std::shared_ptr<MPC> mpc_;
  std::shared_ptr<Integrator> integrator_;
  std::shared_ptr<Plotting> plotter_;

  std::list<MPCReturn> log_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sim_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pred_traj_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_center_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_inner_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_outer_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<mpcc_ros::msg::TrackPoints>::SharedPtr sub_track_points_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPCCNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}