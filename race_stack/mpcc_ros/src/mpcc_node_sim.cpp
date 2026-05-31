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
      track_xy_ = track.getTrack();
      mpc_->setTrack(track_xy_.X, track_xy_.Y);

      x0_ = {0, 0, 0, v0_, 0, 0, 0, 0, 0, v0_};

      // Simulated pose pub
      sim_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("sim/pose", 10);

      // pred_traj pub
      pred_traj_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mpcc_predicted_traj", 10);


      // track pub (시각화 목적)
      track_center_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_center", 1);
      track_inner_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_inner", 1);
      track_outer_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_outer", 1);

      // timer
      timer_ = this->create_wall_timer(std::chrono::duration<double>(Ts_),std::bind(&MPCCNode::timer_callback, this));

      // 시각화 목적
      publish_track_center(track_xy_.X, track_xy_.Y);
      publish_track_inner(track_xy_.X_inner, track_xy_.Y_inner);
      publish_track_outer(track_xy_.X_outer, track_xy_.Y_outer);
      
      rclcpp::on_shutdown([this]() 
      {
        RCLCPP_INFO(this->get_logger(), "Node shutting down, plotting results...");
        plotter_->plotRun(log_, track_xy_);
        // plotter_->plotSim(log_, track_xy_);
      });
  }

private:
  // 디버깅용 timer callback
  // void timer_callback() {
  //   auto t_start = std::chrono::steady_clock::now();

  //   auto t1 = std::chrono::steady_clock::now();
  //   auto mpc_sol = mpc_->runMPC(x0_);
  //   auto t2 = std::chrono::steady_clock::now();
  //   x0_ = integrator_->simTimeStep(x0_, mpc_sol.u0, Ts_);
  //   auto t3 = std::chrono::steady_clock::now();
  //   pub_sim_pose(x0_);

  //   //시각화 목적
  //   pub_pred_traj(mpc_sol.mpc_horizon);
  //   publish_track_center(track_xy_.X, track_xy_.Y);
  //   publish_track_inner(track_xy_.X_inner, track_xy_.Y_inner);
  //   publish_track_outer(track_xy_.X_outer, track_xy_.Y_outer);
  //   log_.push_back(mpc_sol);

  //   auto t4 = std::chrono::steady_clock::now();

  //   auto dt_mpc_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  //   auto dt_sim_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
  //   auto dt_pub_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();

  //   std::cout << "runMPC(): " << dt_mpc_ms << " ms, ";
  //   std::cout << "simTimeStep(): " << dt_sim_ms << " ms, ";
  //   std::cout << "pub(): " << dt_pub_ms << " ms" << std::endl;
  // }

  void timer_callback() {
    auto mpc_sol = mpc_->runMPC(x0_);
    x0_ = integrator_->simTimeStep(x0_, mpc_sol.u0, Ts_);
    

    //시각화 목적
    pub_sim_pose(x0_);
    pub_pred_traj(mpc_sol.mpc_horizon);
    publish_track_center(track_xy_.X, track_xy_.Y);
    publish_track_inner(track_xy_.X_inner, track_xy_.Y_inner);
    publish_track_outer(track_xy_.X_outer, track_xy_.Y_outer);
    log_.push_back(mpc_sol);
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
    marker.scale.x = 0.015;
    marker.scale.y = 0.015;
    marker.scale.z = 0.0; 
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
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;


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
    auto marker_array = visualization_msgs::msg::MarkerArray();
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "sim";
    marker.header.stamp = this->now();
    marker.ns = "track_inner";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;


    for (int i = 0; i < X.size(); ++i) {
        geometry_msgs::msg::Point pt;
        pt.x = X[i];
        pt.y = Y[i];
        pt.z = 0.0;
        marker.points.push_back(pt);
    }

    marker_array.markers.push_back(marker);
    track_inner_pub_->publish(marker_array);
  };

  void publish_track_outer(const Eigen::VectorXd &X, const Eigen::VectorXd &Y) {
    auto marker_array = visualization_msgs::msg::MarkerArray();
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "sim";
    marker.header.stamp = this->now();
    marker.ns = "track_outer";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    for (int i = 0; i < X.size(); ++i) {
        geometry_msgs::msg::Point pt;
        pt.x = X[i];
        pt.y = Y[i];
        pt.z = 0.0;
        marker.points.push_back(pt);
    }

    marker_array.markers.push_back(marker);
    track_outer_pub_->publish(marker_array);
  };

  double Ts_, v0_;

  State x0_;

  TrackPos track_xy_;
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
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPCCNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}