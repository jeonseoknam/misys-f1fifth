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
#include <chrono>

#include <mutex>
#include <cmath>

#include <chrono>


using json = nlohmann::json;
using namespace mpcc;
using std::placeholders::_1;

class MPCCSim : public rclcpp::Node {
public:
  MPCCSim() : Node("mpcc_sim") {
    // config 파일 읽기
    std::ifstream iConfig("/home/misys/forza_ws/race_stack/MPCC/C++/Params/config_sim.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    Ts_ = jsonConfig["Ts"];
    v0_ = jsonConfig["v0"];
    state_ = {4.6689, -3.4396, 0, v0_, 0, 0, 0, 0, 0, v0_};

    json_paths_ = PathToJson{
        jsonConfig["model_path"],
        jsonConfig["cost_path"],
        jsonConfig["bounds_path"],
        jsonConfig["track_path"],
        jsonConfig["normalization_path"]
    };

    integrator_ = std::make_shared<Integrator>(Ts_, json_paths_);
    Track track(json_paths_.track_path);
    track_xy_ = track.getTrack();

    // track pub (시각화 목적)
    track_center_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_center", 1);
    track_inner_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_inner", 1);
    track_outer_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_outer", 1);
    publish_track_center(track_xy_.X, track_xy_.Y);
    publish_track_inner(track_xy_.X_inner, track_xy_.Y_inner);
    publish_track_outer(track_xy_.X_outer, track_xy_.Y_outer);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("sim/pose", rclcpp::QoS{10});
  }

  static State read_state() {
    std::lock_guard<std::mutex> lock(mutex_); // scope 끝나면 자동 unlock
    return state_;
  }

  static void update_cmd(const Input &u, const State &x) {
  geometry_msgs::msg::PoseStamped msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    state_.s = x.s;
    state_.phi = x.phi;
    state_ = integrator_->simTimeStep(state_, u, Ts_);

    msg.header.frame_id = "sim";
    msg.pose.position.x = state_.X;
    msg.pose.position.y = state_.Y;
    msg.pose.position.z = 0.0;
    tf2::Quaternion q; q.setRPY(0.0, 0.0, state_.phi);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
  }
  pose_pub_->publish(msg);
  }

private:
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
    track_inner_pub_->publish(marker_array);
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

  double v0_;
  PathToJson json_paths_;
  TrackPos track_xy_;

  static State state_;
  static double Ts_;
  static std::shared_ptr<Integrator> integrator_;
  static std::mutex mutex_;


  static rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_center_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_inner_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_outer_pub_;
};

class MPCCOne : public rclcpp::Node {
public:
  MPCCOne() : Node("mpcc_one") {
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
      x0_ = {0, 0, 0, v0_, 0, 0, 0, 0, 0, v0_};

      mpc_ = std::make_shared<MPC>(jsonConfig["n_sqp"], jsonConfig["n_reset"], jsonConfig["sqp_mixing"], Ts_, json_paths_);
      integrator_ = std::make_shared<Integrator>(Ts_, json_paths_);
      plotter_ = std::make_shared<Plotting>(Ts_, json_paths_);
      Track track(json_paths_.track_path);
      track_xy_ = track.getTrack();
      mpc_->setTrack(track_xy_.X, track_xy_.Y);

      timer_ = this->create_wall_timer(std::chrono::duration<double>(Ts_),std::bind(&MPCCOne::timer_callback, this));
  }

private:
  // void timer_callback() {
  //   x0_ = MPCCSim::read_state(); //MPCCSim 클래스의 대표 객체의 공유 멤버변수(상태변수)들을 읽어옴 (static으로 작성)
  //   auto mpc_sol = mpc_->runMPC(x0_);
  //   MPCCSim::update_cmd(mpc_sol.u0, x0_); // MPCCSim 클래스의 대표 객체의 공유 멤버변수(제어입력변수)들의 값을 업데이트 (static으로 작성)
  // }
  
  void timer_callback() {
    using clock = std::chrono::steady_clock;

    // --- read_state() ---
    auto t0 = clock::now();
    x0_ = MPCCSim::read_state();
    auto t1 = clock::now();

    // --- runMPC() ---
    auto t2_start = clock::now();
    auto mpc_sol = mpc_->runMPC(x0_);
    auto t2_end = clock::now();

    // --- update_cmd() ---
    auto t3_start = clock::now();
    MPCCSim::update_cmd(mpc_sol.u0, x0_);
    auto t3_end = clock::now();

    // 각 구간 실행시간 (밀리초)
    const double read_ms   = std::chrono::duration<double, std::milli>(t1 - t0).count();
    const double mpc_ms    = std::chrono::duration<double, std::milli>(t2_end - t2_start).count();
    const double update_ms = std::chrono::duration<double, std::milli>(t3_end - t3_start).count();

    // 로그 출력
    RCLCPP_INFO(this->get_logger(),
      "[MPCCOne] read_state=%.3f ms | runMPC=%.3f ms | update_cmd=%.3f ms | total=%.3f ms",
      read_ms, mpc_ms, update_ms, (read_ms + mpc_ms + update_ms));
  }

  double Ts_, v0_;

  State x0_;
  TrackPos track_xy_;
  PathToJson json_paths_;

  std::shared_ptr<MPC> mpc_;
  std::shared_ptr<Integrator> integrator_;
  std::shared_ptr<Plotting> plotter_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class MPCCTwo : public rclcpp::Node {
public:
  MPCCTwo() : Node("mpcc_two") {
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
      x0_ = {0, 0, 0, v0_, 0, 0, 0, 0, 0, v0_};

      mpc_ = std::make_shared<MPC>(jsonConfig["n_sqp"], jsonConfig["n_reset"], jsonConfig["sqp_mixing"], Ts_, json_paths_);
      integrator_ = std::make_shared<Integrator>(Ts_, json_paths_);
      plotter_ = std::make_shared<Plotting>(Ts_, json_paths_);
      Track track(json_paths_.track_path);
      track_xy_ = track.getTrack();
      mpc_->setTrack(track_xy_.X, track_xy_.Y);

      timer_ = this->create_wall_timer(std::chrono::duration<double>(Ts_),std::bind(&MPCCTwo::timer_callback, this));
  }

private:
  // void timer_callback() {
  //   x0_ = MPCCSim::read_state(); //MPCCSim 클래스의 대표 객체의 공유 멤버변수(상태변수)들을 읽어옴 (static으로 작성)
  //   auto mpc_sol = mpc_->runMPC(x0_);
  //   MPCCSim::update_cmd(mpc_sol.u0, x0_); // MPCCSim 클래스의 대표 객체의 공유 멤버변수(제어입력변수)들의 값을 업데이트 (static으로 작성)
  // }
  
  void timer_callback() {
    using clock = std::chrono::steady_clock;

    // --- read_state() ---
    auto t0 = clock::now();
    x0_ = MPCCSim::read_state();
    auto t1 = clock::now();

    // --- runMPC() ---
    auto t2_start = clock::now();
    auto mpc_sol = mpc_->runMPC(x0_);
    auto t2_end = clock::now();

    // --- update_cmd() ---
    auto t3_start = clock::now();
    MPCCSim::update_cmd(mpc_sol.u0, x0_);
    auto t3_end = clock::now();

    // 각 구간 실행시간 (밀리초)
    const double read_ms   = std::chrono::duration<double, std::milli>(t1 - t0).count();
    const double mpc_ms    = std::chrono::duration<double, std::milli>(t2_end - t2_start).count();
    const double update_ms = std::chrono::duration<double, std::milli>(t3_end - t3_start).count();

    // 로그 출력
    RCLCPP_INFO(this->get_logger(),
      "[MPCCTwo] read_state=%.3f ms | runMPC=%.3f ms | update_cmd=%.3f ms | total=%.3f ms",
      read_ms, mpc_ms, update_ms, (read_ms + mpc_ms + update_ms));
  }

  double Ts_, v0_;

  State x0_;
  TrackPos track_xy_;
  PathToJson json_paths_;

  std::shared_ptr<MPC> mpc_;
  std::shared_ptr<Integrator> integrator_;
  std::shared_ptr<Plotting> plotter_;
  rclcpp::TimerBase::SharedPtr timer_;
};


// static 멤버 정의
State MPCCSim::state_;
double MPCCSim::Ts_;
std::mutex MPCCSim::mutex_;

std::shared_ptr<Integrator> MPCCSim::integrator_ = nullptr;

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr MPCCSim::pose_pub_ = nullptr;


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    size_t num_threads = 6; //orin nano는 코어가  총 6개 (num_threads <= 6)
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), num_threads);
    
    rclcpp::Node::SharedPtr mpcc_sim = std::make_shared<MPCCSim>();
    rclcpp::Node::SharedPtr mpcc_one = std::make_shared<MPCCOne>();
    rclcpp::Node::SharedPtr mpcc_two = std::make_shared<MPCCTwo>();
    

    executor.add_node(mpcc_sim);
    executor.add_node(mpcc_one);
    executor.add_node(mpcc_two);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}