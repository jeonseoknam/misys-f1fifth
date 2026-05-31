#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
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
#include <Eigen/Dense>
#include <fstream>
#include <chrono>
#include <sstream>



#include "shared_state.hpp"

using json = nlohmann::json;
using namespace mpcc;

class MPCCSim : public rclcpp::Node {
public:
  explicit MPCCSim(const rclcpp::NodeOptions& options)
  : Node("mpcc_sim", options)
  {
    // config
    std::ifstream iConfig("/home/misys/forza_ws/race_stack/MPCC/C++/Params/config_sim.json");
    json cfg; iConfig >> cfg;

    Ts_ = cfg["Ts"];
    v0_ = cfg["v0"];
    state_init_ = {1.96484, -1.00673, 0, v0_, 0, 0, 0, 0, 0, v0_};

    json_paths_ = PathToJson{
      cfg["model_path"], cfg["cost_path"], cfg["bounds_path"],
      cfg["track_path"], cfg["normalization_path"]
    };

    // ---- 전역 공유 상태 초기화 ----
    {
      std::lock_guard<std::mutex> lk(sim_shared::g_mutex);
      sim_shared::g_Ts = Ts_;
      sim_shared::g_integrator = std::make_shared<Integrator>(Ts_, json_paths_);
      sim_shared::g_state = state_init_;
    }

    Track track(json_paths_.track_path);
    track_xy_ = track.getTrack();

    // 시각화 퍼블리셔
    track_center_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_center", 1);
    track_inner_pub_  = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_inner", 1);
    track_outer_pub_  = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_outer", 1);

    // Pose 퍼블리셔 전역 등록
    sim_shared::g_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("sim/pose", rclcpp::QoS{10});

    publish_track_center(track_xy_.X, track_xy_.Y);
    publish_track_inner(track_xy_.X_inner, track_xy_.Y_inner);
    publish_track_outer(track_xy_.X_outer, track_xy_.Y_outer);
  }

private:
  void publish_track_center(const Eigen::VectorXd &X, const Eigen::VectorXd &Y) {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "sim";
    m.header.stamp = now();
    m.ns = "track_center";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = 0.05;
    m.color.a = 1.0; m.color.r = 1.0;
    for (int i=0;i<X.size();++i){ geometry_msgs::msg::Point p; p.x=X[i]; p.y=Y[i]; p.z=0.0; m.points.push_back(p); }
    arr.markers.push_back(m);
    track_center_pub_->publish(arr);
  }

  void publish_track_inner(const Eigen::VectorXd &X, const Eigen::VectorXd &Y) {
    publish_wall("track_inner_wall_surface", X, Y, track_inner_pub_);
  }
  void publish_track_outer(const Eigen::VectorXd &X, const Eigen::VectorXd &Y) {
    publish_wall("track_outer_wall_surface", X, Y, track_outer_pub_);
  }
  void publish_wall(const std::string& ns, const Eigen::VectorXd& X, const Eigen::VectorXd& Y,
                    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& pub)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "sim";
    m.header.stamp = now();
    m.ns = ns;
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = 1.0;
    m.color.a = 0.5;  // 반투명
    m.color.r = m.color.g = m.color.b = 0.0;

    const double h = 0.2;
    for (int i=0;i<X.size()-1;++i){
      geometry_msgs::msg::Point p1, p2, p1t, p2t;
      p1.x=X[i];   p1.y=Y[i];   p1.z=0.0;
      p2.x=X[i+1]; p2.y=Y[i+1]; p2.z=0.0;
      p1t = p1; p1t.z = h;
      p2t = p2; p2t.z = h;
      // tri 1
      m.points.push_back(p1); m.points.push_back(p2); m.points.push_back(p1t);
      // tri 2
      m.points.push_back(p1t); m.points.push_back(p2); m.points.push_back(p2t);
    }
    visualization_msgs::msg::MarkerArray arr; arr.markers.push_back(m);
    pub->publish(arr);
  }

  double Ts_, v0_;
  State   state_init_;
  PathToJson json_paths_;
  TrackPos track_xy_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_center_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_inner_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_outer_pub_;
};



namespace {
template<typename Vec>
std::string vec_to_string(const Vec& v) {
  std::ostringstream oss;
  oss << "[";
  for (int i = 0; i < v.size(); ++i) {
    oss << v(i);
    if (i + 1 < v.size()) oss << ", ";
  }
  oss << "]";
  return oss.str();
}
} // namespace


// ---- MPC 공통 베이스: read/step/publish 타이밍 로깅 ----
class MPCC_Base : public rclcpp::Node {
public:
  MPCC_Base(const std::string& name, const rclcpp::NodeOptions& options)
  : Node(name, options)
  {
    std::ifstream iConfig("/home/misys/forza_ws/race_stack/MPCC/C++/Params/config_sim.json");
    json cfg; iConfig >> cfg;
    json_paths_ = PathToJson{
      cfg["model_path"], cfg["cost_path"], cfg["bounds_path"],
      cfg["track_path"], cfg["normalization_path"]
    };
    Ts_ = cfg["Ts"];
    v0_ = cfg["v0"];
    x0_ = {0, 0, 0, v0_, 0, 0, 0, 0, 0, v0_};

    mpc_ = std::make_shared<MPC>(cfg["n_sqp"], cfg["n_reset"], cfg["sqp_mixing"], Ts_, json_paths_);
    integrator_ = std::make_shared<Integrator>(Ts_, json_paths_);
    plotter_ = std::make_shared<Plotting>(Ts_, json_paths_);

    Track track(json_paths_.track_path);
    track_xy_ = track.getTrack();
    mpc_->setTrack(track_xy_.X, track_xy_.Y);

    timer_ = this->create_wall_timer(std::chrono::duration<double>(Ts_),
      std::bind(&MPCC_Base::timer_callback, this));
  }

protected:
  void timer_callback() {

    x0_ = sim_shared::read_state();
    auto mpc_sol = mpc_->runMPC(x0_);

    // 통합 + Pose 퍼블리시 (전역 공유 integrator/pose_pub 사용, mutex 보호)
    sim_shared::step_and_publish_pose(mpc_sol.u0, x0_);

    const auto u_vec = inputToVector(mpc_sol.u0);
    const auto x_vec = stateToVector(x0_);

    RCLCPP_INFO(this->get_logger(),"[%s] u0=%s | x0=%s",this->get_name(), vec_to_string(u_vec).c_str(),vec_to_string(x_vec).c_str());
  }

  double Ts_{}, v0_{};
  State x0_;
  TrackPos track_xy_;
  PathToJson json_paths_;
  std::shared_ptr<MPC> mpc_;
  std::shared_ptr<Integrator> integrator_;
  std::shared_ptr<Plotting> plotter_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class MPCCOne : public MPCC_Base {
public:
  explicit MPCCOne(const rclcpp::NodeOptions& options)
  : MPCC_Base("mpcc_one", options) {}
};

class MPCCTwo : public MPCC_Base {
public:
  explicit MPCCTwo(const rclcpp::NodeOptions& options)
  : MPCC_Base("mpcc_two", options) {}
};

// --- 컴포넌트 등록 ---
RCLCPP_COMPONENTS_REGISTER_NODE(MPCCSim)
RCLCPP_COMPONENTS_REGISTER_NODE(MPCCOne)
RCLCPP_COMPONENTS_REGISTER_NODE(MPCCTwo)
