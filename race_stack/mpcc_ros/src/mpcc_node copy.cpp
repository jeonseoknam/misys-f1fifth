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
      // std::ifstream iConfig("/home/misys/forza_ws/race_stack/MPCC/C++/Params/config_hall.json");
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

      // 초기화
      // latest_state_ = {0.619071, -0.426088, 0.03487, v0_, 0, 0, 0, 0., 0, v0_};
      latest_state_ = {0.6, 0.4, 0, v0_, 0, 0, 0, 0, 0, v0_};

      next_state_ = {0, 0, 0, v0_, 0, 0, 0, 0., 0, v0_};

      duty_cycle_ = 0.;
      steering_angle_ = 0.;

      // gain과 offset은 vesc.yaml에서 그대로 가져옴
      steering_angle_to_servo_gain_ = -1.0;
      steering_angle_to_servo_offset_ = 0.5;

      // state_update_ = false;


      // localization sub
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/car_state/odom", 10, std::bind(&MPCCNode::odom_callback, this, _1));

      // duty cycle sub
      duty_cycle_sub_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>("/sensors/core", 10, std::bind(&MPCCNode::duty_cycle_callback, this, _1));
      
      // servo sub
      servo_sub_ = create_subscription<std_msgs::msg::Float64>("sensors/servo_position_command", 10, std::bind(&MPCCNode::servo_callback, this, _1));


      // duty cycle pub
      duty_cycle_pub_ = this->create_publisher<std_msgs::msg::Float64>("commands/motor/duty_cycle", 10);

      // servo pub
      servo_pub_ = this->create_publisher<std_msgs::msg::Float64>("commands/servo/position", 10);

      // pred_traj pub
      pred_traj_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mpcc_predicted_traj", 10);
      
      drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

      // track pub (시각화 목적)
      track_center_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_center", 1);
      track_inner_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_inner", 1);
      track_outer_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("track_outer", 1);

      // timer
      timer_ = this->create_wall_timer(std::chrono::duration<double>(Ts_*0.5),std::bind(&MPCCNode::timer_callback, this));

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
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double curr_x = msg->pose.pose.position.x;
    double curr_y = msg->pose.pose.position.y;

    tf2::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double curr_roll, curr_pitch, curr_yaw;
    tf2::Matrix3x3(quat).getRPY(curr_roll, curr_pitch, curr_yaw);
    

    double curr_vx = (msg->twist.twist.linear.x <= v0_) ? v0_ : msg->twist.twist.linear.x;
    double curr_vy = msg->twist.twist.linear.y;
    double omega = msg->twist.twist.angular.z;

    // s, vs가 살짝 어색함 (이런식으로 센서 같은 것에 의한 측정이 아닌 계산을 통해 값을 넣어줘도 될지...)
    latest_state_ = {curr_x, curr_y, curr_yaw, curr_vx, curr_vy, omega, next_state_.s, duty_cycle_, steering_angle_, next_state_.vs};

    // state_update_ = true;
  }

  void duty_cycle_callback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg) {
    duty_cycle_ = msg->state.duty_cycle;

    std::cout << "duty cycle: " << duty_cycle_ << std::endl;
  }

  void servo_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // servo value (0 to 1) =  steering_angle_to_servo_gain * steering angle (radians) + steering_angle_to_servo_offset
    // steering angle = (servo value - steering_angle_to_servo_offset) / steering_angle_to_servo_gain

    double servo = msg->data;
    steering_angle_ = (servo - steering_angle_to_servo_offset_) / steering_angle_to_servo_gain_;

    std::cout << "delta: " << steering_angle_ << std::endl;
  }


  void timer_callback() {
    // if (state_update_) return;

    auto mpc_sol = mpc_->runMPC(latest_state_);

    
    next_state_ = integrator_->simTimeStep(latest_state_, mpc_sol.u0, Ts_);

    pub_control_drive(next_state_);
    pub_pred_traj(mpc_sol.mpc_horizon);

    // 디버깅을 위해 log 기록
    log_.push_back(mpc_sol);
  }

  void pub_control_drive(const State x1)
  {
    // duty cycle pub
    auto duty_cycle_msg = std_msgs::msg::Float64();
    duty_cycle_msg.data = x1.D;
    duty_cycle_pub_->publish(duty_cycle_msg);

    // servo pub
    auto servo_msg = std_msgs::msg::Float64();
    servo_msg.data = steering_angle_to_servo_gain_ * x1.delta + steering_angle_to_servo_offset_;
    servo_pub_->publish(servo_msg);

    steering_angle_ = x1.delta;
  }

  void pub_pred_traj(const std::array<OptVariables,N+1> mpc_horizon)
  {
    auto marker_array = visualization_msgs::msg::MarkerArray();
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "map";
    // marker.header.frame_id = "sim";
    marker.header.stamp = this->now();
    marker.ns = "mpcc_pred_traj";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS; 
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
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

    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "track_center";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
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

    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "track_inner";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
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

    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "track_outer";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
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
    track_outer_pub_->publish(marker_array);
  };

   

  double duty_cycle_, steering_angle_;
  double Ts_, v0_;
  double steering_angle_to_servo_gain_;
  double steering_angle_to_servo_offset_;
  // bool state_update_;

  State latest_state_, next_state_;

  TrackPos track_xy_;
  PathToJson json_paths_;

  std::shared_ptr<MPC> mpc_;
  std::shared_ptr<Integrator> integrator_;
  std::shared_ptr<Plotting> plotter_;

  std::list<MPCReturn> log_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr duty_cycle_sub_;  
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr duty_cycle_pub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_pub_;
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
