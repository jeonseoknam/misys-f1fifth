#pragma once
#include <mutex>
#include <memory>
#include "Model/integrator.h"
#include "types.h"  // State, Input 타입이 선언된 헤더로 교체
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace sim_shared {

// ---- 전역 공유 자원 선언부 ----
extern mpcc::State g_state;
extern double g_Ts;
extern std::shared_ptr<mpcc::Integrator> g_integrator;
extern std::mutex g_mutex;

// Pose 퍼블리셔(선택): 외부에서 상태 스텝 후 Pose를 내보낼 때 사용
extern rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr g_pose_pub;

// ---- thread-safe helper ----
inline mpcc::State read_state() {
  std::lock_guard<std::mutex> lk(g_mutex);
  return g_state;
}

// u, x를 받아 1-스텝 시뮬레이터를 돌리고 최신 상태를 반환 (thread-safe)
inline mpcc::State step_with_input_and_get(const mpcc::Input& u, const mpcc::State& x) {
  std::lock_guard<std::mutex> lk(g_mutex);
  // 외부에서 받은 레퍼런스들로 필요한 필드를 동기화
  g_state.s   = x.s;
  g_state.phi = x.phi;
  // 통합
  g_state = g_integrator->simTimeStep(g_state, u, g_Ts);
  return g_state;
}

// 필요 시 Pose까지 바로 퍼블리시
inline void step_and_publish_pose(const mpcc::Input& u, const mpcc::State& x) {
  const auto st = step_with_input_and_get(u, x);
  if (g_pose_pub) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "sim";
    msg.pose.position.x = st.X;
    msg.pose.position.y = st.Y;
    msg.pose.position.z = 0.0;
    tf2::Quaternion q; q.setRPY(0.0, 0.0, st.phi);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    g_pose_pub->publish(msg);
  }
}

} // namespace sim_shared
