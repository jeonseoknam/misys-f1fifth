#include "shared_state.hpp"

namespace sim_shared {
mpcc::State g_state{};
double g_Ts = 0.0;
std::shared_ptr<mpcc::Integrator> g_integrator = nullptr;
std::mutex g_mutex;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr g_pose_pub = nullptr;
}
