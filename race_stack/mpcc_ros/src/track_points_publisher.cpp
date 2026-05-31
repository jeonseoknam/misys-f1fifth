#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mpcc_ros/msg/track_points.hpp>
#include <Eigen/Dense>
#include <cmath>

#include "Params/track.h"

using mpcc::Track;
using mpcc::TrackPos;
using std::placeholders::_1;

class TrackPointsPublisher : public rclcpp::Node
{
public:
    TrackPointsPublisher()
    : Node("track_points_publisher")
    {
        Track track("/home/misys/forza_ws/race_stack/MPCC/C++/Params/track_sim.json");
        track_xy_ = track.getTrack();

        sub_initial_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&TrackPointsPublisher::initial_pose_callback, this, _1)
        );

        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/sim/pose", 10,
            std::bind(&TrackPointsPublisher::pose_callback, this, _1)
        );

        pub_track_points_ = this->create_publisher<mpcc_ros::msg::TrackPoints>("/track_points", 10);
        pub_track_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/track_points_markers", 10);

    }

private:
    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        publish_track_points(msg->pose.pose.position.x, msg->pose.pose.position.y);
        
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        publish_track_points(msg->pose.position.x, msg->pose.position.y);
    }

    void publish_track_points(double car_x, double car_y)
    {
        int closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        for (int i = 0; i < track_xy_.X.size(); i++) {
            double dx = track_xy_.X[i] - car_x;
            double dy = track_xy_.Y[i] - car_y;
            double dist = dx * dx + dy * dy;
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        mpcc_ros::msg::TrackPoints track_msg;
        track_msg.header.stamp = this->now();
        track_msg.header.frame_id = "sim";

        int num_points = 1000;
        int track_size = track_xy_.X.size();

        for (int i = -100; i < num_points; i++) {
            int idx = (closest_idx + i) % track_size;

            geometry_msgs::msg::Point pc;
            pc.x = track_xy_.X[idx];
            pc.y = track_xy_.Y[idx];
            pc.z = 0.0;
            track_msg.center_points.push_back(pc);

            geometry_msgs::msg::Point pi;
            pi.x = track_xy_.X_inner[idx];
            pi.y = track_xy_.Y_inner[idx];
            pi.z = 0.0;
            track_msg.inner_points.push_back(pi);

            geometry_msgs::msg::Point po;
            po.x = track_xy_.X_outer[idx];
            po.y = track_xy_.Y_outer[idx];
            po.z = 0.0;
            track_msg.outer_points.push_back(po);
        }

        pub_track_points_->publish(track_msg);

        // RViz MarkerArray 변환
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker marker_center;
        marker_center.header = track_msg.header;
        marker_center.ns = "center_line";
        marker_center.id = 0;
        marker_center.type = visualization_msgs::msg::Marker::POINTS;
        marker_center.action = visualization_msgs::msg::Marker::ADD;
        marker_center.scale.x = 0.1;
        marker_center.scale.y = 0.1;
        marker_center.color.r = 0.0;
        marker_center.color.g = 0.0;
        marker_center.color.b = 1.0;
        marker_center.color.a = 1.0;
        marker_center.points = track_msg.center_points;

        visualization_msgs::msg::Marker marker_inner = marker_center;
        marker_inner.ns = "inner_line";
        marker_inner.id = 1;
        marker_inner.color.r = 0.0;
        marker_inner.color.g = 0.0;
        marker_inner.color.b = 1.0;
        marker_inner.points = track_msg.inner_points;

        visualization_msgs::msg::Marker marker_outer = marker_center;
        marker_outer.ns = "outer_line";
        marker_outer.id = 2;
        marker_outer.color.r = 0.0;
        marker_outer.color.g = 0.0;
        marker_outer.color.b = 1.0;
        marker_outer.points = track_msg.outer_points;

        marker_array.markers.push_back(marker_center);
        marker_array.markers.push_back(marker_inner);
        marker_array.markers.push_back(marker_outer);

        pub_track_markers_->publish(marker_array);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Publisher<mpcc_ros::msg::TrackPoints>::SharedPtr pub_track_points_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_track_markers_;
    TrackPos track_xy_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackPointsPublisher>());
    rclcpp::shutdown();
    return 0;
}
