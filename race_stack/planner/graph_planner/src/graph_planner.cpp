#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "nav_msgs/msg/odometry.hpp"

#include "f110_msgs/msg/obstacle.hpp"
#include "f110_msgs/msg/obstacle_array.hpp"
#include "f110_msgs/msg/ot_wpnt_array.hpp"
#include "f110_msgs/msg/wpnt.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "f110_msgs/msg/ot_wpnt_array.hpp"
#include "f110_msgs/msg/ltpl_wpnt.hpp"
#include "f110_msgs/msg/ltpl_wpnt_array.hpp"

#include "graph_planner.hpp"
#include "NodeGraph.hpp"
#include "offline_params.hpp"
#include "velocity_profile.hpp"
#include "frenet_conversion_cpp/frenet_converter_cpp.hpp"
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
#include <iostream>

double get_wall_time() {
    struct timeval time;
    if (gettimeofday(&time, NULL)) {
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}


double get_cpu_time() {
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    double user = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec * 1e-6;
    double sys  = usage.ru_stime.tv_sec + usage.ru_stime.tv_usec * 1e-6;
    return user + sys;
}

// using std:::placeholders:_1;

class ObstacleSpliner : public rclcpp::Node {
public:
    ObstacleSpliner(): Node("graph_planner"), planning_done(false) {
        // offline parameter
        params = load_offline_params(this);
        // online parameter
        this->declare_parameter<double>("obs_delay_d", 0.0);
        this->declare_parameter<double>("obs_delay_s", 0.0);
        this->declare_parameter<int>("inflate_idx", 6);  
        this->declare_parameter<int>("min_plan_horizon", 11);
        this->declare_parameter<double>("obs_traj_tresh", 2.5);
        this->declare_parameter<double>("closest_obs", 2.0);
        this->declare_parameter<double>("obs_lookahead", 5.0);
        this->declare_parameter<double>("safety_margin", 0.3);
        this->declare_parameter<double>("hyst_time", 0.0);
        this->declare_parameter<double>("overtake_speed_gain", 1.2);

        obs_delay_s_    = this->get_parameter("obs_delay_s").as_double();
        obs_delay_d_        = this->get_parameter("obs_delay_d").as_double();
        min_plan_horizon_   = this->get_parameter("min_plan_horizon").as_int();
        inflate_idx_        = this->get_parameter("inflate_idx").as_int();
        obs_traj_tresh_     = this->get_parameter("obs_traj_tresh").as_double();
        closest_obs_        = this->get_parameter("closest_obs").as_double();
        obs_lookahead_      = this->get_parameter("obs_lookahead").as_double();
        safety_margin_      = this->get_parameter("safety_margin").as_double();
        hyst_time_          = this->get_parameter("hyst_time").as_double();
        overtake_speed_gain_ = this->get_parameter("overtake_speed_gain").as_double();
        
        param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ObstacleSpliner::paramCB, this, std::placeholders::_1));

        this->declare_parameter<bool>("from_bag", false);
        this->declare_parameter<bool>("measure", false);
        from_bag = this->get_parameter("from_bag").as_bool();
        measuring = this->get_parameter("measure").as_bool();
        
        rclcpp::QoS qos(10);
        qos.transient_local().reliable();

        // Subscriber - Offline
        ltpl_waypoints_sub = this->create_subscription<f110_msgs::msg::LtplWpntArray>(
            "/ltpl_waypoints", 10, std::bind(&ObstacleSpliner::ltpl_cb, this, std::placeholders::_1));

        // Subscriber - Online
        obs_sub = this->create_subscription<f110_msgs::msg::ObstacleArray>(
            "/perception/obstacles", 10, std::bind(&ObstacleSpliner::obs_cb, this, std::placeholders::_1));
        state_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/car_state/frenet/odom", 10, std::bind(&ObstacleSpliner::state_cb, this, std::placeholders::_1));

        // publisher 
        mrks_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planner/avoidance/markers", qos);
        evasion_pub = this->create_publisher<f110_msgs::msg::OTWpntArray>("/planner/avoidance/otwpnts", qos);
        pub_propagated = this->create_publisher<visualization_msgs::msg::Marker>("/planner/avoidance/propagated_obs", qos);

        if (measuring) {
            latency_pub = this->create_publisher<std_msgs::msg::Float32>("/graph_planner/avoidance/latency", qos);
        }

        // Wait for initial messages (blocking-like behavior but safe)
        RCLCPP_INFO(this->get_logger(), "Waiting for initial messages...");
        wait_for_initial_messages();

        // Offline Part run -> once
        if (!planning_done) runOffline();

        // create timer at 20 Hz
        timer_ = this->create_wall_timer(50ms, std::bind(&ObstacleSpliner::online_loop, this));
    }
    
private:
    // Subscriber
    rclcpp::Subscription<f110_msgs::msg::LtplWpntArray>::SharedPtr ltpl_waypoints_sub;
    rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obs_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mrks_pub;
    rclcpp::Publisher<f110_msgs::msg::OTWpntArray>::SharedPtr evasion_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_propagated;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr latency_pub;

    // OfflineParams params;
    OfflineParams params;  
    bool measuring{false};
    bool from_bag{false};
    bool planning_done;

    // last_switch_time/side
    rclcpp::Time last_switch_time{0,0,RCL_ROS_TIME};    
    string last_side;   

    // state variables
    double cur_s{0.0}, cur_d{0.0}, cur_vs{0.0};
    double wpnt_max_s{0.0};
    int inflate_idx_{2};
    int min_plan_horizon_;
    double obs_traj_tresh_;
    double closest_obs_;
    double obs_lookahead_;
    double obs_delay_d_;
    double safety_margin_;
    double obs_delay_s_;
    double hyst_time_;
    double overtake_speed_gain_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    DMap gtMap;
    DMap stMap;
    NodeMap nodeMap;
    IVector nodeIndicesOnRaceline;
    f110_msgs::msg::ObstacleArray obs_msg;
    f110_msgs::msg::LtplWpntArray ltpl_wpnts_msg;
    f110_msgs::msg::OTWpntArray last_wpnts;
    rclcpp::TimerBase::SharedPtr timer_;
    bool have_state{false}, have_ltpl{false};
    NodeGraph nodeGraph;
    std::string final_csv_path;
    std::unique_ptr<FrenetConverter> converter;
    double gb_vmax;

    // ---- 동적 파라미터 콜백 ----
    rcl_interfaces::msg::SetParametersResult paramCB(const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &p : params) {
            if (p.get_name() == "inflate_idx") {
                inflate_idx_ = p.as_int();
                RCLCPP_INFO(this->get_logger(), "inflate_idx updated: %d", inflate_idx_);
            } else if (p.get_name() == "obs_traj_tresh") {
                obs_traj_tresh_ = p.as_double();
                RCLCPP_INFO(get_logger(), "obs_traj_tresh updated: %.3f", obs_traj_tresh_);
            } else if (p.get_name() == "closest_obs") {
                closest_obs_ = p.as_double();
                RCLCPP_INFO(get_logger(), "closest_obs updated: %.3f", closest_obs_);
            } else if (p.get_name() == "obs_lookahead") {
                obs_lookahead_ = p.as_double();
                RCLCPP_INFO(get_logger(), "obs_lookahead updated: %.3f", obs_lookahead_);
            } else if (p.get_name() == "obs_delay_d") {
                obs_delay_d_ = p.as_double();
                RCLCPP_INFO(get_logger(), "obs_delay_d updated: %.3f", obs_delay_d_);
            } else if (p.get_name() == "safety_margin") {
                safety_margin_ = p.as_double();
                RCLCPP_INFO(get_logger(), "safety_margin updated: %.3f", safety_margin_);
            } else if (p.get_name() == "obs_delay_s") {
                obs_delay_s_ = p.as_double();
                RCLCPP_INFO(get_logger(), "obs_delay_s updated: %.3f", obs_delay_s_);
            } else if (p.get_name() == "min_plan_horizon") {
                min_plan_horizon_ = p.as_int();
                RCLCPP_INFO(get_logger(), "min_plan_horizon updated: %d", min_plan_horizon_);
            } else if (p.get_name() == "hyst_time") {
                hyst_time_ = p.as_double();
                RCLCPP_INFO(get_logger(), "hyst_time updated: %.3f", hyst_time_);
            } else if (p.get_name() == "overtake_speed_gain") {
                overtake_speed_gain_ = p.as_double();
                RCLCPP_INFO(get_logger(), "overtake_speed_gain updated: %.3f", overtake_speed_gain_);
            }
        }
        return result;
    }

    // Callback
    void ltpl_cb(const f110_msgs::msg::LtplWpntArray::SharedPtr msg) {
        ltpl_wpnts_msg = *msg;
        try {
            gtMap.clear();
            for (const auto &wp : msg->ltplwpnts) {
                gtMap["x_ref_m"].push_back(wp.x_ref_m);
                gtMap["y_ref_m"].push_back(wp.y_ref_m);
                gtMap["width_right_m"].push_back(wp.width_right_m);
                gtMap["width_left_m"].push_back(wp.width_left_m);
                gtMap["x_normvec_m"].push_back(wp.x_normvec_m);
                gtMap["y_normvec_m"].push_back(wp.y_normvec_m);
                gtMap["alpha_m"].push_back(wp.alpha_m);
                gtMap["s_racetraj_m"].push_back(wp.s_racetraj_m);
                gtMap["psi_racetraj_rad"].push_back(wp.psi_racetraj_rad);
                gtMap["kappa_racetraj_radpm"].push_back(wp.kappa_racetraj_radpm);
                gtMap["vx_racetraj_mps"].push_back(wp.vx_racetraj_mps);
                gtMap["ax_racetraj_mps2"].push_back(wp.ax_racetraj_mps2);

                auto vmax_it = std::max_element(gtMap["vx_racetraj_mps"].begin(),
                                gtMap["vx_racetraj_mps"].end());
                double gb_vmax = (vmax_it != gtMap["vx_racetraj_mps"].end()) ? *vmax_it : 0.0;
                have_ltpl = true;
            }
            wpnt_max_s = msg->ltplwpnts.back().s_racetraj_m;

            // RCLCPP_INFO(this->get_logger(), "gtMap updated with %zu waypoints", msg->ltplwpnts.size());
            // RCLCPP_INFO(this->get_logger(), "Offline planning done, shutting down node.");

        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in offline planning %s", e.what());
            rclcpp::shutdown();
        }
        catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Unkown exception in offline planning.");
            rclcpp::shutdown();
        }
    }

    void obs_cb(const f110_msgs::msg::ObstacleArray::SharedPtr msg) {
        obs_msg = *msg;
    }

    void state_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        cur_s = msg->pose.pose.position.x;
        cur_d = msg->pose.pose.position.y;
        cur_vs = msg->twist.twist.linear.x;
        have_state = true;
    }

    void wait_for_initial_messages() {
        rclcpp::Rate r(10);
        while (rclcpp::ok()) {
            if (have_state && have_ltpl) break;
            r.sleep();
            rclcpp::spin_some(this->get_node_base_interface());
        }
        RCLCPP_INFO(this->get_logger(), "All required messages received. Continuing...");
    }

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////// Offline Part ////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    void runOffline() {
        try {
            // map_size(gtMap); 
            loadGlobalTrajectoryMap();
            stMap = createSampledTrajectoryMap(gtMap);
            auto [nodeMap, nodeIndicesOnRaceline] = createNodeMap(stMap);

            nodeGraph.setParams(params);
            nodeGraph.setNumLayers(nodeMap);
            nodeGraph.genEdges(nodeMap, nodeIndicesOnRaceline, this->get_logger());
            nodeGraph.pruneEdges(nodeMap, stMap[RL_VX]);
            nodeGraph.computeSplineCost(nodeIndicesOnRaceline);
 
            // if (!nodeGraph.writeSplineMapToCSV(paramsfinal_csv_path)) {
            //     RCLCPP_ERROR(this->get_logger(), "Failed to write CSV to path: %s", params.csv_output_path);
            //     throw std::runtime_error("CSV write failed");
            // }
            
            nodeGraph.printGraph(this->get_logger());
            final_csv_path = params.csv_output_path + params.map_name + "/SplineMap.csv";
            // nodeGraph.writeSplineMapToCSV(final_csv_path, this->get_logger());

            RCLCPP_INFO(this->get_logger(), "Offline planning completed successfully!");    
            planning_done = true;

            // Visualize Offline Result 
            // visualizeTrajectories(gtMap, stMap, nodeMap, nodeGraph.getSplineMap());
        }   
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in runOffline: %s", e.what());
            throw;
        }
    }
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Online Part /////////////////////////////
    ////////////////////////////////////////////////////////////////////////

    auto runOnline(const f110_msgs::msg::ObstacleArray &obstacles) -> pair<f110_msgs::msg::OTWpntArray, visualization_msgs::msg::MarkerArray> {
        f110_msgs::msg::OTWpntArray wpnts;
        visualization_msgs::msg::MarkerArray mrks;

        // 회피 대상 obstacle 필터링
        auto obs = obs_filtering(obstacles);
        
        // 회피 대상 obstacle 없는 경우 Online 마무리
        if (obs.empty()) return {wpnts, mrks};
    
        // obs가 존재하는 영역에 대하여 노드/엣지 차단
        std::vector<std::tuple<int,int,int>> blocked_zones;
        bool avoidance_possible = true;

        f110_msgs::msg::Obstacle target_obstacle;  // 가장 가까운 앞쪽 장애물
        double min_gap = 1e9;

        for (auto &target_obs : obs) {
            double gap = target_obs.s_center - cur_s;

            // ego 앞쪽에 있는 장애물만 고려
            if (gap > 0.0 && gap < min_gap) {
                min_gap = gap;
                target_obstacle = target_obs;   
            }

            auto blocked_ranges = find_obs_jone(target_obs);
            // for (auto &[layer, idx_min, idx_max] : blocked_ranges) {
            //     int min_free_nodes = static_cast<int>(std::ceil(
            //         (params.veh_width + 2.0 * safety_margin_) / params.lat_resolution));
            //     int total_nodes = nodeMap[layer].size();
            //     int blocked = idx_max - idx_min + 1;
                
            //     if (blocked >= total_nodes - min_free_nodes) {
            //         RCLCPP_INFO(this->get_logger(), "Layer %d blocked too much (%d/%d). Avoidance impossible.",
            //                     layer, blocked, total_nodes);
            //         avoidance_possible = false;
            //         break;
            //     }
            // }

            if (!avoidance_possible) break;

            blocked_zones.insert(blocked_zones.end(),
                                blocked_ranges.begin(),
                                blocked_ranges.end());
        }

        if (!avoidance_possible) {
            RCLCPP_INFO(this->get_logger(), "Skipping spline generation, fallback to raceline.");
            return {wpnts, mrks};
        }
        
        // obs가 점유하고 있는 공간 -> Edge COST UP!!
        nodeGraph.apply_node_filter(blocked_zones);
        // nodeGraph.apply_node_filter(blocked_zones, target_obstacle, nodeMap);
        // 회피 경로의 시작 노드/목표 노드 정의
        auto [startIdx, endIdx] = findDestination(blocked_zones);

        // 회피 방향 일관성 유지
        // dynamic parameter: hyst_time_ -> 바뀌고 해당 시간만큼은 회피 방향 일관성 유지
        // TODO! 지능적으로 회피 방향 유지할 수 있게 로직 추가해야 한다.
        // if ((this->now() - last_switch_time).seconds() < hyst_time_) {
        //     nodeGraph.hysteresisBias(last_side, startIdx.first, nodeIndicesOnRaceline, nodeMap, 5);
        // }

        // 최소 비용 경로 탐색
        IPairVector nodeArray = nodeGraph.graph_search(startIdx, endIdx.first, nodeIndicesOnRaceline, this->get_logger());

        // Edge COST 원상복구
        nodeGraph.deactivateFiltering();
    
        // RCLCPP_INFO(this->get_logger(), "ComputeSplines input: %zu nodes", nodeArray.size());
        if (nodeArray.size() < 2) return {wpnts, mrks};
        
        // 경로 탐색 시 endIdx가 raceline 위가 아닌 경우 강제 맞춤(쿠션용)
        // auto [check_layer, check_idx] = nodeArray.back();
        // if (!nodeIndicesOnRaceline[check_layer] == check_idx) {
        //     RCLCPP_INFO(this->get_logger(), "Last Node isn't on Raceline!!!");
        //     int rl_idx = nodeIndicesOnRaceline[check_layer]; // 강제로 가장 가까운 raceline 노드로 수정
        //     nodeArray.back() = {check_layer, rl_idx};
        // }

        // 현재 spline 상태 x 노드 시퀀스이므로 spline 게산을 위한 작업 실행
        MatrixXd path(nodeArray.size(), 2);
        // 경로를 Eigen::Matrix 타입으로 변경 
        // 기존 node 구조체에 들어있는 x, y, psi 재사용
        for (size_t i = 0; i < nodeArray.size(); ++i) {
            auto [layer, idx] = nodeArray[i];
            const ::Node &n = nodeMap[layer][idx];  
            path(i, 0) = n.x;
            path(i, 1) = n.y;
        }
        double psi_s = nodeMap[nodeArray.front().first][nodeArray.front().second].psi;
        double psi_e = nodeMap[nodeArray.back().first][nodeArray.back().second].psi;

        // 최소 비용 경로의 뼈대인 노드 시퀀스를 구간별 spline으로 잇는 작업
        auto evasion_spline = nodeGraph.computeSplines(path, psi_s, psi_e, true);

        if (evasion_spline->coeffs_x.size() == 0 || evasion_spline->coeffs_y.size() == 0) {
            RCLCPP_WARN(this->get_logger(),
                        "Skipping waypoint generation: empty spline coefficients (path too short)");
            return {wpnts, mrks};
        }

        // 토픽으로 내보내기 위해 spline 위 점 sampling
        auto evasion_points = nodeGraph.interpSpline(evasion_spline->coeffs_x,
                                                evasion_spline->coeffs_y);
        
        // 곡률/구간거리(el_lengths) 계산
        int N = (int)evasion_points.size();
        Eigen::VectorXd kappa(N);
        Eigen::VectorXd el_lengths(N-1);
        
        kappa(0) = evasion_points[0].kappa;
        for (int i=1;i<N;++i) {
            kappa(i) = evasion_points[i].kappa;
            double dx = evasion_points[i].x - evasion_points[i-1].x;
            double dy = evasion_points[i].y - evasion_points[i-1].y;
            el_lengths(i-1) = std::hypot(dx, dy);
        }

        // s 누적 벡터 계산
        auto [cur_layer, cur_idx] = nodeArray.front();
        
        std::vector<double> s_vec(N);
        s_vec[0] = stMap[RL_S][cur_layer];   // 현재 위치 raceline s에서 시작
        double track_length = stMap[RL_S].back();
        for (int i = 1; i < N; ++i) {
            s_vec[i] = s_vec[i-1] + el_lengths(i-1);
                if (s_vec[i] >= track_length) {
                    s_vec[i] -= track_length; // wrap-around 처리
                }
        }

        // 속도 프로파일 생성
        auto [end_layer, end_idx] = nodeArray.back();
        double v_start = stMap[RL_VX][cur_layer];
        // double v_start = cur_vs;
        double v_end   = stMap[RL_VX][end_layer]; 

        // VpForwardBackward vp(3.0, 6.0, params.vel_max, params.gg_scale);
        // vp.updateDynParameters(params.vel_max, params.gg_scale);

        // Eigen::VectorXd vx = vp.calcVelProfile(
        //     kappa, el_lengths,
        //     v_start, v_end,
        //     target_obstacle,
        //     overtake_speed_gain_     
        // );
        // Eigen::VectorXd ax = accelFromProfile(vx, el_lengths);
        VpForwardBackward vp(params);


        // 속도 프로파일 계산
        Eigen::VectorXd vx = vp.calcVelProfile(kappa, el_lengths, v_start, v_end);


        // 가속도 계산 (간단한 finite difference)
        Eigen::VectorXd ax(vx.size());
        for (int i = 0; i < vx.size() - 1; ++i) {
            double ds = std::max(1e-6, el_lengths(i));
            ax(i) = (vx(i + 1) * vx(i + 1) - vx(i) * vx(i)) / (2.0 * ds);
        }
        ax(vx.size() - 1) = ax(vx.size() - 2);



        // fill wpnts
        for (int i=0; i<N; ++i) {
            std::vector<double> xs = {evasion_points[i].x};
            std::vector<double> ys = {evasion_points[i].y};
            std::vector<double> s_hint = {s_vec[i]};

            auto result = converter->get_frenet(xs, ys, &s_hint);
            auto& d_vec = result.second;

            double psi = std::atan2(evasion_points[i].y_d, evasion_points[i].x_d);
            auto w = xypsi_to_wpnt(evasion_points[i].x, evasion_points[i].y,
                                s_vec[i], d_vec[0],
                                psi, kappa(i),
                                vx(i), ax(i), i);
            wpnts.wpnts.push_back(w);

            visualization_msgs::msg::Marker m = xyv_to_marker(evasion_points[i].x, evasion_points[i].y, vx(i), i);
            mrks.markers.push_back(m);

            // RCLCPP_INFO(this->get_logger(),
                // "Wpnt[%d]: x=%.3f, y=%.3f, s=%.3f, d=%.3f, psi=%.3f, kappa=%.3f, vx=%.3f",
                // i,
                // evasion_points[i].x, evasion_points[i].y,
                // s_vec[i], d_vec[0],
                // psi, kappa(i), vx(i));
        }

        // RCLCPP_INFO(this->get_logger(),
        //     "[Timing] obs_filter=%.3f ms | node_filter=%.3f ms | graph_search=%.3f ms | computeSplines=%.3f ms | interpSpline=%.3f ms | velProfile=%.3f ms",
        //     (t1-t0)*1000.0, (t2-t1)*1000.0, (t3-t2)*1000.0, (t4-t3)*1000.0, (t5-t4)*1000.0, (t6-t5)*1000.0
        // );
        // RCLCPP_INFO(this->get_logger(), "-----------------------------");

        return std::make_pair(wpnts, mrks);
    }

    pair<DVector, DVector> computeBoundRight(DVector &pos_x, DVector &pos_y,
                                            DVector &norm_x, DVector &norm_y,
                                            DVector &width_r) {
        if (pos_x.empty() || pos_y.empty() || norm_x.empty() || norm_y.empty() || width_r.empty()) {
            throw runtime_error("computeBoundRight() - Empty DVector !!");
        }

        int len = pos_x.size();
        DVector x_bound_r(len), y_bound_r(len);
        
        for (size_t i = 0; i < len; ++i) {
            x_bound_r[i] = pos_x[i] + norm_x[i] * width_r[i];
            y_bound_r[i] = pos_y[i] + norm_y[i] * width_r[i];
        }
        
        return {x_bound_r, y_bound_r};

    }

    pair<DVector, DVector> computeBoundLeft(DVector &pos_x, DVector &pos_y,
                                            DVector &norm_x, DVector &norm_y,
                                            DVector &width_l) {
        if (pos_x.empty() || pos_y.empty() || norm_x.empty() || norm_y.empty() || width_l.empty()) {
            throw runtime_error("computeBoundLeft() - Empty DVector !!");
        }

        int len = pos_x.size();
        DVector x_bound_l(len), y_bound_l(len);
        
        for (size_t i = 0; i < len; ++i) {
            x_bound_l[i] = pos_x[i] - norm_x[i] * width_l[i];
            y_bound_l[i] = pos_y[i] - norm_y[i] * width_l[i];
        }
        
        return {x_bound_l, y_bound_l};

    }

    pair<DVector, DVector> computeRaceline(DVector &pos_x, DVector &pos_y,
                                            DVector &norm_x, DVector &norm_y,
                                            DVector &norm_l) {
        if (pos_x.empty() || pos_y.empty() || norm_x.empty() || norm_y.empty() || norm_l.empty()) {
            throw runtime_error("computeBoundRaceline() - Empty DVector !!");
        }

        int len = pos_x.size();
        DVector x_raceline(len), y_raceline(len);
        
        for (size_t i = 0; i < len; ++i) {
            x_raceline[i] = pos_x[i] + norm_x[i] * norm_l[i];
            y_raceline[i] = pos_y[i] + norm_y[i] * norm_l[i];
        }
        
        return {x_raceline, y_raceline};

    }

    DVector computeDeltaS(DVector &rl_s) {
        if (rl_s.empty()) {
            throw runtime_error("computeDeltaS() - Empty DVector !!");
        }

        int len = rl_s.size();
        DVector rl_ds(len);

        // 마지막 원소는 0
        for (size_t i = 0; i < len - 1; ++i) {
            rl_ds[i] = rl_s[i+1] - rl_s[i];
        }
        
        return rl_ds;

    }

    DVector computeHeading(DVector &x_raceline, DVector &y_raceline) {

        DVector psi;
        size_t N = x_raceline.size();
        psi.resize(N);

        // 닫힌 회로 가정. 예외 처리 필요
        double dx, dy;
        for (size_t i = 0; i < N; ++i) {
            
            if (i != N -1) {
                dx = x_raceline[i+1] - x_raceline[i];
                dy = y_raceline[i+1] - y_raceline[i];
            } else {
                dx = x_raceline[0] - x_raceline[N - 1];
                dy = y_raceline[0] - y_raceline[N - 1];
            } 
        psi[i] = atan2(dy, dx) - M_PI_2;
            
        normalizeAngle(psi[i]);

        }

        return psi;
    }

    void loadGlobalTrajectoryMap() {
        // DMap gtMap = readDMapFromCSV(fname);

        auto [rb_x, rb_y] = computeBoundRight(gtMap[POS_X], gtMap[POS_Y],
                                                gtMap[NORM_X], gtMap[NORM_Y],
                                                gtMap[WIDTH_R]);
        gtMap[RB_X] = rb_x;
        gtMap[RB_Y] = rb_y;

        auto [lb_x, lb_y] = computeBoundLeft(gtMap[POS_X], gtMap[POS_Y],
                                            gtMap[NORM_X], gtMap[NORM_Y],
                                            gtMap[WIDTH_L]);
        gtMap[LB_X] = lb_x;
        gtMap[LB_Y] = lb_y;

        auto [rl_x, rl_y] = computeRaceline(gtMap[POS_X], gtMap[POS_Y],
                                            gtMap[NORM_X], gtMap[NORM_Y],
                                            gtMap[NORM_L]);
        gtMap[RL_X] = rl_x;
        gtMap[RL_Y] = rl_y;

        DVector rl_ds = computeDeltaS(gtMap[RL_S]);
        gtMap[RL_dS] = rl_ds;

        converter = std::make_unique<FrenetConverter>(rl_x, rl_y, gtMap[RL_PSI]);

    }

    IVector sampleLayersFromRaceline(const DVector& kappaVector, const DVector& distVector) {
        // RCLCPP_INFO(this->get_logger(), "Reached sampleLayersFromRaceline!");

        IVector layerIndexesSampled;
        const size_t n = kappaVector.size();
        double cur_dist = 0.0;
        double next_dist = 0.0;
        double next_dist_min = 0.0;

        for (size_t i = 0; i < n; ++i) {
            // 곡선이면 최소 거리 갱신
            if ((cur_dist + distVector[i]) > next_dist_min && fabs(kappaVector[i]) > params.curve_thr) {
                next_dist = cur_dist;
            }

            // 다음 샘플링 지점 도달
            if ((cur_dist + distVector[i]) > next_dist) {
                layerIndexesSampled.push_back(static_cast<int>(i));
                if (fabs(kappaVector[i]) < params.curve_thr) {  // 직선 구간
                    next_dist += params.d_straight;
                } else {  // 곡선 구간
                    next_dist += params.d_curve;
                }

                next_dist_min = cur_dist + params.d_curve;
            }

            cur_dist += distVector[i];
        }

        RCLCPP_INFO(this->get_logger(), "[INFO] Total number of track layers: %zu", layerIndexesSampled.size());

        return layerIndexesSampled;
    }
    
    DMap createSampledTrajectoryMap(DMap gtMap) {
        DMap stMap;
        
        IVector layerIndexesSampled = sampleLayersFromRaceline(gtMap[RL_KAPPA], gtMap[RL_dS]);

        for (const auto& [key, vec] : gtMap) {
            for (int idx : layerIndexesSampled) {
            if (idx >= 0 && idx < vec.size()) {
                stMap[key].push_back(vec[idx]);
                } 
            }
        }

        stMap[RL_dS] = computeDeltaS(stMap[RL_S]);
        stMap[RL_PSI] = computeHeading(stMap[RL_X], stMap[RL_Y]);
        stMap[LB_PSI] = computeHeading(stMap[LB_X], stMap[LB_Y]);
        stMap[RB_PSI] = computeHeading(stMap[RB_X], stMap[RB_Y]);  
        // RCLCPP_INFO(this->get_logger(), "Finished createSampledTrajectoryMap!");
        return stMap;
    }
auto createNodeMap(DMap &stMap) -> pair<NodeMap, IVector> {

    const int N = stMap[NORM_L].size();
    nodeMap.resize(N);
    nodeIndicesOnRaceline.clear();

    for (int i = 0; i < N; ++i) {
        ::Node node_;
        int raceline_index = floor((stMap[WIDTH_L][i] + stMap[NORM_L][i] - params.veh_width / 2) / params.lat_resolution);
        nodeIndicesOnRaceline.push_back(raceline_index);

        Vector2d ref_xy(stMap[POS_X][i], stMap[POS_Y][i]);
        Vector2d norm_vec(stMap[NORM_X][i], stMap[NORM_Y][i]);

        // === 노드 수 계산 ===
        int num_nodes = (stMap[WIDTH_R][i] + stMap[WIDTH_L][i] - params.veh_width) / params.lat_resolution;
        if (num_nodes == raceline_index) num_nodes++;
        nodeMap[i].resize(num_nodes);

        // === 좌우 PSI 보간 방향 보정 ===
        double psi_LB = normalizeAngle(stMap[LB_PSI][i]);
        double psi_RL = normalizeAngle(stMap[RL_PSI][i]);
        double psi_RB = normalizeAngle(stMap[RB_PSI][i]);

        // wrap-around 방지: ψ 차이가 180도 이상이면 보정
        auto psi_diff = [](double a, double b){
            double d = b - a;
            while (d > M_PI) d -= 2 * M_PI;
            while (d < -M_PI) d += 2 * M_PI;
            return d;
        };

        double diff_L_to_R = psi_diff(psi_LB, psi_RL);
        double diff_R_to_B = psi_diff(psi_RL, psi_RB);

        for (int idx = 0; idx < num_nodes; ++idx) {
            double alpha = stMap[NORM_L][i] - raceline_index * params.lat_resolution + idx * params.lat_resolution;
            Vector2d node_pos = ref_xy + alpha * norm_vec;

            node_.x = node_pos.x();
            node_.y = node_pos.y();
            node_.raceline = (idx == raceline_index);

            double psi_interp = 0.0;

            if (idx < raceline_index) {
                // 왼쪽 구간: LB → RL로 보간
                double t = static_cast<double>(idx) / std::max(raceline_index, 1);
                psi_interp = psi_LB + diff_L_to_R * t;
            }
            else if (idx == raceline_index) {
                // 레이싱라인: 그대로 사용
                psi_interp = psi_RL;
            }
            else {
                // 오른쪽 구간: RL → RB로 보간
                int remain = num_nodes - raceline_index - 1;
                double t = static_cast<double>(idx - raceline_index) / std::max(remain, 1);
                psi_interp = psi_RL + diff_R_to_B * t;
            }

            node_.psi = normalizeAngle(psi_interp);
            // node_.kappa = stMap[RL_KAPPA][i];  // 필요시 곡률도 raceline 기준으로
            nodeMap[i][idx] = node_;
        }
    }

    RCLCPP_INFO(this->get_logger(), "createNodeMap: total %zu layers built", nodeMap.size());
    return {nodeMap, nodeIndicesOnRaceline};
}


    IPair getClosestNodes(const Vector2d& pos, int limit=1) {
        IPair closestIdx;
        int num_nodes = 0;
        for (const auto& layer : nodeMap) {
            num_nodes += layer.size();
        }

        MatrixXd node_xy(num_nodes, 2);
        int idx = 0;

        for (size_t i = 0; i < nodeMap.size(); ++i) {
            for (size_t j = 0; j < nodeMap[i].size(); ++j) {
                const ::Node& node = nodeMap[i][j];
                node_xy(idx, 0) = node.x;
                node_xy(idx, 1) = node.y;
                ++idx;
            }
        }   
        // pos(2, 1) -> pos.transpose() -> (1, 2)
        MatrixXd diff = node_xy.rowwise() - pos.transpose();
        VectorXd dist2 = diff.rowwise().squaredNorm();
        vector<tuple<double, int, int>> dist_info;

        int re_idx = 0;
        for (size_t i = 0; i < nodeMap.size(); ++i) {
            for (size_t j = 0; j < nodeMap[i].size(); ++j) {
                dist_info.emplace_back(dist2(re_idx++), i, j);
            }
        }

        // 최소 거리 limit개만 앞으로 정렬
        nth_element(dist_info.begin(), dist_info.begin() + limit, dist_info.end());

        // 결과 저장
        for (int k = 0; k < limit; ++k) {
            auto [dist, i, j] = dist_info[k];
            closestIdx = make_pair(i, j);
            // RCLCPP_INFO(this->get_logger(), "Closest node: layer=%d, idx=%d", i, j);
        }
        return closestIdx;
    }

    bool is_same_path(const f110_msgs::msg::OTWpntArray& a,
                  const f110_msgs::msg::OTWpntArray& b,
                  double tol)
    {
        if (a.wpnts.size() != b.wpnts.size()) return false;
        for (size_t i=0; i<a.wpnts.size(); i++) {
            if (fabs(a.wpnts[i].x_m - b.wpnts[i].x_m) > tol) return false;
            if (fabs(a.wpnts[i].y_m - b.wpnts[i].y_m) > tol) return false;
        }
        return true;
    }

    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Online Loop /////////////////////////////
    ////////////////////////////////////////////////////////////////////////

    void online_loop() {
        auto start_time = std::chrono::high_resolution_clock::now();
        // double wall_start = get_wall_time();
        // double cpu_start  = get_cpu_time();

        f110_msgs::msg::OTWpntArray wpnts;
        visualization_msgs::msg::MarkerArray mrks;

        // 마커 지우개
        visualization_msgs::msg::Marker del;
        del.header.stamp = this->now();
        del.header.frame_id = "map";   // frame도 반드시 지정
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        visualization_msgs::msg::MarkerArray clear;
        clear.markers.push_back(del);
        mrks_pub->publish(clear);
    
        // obs 있는 경우에만 online part 수행 
        if (!obs_msg.obstacles.empty()) {
            std::tie(wpnts, mrks) = runOnline(obs_msg);
            last_switch_time = this->now();
            last_wpnts = wpnts; 
        } 

        wpnts.header.stamp = this->now();
        wpnts.header.frame_id = "map";
        evasion_pub->publish(wpnts);

        if (measuring) {
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end_time - start_time;
            std_msgs::msg::Float32 latency;
            latency.data = static_cast<float>(elapsed.count());
            latency_pub->publish(latency);
        }

        mrks_pub->publish(mrks);

        // CPU 사용량 측정
        // double wall_end = get_wall_time();
        // double cpu_end  = get_cpu_time();

        // double wall_elapsed = wall_end - wall_start;
        // double cpu_elapsed  = cpu_end - cpu_start;

        // double cpu_usage = (cpu_elapsed / wall_elapsed) * 100.0;

        // std::cout << "Wall time: " << wall_elapsed << " sec\n";
        // std::cout << "CPU time : " << cpu_elapsed << " sec\n";
        // std::cout << "CPU usage: " << cpu_usage  << " %\n";
    }

f110_msgs::msg::Obstacle predict_obs_movement(f110_msgs::msg::Obstacle obs) {

    double front_dist = fmod((obs.s_center - cur_s + wpnt_max_s), wpnt_max_s);

    if (front_dist < closest_obs_) {
        double delta_s = 0.0, delta_d = 0.0;
        double ot_distance = fmod((obs.s_center - cur_s + wpnt_max_s), wpnt_max_s);

        int idx = std::min<int>(
            std::max<int>(0, static_cast<int>(cur_s * 10)),
            static_cast<int>(ltpl_wpnts_msg.ltplwpnts.size()) - 1
        );

        // 상대 속도 (내 racetraj 속도 - 상대 속도)
        double rel_speed = cur_vs - obs.vs;

        // 상대가 더 빠른 경우 → 나는 추월당하는 상황
        if (rel_speed <= 0.0) {
            // 이미 멀어지고 있으니 예측 보정 안 함
            return obs;
        }

        // 내가 더 빠른 경우 → 추월하는 상황
        // 예측 시간 계산 (최대 5초까지)
        double ot_time_distance = std::clamp(ot_distance / std::max(rel_speed, 0.1), 0.0, 2.0) * 0.5;

        // 위치 보정
        // dynamic parameter: obs_delay_s
        // dynamic parameter: obs_delay_d
        delta_s = ot_time_distance * obs.vs + obs_delay_s_; // 뒤쪽으로 offset
        delta_d = ot_time_distance * obs.vd + obs_delay_d_; // 좌우로 보정

        // s 업데이트
        obs.s_start = fmod(obs.s_start + delta_s + wpnt_max_s, wpnt_max_s);
        obs.s_center = fmod(obs.s_center + delta_s + wpnt_max_s, wpnt_max_s);
        obs.s_end   = fmod(obs.s_end   + delta_s + wpnt_max_s, wpnt_max_s);

        // d 업데이트
        obs.d_left   += delta_d;
        obs.d_center += delta_d;
        obs.d_right  += delta_d;

        // 디버그 마커 발행
        visualization_msgs::msg::Marker zone;
        zone.header.frame_id = "map";
        zone.header.stamp = this->now();
        zone.ns = "predicted_obs_zone";
        zone.id = obs.id;
        zone.type = visualization_msgs::msg::Marker::LINE_STRIP;
        zone.action = visualization_msgs::msg::Marker::ADD;
        zone.scale.x = 0.05; // 선 두께
        zone.color.a = 1.0;
        zone.color.r = 1.0; zone.color.g = 0.0; zone.color.b = 0.0;

        // s,d 좌표 → x,y 변환
        std::vector<double> s_vec = {obs.s_start, obs.s_start, obs.s_end, obs.s_end};
        std::vector<double> d_vec = {obs.d_left,  obs.d_right, obs.d_right, obs.d_left};
        auto resp = converter->get_cartesian(s_vec, d_vec);

        // 네 모서리 점
        for (size_t i = 0; i < resp.first.size(); i++) {
            geometry_msgs::msg::Point p;
            p.x = resp.first[i];
            p.y = resp.second[i];
            p.z = 0.0;
            zone.points.push_back(p);
        }
        // 닫아주기 (첫 점 다시 push)
        zone.points.push_back(zone.points.front());

        pub_propagated->publish(zone);

    }
    return obs;
}

    std::vector<f110_msgs::msg::Obstacle> obs_filtering(const f110_msgs::msg::ObstacleArray &obstacles) {
        std::vector<f110_msgs::msg::Obstacle> obs_on_traj;
        
        // dynamic parameter: obs_traj_tresh_ -> raceline에 가까운지 판단하는 기준 offset
        // raceline에 붙어있지 않으면 회피할 필요X
        for (const auto & obs : obstacles.obstacles) {
            // if (std::abs(obs.d_center) < obs_traj_tresh_) obs_on_traj.push_back(obs);
            obs_on_traj.push_back(obs);
        }


        // RCLCPP_INFO(this->get_logger(), "Total obstacles: %zu", obstacles.obstacles.size());
        // RCLCPP_INFO(this->get_logger(), "On traj obstacles: %zu", obs_on_traj.size());

        std::vector<f110_msgs::msg::Obstacle> close_obs;
        for (auto obs : obs_on_traj) {
            double dist = 0.0;
            obs = predict_obs_movement(obs);
            // if (cur_s > obs.s_center) {
            //     dist = std::min(cur_s - obs.s_center, wpnt_max_s - cur_s + obs.s_center);
            // }
            // else {
            //     dist = std::min(obs.s_center - cur_s, wpnt_max_s - obs.s_center + cur_s);
            // }

            // RCLCPP_INFO(this->get_logger(), "Candidate obs_s: %.2f, cur_s: %.2f, dist=%.2f",
                        // obs.s_center, cur_s, dist);
            // dynamic parameter: obs_lookahead_ -> 움직임 예측 후 보정된 obs 기준 회피 경로를 생성을 해야될 거리에 있는지(s값)
            // if (dist < obs_lookahead_) close_obs.push_back(obs); //obs_lookahead_: 회피경로를 생성할 최대 예측 후의 장애물과 나와의 거리
            close_obs.push_back(obs);
        }

        // RCLCPP_INFO(this->get_logger(), "Obs_filtering got %zu close obs", close_obs.size());
        // RCLCPP_INFO(this->get_logger(), "------------------------------------------------");
        
        return close_obs;
    }
    
    std::vector<std::tuple<int,int,int>> find_obs_jone(
        const f110_msgs::msg::Obstacle &target_obs) {
        // TODO! getClosestNodes가 (s, d) 기준이면 이 과정이 없어도 된다. 
        std::vector<std::tuple<int,int,int>> blocked;

        auto obs_point_1 = converter->get_cartesian(target_obs.s_start, target_obs.d_left);
        auto obs_point_2 = converter->get_cartesian(target_obs.s_start, target_obs.d_right);
        auto obs_point_3 = converter->get_cartesian(target_obs.s_end,   target_obs.d_left);
        auto obs_point_4 = converter->get_cartesian(target_obs.s_end,   target_obs.d_right);
    
        Eigen::Vector2d obs_vec1(obs_point_1.first, obs_point_1.second);
        Eigen::Vector2d obs_vec2(obs_point_2.first, obs_point_2.second);
        Eigen::Vector2d obs_vec3(obs_point_3.first, obs_point_3.second);
        Eigen::Vector2d obs_vec4(obs_point_4.first, obs_point_4.second);

        IPair obs_front_l = getClosestNodes(obs_vec1);
        IPair obs_front_r = getClosestNodes(obs_vec2);
        IPair obs_back_l  = getClosestNodes(obs_vec3);
        IPair obs_back_r  = getClosestNodes(obs_vec4);

        // front랑 back 노드 인덱스가 같은 경우
        if (obs_front_l == obs_front_r) {
        // 좌우가 같은 노드라면 인덱스를 조금 퍼뜨려줌(수정 필요. +1한 상태에서 해당 노드가 없을 수 있음.)
        obs_front_l.second = std::max(0, obs_front_l.second - 1);
        obs_front_r.second = std::min((int)nodeMap[obs_front_r.first].size()-1,
                                    obs_front_r.second + 1);
        }
        if (obs_back_l == obs_back_r) {
            obs_back_l.second = std::max(0, obs_back_l.second - 1);
            obs_back_r.second = std::min((int)nodeMap[obs_back_r.first].size()-1,
                                        obs_back_r.second + 1);
        }

            int start_layer = std::max(obs_front_l.first, obs_front_r.first);
            int end_layer   = std::min(obs_back_l.first,  obs_back_r.first);
            if (start_layer == end_layer) {
                if (start_layer < (int)nodeMap.size()-1) {
                    end_layer = start_layer + 1; // 뒤쪽으로 한 레이어 확장
                } else if (start_layer > 0) {
                    start_layer = start_layer - 1; // 앞쪽으로 한 레이어 확장
                }
            }

        // front 기준 lateral index 범위
        int idx_min = std::max(0, std::min(obs_front_l.second, obs_front_r.second) - inflate_idx_);
        int idx_max = std::min((int)nodeMap[start_layer].size()-1,
                            std::max(obs_front_l.second, obs_front_r.second) + inflate_idx_);

        // start_layer ~ end_layer 전부 push
        for (int l = start_layer; l <= end_layer; ++l) {
            blocked.push_back(std::make_tuple(l, idx_min, idx_max));
            // RCLCPP_INFO(rclcpp::get_logger("graph_planner"),
            //             "Blocked zone: layer=%d, idx_min=%d, idx_max=%d",
            //             l, idx_min, idx_max);
        }

        // RCLCPP_INFO(rclcpp::get_logger("graph_planner"),
        //             "find_obs_jone -> total blocked ranges: %zu", blocked.size());

        return blocked;
    }

    auto findDestination(const vector<std::tuple<int,int,int>> &blocked_zones) -> pair<IPair, IPair> {
        
        // (s, d) -> (x, y)로 변경
        // TODO! Node 구조체에 s, d 정보 추가 시 함수가 간단해진다.
        auto [x,y] = converter->get_cartesian(cur_s, cur_d);
        Eigen::Vector2d cur_xy(x,y);

        IPair startIdx = getClosestNodes(cur_xy, 1); 

        // 목적지 거리 계산
        // int max_blocked_layer = startIdx.first;
        // for (auto &[layer, idx_min, idx_max] : blocked_zones) {
        //     if (layer > max_blocked_layer) {
        //         max_blocked_layer = layer;
        //     }
        // }

        // int dest_layer = std::min(max_blocked_layer + 1, (int)nodeMap.size() - 1) + min_plan_horizon_;
        int dest_layer = startIdx.first + min_plan_horizon_;
        // 레이어 wrapping
        if (dest_layer >= (int)nodeIndicesOnRaceline.size()) {
            dest_layer = dest_layer % nodeIndicesOnRaceline.size();
        }

        int dest_index = nodeIndicesOnRaceline[dest_layer];
        IPair endIdx = {dest_layer, dest_index};
        // RCLCPP_INFO(this->get_logger(),
        //     "[findDestination] start=(layer=%d, idx=%d), dest=(layer=%d, idx=%d)",
        //     startIdx.first, startIdx.second,
        //     endIdx.first, endIdx.second
        // );
        return {startIdx, endIdx};
    }


    visualization_msgs::msg::Marker xy_to_point(double x, double y, bool opponent=true) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.scale.x = 0.5; m.scale.y = 0.5; m.scale.z = 0.5;
        m.color.a = 0.8;
        m.color.b = 0.65;
        m.color.r = opponent ? 1.0 : 0.0;
        m.color.g = 0.65;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = 0.01;
        m.pose.orientation.w = 1.0;
        return m;
    }

    visualization_msgs::msg::Marker xyv_to_marker(double x, double y, double v, int id) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.scale.x = 0.1; 
        m.scale.y = 0.1; 
        m.scale.z = 0.1;
        m.color.a = 1.0;
        m.color.b = 0.75; m.color.r = 0.75;
        if (from_bag) m.color.g = 0.75;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.ns = "otwpnts";
        m.id = id;
        m.pose.position.x = x; m.pose.position.y = y;
        m.pose.position.z = 0.0;
        m.pose.orientation.w = 1.0;
        return m;
    }

    f110_msgs::msg::Wpnt xypsi_to_wpnt(
        double x, double y, double s, double d,
        double psi, double kappa,
        double v, double ax, int id)
    {
        f110_msgs::msg::Wpnt w;
        w.id = id;
        w.s_m = s;
        w.d_m = d;
        w.x_m = x;
        w.y_m = y;
        w.psi_rad = psi;
        w.kappa_radpm = kappa;
        w.vx_mps = v;
        w.ax_mps2 = ax;
        return w;
    }
     
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleSpliner>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}