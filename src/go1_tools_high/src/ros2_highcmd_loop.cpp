#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "geometry_msgs/msg/twist.hpp"  // NEW
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace std::chrono_literals;
using ros2_unitree_legged_msgs::msg::HighCmd;

static inline double clamp(double x, double lo, double hi) {
  return std::min(std::max(x, lo), hi);
}

static inline double smoothstep(double s) {
  s = clamp(s, 0.0, 1.0);
  return s * s * (3.0 - 2.0 * s);
}

struct Segment {
  std::string name;
  double duration_s{1.0};
  double vx{0.0}, vy{0.0}, wz{0.0};
  uint8_t mode{2};      // 2 walk, 1 stand, 0 idle
  uint8_t gait{1};
  double foot_raise{0.08};
  double body_height{0.0};
  double ramp_in_s{0.3};
  double ramp_out_s{0.3};
};

class Go1HighCmdProfileLoop : public rclcpp::Node
{
public:
  Go1HighCmdProfileLoop() : Node("go1_highcmd_profile_loop")
  {
    pub_ = this->create_publisher<HighCmd>("/high_cmd", 1);

    // NEW: publish a standard cmd topic for logging/analysis
    cmdvel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    rate_hz_ = this->declare_parameter<double>("rate_hz", 200.0);
    dt_ = 1.0 / rate_hz_;

    gait_   = this->declare_parameter<int>("gait_type", 1);
    foot_h_ = this->declare_parameter<double>("foot_raise_height", 0.08);
    body_h_ = this->declare_parameter<double>("body_height", 0.0);

    loop_forever_ = this->declare_parameter<bool>("loop_forever", true);
    max_loops_    = this->declare_parameter<int>("max_loops", 0); // if loop_forever=false

    t_stand_ = this->declare_parameter<double>("t_stand", 2.0);
    t_hold_  = this->declare_parameter<double>("t_hold", 2.0);
    t_stop_  = this->declare_parameter<double>("t_stop", 1.5);

    vx_amp_ = this->declare_parameter<double>("vx_amp", 0.25);
    vy_amp_ = this->declare_parameter<double>("vy_amp", 0.20);
    wz_amp_ = this->declare_parameter<double>("wz_amp", 0.60);

    ramp_in_  = this->declare_parameter<double>("ramp_in", 0.30);
    ramp_out_ = this->declare_parameter<double>("ramp_out", 0.30);

    log_dt_ = this->declare_parameter<double>("log_dt", 0.20); // 5 Hz
    log_t_  = 0.0;

    // launch-friendly start control (no stdin by default)
    wait_for_enter_ = this->declare_parameter<bool>("wait_for_enter", false);
    start_delay_s_  = this->declare_parameter<double>("start_delay_s", 2.0);

    build_profile();

    std::cout << "Looping HIGH cmd profile (C++)\n"
              << "Segments: " << segments_.size() << "\n";

    if (wait_for_enter_) {
      std::cout << "Press Enter to start...\n";
      std::cin.ignore();
      started_ = true;
    } else {
      std::cout << "Auto-start enabled. Start delay = " << start_delay_s_ << " s\n";
      started_ = false;
      start_time_ = this->now();
    }

    seg_idx_ = 0;
    seg_t_   = 0.0;
    loops_done_ = 0;

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&Go1HighCmdProfileLoop::tick, this));
  }

private:
  void fill_defaults(HighCmd & cmd)
  {
    cmd.head[0] = 0xFE;
    cmd.head[1] = 0xEF;
    cmd.level_flag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    cmd.frame_reserve = 0;

    cmd.mode = 0;
    cmd.gait_type = 0;
    cmd.speed_level = 0;

    cmd.foot_raise_height = 0.0f;
    cmd.body_height = 0.0f;

    cmd.position[0] = 0.0f;
    cmd.position[1] = 0.0f;

    cmd.euler[0] = 0.0f;
    cmd.euler[1] = 0.0f;
    cmd.euler[2] = 0.0f;

    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yaw_speed   = 0.0f;

    cmd.reserve = 0;
  }

  void publish_cmd_vel(double vx, double vy, double wz)
  {
    geometry_msgs::msg::Twist tw;
    tw.linear.x  = vx;
    tw.linear.y  = vy;
    tw.linear.z  = 0.0;
    tw.angular.x = 0.0;
    tw.angular.y = 0.0;
    tw.angular.z = wz;
    cmdvel_pub_->publish(tw);
  }

  double segment_gain(const Segment& s, double seg_t) const
  {
    const double T = s.duration_s;
    if (T <= 1e-6) return 0.0;

    double g_in = 1.0;
    if (s.ramp_in_s > 1e-6) g_in = smoothstep(seg_t / s.ramp_in_s);

    double g_out = 1.0;
    if (s.ramp_out_s > 1e-6) {
      const double t_to_end = T - seg_t;
      g_out = smoothstep(t_to_end / s.ramp_out_s);
    }
    return std::min(g_in, g_out);
  }

  Segment vel_seg(const std::string& name, double vx, double vy, double wz) const
  {
    Segment s;
    s.name = name;
    s.duration_s = t_hold_;
    s.vx = vx; s.vy = vy; s.wz = wz;
    s.mode = 2;
    s.gait = static_cast<uint8_t>(gait_);
    s.foot_raise = foot_h_;
    s.body_height = body_h_;
    s.ramp_in_s = ramp_in_;
    s.ramp_out_s = ramp_out_;
    return s;
  }

  void build_profile()
  {
    segments_.clear();

    Segment stand;
    stand.name = "stand";
    stand.duration_s = t_stand_;
    stand.mode = 1;
    stand.gait = static_cast<uint8_t>(gait_);
    stand.foot_raise = foot_h_;
    stand.body_height = body_h_;
    stand.ramp_in_s = 0.0;
    stand.ramp_out_s = 0.0;
    segments_.push_back(stand);

    segments_.push_back(vel_seg("vx +", +vx_amp_, 0.0, 0.0));
    segments_.push_back(vel_seg("vx -", -vx_amp_, 0.0, 0.0));
    segments_.push_back(vel_seg("vy +", 0.0, +vy_amp_, 0.0));
    segments_.push_back(vel_seg("vy -", 0.0, -vy_amp_, 0.0));
    segments_.push_back(vel_seg("wz +", 0.0, 0.0, +wz_amp_));
    segments_.push_back(vel_seg("wz -", 0.0, 0.0, -wz_amp_));

    segments_.push_back(vel_seg("coupled +++", +0.6*vx_amp_, +0.6*vy_amp_, +0.6*wz_amp_));
    segments_.push_back(vel_seg("coupled +--", +0.6*vx_amp_, -0.6*vy_amp_, -0.6*wz_amp_));
    segments_.push_back(vel_seg("coupled -+-", -0.6*vx_amp_, +0.6*vy_amp_, -0.6*wz_amp_));
    segments_.push_back(vel_seg("coupled --+", -0.6*vx_amp_, -0.6*vy_amp_, +0.6*wz_amp_));

    Segment stop;
    stop.name = "stop";
    stop.duration_s = t_stop_;
    stop.mode = 0;
    stop.ramp_in_s = 0.0;
    stop.ramp_out_s = 0.0;
    segments_.push_back(stop);
  }

  void advance_segment()
  {
    seg_idx_++;
    seg_t_ = 0.0;

    if (seg_idx_ >= static_cast<int>(segments_.size())) {
      seg_idx_ = 0;
      loops_done_++;

      if (!loop_forever_) {
        const int target = (max_loops_ > 0) ? max_loops_ : 1;
        if (loops_done_ >= target) {
          rclcpp::shutdown();
          return;
        }
      }
    }
  }

  void tick()
  {
    // Launch-friendly delay gate (no stdin)
    if (!wait_for_enter_ && !started_) {
      const double elapsed = (this->now() - start_time_).seconds();
      if (elapsed < start_delay_s_) {
        HighCmd cmd;
        fill_defaults(cmd);
        cmd.mode = 1;  // stand while waiting
        cmd.gait_type = static_cast<uint8_t>(gait_);
        cmd.foot_raise_height = static_cast<float>(foot_h_);
        cmd.body_height       = static_cast<float>(body_h_);
        pub_->publish(cmd);

        // publish /cmd_vel = 0 during waiting
        publish_cmd_vel(0.0, 0.0, 0.0);
        return;
      }
      started_ = true;
      RCLCPP_INFO(this->get_logger(), "HighCmd loop started after %.2f s delay.", start_delay_s_);
    }

    if (segments_.empty()) return;

    const Segment& s = segments_[seg_idx_];
    const double g = segment_gain(s, seg_t_);

    double vx_cmd = 0.0, vy_cmd = 0.0, wz_cmd = 0.0;

    HighCmd cmd;
    fill_defaults(cmd);

    cmd.mode = s.mode;
    cmd.gait_type = s.gait;
    cmd.foot_raise_height = static_cast<float>(s.foot_raise);
    cmd.body_height       = static_cast<float>(s.body_height);

    if (s.mode == 2) {
      vx_cmd = g * s.vx;
      vy_cmd = g * s.vy;
      wz_cmd = g * s.wz;

      cmd.velocity[0] = static_cast<float>(vx_cmd);
      cmd.velocity[1] = static_cast<float>(vy_cmd);
      cmd.yaw_speed   = static_cast<float>(wz_cmd);
    }

    pub_->publish(cmd);
    publish_cmd_vel(vx_cmd, vy_cmd, wz_cmd);

    // Throttled log
    log_t_ += dt_;
    if (log_t_ >= log_dt_) {
      log_t_ = 0.0;
      RCLCPP_INFO(this->get_logger(),
        "[Loop %d | Seg %d/%zu | %s | t=%.2f/%.2f] CMD: vx=%+.3f vy=%+.3f wz=%+.3f (gain=%.2f, mode=%d, gait=%d)",
        loops_done_,
        seg_idx_ + 1, segments_.size(),
        s.name.c_str(),
        seg_t_, s.duration_s,
        vx_cmd, vy_cmd, wz_cmd,
        g, s.mode, s.gait
      );
    }

    seg_t_ += dt_;
    if (seg_t_ >= s.duration_s) {
      advance_segment();
    }
  }

  rclcpp::Publisher<HighCmd>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdvel_pub_;  // NEW
  rclcpp::TimerBase::SharedPtr timer_;

  double rate_hz_{200.0}, dt_{0.005};

  int gait_{1};
  double foot_h_{0.08}, body_h_{0.0};

  bool loop_forever_{true};
  int max_loops_{0};
  int loops_done_{0};

  double t_stand_{2.0}, t_hold_{2.0}, t_stop_{1.5};
  double vx_amp_{0.25}, vy_amp_{0.20}, wz_amp_{0.60};
  double ramp_in_{0.30}, ramp_out_{0.30};

  double log_dt_{0.2}, log_t_{0.0};

  std::vector<Segment> segments_;
  int seg_idx_{0};
  double seg_t_{0.0};

  // start control
  bool wait_for_enter_{false};
  double start_delay_s_{2.0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  bool started_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go1HighCmdProfileLoop>());
  rclcpp::shutdown();
  return 0;
}
