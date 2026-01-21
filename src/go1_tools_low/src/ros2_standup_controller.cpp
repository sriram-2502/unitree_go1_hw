#include <rclcpp/rclcpp.hpp>
#include <ros2_unitree_legged_msgs/msg/low_cmd.hpp>
#include <ros2_unitree_legged_msgs/msg/low_state.hpp>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

using UNITREE_LEGGED_SDK::LOWLEVEL;

static inline float clampf(float x, float lo, float hi) {
  return std::max(lo, std::min(x, hi));
}

static inline const char* joint_name(int i) {
  // Unitree ROS2 low-level motor order (as seen in /low_state print):
  // 0..2 FR, 3..5 FL, 6..8 RR, 9..11 RL
  static const char* names[12] = {
    "FR_0(hip)", "FR_1(thigh)", "FR_2(calf)",
    "FL_0(hip)", "FL_1(thigh)", "FL_2(calf)",
    "RR_0(hip)", "RR_1(thigh)", "RR_2(calf)",
    "RL_0(hip)", "RL_1(thigh)", "RL_2(calf)"
  };
  if (i < 0 || i >= 12) return "UNKNOWN";
  return names[i];
}

class StandUpController : public rclcpp::Node {
public:
  StandUpController()
  : Node("ros2_standup_controller"),
    steady_clock_(RCL_STEADY_TIME)
  {
    // ---------------- Parameters ----------------
    this->declare_parameter<std::string>("state_topic", "/low_state");
    this->declare_parameter<std::string>("cmd_topic",   "low_cmd");
    this->declare_parameter<double>("rate_hz", 500.0);

    // Ramp to pose (seconds)e low-level driver use
    this->declare_parameter<double>("ramp_sec", 2.0);

    // PD gains (start moderate; tune up if it "half-stands")
    // You can safely bump these if needed.
    this->declare_parameter<double>("kp_hip",   30.0);
    this->declare_parameter<double>("kp_thigh", 45.0);
    this->declare_parameter<double>("kp_calf",  45.0);

    this->declare_parameter<double>("kd_hip",   1.0);
    this->declare_parameter<double>("kd_thigh", 1.5);
    this->declare_parameter<double>("kd_calf",  1.5);

    // Feedforward torque (keep 0 for now)
    this->declare_parameter<double>("tau_ff", 0.0);

    // Hard clamp for tau_ff only (PD is inside motor)
    this->declare_parameter<double>("tau_ff_limit", 5.0);

    // Stand target (12 joints). Default set from your measured “standing-ish” snapshot:
    // 0 FR_0, 1 FR_1, 2 FR_2, 3 FL_0, 4 FL_1, 5 FL_2, 6 RR_0, 7 RR_1, 8 RR_2, 9 RL_0, 10 RL_1, 11 RL_2
    this->declare_parameter<std::vector<double>>(
      "q_stand",
      std::vector<double>{
        -0.4305464625,  1.1287462711, -2.7049944401,
         0.3420755267,  1.2028657198, -2.7525908947,
        -0.4984287024,  1.1093686819, -2.6805303097,
         0.3944556713,  1.2287832499, -2.7576370239
      }
    );

    // ---------------- Topics ----------------
    state_topic_ = this->get_parameter("state_topic").as_string();
    cmd_topic_   = this->get_parameter("cmd_topic").as_string();

    pub_ = this->create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>(cmd_topic_, 1);

    sub_ = this->create_subscription<ros2_unitree_legged_msgs::msg::LowState>(
      state_topic_, 1,
      [this](const ros2_unitree_legged_msgs::msg::LowState::SharedPtr msg) {
        this->low_state_cb(msg);
      }
    );

    // ---------------- Command init ----------------
    cmd_.head[0] = 0xFE;
    cmd_.head[1] = 0xEF;
    cmd_.level_flag = LOWLEVEL;

    for (int i = 0; i < 12; i++) {
      cmd_.motor_cmd[i].mode = 0x0A;   // servo mode
      cmd_.motor_cmd[i].q  = 0.0f;     // will fill
      cmd_.motor_cmd[i].dq = 0.0f;     // will fill
      cmd_.motor_cmd[i].kp = 0.0f;     // will fill
      cmd_.motor_cmd[i].kd = 0.0f;     // will fill
      cmd_.motor_cmd[i].tau = 0.0f;    // keep 0 for now (optional small FF later)
    }

    // Timer
    const double rate_hz = this->get_parameter("rate_hz").as_double();
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { this->update(); }
    );

    RCLCPP_INFO(this->get_logger(),
      "StandUpController: sub='%s' pub='%s' rate=%.1f Hz",
      state_topic_.c_str(), cmd_topic_.c_str(), rate_hz
    );
    RCLCPP_INFO(this->get_logger(),
      "Default q_stand loaded from parameters. You can override with --ros-args -p q_stand:=[...]"
    );
  }

private:
  void low_state_cb(const ros2_unitree_legged_msgs::msg::LowState::SharedPtr &msg) {
    for (int i = 0; i < 12; i++) {
      q_[i]       = static_cast<float>(msg->motor_state[i].q);
      dq_[i]      = static_cast<float>(msg->motor_state[i].dq);
      tau_est_[i] = static_cast<float>(msg->motor_state[i].tau_est);
    }

    if (!have_state_) {
      have_state_ = true;
      q0_ = q_;
      steady_start_ = steady_clock_.now();

      RCLCPP_INFO(this->get_logger(), "First /low_state received. Starting ramp to q_stand.");
      for (int i = 0; i < 12; i++) {
        RCLCPP_INFO(this->get_logger(), "q0[%02d] %-10s = %+0.4f", i, joint_name(i), q0_[i]);
      }
    }
  }

  void update() {
    if (!have_state_) return;

    // Read params
    const double ramp_sec = this->get_parameter("ramp_sec").as_double();

    const double kp_hip   = this->get_parameter("kp_hip").as_double();
    const double kp_thigh = this->get_parameter("kp_thigh").as_double();
    const double kp_calf  = this->get_parameter("kp_calf").as_double();

    const double kd_hip   = this->get_parameter("kd_hip").as_double();
    const double kd_thigh = this->get_parameter("kd_thigh").as_double();
    const double kd_calf  = this->get_parameter("kd_calf").as_double();

    const double tau_ff = this->get_parameter("tau_ff").as_double();
    const double tau_ff_limit = this->get_parameter("tau_ff_limit").as_double();

    // Get q_stand param
    const auto qv = this->get_parameter("q_stand").as_double_array();
    if (qv.size() != 12) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Parameter q_stand must have 12 elements. Currently has %zu. Using q0 (hold).",
        qv.size()
      );
      return;
    }

    std::array<float, 12> q_stand{};
    for (int i = 0; i < 12; i++) q_stand[i] = static_cast<float>(qv[i]);

    // Ramp alpha in [0,1] using steady time
    const double t = (steady_clock_.now() - steady_start_).seconds();
    const float alpha = (ramp_sec > 1e-6) ? clampf(static_cast<float>(t / ramp_sec), 0.0f, 1.0f) : 1.0f;

    // Fill LowCmd with position+PD
    for (int i = 0; i < 12; i++) {
      const int j = i % 3; // 0 hip, 1 thigh, 2 calf

      const float kp = static_cast<float>((j == 0) ? kp_hip : (j == 1) ? kp_thigh : kp_calf);
      const float kd = static_cast<float>((j == 0) ? kd_hip : (j == 1) ? kd_thigh : kd_calf);

      const float qdes = (1.0f - alpha) * q0_[i] + alpha * q_stand[i];
      const float dqdes = 0.0f;

      // IMPORTANT:
      // - We rely on the motor's internal PD: (q, dq, kp, kd)
      // - tau is just *feedforward* (keep 0 until you need a small assist)
      const float tau_ff_clamped = clampf(static_cast<float>(tau_ff),
                                          static_cast<float>(-tau_ff_limit),
                                          static_cast<float>(tau_ff_limit));

      cmd_.motor_cmd[i].mode = 0x0A;
      cmd_.motor_cmd[i].q    = qdes;
      cmd_.motor_cmd[i].dq   = dqdes;
      cmd_.motor_cmd[i].kp   = kp;
      cmd_.motor_cmd[i].kd   = kd;
      cmd_.motor_cmd[i].tau  = tau_ff_clamped;
    }

    pub_->publish(cmd_);

    // Debug print ~2 Hz (at 500 Hz loop: every 250 ticks is 2 Hz)
    if (++dbg_count_ % 250 == 0) {
      const int a = 0, b = 1, c = 2; // FR
      const float e0 = cmd_.motor_cmd[a].q - q_[a];
      const float e1 = cmd_.motor_cmd[b].q - q_[b];
      const float e2 = cmd_.motor_cmd[c].q - q_[c];

      RCLCPP_INFO(this->get_logger(),
        "alpha=%.2f  FR q=[%.3f %.3f %.3f] qdes=[%.3f %.3f %.3f] err=[%.3f %.3f %.3f] tauEst=[%.3f %.3f %.3f]",
        alpha,
        q_[a], q_[b], q_[c],
        cmd_.motor_cmd[a].q, cmd_.motor_cmd[b].q, cmd_.motor_cmd[c].q,
        e0, e1, e2,
        tau_est_[a], tau_est_[b], tau_est_[c]
      );
    }
  }

private:
  std::string state_topic_;
  std::string cmd_topic_;

  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr sub_;
  rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  ros2_unitree_legged_msgs::msg::LowCmd cmd_{};

  bool have_state_{false};

  rclcpp::Clock steady_clock_;
  rclcpp::Time steady_start_{0,0,RCL_STEADY_TIME};

  std::array<float, 12> q_{};
  std::array<float, 12> dq_{};
  std::array<float, 12> tau_est_{};
  std::array<float, 12> q0_{};

  int dbg_count_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StandUpController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
