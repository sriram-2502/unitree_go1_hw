// ros2_torque_example.cpp
//
// Torque "poke" tool to determine sign conventions for Unitree Go1 joints.
//
// Subscribes:  /low_state   (ros2_unitree_legged_msgs/msg/LowState)
// Publishes:   /low_cmd     (ros2_unitree_legged_msgs/msg/LowCmd)
//
// Behavior: repeats sequence
//   WARMUP -> +TAU -> REST -> -TAU -> REST -> repeat
//
// Prints:
//   - joint index + name
//   - commanded torque and sign
//   - q/dq/tau_est on phase enter/exit, plus Δq
//
// Run (example):
/* 
   ros2 run unitree_legged_real ros2_torque_sign_probe --ros-args \
   ...
*/
//
// Safety:
//   - Robot must be HUNG.
//   - Start with tau 0.3~0.8 Nm.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"

// Only use the Unitree SDK for constants (LOWLEVEL, PosStopF, VelStopF).
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace std::chrono_literals;
using UNITREE_LEGGED_SDK::LOWLEVEL;
using UNITREE_LEGGED_SDK::PosStopF;
using UNITREE_LEGGED_SDK::VelStopF;

static inline float clampf(float x, float lo, float hi) {
  return std::max(lo, std::min(x, hi));
}

// Unitree LOW-level motor order (confirmed on your Go1):
// 0..2 FR_0..2, 3..5 FL_0..2, 6..8 RR_0..2, 9..11 RL_0..2
static const char* JOINT_NAMES[12] = {
  "FR_0 (hip)", "FR_1 (thigh)", "FR_2 (calf)",
  "FL_0 (hip)", "FL_1 (thigh)", "FL_2 (calf)",
  "RR_0 (hip)", "RR_1 (thigh)", "RR_2 (calf)",
  "RL_0 (hip)", "RL_1 (thigh)", "RL_2 (calf)"
};

class TorqueSignProbe : public rclcpp::Node {
public:
  TorqueSignProbe() : Node("ros2_torque_sign_probe") {
    // Parameters
    low_cmd_topic_   = this->declare_parameter<std::string>("low_cmd_topic", "low_cmd");
    low_state_topic_ = this->declare_parameter<std::string>("low_state_topic", "/low_state");

    joint_      = this->declare_parameter<int>("joint", 2);            // default FR_2
    tau_mag_    = this->declare_parameter<double>("tau", 0.5);         // Nm magnitude
    tau_limit_  = this->declare_parameter<double>("tau_limit", 3.0);   // clamp
    hold_sec_   = this->declare_parameter<double>("hold_sec", 0.35);
    rest_sec_   = this->declare_parameter<double>("rest_sec", 0.35);
    warmup_sec_ = this->declare_parameter<double>("warmup_sec", 0.25);
    publish_hz_ = this->declare_parameter<double>("publish_hz", 500.0);
    state_timeout_sec_ = this->declare_parameter<double>("state_timeout_sec", 0.10);

    if (joint_ < 0 || joint_ > 11) {
      throw std::runtime_error("Parameter 'joint' must be in [0, 11].");
    }
    tau_mag_ = std::max(0.0, std::min(tau_mag_, tau_limit_));

    // Pub/Sub
    pub_ = this->create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>(low_cmd_topic_, 1);

    sub_ = this->create_subscription<ros2_unitree_legged_msgs::msg::LowState>(
      low_state_topic_, 50,
      std::bind(&TorqueSignProbe::on_low_state, this, std::placeholders::_1));

    // Init command
    cmd_.head[0] = 0xFE;
    cmd_.head[1] = 0xEF;
    cmd_.level_flag = LOWLEVEL;
    cmd_.frame_reserve = 0;

    // Safe init all motors
    for (size_t i = 0; i < cmd_.motor_cmd.size(); i++) {
      auto &m = cmd_.motor_cmd[i];
      m.mode = 0x0A;
      m.q = PosStopF;
      m.dq = VelStopF;
      m.kp = 0.0f;
      m.kd = 0.0f;
      m.tau = 0.0f;
      m.reserve = {0,0,0};
    }

    // Timer
    const double period_s = 1.0 / std::max(1.0, publish_hz_);
    timer_ = this->create_wall_timer(std::chrono::duration<double>(period_s),
                                     std::bind(&TorqueSignProbe::on_timer, this));

    RCLCPP_INFO(this->get_logger(),
                "TorqueSignProbe: pub='%s' sub='%s' joint=%d (%s) tau_mag=%.3fNm (limit=%.1f) hold=%.2fs rest=%.2fs",
                low_cmd_topic_.c_str(), low_state_topic_.c_str(),
                joint_, JOINT_NAMES[joint_], tau_mag_, tau_limit_, hold_sec_, rest_sec_);
  }

private:
  enum class Phase { WARMUP, POS, REST1, NEG, REST2 };

  void on_low_state(const ros2_unitree_legged_msgs::msg::LowState::SharedPtr msg) {
    last_state_ = msg;
    last_state_time_ = this->now();
  }

  bool state_ok() const {
    if (!last_state_) return false;
    const double age = (this->now() - last_state_time_).seconds();
    return age < state_timeout_sec_;
  }

  float q(int j) const {
    if (!last_state_) return 0.0f;
    if ((int)last_state_->motor_state.size() <= j) return 0.0f;
    return (float)last_state_->motor_state[j].q;
  }
  float dq(int j) const {
    if (!last_state_) return 0.0f;
    if ((int)last_state_->motor_state.size() <= j) return 0.0f;
    return (float)last_state_->motor_state[j].dq;
  }
  float tau_est(int j) const {
    if (!last_state_) return 0.0f;
    if ((int)last_state_->motor_state.size() <= j) return 0.0f;
    return (float)last_state_->motor_state[j].tau_est;
  }

  void set_all_tau_zero() {
    for (int i = 0; i < 12; i++) cmd_.motor_cmd[i].tau = 0.0f;
  }

  const char* phase_name(Phase p) const {
    switch (p) {
      case Phase::WARMUP: return "WARMUP";
      case Phase::POS:    return "+TAU";
      case Phase::REST1:  return "REST1";
      case Phase::NEG:    return "-TAU";
      case Phase::REST2:  return "REST2";
      default:            return "UNKNOWN";
    }
  }

  float phase_tau_cmd(Phase p) const {
    if (p == Phase::POS) return (float)(+tau_mag_);
    if (p == Phase::NEG) return (float)(-tau_mag_);
    return 0.0f;
  }

  const char* sign_str(float x) const {
    if (x > 0.f) return "+";
    if (x < 0.f) return "-";
    return "0";
  }

  void phase_enter(Phase new_phase) {
    phase_ = new_phase;
    phase_start_ = this->now();

    q0_ = q(joint_);
    dq0_ = dq(joint_);
    tau_est0_ = tau_est(joint_);

    const float tau_cmd = phase_tau_cmd(phase_);

    RCLCPP_INFO(this->get_logger(),
      "[%s] ENTER joint=%d (%s)  tau_cmd=%.3f Nm (%s)  | q=%.4f dq=%.4f tau_est=%.4f",
      phase_name(phase_), joint_, JOINT_NAMES[joint_],
      tau_cmd, sign_str(tau_cmd),
      q0_, dq0_, tau_est0_);
  }

  void phase_exit(Phase old_phase) {
    const float q1 = q(joint_);
    const float dq1 = dq(joint_);
    const float tau_est1 = tau_est(joint_);
    const float dq_delta = dq1 - dq0_;
    const float q_delta  = q1 - q0_;
    const float tau_cmd  = phase_tau_cmd(old_phase);

    RCLCPP_INFO(this->get_logger(),
      "[%s] EXIT  joint=%d (%s)  tau_cmd=%.3f Nm (%s)  | q_end=%.4f dq_end=%.4f tau_est_end=%.4f | Δq=%.5f Δdq=%.5f",
      phase_name(old_phase), joint_, JOINT_NAMES[joint_],
      tau_cmd, sign_str(tau_cmd),
      q1, dq1, tau_est1,
      q_delta, dq_delta);
  }

  void on_timer() {
    // If no recent /low_state, publish zeros
    if (!state_ok()) {
      set_all_tau_zero();
      pub_->publish(cmd_);
      return;
    }

    if (!started_) {
      started_ = true;
      phase_enter(Phase::WARMUP);
    }

    const double t_phase = (this->now() - phase_start_).seconds();

    // transitions
    if (phase_ == Phase::WARMUP && t_phase >= warmup_sec_) {
      phase_exit(phase_);
      phase_enter(Phase::POS);
    } else if (phase_ == Phase::POS && t_phase >= hold_sec_) {
      phase_exit(phase_);
      phase_enter(Phase::REST1);
    } else if (phase_ == Phase::REST1 && t_phase >= rest_sec_) {
      phase_exit(phase_);
      phase_enter(Phase::NEG);
    } else if (phase_ == Phase::NEG && t_phase >= hold_sec_) {
      phase_exit(phase_);
      phase_enter(Phase::REST2);
    } else if (phase_ == Phase::REST2 && t_phase >= rest_sec_) {
      phase_exit(phase_);
      phase_enter(Phase::POS);
    }

    // apply command
    set_all_tau_zero();
    float tau_cmd = phase_tau_cmd(phase_);
    tau_cmd = clampf(tau_cmd, (float)-tau_limit_, (float)tau_limit_);
    cmd_.motor_cmd[joint_].tau = tau_cmd;

    pub_->publish(cmd_);
  }

private:
  // params
  std::string low_cmd_topic_;
  std::string low_state_topic_;
  int joint_{2};
  double tau_mag_{0.5};
  double tau_limit_{3.0};
  double hold_sec_{0.35};
  double rest_sec_{0.35};
  double warmup_sec_{0.25};
  double publish_hz_{500.0};
  double state_timeout_sec_{0.10};

  // ros
  rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr pub_;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  ros2_unitree_legged_msgs::msg::LowCmd cmd_;
  ros2_unitree_legged_msgs::msg::LowState::SharedPtr last_state_{nullptr};
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};

  // phase machine
  bool started_{false};
  Phase phase_{Phase::WARMUP};
  rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};

  // per-phase log data
  float q0_{0.f};
  float dq0_{0.f};
  float tau_est0_{0.f};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::cout << "LOW-level torque sign probe.\n"
            << "WARNING: Make sure the robot is HUNG.\n"
            << "Press Enter to continue...\n";
  std::cin.ignore();

  auto node = std::make_shared<TorqueSignProbe>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
