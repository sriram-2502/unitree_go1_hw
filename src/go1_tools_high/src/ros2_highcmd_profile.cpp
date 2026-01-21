#include <chrono>
#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace std::chrono_literals;
using ros2_unitree_legged_msgs::msg::HighCmd;

class Go1HighCmdProfile : public rclcpp::Node
{
public:
  Go1HighCmdProfile() : Node("go1_highcmd_profile")
  {
    pub_ = this->create_publisher<HighCmd>("/high_cmd", 1);

    rate_hz_ = this->declare_parameter<double>("rate_hz", 200.0);
    vx_      = this->declare_parameter<double>("vx", 0.20);       // m/s-ish (Unitree uses normalized-ish, but works)
    vy_      = this->declare_parameter<double>("vy", 0.0);
    wz_      = this->declare_parameter<double>("wz", 0.0);         // rad/s-ish
    gait_    = this->declare_parameter<int>("gait_type", 1);       // 1 or 2
    foot_h_  = this->declare_parameter<double>("foot_raise_height", 0.08);
    body_h_  = this->declare_parameter<double>("body_height", 0.0);

    // segments (seconds)
    t_stand_   = this->declare_parameter<double>("t_stand", 2.0);
    t_walk_    = this->declare_parameter<double>("t_walk", 6.0);
    t_turn_    = this->declare_parameter<double>("t_turn", 4.0);
    t_stop_    = this->declare_parameter<double>("t_stop", 2.0);

    dt_ = 1.0 / rate_hz_;
    t_  = 0.0;

    std::cout
      << "HIGH-level profile publisher\n"
      << "Make sure robot is on ground. Press Enter to start...\n";
    std::cin.ignore();

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&Go1HighCmdProfile::tick, this)
    );
  }

private:
  void fill_defaults(HighCmd & cmd)
  {
    cmd.head[0] = 0xFE;
    cmd.head[1] = 0xEF;
    cmd.level_flag = UNITREE_LEGGED_SDK::HIGHLEVEL;   // 0xEE (238)
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
    // Leave sn/version/band_width/crc as default; udp_high usually handles packing.
  }

  void tick()
  {
    HighCmd cmd;
    fill_defaults(cmd);

    // Time schedule
    const double T0 = t_stand_;
    const double T1 = T0 + t_walk_;
    const double T2 = T1 + t_turn_;
    const double T3 = T2 + t_stop_;

    if (t_ < T0) {
      // Stand posture mode (optional): mode=1, or mode=0 works too
      cmd.mode = 1;
    }
    else if (t_ < T1) {
      // Walk forward (velocity tracking)
      cmd.mode = 2;
      cmd.gait_type = static_cast<uint8_t>(gait_);
      cmd.velocity[0] = static_cast<float>(vx_);
      cmd.velocity[1] = static_cast<float>(vy_);
      cmd.yaw_speed   = static_cast<float>(0.0);
      cmd.foot_raise_height = static_cast<float>(foot_h_);
      cmd.body_height       = static_cast<float>(body_h_);
    }
    else if (t_ < T2) {
      // Turn while moving slowly (good EKF excitation)
      cmd.mode = 2;
      cmd.gait_type = static_cast<uint8_t>(gait_);
      cmd.velocity[0] = static_cast<float>(0.15);
      cmd.velocity[1] = static_cast<float>(0.0);
      cmd.yaw_speed   = static_cast<float>(0.6);
      cmd.foot_raise_height = static_cast<float>(foot_h_);
      cmd.body_height       = static_cast<float>(body_h_);
    }
    else if (t_ < T3) {
      // Stop
      cmd.mode = 0;
    }
    else {
      // End: keep standing
      cmd.mode = 1;
      // Optionally: shutdown after one run
      // rclcpp::shutdown();
    }

    pub_->publish(cmd);
    t_ += dt_;
  }

  rclcpp::Publisher<HighCmd>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double rate_hz_, dt_, t_;
  double vx_, vy_, wz_;
  int gait_;
  double foot_h_, body_h_;
  double t_stand_, t_walk_, t_turn_, t_stop_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go1HighCmdProfile>());
  rclcpp::shutdown();
  return 0;
}
