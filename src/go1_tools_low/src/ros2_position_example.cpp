#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <cmath>
#include <iostream>

using namespace UNITREE_LEGGED_SDK;

static inline float clampf(float x, float lo, float hi)
{
    return std::max(lo, std::min(x, hi));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    auto node = rclcpp::Node::make_shared("node_ros2_torque_example");
    rclcpp::WallRate loop_rate(500);

    long motiontime = 0;  // ms-like counter (your position example increments by 2 each tick)

    ros2_unitree_legged_msgs::msg::LowCmd low_cmd_ros;

    bool initiated_flag = false;
    int count = 0;

    auto pub = node->create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 1);

    // --- Header + level flag (COPY EXACTLY FROM YOUR POSITION EXAMPLE) ---
    low_cmd_ros.head[0] = 0xFE;
    low_cmd_ros.head[1] = 0xEF;
    low_cmd_ros.level_flag = LOWLEVEL;
    // LOWLEVEL has int value of 255


    // --- Init all joints to "torque mode" settings (close to SDK torque example) ---
    for (int i = 0; i < 12; i++)
    {
        low_cmd_ros.motor_cmd[i].mode = 0x0A;     // same as your position example
        low_cmd_ros.motor_cmd[i].q = PosStopF;    // disable position loop
        low_cmd_ros.motor_cmd[i].dq = VelStopF;   // disable velocity loop
        low_cmd_ros.motor_cmd[i].kp = 0.0f;
        low_cmd_ros.motor_cmd[i].kd = 0.0f;
        low_cmd_ros.motor_cmd[i].tau = 0.0f;      // start with zero torque
    }

    // ---- Tunables (keep small for first test) ----
    const float tau_amp = 1.0f;      // Nm amplitude (start 0.3~1.0)
    const float tau_freq = 0.5f;     // Hz
    const float tau_ramp_sec = 1.0f; // seconds to ramp up
    const float tau_limit = 3.0f;    // hard clamp for safety

    // Choose ONE joint to torque-test (safe while hanging).
    // You can change these indices.
    const int test_joint = FR_2;     // calf joint front-right (often safe for demo)

    while (rclcpp::ok())
    {
        if (initiated_flag)
        {
            motiontime += 2; // matches your position example timing

            // time in seconds (motiontime is in ms-ish units)
            const float t = static_cast<float>(motiontime) * 1e-3f;

            // smooth ramp 0->1 over tau_ramp_sec
            float ramp = 1.0f;
            if (tau_ramp_sec > 1e-6f)
                ramp = clampf(t / tau_ramp_sec, 0.0f, 1.0f);

            // sinusoid torque on one joint
            float tau = ramp * tau_amp * std::sin(2.0f * static_cast<float>(M_PI) * tau_freq * t);
            tau = clampf(tau, -tau_limit, tau_limit);

            // keep everyone else at zero torque
            for (int i = 0; i < 12; i++)
                low_cmd_ros.motor_cmd[i].tau = 0.0f;

            // apply torque to just one joint
            low_cmd_ros.motor_cmd[test_joint].tau = tau;

            // OPTIONAL: a tiny anti-splay hip torque bias like your position example,
            // comment out initially if you want minimal torque-only test:
            // low_cmd_ros.motor_cmd[FR_0].tau = -0.2f;
            // low_cmd_ros.motor_cmd[FL_0].tau = +0.2f;
            // low_cmd_ros.motor_cmd[RR_0].tau = -0.2f;
            // low_cmd_ros.motor_cmd[RL_0].tau = +0.2f;
        }

        count++;
        if (count > 10)
        {
            count = 10;
            initiated_flag = true;
        }

        pub->publish(low_cmd_ros);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    return 0;
}
