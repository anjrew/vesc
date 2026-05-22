// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std::placeholders::_1;
using std_msgs::msg::Float64;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options)
{
  // declare parameters
  declare_parameter("speed_to_erpm_gain", 0.0);
  declare_parameter("speed_to_erpm_offset", 0.0);
  declare_parameter("steering_angle_to_servo_gain", 0.0);
  declare_parameter("steering_angle_to_servo_offset", 0.0);
  
  // get conversion parameters
  speed_to_erpm_gain_ = get_parameter("speed_to_erpm_gain").get_value<double>();
  speed_to_erpm_offset_ = get_parameter("speed_to_erpm_offset").get_value<double>();
  steering_to_servo_gain_ = get_parameter("steering_angle_to_servo_gain").get_value<double>();
  steering_to_servo_offset_ = get_parameter("steering_angle_to_servo_offset").get_value<double>();

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));

  // Register the dynamic-param callback. Only speed_to_erpm_gain /
  // speed_to_erpm_offset refresh at runtime. The steering mapping is read at
  // construction only — on this car, steering is driven by the PCA9685, not
  // the VESC servo channel, so live-tuning it has no physical effect.
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&AckermannToVesc::parameter_callback, this, std::placeholders::_1));
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  // calc vesc electric RPM (speed)
  Float64 erpm_msg;
  erpm_msg.data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

  // calc steering angle (servo)
  Float64 servo_msg;
  servo_msg.data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  // publish
  if (rclcpp::ok()) {
    erpm_pub_->publish(erpm_msg);
    servo_pub_->publish(servo_msg);
  }
}

rcl_interfaces::msg::SetParametersResult AckermannToVesc::parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // Same-executor as ackermannCmdCallback, so the assignment below cannot
  // race with the multiplication in that callback. No mutex needed.
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    const auto & name = param.get_name();
    if (name == "speed_to_erpm_gain" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      const double new_gain = param.as_double();
      if (new_gain <= 0.0) {
        // Zero gain sends 0 ERPM regardless of commanded speed — a silent
        // estop. Negative gain inverts the commanded direction (commanding
        // +1 m/s drives the motor backwards). Reject anything non-positive
        // at the node, not just at the helper script, so direct
        // `ros2 param set` calls are also safe.
        result.successful = false;
        result.reason = "speed_to_erpm_gain must be > 0";
        break;
      }
      speed_to_erpm_gain_ = new_gain;
    } else if (name == "speed_to_erpm_offset" &&
               param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      speed_to_erpm_offset_ = param.as_double();
    } else if (name == "steering_angle_to_servo_gain" ||
               name == "steering_angle_to_servo_offset") {
      // Steering on tron is driven by the PCA9685 — this mapping only exists
      // for /commands/servo/position round-tripping. Live-tuning here has
      // no physical effect on the car. Reject so the user knows to tune the
      // PCA9685 path instead (config/.../control/ackermann_to_pwm.yaml).
      result.successful = false;
      result.reason =
        "Parameter '" + name + "' does not drive steering on this car (PCA9685 owns the servo). "
        "Tune ackermann_to_pwm.yaml instead.";
      break;
    } else {
      result.successful = false;
      result.reason =
        "Parameter '" + name + "' is read at construction only; relaunch the node to change it.";
      break;
    }
  }

  // Log every accepted runtime parameter change so the operator sees the
  // new value land. Gated on result.successful so rejected batches don't
  // mislead the operator (the framework refuses the whole batch on any
  // rejection).
  if (result.successful && !parameters.empty()) {
    std::string summary;
    for (const auto & p : parameters) {
      if (!summary.empty()) summary += ", ";
      summary += p.get_name() + "=" + p.value_to_string();
    }
    RCLCPP_INFO(this->get_logger(), "Parameters updated: %s", summary.c_str());
  }
  return result;
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
