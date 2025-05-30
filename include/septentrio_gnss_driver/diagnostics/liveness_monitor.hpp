// *****************************************************************************
//
// © Copyright 2025, TIER IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//    3. Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
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
//
// *****************************************************************************

#pragma once

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace custom_diagnostic_tasks
{

/**
 * @brief Monitor whether a routine is alive (is running at least once in a given period).
 *
 * This can be used for getting notified when a certain routine gets stuck, e.g. when packets stop
 * arriving or when pointclouds stop being published.
 */
class LivenessMonitor : public diagnostic_updater::DiagnosticTask
{
public:
  /**
   * @brief Create and activate a new liveness monitor.
   *
   * The monitored routine has to call `tick()` on every iteration to prove its liveness. If there
   * is no call for a length of `timeout`, the routine is declared dead.
   *
   * @param name The name of the task
   * @param parent_node The node from which clock type and parameters are read.
   * @param timeout The time after the last call to `tick()` where the routine is declared dead
   */
  LivenessMonitor(
    const std::string & name, const rclcpp::Node * parent_node, const rclcpp::Duration & timeout)
  : DiagnosticTask(name), timeout_(timeout)
  {
    // select clock according to the use_sim_time paramter set to the parent
    bool use_sim_time = false;
    if (parent_node->has_parameter("use_sim_time")) {
      use_sim_time = parent_node->get_parameter("use_sim_time").as_bool();
    }
    if (use_sim_time) {
      clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    } else {
      clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    }

    last_tick_ = clock_->now();
  }

  /**
   * @brief Proves the liveness of a routine. Has to be called frequently.
   *
   * A call to `tick()` resets the internal timer. If the timer is not reset at least once before it
   * expires (within `timeout`), the monitored routine is pronounced dead.
   */
  void tick() { last_tick_ = clock_->now(); }

private:
  void run(diagnostic_updater::DiagnosticStatusWrapper & status) override
  {
    using diagnostic_msgs::msg::DiagnosticStatus;

    rclcpp::Time now = clock_->now();
    rclcpp::Duration lateness = now - last_tick_;
    bool is_live = lateness < timeout_;

    uint8_t severity = DiagnosticStatus::OK;
    std::string message = "Alive";
    std::string value = "true";

    if (!is_live) {
      severity = DiagnosticStatus::ERROR;
      message = "Dead";
      value = "false";
    }

    status.summary(severity, message);
    status.add("Is alive", value);
    status.add("Last tick [s]", std::to_string(last_tick_.seconds()));
    status.add("Lateness [ms]", std::to_string(lateness.seconds() * 1000));
  }

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Duration timeout_;
  rclcpp::Time last_tick_;
};

}  // namespace custom_diagnostic_tasks
