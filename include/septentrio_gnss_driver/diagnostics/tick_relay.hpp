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

#include "septentrio_gnss_driver/abstraction/typedefs.hpp"
#include "septentrio_gnss_driver/communication/settings.hpp"
#include "septentrio_gnss_driver/diagnostics/liveness_monitor.hpp"
#include "septentrio_gnss_driver/diagnostics/rate_bound_status.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace custom_diagnostic_tasks {

    class TickRelay
    {
    public:
        TickRelay(ROSaicNodeBase* node) : node_(node), settings_(node->settings()) {}

        void tick_topic(const std::string& topic)
        {
            if (!settings_->publish_diagnostics)
                return;
            std::call_once(initialized_, [this]() { initialize(); });
            auto it = rate_bound_status_map_.find(topic);
            if (it != rate_bound_status_map_.end())
                it->second->tick();
        }

        void tick_liveness()
        {
            if (!settings_->publish_diagnostics)
                return;
            std::call_once(initialized_, [this]() { initialize(); });
            liveness_monitor_->tick();
        }

    private:
        void initialize()
        {
            auto ms_to_s = [](uint32_t ms) {
                return static_cast<double>(ms) / 1000.0;
            };
            auto ms_to_hz = [](uint32_t ms) {
                return 1000.0 / static_cast<double>(ms);
            };

            liveness_diagnostics_updater_ =
                std::make_unique<diagnostic_updater::Updater>(node_);
            const auto liveness_period =
                ms_to_s(std::max(settings_->polling_period_rest,
                                 settings_->polling_period_pvt) *
                        2.0);
            liveness_diagnostics_updater_->setPeriod(liveness_period);
            liveness_diagnostics_updater_->setHardwareID(settings_->frame_id);

            liveness_monitor_ =
                std::make_unique<custom_diagnostic_tasks::LivenessMonitor>(
                    "Liveness", node_,
                    rclcpp::Duration::from_seconds(liveness_period));
            liveness_diagnostics_updater_->add(*liveness_monitor_);
            liveness_diagnostics_updater_->force_update();

            pvt_diagnostics_updater_ =
                std::make_unique<diagnostic_updater::Updater>(node_);
            pvt_diagnostics_updater_->setPeriod(
                ms_to_s(settings_->polling_period_pvt));
            pvt_diagnostics_updater_->setHardwareID(settings_->frame_id);

            rest_diagnostics_updater_ =
                std::make_unique<diagnostic_updater::Updater>(node_);
            rest_diagnostics_updater_->setPeriod(
                ms_to_s(settings_->polling_period_rest));
            rest_diagnostics_updater_->setHardwareID(settings_->frame_id);

            auto pvt_ok_params = RateBoundStatusParam{
                ms_to_hz(settings_->monitor_pvt_period_ok_max_ms),
                ms_to_hz(settings_->monitor_pvt_period_ok_min_ms)};
            auto pvt_warn_params = RateBoundStatusParam{
                ms_to_hz(settings_->monitor_pvt_period_warn_max_ms),
                ms_to_hz(settings_->monitor_pvt_period_warn_min_ms)};

            auto rest_ok_params = RateBoundStatusParam{
                ms_to_hz(settings_->monitor_rest_period_ok_max_ms),
                ms_to_hz(settings_->monitor_rest_period_ok_min_ms)};
            auto rest_warn_params = RateBoundStatusParam{
                ms_to_hz(settings_->monitor_rest_period_warn_max_ms),
                ms_to_hz(settings_->monitor_rest_period_warn_min_ms)};

            if (settings_->monitor_gpst)
                update_map(rest_diagnostics_updater_, "gpst", rest_ok_params,
                           rest_warn_params);
            if (settings_->monitor_navsatfix)
                update_map(rest_diagnostics_updater_, "navsatfix", rest_ok_params,
                           rest_warn_params);
            if (settings_->monitor_gpsfix)
                update_map(rest_diagnostics_updater_, "gpsfix", rest_ok_params,
                           rest_warn_params);
            if (settings_->monitor_pose)
                update_map(rest_diagnostics_updater_, "pose", rest_ok_params,
                           rest_warn_params);
            if (settings_->monitor_aimplusstatus)
                update_map(rest_diagnostics_updater_, "aimplusstatus",
                           rest_ok_params, rest_warn_params);
            if (settings_->monitor_galauthstatus)
                update_map(rest_diagnostics_updater_, "galauthstatus",
                           rest_ok_params, rest_warn_params);
            if (settings_->monitor_gpgga)
                update_map(rest_diagnostics_updater_, "gpgga", rest_ok_params,
                           rest_warn_params);
            if (settings_->monitor_gprmc)
                update_map(rest_diagnostics_updater_, "gprmc", rest_ok_params,
                           rest_warn_params);
            if (settings_->monitor_gpgsa)
                update_map(rest_diagnostics_updater_, "gpgsa", rest_ok_params,
                           rest_warn_params);
            if (settings_->monitor_gpgsv)
                update_map(rest_diagnostics_updater_, "gpgsv", rest_ok_params,
                           rest_warn_params);
            if (settings_->monitor_measepoch)
                update_map(rest_diagnostics_updater_, "measepoch", rest_ok_params,
                           rest_warn_params);
            if (settings_->monitor_pvtcartesian) // pvt
                update_map(pvt_diagnostics_updater_, "pvtcartesian", pvt_ok_params,
                           pvt_warn_params);
            if (settings_->monitor_pvtgeodetic) // pvt
                update_map(pvt_diagnostics_updater_, "pvtgeodetic", pvt_ok_params,
                           pvt_warn_params);
            if (settings_->monitor_basevectorcart)
                update_map(rest_diagnostics_updater_, "basevectorcart",
                           rest_ok_params, rest_warn_params);
            if (settings_->monitor_basevectorgeod)
                update_map(rest_diagnostics_updater_, "basevectorgeod",
                           rest_ok_params, rest_warn_params);
            if (settings_->monitor_poscovcartesian) // pvt
                update_map(pvt_diagnostics_updater_, "poscovcartesian",
                           pvt_ok_params, pvt_warn_params);
            if (settings_->monitor_poscovgeodetic) // pvt
                update_map(pvt_diagnostics_updater_, "poscovgeodetic", pvt_ok_params,
                           pvt_warn_params);
            if (settings_->monitor_velcovgeodetic)
                update_map(rest_diagnostics_updater_, "velcovgeodetic",
                           rest_ok_params, rest_warn_params);
            if (settings_->monitor_atteuler)
                update_map(rest_diagnostics_updater_, "atteuler", rest_ok_params,
                           rest_warn_params);
            if (settings_->monitor_attcoveuler)
                update_map(rest_diagnostics_updater_, "attcoveuler", rest_ok_params,
                           rest_warn_params);
            if (settings_->monitor_insnavcart)
                update_map(rest_diagnostics_updater_, "insnavcart", rest_ok_params,
                           rest_warn_params);
            if (settings_->monitor_insnavgeod)
                update_map(rest_diagnostics_updater_, "insnavgeod", rest_ok_params,
                           rest_warn_params);
            if (settings_->monitor_imusetup)
                update_map(rest_diagnostics_updater_, "imusetup", rest_ok_params,
                           rest_warn_params);
            if (settings_->monitor_velsensorsetup)
                update_map(rest_diagnostics_updater_, "velsensorsetup",
                           rest_ok_params, rest_warn_params);
            if (settings_->monitor_exteventinsnavgeod)
                update_map(rest_diagnostics_updater_, "exteventinsnavgeod",
                           rest_ok_params, rest_warn_params);
            if (settings_->monitor_exteventinsnavcart)
                update_map(rest_diagnostics_updater_, "exteventinsnavcart",
                           rest_ok_params, rest_warn_params);
            if (settings_->monitor_extsensormeas)
                update_map(rest_diagnostics_updater_, "extsensormeas",
                           rest_ok_params, rest_warn_params);
            if (settings_->monitor_imu)
                update_map(rest_diagnostics_updater_, "imu", rest_ok_params,
                           rest_warn_params);
            if (settings_->monitor_localization)
                update_map(rest_diagnostics_updater_, "localization", rest_ok_params,
                           rest_warn_params);
            if (settings_->monitor_localization_ecef)
                update_map(rest_diagnostics_updater_, "localization_ecef",
                           rest_ok_params, rest_warn_params);
            if (settings_->monitor_twist)
                update_map(rest_diagnostics_updater_, "twist", rest_ok_params,
                           rest_warn_params);

            pvt_diagnostics_updater_->force_update();
            rest_diagnostics_updater_->force_update();
        }

        void
        update_map(std::unique_ptr<diagnostic_updater::Updater>& diagnostics_updater,
                   const std::string& topic, const RateBoundStatusParam& ok_params,
                   const RateBoundStatusParam& warn_params)
        {
            rate_bound_status_map_.emplace(
                topic, std::make_unique<custom_diagnostic_tasks::RateBoundStatus>(
                           node_, ok_params, warn_params, 3, true,
                           topic + " rate bound status"));
            diagnostics_updater->add(*rate_bound_status_map_.at(topic));
        }

        ROSaicNodeBase* node_;
        const Settings* settings_;
        std::once_flag initialized_;
        std::unique_ptr<diagnostic_updater::Updater> pvt_diagnostics_updater_;
        std::unique_ptr<diagnostic_updater::Updater> rest_diagnostics_updater_;
        std::unique_ptr<diagnostic_updater::Updater> liveness_diagnostics_updater_;
        std::unordered_map<std::string,
                           std::unique_ptr<custom_diagnostic_tasks::RateBoundStatus>>
            rate_bound_status_map_;
        std::unique_ptr<custom_diagnostic_tasks::LivenessMonitor> liveness_monitor_;
    };

} // namespace custom_diagnostic_tasks