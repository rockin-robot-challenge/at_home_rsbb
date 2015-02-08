/*
 * Copyright 2014 Instituto de Sistemas e Robotica, Instituto Superior Tecnico
 *
 * This file is part of RoAH RSBB.
 *
 * RoAH RSBB is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RoAH RSBB is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with RoAH RSBB.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __CORE_GUI_H__
#define __CORE_GUI_H__

#include "core_includes.h"

#include "core_shared_state.h"
#include "core_public_channel.h"
#include "core_zone_manager.h"



class CoreGui
  : boost::noncopyable
{
    CoreSharedState& ss_;
    CorePublicChannel& public_channel_;
    CoreZoneManager& zone_manager_;

    Publisher pub_;
    Timer pub_timer_;

    ServiceServer set_score_srv_;
    ServiceServer manual_operation_complete_srv_;
    ServiceServer omf_complete_srv_;
    ServiceServer omf_damaged_srv_;
    ServiceServer omf_button_srv_;
    ServiceServer connect_srv_;
    ServiceServer disconnect_srv_;
    ServiceServer start_srv_;
    ServiceServer stop_srv_;
    ServiceServer previous_srv_;
    ServiceServer next_srv_;

    void
    transmit (const TimerEvent& = TimerEvent())
    {
      Time now = Time::now();

      // ROS_DEBUG ("Transmitting CoreToGui message");

      auto msg = boost::make_shared<roah_rsbb::CoreToGui>();
      msg->clock = now;
      msg->status = ss_.status;
      msg->addr = public_channel_.host();
      msg->port = to_string (public_channel_.port());
      ss_.active_robots.msg (msg->active_robots);
      zone_manager_.msg (now, msg->zones);

      msg->tablet_last_beacon = ss_.last_tablet_time;
      msg->tablet_display_map = ss_.tablet_display_map;
      if (ss_.last_tablet) {
        msg->tablet_call_time = roah_rsbb::proto_to_ros_time (ss_.last_tablet->last_call());
        msg->tablet_position_time = roah_rsbb::proto_to_ros_time (ss_.last_tablet->last_pos());
        msg->tablet_position_x = ss_.last_tablet->x();
        msg->tablet_position_y = ss_.last_tablet->y();
      }
      else {
        msg->tablet_call_time = TIME_MIN;
        msg->tablet_position_time = TIME_MIN;
        msg->tablet_position_x = 0;
        msg->tablet_position_y = 0;
      }

      pub_.publish (msg);
    }

    bool
    set_score_callback (roah_rsbb::ZoneScore::Request& req,
                        roah_rsbb::ZoneScore::Response& res)
    {
      Zone::Ptr zone = zone_manager_.get (req.zone);
      if (! zone) {
        ROS_WARN_STREAM ("set_score_callback: Could not find zone: " << req.zone);
        return false;
      }
      zone->set_score (req.score);
      return true;
    }

    bool
    manual_operation_complete_callback (roah_rsbb::Zone::Request& req,
                                        roah_rsbb::Zone::Response& res)
    {
      Zone::Ptr zone = zone_manager_.get (req.zone);
      if (! zone) {
        ROS_WARN_STREAM ("manual_operation_complete_callback: Could not find zone: " << req.zone);
        return false;
      }
      zone->manual_operation_complete();
      return true;
    }

    bool
    omf_complete_callback (roah_rsbb::Zone::Request& req,
                           roah_rsbb::Zone::Response& res)
    {
      Zone::Ptr zone = zone_manager_.get (req.zone);
      if (! zone) {
        ROS_WARN_STREAM ("omf_complete_callback: Could not find zone: " << req.zone);
        return false;
      }
      zone->omf_complete();
      return true;
    }

    bool
    omf_damaged_callback (roah_rsbb::ZoneUInt8::Request& req,
                          roah_rsbb::ZoneUInt8::Response& res)
    {
      Zone::Ptr zone = zone_manager_.get (req.zone);
      if (! zone) {
        ROS_WARN_STREAM ("omf_damaged_callback: Could not find zone: " << req.zone);
        return false;
      }
      zone->omf_damaged (req.data);
      return true;
    }

    bool
    omf_button_callback (roah_rsbb::ZoneUInt8::Request& req,
                         roah_rsbb::ZoneUInt8::Response& res)
    {
      Zone::Ptr zone = zone_manager_.get (req.zone);
      if (! zone) {
        ROS_WARN_STREAM ("omf_button_callback: Could not find zone: " << req.zone);
        return false;
      }
      zone->omf_button (req.data);
      return true;
    }

    bool
    connect_callback (roah_rsbb::Zone::Request& req,
                      roah_rsbb::Zone::Response& res)
    {
      Zone::Ptr zone = zone_manager_.get (req.zone);
      if (! zone) {
        ROS_WARN_STREAM ("connect_callback: Could not find zone: " << req.zone);
        return false;
      }
      zone->connect();
      return true;
    }

    bool
    disconnect_callback (roah_rsbb::Zone::Request& req,
                         roah_rsbb::Zone::Response& res)
    {
      Zone::Ptr zone = zone_manager_.get (req.zone);
      if (! zone) {
        ROS_WARN_STREAM ("disconnect_callback: Could not find zone: " << req.zone);
        return false;
      }
      zone->disconnect();
      return true;
    }

    bool
    start_callback (roah_rsbb::Zone::Request& req,
                    roah_rsbb::Zone::Response& res)
    {
      Zone::Ptr zone = zone_manager_.get (req.zone);
      if (! zone) {
        ROS_WARN_STREAM ("start_callback: Could not find zone: " << req.zone);
        return false;
      }
      zone->start();
      return true;
    }

    bool
    stop_callback (roah_rsbb::Zone::Request& req,
                   roah_rsbb::Zone::Response& res)
    {
      Zone::Ptr zone = zone_manager_.get (req.zone);
      if (! zone) {
        ROS_WARN_STREAM ("stop_callback: Could not find zone: " << req.zone);
        return false;
      }
      zone->stop();
      return true;
    }

    bool
    previous_callback (roah_rsbb::Zone::Request& req,
                       roah_rsbb::Zone::Response& res)
    {
      Zone::Ptr zone = zone_manager_.get (req.zone);
      if (! zone) {
        ROS_WARN_STREAM ("previous_callback: Could not find zone: " << req.zone);
        return false;
      }
      zone->previous();
      return true;
    }

    bool
    next_callback (roah_rsbb::Zone::Request& req,
                   roah_rsbb::Zone::Response& res)
    {
      Zone::Ptr zone = zone_manager_.get (req.zone);
      if (! zone) {
        ROS_WARN_STREAM ("next_callback: Could not find zone: " << req.zone);
        return false;
      }
      zone->next();
      return true;
    }

  public:
    CoreGui (CoreSharedState& ss,
             CorePublicChannel& public_channel,
             CoreZoneManager& zone_manager)
      : ss_ (ss)
      , public_channel_ (public_channel)
      , zone_manager_ (zone_manager)
      , pub_ (ss_.nh.advertise<roah_rsbb::CoreToGui> ("/core/to_gui", 1, true))
      , pub_timer_ (ss_.nh.createTimer (Duration (0.1), &CoreGui::transmit, this))
      , set_score_srv_ (ss_.nh.advertiseService ("/core/set_score", &CoreGui::set_score_callback, this))
      , manual_operation_complete_srv_ (ss_.nh.advertiseService ("/core/manual_operation_complete", &CoreGui::manual_operation_complete_callback, this))
      , omf_complete_srv_ (ss_.nh.advertiseService ("/core/omf_switches/complete", &CoreGui::omf_complete_callback, this))
      , omf_damaged_srv_ (ss_.nh.advertiseService ("/core/omf_switches/damaged", &CoreGui::omf_damaged_callback, this))
      , omf_button_srv_ (ss_.nh.advertiseService ("/core/omf_switches/button", &CoreGui::omf_button_callback, this))
      , connect_srv_ (ss_.nh.advertiseService ("/core/connect", &CoreGui::connect_callback, this))
      , disconnect_srv_ (ss_.nh.advertiseService ("/core/disconnect", &CoreGui::disconnect_callback, this))
      , start_srv_ (ss_.nh.advertiseService ("/core/start", &CoreGui::start_callback, this))
      , stop_srv_ (ss_.nh.advertiseService ("/core/stop", &CoreGui::stop_callback, this))
      , previous_srv_ (ss_.nh.advertiseService ("/core/previous", &CoreGui::previous_callback, this))
      , next_srv_ (ss_.nh.advertiseService ("/core/next", &CoreGui::next_callback, this))
    {
      transmit();
    }
};

#endif
