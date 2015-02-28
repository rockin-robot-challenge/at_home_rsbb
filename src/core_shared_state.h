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

#ifndef __CORE_SHARED_STATE_H__
#define __CORE_SHARED_STATE_H__

#include "core_includes.h"

#include "core_aux.h"



class ActiveRobots
  : boost::noncopyable
{
    Duration robot_timeout_;

    // sum . map size team_robot_map_ == size last_beacon_map_
    map<string, map<string, roah_rsbb::RobotInfo::ConstPtr>> team_robot_map_;
    map<Time, roah_rsbb::RobotInfo::ConstPtr> last_beacon_map_;

    void
    update ()
    {
      auto now = Time::now();

      while ( (! last_beacon_map_.empty())
              && ( (last_beacon_map_.begin()->first + robot_timeout_) < now)) {
        team_robot_map_[last_beacon_map_.begin()->second->team].erase (last_beacon_map_.begin()->second->robot);
        last_beacon_map_.erase (last_beacon_map_.begin());
      }
    }

  public:
    ActiveRobots()
      : robot_timeout_ (param_direct<double> ("~robot_timeout", 30.0))
    {
    }

    void
    add (roah_rsbb::RobotInfo::ConstPtr const& ri)
    {
      auto last_team = team_robot_map_.find (ri->team);
      if (last_team != team_robot_map_.end()) {
        auto last = last_team->second.find (ri->robot);
        if (last != last_team->second.end()) {
          last_beacon_map_.erase (last->second->beacon);
          last_beacon_map_[ri->beacon] = ri;
          last->second = ri;
          return;
        }
      }
      team_robot_map_[ri->team][ri->robot] = ri;
      last_beacon_map_[ri->beacon] = ri;
    }

    void
    add (string const& team,
         string const& robot,
         Duration const& skew,
         Time const& beacon)
    {
      auto msg = boost::make_shared<roah_rsbb::RobotInfo>();
      msg->team = team;
      msg->robot = robot;
      msg->skew = skew;
      msg->beacon = beacon;
      add (msg);
    }

    void
    msg (vector<roah_rsbb::RobotInfo>& msg)
    {
      update();

      for (auto const& iteam : team_robot_map_) {
        for (auto const& i : iteam.second) {
          msg.push_back (* (i.second));
        }
      }
    }

    vector<roah_rsbb::RobotInfo>
    get ()
    {
      update();

      vector<roah_rsbb::RobotInfo> ret;
      ret.reserve (team_robot_map_.size());

      for (auto const& iteam : team_robot_map_) {
        map<string, roah_rsbb::RobotInfo::ConstPtr> rm = iteam.second;

        if (! rm.empty()) {
          ret.push_back (* (rm.begin()->second));
        }
      }

      return ret;
    }

    roah_rsbb::RobotInfo
    get (string const& team)
    {
      update();

      map<string, roah_rsbb::RobotInfo::ConstPtr> rm = team_robot_map_[team];

      if (rm.empty()) {
        return roah_rsbb::RobotInfo();
      }

      return * (rm.begin()->second);
    }
};



struct ScoringItem {
  typedef enum { SCORING_BOOL, SCORING_UINT } scoring_type_t;

  string group;
  string desc;
  scoring_type_t type;
  int32_t current_value;

  ScoringItem (string const& benchmark,
               string const& group_name,
               YAML::Node const& item_node)
    : group (group_name)
    , current_value (0)
  {
    using namespace YAML;

    if (! item_node["type"]) {
      ROS_FATAL_STREAM ("Benchmark \"" << benchmark << "\" scoring item in \"" << group_name << "\" is missing a \"type\" entry! :\n" << item_node);
      abort_rsbb();
    }
    string type_s = item_node["type"].as<string>();
    if (type_s == "bool") {
      type = SCORING_BOOL;
    }
    else if (type_s == "uint") {
      type = SCORING_UINT;
    }
    else {
      ROS_FATAL_STREAM ("Benchmark \"" << benchmark << "\" scoring item in \"" << group_name << "\" type is unknown:" << type_s);
      abort_rsbb();
    }

    if (! item_node["desc"]) {
      ROS_FATAL_STREAM ("Benchmark \"" << benchmark << "\" scoring item in \"" << group_name << "\" is missing a \"desc\" entry! :\n" << item_node);
      abort_rsbb();
    }
    desc = item_node["desc"].as<string>();
  }
};



struct Benchmark {
  string name;
  string desc;
  string code;
  Duration timeout;
  Duration total_timeout;
  vector<ScoringItem> scoring;
};



class Benchmarks
{
    map<string, Benchmark> by_code_;

  public:
    Benchmarks()
    {
      using namespace YAML;

      Node file = LoadFile (param_direct<string> ("~benchmarks_file", "benchmarks.yaml"));
      if (! file.IsSequence()) {
        ROS_FATAL_STREAM ("Benchmarks file is not a sequence!");
        abort_rsbb();
      }
      for (Node const& benchmark_node : file) {
        if (! benchmark_node.IsMap()) {
          ROS_FATAL_STREAM ("Benchmarks file has a benchmark entry that is not a map!");
          abort_rsbb();
        }
        if (! benchmark_node["name"]) {
          ROS_FATAL_STREAM ("Benchmarks file is missing a \"node\" entry!");
          abort_rsbb();
        }
        if (! benchmark_node["desc"]) {
          ROS_FATAL_STREAM ("Benchmarks file is missing a \"desc\" entry!");
          abort_rsbb();
        }
        if (! benchmark_node["code"]) {
          ROS_FATAL_STREAM ("Benchmarks file is missing a \"code\" entry!");
          abort_rsbb();
        }
        if (! benchmark_node["timeout"]) {
          ROS_FATAL_STREAM ("Benchmarks file is missing a \"timeout\" entry!");
          abort_rsbb();
        }
        Benchmark b;
        b.name = benchmark_node["name"].as<string>();
        b.desc = benchmark_node["desc"].as<string>();
        b.code = benchmark_node["code"].as<string>();
        b.timeout = Duration (benchmark_node["timeout"].as<double>());
        if (benchmark_node["total_timeout"]) {
          b.total_timeout = Duration (benchmark_node["total_timeout"].as<double>());
        }
        else {
          b.total_timeout = b.timeout;
        }

        if (benchmark_node["scoring"]) {
          if (! benchmark_node["scoring"].IsSequence()) {
            ROS_FATAL_STREAM ("Benchmark \"" << b.name << "\" \"scoring\" entry is not a sequence! :\n" << benchmark_node["scoring"]);
            abort_rsbb();
          }
          for (Node const& scoring_node : benchmark_node["scoring"]) {
            if (! scoring_node.IsMap()) {
              ROS_FATAL_STREAM ("Benchmark \"" << b.name << "\" \"scoring\" entry is not a sequence of maps! :\n" << scoring_node);
              abort_rsbb();
            }
            for (YAML::const_iterator it = scoring_node.begin(); it != scoring_node.end(); ++it) {
              string group_name = it->first.as<string>();
              if (! it->second.IsSequence()) {
                ROS_FATAL_STREAM ("Benchmark \"" << b.name << "\" scoring \"" << it->first << "\" is not a sequence! :\n" << it->second);
                abort_rsbb();
              }
              for (Node const& item_node : it->second) {
                b.scoring.push_back (ScoringItem (b.name, group_name, item_node));
              }
            }
          }
        }
        by_code_[b.code] = b;
      }
    }

    Benchmark const&
    get (string const& code) const
    {
      auto b = by_code_.find (code);
      if (b == by_code_.end()) {
        ROS_FATAL_STREAM ("Could not find benchmark with code \"" << code << "\"");
        abort_rsbb();
      }
      return b->second;
    }
};



class Passwords
{
    map<string, string> passwords_;

  public:
    Passwords()
    {
      using namespace YAML;

      Node file = LoadFile (param_direct<string> ("~passwords_file", "passwords.yaml"));
      if (! file.IsMap()) {
        ROS_FATAL_STREAM ("Passwords file is not a map!");
        abort_rsbb();
      }
      for (auto const& team_node : file) {
        passwords_[team_node.first.as<string>()] = team_node.second.as<string>();
      }
    }

    string const&
    get (string const& team) const
    {
      auto b = passwords_.find (team);
      if (b == passwords_.end()) {
        ROS_FATAL_STREAM ("Could not find password for team \"" << team << "\"");
        abort_rsbb();
      }
      return b->second;
    }
};



struct CoreSharedState
    : boost::noncopyable {
  NodeHandle nh;
  ActiveRobots active_robots;
  string status;
  const Benchmarks benchmarks;
  const Passwords passwords;
  const string run_uuid;
  map<string, pair<string, uint32_t>> benchmarking_robots;
  bool tablet_display_map;
  roah_devices::DevicesState::ConstPtr last_devices_state;
  Time last_tablet_time;
  std::shared_ptr<const roah_rsbb_msgs::TabletBeacon> last_tablet;

  unsigned short private_port_;

  CoreSharedState()
    : status ("Initializing...")
    , run_uuid (to_string (boost::uuids::random_generator() ()))
    , tablet_display_map (false)
    , last_devices_state (boost::make_shared<roah_devices::DevicesState>())
    , last_tablet_time (TIME_MIN)
    , last_tablet (/*empty*/)
    , private_port_ (param_direct<int> ("~rsbb_port", 6666))
  {
  }

  unsigned short
  private_port()
  {
    return ++private_port_;
  }
};

#endif
