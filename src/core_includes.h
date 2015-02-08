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

#ifndef __CORE_INCLUDES_H__
#define __CORE_INCLUDES_H__

#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>

#include <boost/date_time.hpp>
#include <boost/noncopyable.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>

#include <rockin_benchmarking/BmBoxState.h>
#include <rockin_benchmarking/ClientState.h>
#include <rockin_benchmarking/RefBoxState.h>
#include <roah_devices/Bool.h>
#include <roah_devices/DevicesState.h>
#include <roah_devices/Percentage.h>
#include <roah_rsbb/CoreToGui.h>
#include <roah_rsbb/CoreToPublic.h>
#include <roah_rsbb/RobotInfo.h>
#include <roah_rsbb/Zone.h>
#include <roah_rsbb/ZoneState.h>
#include <roah_rsbb/ZoneUInt8.h>
#include <roah_rsbb/ZoneScore.h>

#include <roah_utils.h>
#include <ros_roah_rsbb.h>



using namespace ros;
using namespace std;

#endif
