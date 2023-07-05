// This file is part of RIO - Radar Inertial Odometry and Radar ego velocity estimation.
// Copyright (C) 2021  Christopher Doer <christopher.doer@kit.edu>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <ros/node_handle.h>

namespace rio
{
enum class RosParameterType
{
  Required,
  Recommended,
  Optional
};

template <typename T>
static bool getRosParameter(const ros::NodeHandle& nh,
                            const std::string kPrefix,
                            const RosParameterType& param_type,
                            const std::string& param_name,
                            T& param)
{
  if (!nh.getParam(param_name, param))
  {
    if (param_type == RosParameterType::Optional)
    {
      ROS_INFO_STREAM(kPrefix << "<" << param_name
                              << "> is optional but not configured. Using default value: " << param);
      nh.setParam(param_name, param);
    }
    else if (param_type == RosParameterType::Recommended)
    {
      ROS_WARN_STREAM(kPrefix << "<" << param_name
                              << "> is recommeded but not configured. Using default value: " << param);
      nh.setParam(param_name, param);
    }
    else
    {
      ROS_ERROR_STREAM(kPrefix << "<" << param_name << "> is required but not configured. Exiting!");
      return false;
    }
  }

  return true;
}

}  // namespace rio
