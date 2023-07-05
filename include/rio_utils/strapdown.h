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

#include <eigen3/Eigen/Dense>
#include "rio_utils/data_types.h"

namespace rio
{
/**
 * @brief The Strapdown class implements a strapdown which propagates a kinematic state using an IMU measurement
 * @note Uses the North East Down (NED) convention
 */
class Strapdown
{
public:
  /**
   * @brief Strapdown constructor
   * @param local_gravity  local gravity using the NED convention
   */
  Strapdown(const double local_gravity = 9.80665);

  /**
   * @brief Propagates the given navigation solution using an IMU measurement
   * @param nav_sol_prev   previous navigation solution
   * @param a_b_ib         measured body frame acceleration
   * @param w_b_ib         measured body frame angular velocity
   * @param dt             propagation time
   * @return
   */
  NavigationSolution
  propagate(const NavigationSolution nav_sol_prev, const Vector3 a_b_ib, const Vector3 w_b_ib, const Real dt);

private:
  Eigen::Matrix4d getQLeftMatrix(const Vector4& v);

  Vector3 local_gravity_;
};
}  // namespace rio
