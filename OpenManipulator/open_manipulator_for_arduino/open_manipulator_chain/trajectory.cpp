/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim */

#include "trajectory.h"
using namespace open_manipulator;

Trajectory::Trajectory(){}

Trajectory::~Trajectory(){}

Eigen::VectorXf Trajectory::minimumJerk(Property start, Property end, float control_period, float mov_time)
{
  float a[6] = {0, };
  float step_time = mov_time/control_period + 1;

  a[0] =     start.pos;
  a[1] =     start.vel;
  a[2] = 2 * start.acc;

  Eigen::Matrix3f A = Eigen::Matrix3f::Identity(3,3);
  Eigen::Vector3f b = Eigen::Vector3f::Zero();
  Eigen::Vector3f x = Eigen::Vector3f::Zero();

  A <<     pow(mov_time,3),     pow(mov_time,4),     pow(mov_time,5),
       3 * pow(mov_time,2), 4 * pow(mov_time,3), 5 * pow(mov_time,4),
       6 * pow(mov_time,1), 12* pow(mov_time,2), 20* pow(mov_time,3);

  b << end.pos, end.vel, end.acc;

  Eigen::ColPivHouseholderQR<Eigen::Matrix3f> dec(A);
  x = dec.solve(b);

  a[3] = x(0);
  a[4] = x(1);
  a[5] = x(2);

  Eigen::VectorXf tra;

  for (int num = 0; num < step_time; num++)
  {
    tra(num) = a[0] +
               a[1]*pow(control_period*num,1) +
               a[2]*pow(control_period*num,2) +
               a[3]*pow(control_period*num,3) +
               a[4]*pow(control_period*num,4) +
               a[5]*pow(control_period*num,5);
  }

  return tra;
}
