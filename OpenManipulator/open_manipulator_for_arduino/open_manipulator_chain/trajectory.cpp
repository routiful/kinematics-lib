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

void minimumJerk(Property* start, Property* end, float control_period, float mov_time)
{
  float a[6] = {0, };

  a[0] = start->pos;
  a[1] = start->vel;
  a[2] = 2 * start->acc;

  Eigen::Matrix3f

}
