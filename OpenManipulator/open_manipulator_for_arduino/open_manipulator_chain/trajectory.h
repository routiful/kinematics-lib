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

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <Arduino.h>
#include <math.h>

namespace open_manipulator
{
class Trajectory
{
 public:
   uint16_t position_value[];

 public:
  Trajectory();
  ~Trajectory();

  uint16_t* trapezoidalVelocityProfile(uint16_t* pos_start, uint16_t* pos_end, float acc, float max_vel);
  uint16_t* trapezoidalTimeProfile(uint16_t* pos_start, uint16_t* pos_end, float acc_time, float total_time);
};
}

#endif // TRAJECTORY_H_
