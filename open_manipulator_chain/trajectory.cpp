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

Trajectory::Trajectory()
{
}

Trajectory::~Trajectory()
{

}

uint16_t* Trajectory::trapezoidalVelocityProfile(uint16_t* pos_start, uint16_t* pos_end, float acc, float max_vel)
{
  //TODO
}

uint16_t* Trajectory::trapezoidalTimeProfile(uint16_t* pos_start, uint16_t* pos_end, float acc_time, float total_time)
{
  // float acc = 0.3;
  // float decel = -acc;
  // float velocity = 1.0;
  // 
  // float dt = 0.007;
  // float decel_time = total_time - acc_time;
  // float vel_time   = total_time - (2 * acc_time);
  //
  // uint16_t acc_count   = acc_time / dt;
  // uint16_t vel_count   = vel_time / dt;
  // uint16_t decel_count = decel_time / dt;
  //
  // uint16_t count = 0;
  //
  // for (int id = 0; id < 4; id++)
  // {
  //   if (count < acc_count)
  //   {
  //     position_value[id] = acc * dt * dt;
  //   }
  //   else if (count < acc_count + vel_count)
  //   {
  //     position_value[id] = velocity * dt;
  //   }
  //   else if (count < acc_count + vel_count + decel_count)
  //   {
  //     position_value[id] = decel * dt * dt;
  //   }
  // }
  //
  // return position_value;
}
