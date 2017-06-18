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

#ifndef CALC_H_
#define CALC_H_

#include <Arduino.h>
#include <math.h>

#include <Eigen.h>        // Calls main Eigen matrix class library
#include <Eigen/LU>       // Calls inverse, determinant, LU decomp., etc.

#include "link.h"

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

namespace open_manipulator
{
class Calc
{
 public:
  Calc();
  ~Calc();

  Eigen::Matrix3f skew(Eigen::Vector3f v);
  float sign(float num);

  Eigen::Matrix3f calcRodrigues(Eigen::Vector3f axis, float angle);
  Eigen::Matrix3f calcRotationMatrix(String notation, float angle);
  Eigen::Vector3f calcVerr(Eigen::Vector3f Cref, Eigen::Vector3f Cnow);
  Eigen::Vector3f calcWerr(Eigen::Matrix3f Cref, Eigen::Matrix3f Cnow);
  Eigen::Vector3f calcAngularVelocity(Eigen::Matrix3f R);
  Eigen::MatrixXf calcJacobian(Link* link, Pose goal_pose, uint8_t joint_num);
};
}

#endif // CALC_H_
