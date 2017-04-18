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

#include "tf.h"
using namespace open_manipulator;

TF::TF()
{
}

TF::~TF()
{

}

Eigen::Matrix3f TF::skew(Eigen::Vector3f v)
{
  Eigen::Matrix3f skew_symmetric_matrix = Eigen::Matrix3f::Zero();

  skew_symmetric_matrix <<     0,  -v(2),  v(1),
                           v(2),      0, -v(0),
                          -v(1),   v(0),     0;

  return skew_symmetric_matrix;
}

Eigen::Matrix3f TF::calcRodrigues(Eigen::Vector3f axis, float angle)
{
  Eigen::Matrix3f skew_symmetric_matrix = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f Identity_matrix = Eigen::Matrix3f::Identity();

  skew_symmetric_matrix = skew(axis);
  rotation_matrix = Identity_matrix + skew_symmetric_matrix * sin(angle) + skew_symmetric_matrix * skew_symmetric_matrix * (1 - cos(angle));

  return rotation_matrix;
}

float TF::sign(float num)
{
  if (num >= 0.0)
  {
    return 1.0;
  }
  else
  {
    return -1.0;
  }
}
