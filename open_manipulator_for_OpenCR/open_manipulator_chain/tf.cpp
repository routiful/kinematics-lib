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

TF::TF(){}

TF::~TF(){}

Eigen::Matrix3f TF::skew(Eigen::Vector3f v)
{
  Eigen::Matrix3f skew_symmetric_matrix = Eigen::Matrix3f::Zero();

  skew_symmetric_matrix <<    0,  -v(2),  v(1),
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
  rotation_matrix = Identity_matrix +
                    skew_symmetric_matrix * sin(angle) +
                    skew_symmetric_matrix * skew_symmetric_matrix * (1 - cos(angle));

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

Eigen::Matrix3f TF::calcRotationMatrix(String notation, float angle)
{
  String roll  = "roll";
  String pitch = "pitch";
  String yaw   = "yaw";

  Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();

  if (notation.equals(roll))
  {
    rotation_matrix << 1.000,  0.000,      0.000,
                       0.000,  cos(angle), sin(angle),
                       0.000, -sin(angle), cos(angle);
  }
  else if (notation.equals(pitch))
  {
    rotation_matrix <<  cos(angle),  0.000, sin(angle),
                        0.000,       1.000, 0.000,
                       -sin(angle),  0.000, cos(angle);
  }
  else if (notation.equals(yaw))
  {
    rotation_matrix << cos(angle), -sin(angle), 0.000,
                       sin(angle),  cos(angle), 0.000,
                       0.000,       0.000,      1.000;
  }
  else
  {
    rotation_matrix = Eigen::Matrix3f::Identity();
  }

  return rotation_matrix;
}

Eigen::Vector3f calcAngularVelocity(Eigen::Matrix3f R)
{
  Eigen::Vector3f l = Eigen::Vector3f::Zero();
  Eigen::Vector3f w = Eigen::Vector3f::Zero();

  float theta = 0.0;
  float diag = 0.0;
  bool diagonal_matrix = true;

  l << R(2,1) - R(1,2),
       R(0,2) - R(2,0),
       R(1,0) - R(0,1);
  theta = atan2(l.norm(), R(0,0) + R(1,1) + R(2,2) - 1);
  diag = R(0,0) + R(1,1) + R(2,2);

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (i != j)
      {
        if (R(i, j) != 0)
        {
          diagonal_matrix = false;
        }
      }
    }
  }

  if (R == Eigen::Matrix3f::Identity())
  {
    w  = Eigen::Vector3f::Zero();
  }
  else if (diagonal_matrix == true)
  {
    w << R(0,0) + 1, R(1,1) + 1, R(2,2) + 1;
    w = w * M_PI_2;
  }
  else
  {
    w = theta * (l / l.norm());
  }

  return w;
}

Eigen::Vector3f calcVerr(Eigen::Vector3f Cref, Eigen::Vector3f Cnow)
{
  Eigen::Vector3f Verr;

  Verr = Cref - Cnow;

  return Verr;
}

Eigen::Vector3f calcWerr(Eigen::Matrix3f Cref, Eigen::Matrix3f Cnow)
{
  Eigen::Matrix3f Rerr;
  Eigen::Vector3f Werr;

  Rerr = Cnow.transpose() * Cref;
  Werr = Cnow * calcAngularVelocity(Rerr);

  return Werr;
}
