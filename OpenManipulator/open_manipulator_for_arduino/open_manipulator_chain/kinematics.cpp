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

#include "kinematics.h"
using namespace open_manipulator;

Kinematics::Kinematics(uint8_t link_num)
{
  calc_     = new Calc;
  link_num_ = link_num;
}

Kinematics::~Kinematics(){}

/*******************************************************************************
* Forward kinematics
*******************************************************************************/
void Kinematics::forward(Link* link, int8_t me)
{
  int8_t mother = 0;

  if (me == -1)
  {
    return;
  }

  if (me != 0)
  {
    mother = link[me].mother_;
    link[me].p_ = link[mother].R_ * link[me].b_ + link[mother].p_;
    link[me].R_ = link[mother].R_ * calc_->Rodrigues(link[me].a_, link[me].q_);
  }

  forward(link, link[me].sibling_);
  forward(link, link[me].child_);
}

/*******************************************************************************
* Inverse kinematics (Numerical Method)
*******************************************************************************/
void Kinematics::inverse(Link* link, uint8_t to, Pose goal_pose)
{
  float lambda = 0.7; // To stabilize the numeric calculation (0 1]
  Eigen::MatrixXf J(6,5);
  Eigen::Vector3f Verr, Werr;
  Eigen::VectorXf VWerr(6);
  Eigen::VectorXf dq(5);

  forward(link, BASE);

  for (int i = 0; i < 10; i++)
  {
    J = calc_->Jacobian(link, goal_pose, link_num_);
    Verr = calc_->Verr(goal_pose.position, link[to].p_);
    Werr = calc_->Werr(goal_pose.orientation, link[to].R_);
    VWerr << Verr(0), Verr(1), Verr(2),
             Werr(0), Werr(1), Werr(2);

    if (VWerr.norm() < 1E-6)
      return;

    Eigen::ColPivHouseholderQR<Eigen::MatrixXf> dec(J);
    dq = lambda * dec.solve(VWerr);

    for (int id = BASE+1; id <= to; id++)
    {
      link[id].q_ = link[id].q_ + dq(id-1);
    }
    forward(link, BASE);
  }
}
