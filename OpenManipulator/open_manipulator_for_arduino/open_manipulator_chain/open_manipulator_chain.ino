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

#include "open_manipulator_chain_config.h"

#define DEBUG
// #define DYNAMIXEL
// #define SIMULATION

void setup()
{
  Serial.begin(SERIAL_RATE);
#ifdef DEBUG
   while(!Serial);
#endif

  initLink();

  establishContact();

  setJointAngle(joint_angle);

  target_pose.position    << 0.179, 0.000, 0.201;   // (0, 20, -30, 30)
  target_pose.orientation << 0.940, 0.000, -0.342,
                              0.000, 1.000, 0.000,
                              0.342, 0.000, 0.940;

  kinematics.inverse(link, END, target_pose);

#ifdef DEBUG
  for (uint8_t num = BASE; num <= END; num++)
  {
    Serial.println(link[num].q_*RAD2DEG);
  }
#endif
}

void loop()
{
  communicationWithProcessing();
  showLedStatus();
}

/*******************************************************************************
* Manipulator link initialization
*******************************************************************************/
void initLink()
{
  link[0].name_    = "Base";
  link[0].mother_  = -1;
  link[0].sibling_ = -1;
  link[0].child_   = 1;
  link[0].mass_    = 1.0;
  link[0].p_       = Eigen::Vector3f::Zero();
  link[0].R_       = Eigen::Matrix3f::Identity(3,3);
  link[0].q_       = 0;
  link[0].dq_      = 0;
  link[0].ddq_     = 0;
  link[0].a_       = Eigen::Vector3f::Zero();
  link[0].b_       = Eigen::Vector3f::Zero();
  link[0].v_       = Eigen::Vector3f::Zero();
  link[0].w_       = Eigen::Vector3f::Zero();

  link[1].name_    = "Joint1";
  link[1].mother_  = 0;
  link[1].sibling_ = -1;
  link[1].child_   = 2;
  link[1].mass_    = 1.0;
  link[1].p_       = Eigen::Vector3f::Zero();
  link[1].R_       = Eigen::Matrix3f::Identity(3,3);
  link[1].q_       = 0.0;
  link[1].dq_      = 0.0;
  link[1].ddq_     = 0.0;
  link[1].a_       << 0, 0, 1;
  link[1].b_       << 0.012, 0, 0.036;
  link[1].v_       = Eigen::Vector3f::Zero();
  link[1].w_       = Eigen::Vector3f::Zero();

  link[2].name_    = "Joint2";
  link[2].mother_  = 1;
  link[2].sibling_ = -1;
  link[2].child_   = 3;
  link[2].mass_    = 1.0;
  link[2].p_       = Eigen::Vector3f::Zero();
  link[2].R_       = Eigen::Matrix3f::Identity(3,3);
  link[2].q_       = 0.0;
  link[2].dq_      = 0.0;
  link[2].ddq_     = 0.0;
  link[2].a_       << 0, -1, 0;
  link[2].b_       << 0, 0, 0.040;
  link[2].v_       = Eigen::Vector3f::Zero();
  link[2].w_       = Eigen::Vector3f::Zero();

  link[3].name_    = "Joint3";
  link[3].mother_  = 2;
  link[3].sibling_ = -1;
  link[3].child_   = 4;
  link[3].mass_    = 1.0;
  link[3].p_       = Eigen::Vector3f::Zero();
  link[3].R_       = Eigen::Matrix3f::Identity(3,3);
  link[3].q_       = 0.0;
  link[3].dq_      = 0.0;
  link[3].ddq_     = 0.0;
  link[3].a_       << 0, -1, 0;
  link[3].b_       << 0.022, 0, 0.122;
  link[3].v_       = Eigen::Vector3f::Zero();
  link[3].w_       = Eigen::Vector3f::Zero();

  link[4].name_    = "Joint4";
  link[4].mother_  = 3;
  link[4].sibling_ = -1;
  link[4].child_   = 5;
  link[4].mass_    = 1.0;
  link[4].p_       = Eigen::Vector3f::Zero();
  link[4].R_       = Eigen::Matrix3f::Identity(3,3);
  link[4].q_       = 0.0;
  link[4].dq_      = 0.0;
  link[4].ddq_     = 0.0;
  link[4].a_       << 0, -1, 0;
  link[4].b_       << 0.124, 0, 0;
  link[4].v_       = Eigen::Vector3f::Zero();
  link[4].w_       = Eigen::Vector3f::Zero();

  link[5].name_    = "End";
  link[5].mother_  = 4;
  link[5].sibling_ = -1;
  link[5].child_   = -1;
  link[5].mass_    = 1.0;
  link[5].p_       = Eigen::Vector3f::Zero();
  link[5].R_       = Eigen::Matrix3f::Identity(3,3);
  link[5].q_       = 0.0;
  link[5].dq_      = 0.0;
  link[5].ddq_     = 0.0;
  link[5].a_       = Eigen::Vector3f::Zero();
  link[5].b_       << 0.070, 0, 0;
  link[5].v_       = Eigen::Vector3f::Zero();
  link[5].w_       = Eigen::Vector3f::Zero();
}

/*******************************************************************************
* Set joint angle
*******************************************************************************/
void setJointAngle(float joint_angle[6])
{
  link[JOINT1].q_ = joint_angle[JOINT1];
  link[JOINT2].q_ = joint_angle[JOINT2];
  link[JOINT3].q_ = joint_angle[JOINT3];
  link[JOINT4].q_ = joint_angle[JOINT4];

  kinematics.forward(link, BASE);

#ifdef DEBUG
  for (uint8_t num = BASE; num <= END; num++)
  {
    Serial.print("link : "); Serial.println(link[num].name_);
    Serial.println("p_ : "); print_vt3f(link[num].p_);
    Serial.println("R_ : "); print_mt3f(link[num].R_);
  }
#endif
}

/*******************************************************************************
* HardwareTimer function
*******************************************************************************/
void communicationWithProcessing()
{
#ifdef SIMULATION
  static String simulator = "";
  static bool communication = false;

  if (Serial.available())
  {
    simulator = Serial.readString();
    if (simulator == "ready")
    {
      communication = true;
    }
    else if (simulator == "stop")
    {
      communication = false;
    }
  }

  if (communication)
  {
    Serial.print(link[JOINT1].q_);
    Serial.print(",");
    Serial.print(link[JOINT2].q_);
    Serial.print(",");
    Serial.print(link[JOINT3].q_);
    Serial.print(",");
    Serial.print(link[JOINT4].q_);
    Serial.print(",");
    Serial.println(gripper_pos);
  }
#endif
}

/*******************************************************************************
* send an initial string
*******************************************************************************/
void establishContact()
{
  if (Serial.available())
  {
    Serial.println("0.0, 0.0, 0.0, 0.0, 0.0");
    delay(300);
  }
}
