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

#include "link.h"
#include "calc.h"

#include <Eigen/Dense>

#define DEBUG
// #define SIMULATION

#define COMMUNICATION_RATE  300
#define GRIPPER_ON          -10.0
#define GRIPPER_OFF         -45.0
#define JOINT_NUM           4
#define LINK_NUM            6

#define BASE   0
#define JOINT1 1
#define JOINT2 2
#define JOINT3 3
#define JOINT4 4
#define END    5

float joint_angle[6] = {0.0, 0.0, 0.0*DEG2RAD, 0.0*DEG2RAD, 0.0*DEG2RAD, 0.0};
float gripper_pos    = GRIPPER_OFF;

HardwareTimer serial_timer(TIMER_CH1);

open_manipulator::Link link[JOINT_NUM+2];
open_manipulator::Calc calc;
open_manipulator::Pose target_pose;

void setup()
{
  Serial.begin(57600);
#ifdef DEBUG
   while(!Serial);
#endif

  initLink();
  timerInit();

  establishContact();

  setJointAngle(joint_angle);
  forwardKinematics(BASE);

#ifdef DEBUG
  for (uint8_t num = BASE; num <= END; num++)
  {
    Serial.print("link : "); Serial.println(link[num].name_);
    Serial.println("p_ : "); print_vt3f(link[num].p_);
    Serial.println("R_ : "); print_mt3f(link[num].R_);
  }
#endif

  target_pose.position    << 0.179, 0.000, 0.201;   // (0, 20, -30, 30)
  target_pose.orientation << 0.940, 0.000, -0.342,
                              0.000, 1.000, 0.000,
                              0.342, 0.000, 0.940;


  inverseKinematics(END, target_pose);
}

void loop()
{
  showLedStatus();
}

/*******************************************************************************
* Hardware timer initialization
*******************************************************************************/
void timerInit()
{
  serial_timer.stop();
  serial_timer.setPeriod(COMMUNICATION_RATE);
  serial_timer.attachInterrupt(handler_serial);
  serial_timer.start();
}

/*******************************************************************************
* Forward kinematics
*******************************************************************************/
void forwardKinematics(int8_t me)
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
    link[me].R_ = link[mother].R_ * calc.Rodrigues(link[me].a_, link[me].q_);
  }

  forwardKinematics(link[me].sibling_);
  forwardKinematics(link[me].child_);
}

/*******************************************************************************
* Inverse kinematics (Numerical Method)
*******************************************************************************/
void inverseKinematics(uint8_t to, open_manipulator::Pose goal_pose)
{
  float lambda = 0.7; // To stabilize the numeric calculation (0 1]
  Eigen::MatrixXf J(6,5);
  Eigen::Vector3f Verr, Werr;
  Eigen::VectorXf VWerr(6);
  Eigen::VectorXf dq(5);

  forwardKinematics(BASE);

  for (int i = 0; i < 10; i++)
  {
    J = calc.Jacobian(link, goal_pose, LINK_NUM);
    Verr = calc.Verr(goal_pose.position, link[to].p_);
    Werr = calc.Werr(goal_pose.orientation, link[to].R_);
    VWerr << Verr(0), Verr(1), Verr(2),
             Werr(0), Werr(1), Werr(2);

    if (VWerr.norm() < 1E-6)
      return;

    Eigen::ColPivHouseholderQR<Eigen::MatrixXf> dec(J);
    dq = lambda * dec.solve(VWerr);

    for (int id = JOINT1; id <= JOINT4; id++)
    {
      link[id].q_ = link[id].q_ + dq(id-1);
    }
    forwardKinematics(BASE);
  }

  #ifdef DEBUG
    Serial.println(VWerr.norm());
    Serial.println(link[JOINT1].q_*RAD2DEG);
    Serial.println(link[JOINT2].q_*RAD2DEG);
    Serial.println(link[JOINT3].q_*RAD2DEG);
    Serial.println(link[JOINT4].q_*RAD2DEG);
    Serial.println("");
  #endif
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
}

/*******************************************************************************
* HardwareTimer function
*******************************************************************************/
void handler_serial()
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

/*******************************************************************************
* show led status
*******************************************************************************/
void showLedStatus(void)
{
  static uint32_t t_time = millis();

  if ((millis()-t_time) >= 500 )
  {
    t_time = millis();
    digitalWrite(13, !digitalRead(13));
  }

  if (getPowerInVoltage() < 11.1)
  {
    setLedOn(2);
  }
  else
  {
    setLedOff(2);
  }

  if (getUsbConnected() > 0)
  {
    setLedOn(3);
  }
  else
  {
    setLedOff(3);
  }

  updateRxTxLed();
}

void updateRxTxLed(void)
{
  static uint32_t rx_led_update_time;
  static uint32_t tx_led_update_time;
  static uint32_t rx_cnt;
  static uint32_t tx_cnt;


  if ((millis()-tx_led_update_time) > 50)
  {
    tx_led_update_time = millis();

    if (tx_cnt != Serial.getTxCnt())
    {
      setLedToggle(0);
    }
    else
    {
      setLedOff(0);
    }

    tx_cnt = Serial.getTxCnt();
  }

  if( (millis()-rx_led_update_time) > 50 )
  {
    rx_led_update_time = millis();

    if (rx_cnt != Serial.getRxCnt())
    {
      setLedToggle(1);
    }
    else
    {
      setLedOff(1);
    }

    rx_cnt = Serial.getRxCnt();
  }
}

/*******************************************************************************
* print matrix(3x3)
*******************************************************************************/
void print_mt3f(const Eigen::Matrix3f& m)
{
   uint8_t i, j;

   for (i=0; i<3; i++)
   {
       for (j=0; j<3; j++)
       {
           Serial.print(m(i,j), 3);   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();
}

/*******************************************************************************
* print vector(1x3)
*******************************************************************************/
void print_vt3f(const Eigen::Vector3f& v)
{
   uint8_t i, j;

   for (i=0; i<3; i++)
   {
     Serial.print(v(i), 3);
     Serial.print(", ");
   }
   Serial.println();
}
