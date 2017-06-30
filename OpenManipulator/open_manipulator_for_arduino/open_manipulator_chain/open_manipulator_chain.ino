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

// #define DEBUG
// #define DYNAMIXEL
#define SIMULATION

/*******************************************************************************
* Setup
*******************************************************************************/
void setup()
{
  Serial.begin(SERIAL_RATE);
#ifdef SIMULATION
   while(!Serial);
#endif

  initTimer();

  initLink();
  initKinematics();

  initTrajectory();

#ifdef DYNAMIXEL
  initMotor();
  initMotorDriver(FALSE);
#endif

  kinematics->forward(link, BASE);

  start_prop.pos = 0.0;
  start_prop.vel = 0.0;
  start_prop.acc = 0.0;

  end_prop.pos   = 30.0 * DEG2RAD;
  end_prop.vel   = 0.0;
  end_prop.acc   = 0.0;

  tra = trajectory->minimumJerk(start_prop, end_prop, control_period, mov_time);

  moving = true;

  // open_manipulator::Pose goal_pose;
  // goal_pose.position    << 0.179, 0.000, 0.201;   // (0, 20, -30, 30, 0)
  // goal_pose.orientation << 0.940, 0.000, -0.342,
  //                           0.000, 1.000, 0.000,
  //                           0.342, 0.000, 0.940;
  //
  // kinematics->sr_inverse(link, END, goal_pose);

#ifdef SIMULATION
  establishContactToProcessing();
#endif
}

/*******************************************************************************
* Loop
*******************************************************************************/
void loop()
{
#ifdef SIMULATION
  if (getDataFromProcessing())
  {
    Serial.println("fuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuk");
    timer.start();
  }
#endif

#ifdef DYNAMIXEL
  getDynamixelPosition();
#endif

  showLedStatus();
}

/*******************************************************************************
* Timer (8mm)
*******************************************************************************/
void handler_control()
{
  // comm = getDataFromProcessing();

#ifdef SIMULATION
  if (moving)
  {
    for (int num = 0; num < mov_time/control_period + 1; num++)
    {
      link[JOINT2].q_ = tra(num);
      // Serial.println(link[JOINT2].q_);
      sendJointDatatoProcessing(comm);
    }
    moving = FALSE;
  }
#endif

#ifdef DYNAMIXEL
  // writeDynamixelPosition();
#endif
}

/*******************************************************************************
* Send Joint Data to Processing
*******************************************************************************/
void sendJointDatatoProcessing(uint8_t onoff)
{
  if (onoff)
  {
    Serial.print(link[JOINT1].q_);
    Serial.print(",");
    Serial.print(link[JOINT2].q_);
    Serial.print(",");
    Serial.print(link[JOINT3].q_);
    Serial.print(",");
    Serial.print(link[JOINT4].q_);
    Serial.print(",");
    Serial.println(link[END].q_);
  }
}

/*******************************************************************************
* Get Data From Processing
*******************************************************************************/
uint8_t getDataFromProcessing()
{
  String simulator = "";
  uint8_t communication = false;

  if (Serial.available())
  {
    simulator = Serial.readString();
    Serial.println(simulator);
    if (simulator == "ready")
      communication = true;
    else if (simulator == "end")
      communication = false;
  }

  Serial.println("fuck");

  return communication;
}

/*******************************************************************************
* Initialization Timer
*******************************************************************************/
void initTimer()
{
  timer.stop();
  timer.setPeriod(CONTROL_RATE);
  timer.attachInterrupt(handler_control);
  // timer.start();
}

/*******************************************************************************
* Get Dynamixel Position (rad)
*******************************************************************************/
void getDynamixelPosition()
{
  int32_t* pos = motor_driver->readPosition();

  for (int num = 0; num < JOINT_NUM+GRIP_NUM; num++)
  {
    link[num+1].q_ = motor_driver->convertValue2Radian(pos[num]);
  }
}

/*******************************************************************************
* Get Link Position (rad)
*******************************************************************************/
float* getLinkAngle()
{
  float angle[JOINT_NUM+GRIP_NUM];

  angle[JOINT1] = link[JOINT1].q_;
  angle[JOINT2] = link[JOINT2].q_;
  angle[JOINT3] = link[JOINT3].q_;
  angle[JOINT4] = link[JOINT4].q_;
  angle[END]    = link[END].q_;

  return angle;
}

/*******************************************************************************
* Write Dynamixel Position (rad)
*******************************************************************************/
void writeDynamixelPosition(float* angle)
{
  int32_t joint_value[JOINT_NUM] = {0, };

  setFK(angle);

  for (int num = 0; num < JOINT_NUM; num++)
  {
    joint_value[num] = motor_driver->convertRadian2Value(angle[num]);
  }
  motor_driver->jointControl(joint_value);
}

/*******************************************************************************
* Set joint angle
*******************************************************************************/
void setFK(float* angle)
{
  for (int num = JOINT1; num < END; num++)
  {
    link[num].q_ = angle[num-1];
  }

  kinematics->forward(link, BASE);
}

/*******************************************************************************
* Set Gripper status
*******************************************************************************/
void setGripper(bool onoff)
{
  float   gripper_pos   = grip_off;
  int32_t gripper_value = 0;

  if (onoff)
    gripper_pos = grip_on;
  else
    gripper_pos = grip_off;

  gripper_value = motor_driver->convertRadian2Value(gripper_pos);
  motor_driver->gripControl(gripper_value);
}

/*******************************************************************************
* Manipulator link initialization
*******************************************************************************/
void initLink()
{
  link[BASE].name_      = "Base";
  link[BASE].mother_    = -1;
  link[BASE].sibling_   = -1;
  link[BASE].child_     = 1;
  link[BASE].mass_      = 1.0;
  link[BASE].p_         = Eigen::Vector3f::Zero();
  link[BASE].R_         = Eigen::Matrix3f::Identity(3,3);
  link[BASE].q_         = 0.0;
  link[BASE].dq_        = 0.0;
  link[BASE].ddq_       = 0.0;
  link[BASE].a_         = Eigen::Vector3f::Zero();
  link[BASE].b_         = Eigen::Vector3f::Zero();
  link[BASE].v_         = Eigen::Vector3f::Zero();
  link[BASE].w_         = Eigen::Vector3f::Zero();

  link[JOINT1].name_    = "Joint1";
  link[JOINT1].mother_  = 0;
  link[JOINT1].sibling_ = -1;
  link[JOINT1].child_   = 2;
  link[JOINT1].mass_    = 1.0;
  link[JOINT1].p_       = Eigen::Vector3f::Zero();
  link[JOINT1].R_       = Eigen::Matrix3f::Identity(3,3);
  link[JOINT1].q_       = 0.0;
  link[JOINT1].dq_      = 0.0;
  link[JOINT1].ddq_     = 0.0;
  link[JOINT1].a_       << 0, 0, 1;
  link[JOINT1].b_       << 0.012, 0, 0.036;
  link[JOINT1].v_       = Eigen::Vector3f::Zero();
  link[JOINT1].w_       = Eigen::Vector3f::Zero();

  link[JOINT2].name_    = "Joint2";
  link[JOINT2].mother_  = 1;
  link[JOINT2].sibling_ = -1;
  link[JOINT2].child_   = 3;
  link[JOINT2].mass_    = 1.0;
  link[JOINT2].p_       = Eigen::Vector3f::Zero();
  link[JOINT2].R_       = Eigen::Matrix3f::Identity(3,3);
  link[JOINT2].q_       = 0.0;
  link[JOINT2].dq_      = 0.0;
  link[JOINT2].ddq_     = 0.0;
  link[JOINT2].a_       << 0, -1, 0;
  link[JOINT2].b_       << 0, 0, 0.040;
  link[JOINT2].v_       = Eigen::Vector3f::Zero();
  link[JOINT2].w_       = Eigen::Vector3f::Zero();

  link[JOINT3].name_    = "Joint3";
  link[JOINT3].mother_  = 2;
  link[JOINT3].sibling_ = -1;
  link[JOINT3].child_   = 4;
  link[JOINT3].mass_    = 1.0;
  link[JOINT3].p_       = Eigen::Vector3f::Zero();
  link[JOINT3].R_       = Eigen::Matrix3f::Identity(3,3);
  link[JOINT3].q_       = 0.0;
  link[JOINT3].dq_      = 0.0;
  link[JOINT3].ddq_     = 0.0;
  link[JOINT3].a_       << 0, -1, 0;
  link[JOINT3].b_       << 0.022, 0, 0.122;
  link[JOINT3].v_       = Eigen::Vector3f::Zero();
  link[JOINT3].w_       = Eigen::Vector3f::Zero();

  link[JOINT4].name_    = "Joint4";
  link[JOINT4].mother_  = 3;
  link[JOINT4].sibling_ = -1;
  link[JOINT4].child_   = 5;
  link[JOINT4].mass_    = 1.0;
  link[JOINT4].p_       = Eigen::Vector3f::Zero();
  link[JOINT4].R_       = Eigen::Matrix3f::Identity(3,3);
  link[JOINT4].q_       = 0.0;
  link[JOINT4].dq_      = 0.0;
  link[JOINT4].ddq_     = 0.0;
  link[JOINT4].a_       << 0, -1, 0;
  link[JOINT4].b_       << 0.124, 0, 0;
  link[JOINT4].v_       = Eigen::Vector3f::Zero();
  link[JOINT4].w_       = Eigen::Vector3f::Zero();

  link[END].name_       = "Gripper";
  link[END].mother_     = 4;
  link[END].sibling_    = -1;
  link[END].child_      = -1;
  link[END].mass_       = 1.0;
  link[END].p_          = Eigen::Vector3f::Zero();
  link[END].R_          = Eigen::Matrix3f::Identity(3,3);
  link[END].q_          = 0.0;
  link[END].dq_         = 0.0;
  link[END].ddq_        = 0.0;
  link[END].a_          = Eigen::Vector3f::Zero();
  link[END].b_          << 0.070, 0, 0;
  link[END].v_          = Eigen::Vector3f::Zero();
  link[END].w_          = Eigen::Vector3f::Zero();
}

/*******************************************************************************
* Initialization Kinematics Library
*******************************************************************************/
void initKinematics()
{
  kinematics = new open_manipulator::Kinematics();
}

/*******************************************************************************
* Initialization Trajectory Library
*******************************************************************************/
void initTrajectory()
{
  trajectory = new open_manipulator::Trajectory();
}

/*******************************************************************************
* Dynamixel Initialization
*******************************************************************************/
void initMotor()
{
  motor[0].motor_id   = 1;
  motor[0].joint_id   = 1;
  motor[0].grip_id    = 0;
  motor[0].name       = "Joint1";

  motor[1].motor_id   = 2;
  motor[1].joint_id   = 2;
  motor[1].grip_id    = 0;
  motor[1].name       = "Joint2";

  motor[2].motor_id   = 3;
  motor[2].joint_id   = 3;
  motor[2].grip_id    = 0;
  motor[2].name       = "Joint3";

  motor[3].motor_id   = 4;
  motor[3].joint_id   = 4;
  motor[3].grip_id    = 0;
  motor[3].name       = "Joint4";

  motor[4].motor_id   = 5;
  motor[4].joint_id   = 0;
  motor[4].grip_id    = 1;
  motor[4].name       = "Gripper";
}

/*******************************************************************************
* Initialization Motor Driver Library
*******************************************************************************/
void initMotorDriver(uint8_t torque)
{
  motor_driver = new open_manipulator::MotorDriver(PROTOCOL_VERSION, BAUE_RATE);
  if (motor_driver->init(motor, 5))
    motor_driver->setTorque(torque);
  else
    return;
}

/*******************************************************************************
* send an initial string
*******************************************************************************/
void establishContactToProcessing()
{
  if (Serial.available())
  {
    Serial.println("0.0, 0.0, 0.0, 0.0, 0.0");
    delay(300);
  }
}
