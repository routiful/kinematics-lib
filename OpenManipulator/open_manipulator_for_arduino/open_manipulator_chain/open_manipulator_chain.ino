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
#define DYNAMIXEL
// #define SIMULATION

/*******************************************************************************
* Setup
*******************************************************************************/
void setup()
{
  Serial.begin(SERIAL_RATE);
#ifdef DEBUG
   while(!Serial);
#endif

  initLinkAndMotor();
  initKinematics();

  initTrajectory();

#ifdef DYNAMIXEL
  initMotorDriver(false);
#endif

  initTimer();

#ifdef SIMULATION
  establishContactToProcessing();
#endif

  setFK(link, BASE);

  Serial.println("OpenManipulator Chain Initialization Success!!");
}

/*******************************************************************************
* Loop
*******************************************************************************/
void loop()
{
  // getDynamixelPosition();
  //
  //
  // Serial.print(motor[JOINT1].present_position,5);
  // Serial.print(",");
  // Serial.print(motor[JOINT2].present_position,5);
  // Serial.print(",");
  // Serial.print(motor[JOINT3].present_position,5);
  // Serial.print(",");
  // Serial.print(motor[JOINT4].present_position,5);
  // Serial.print(",");
  // Serial.println(motor[END].present_position,5);

  getDataFromProcessing(comm);

  showLedStatus();
}

/*******************************************************************************
* Timer (8mm)
*******************************************************************************/
void handler_control()
{
  if (moving && comm)
  {
    uint8_t step_time = mov_time/control_period + 1;
    static uint32_t cnt = 0;

    if (cnt >= step_time)
    {
      getDynamixelPosition();
      kinematics->forward(link, BASE);

      Serial.print(motor[JOINT1].present_position ,5);
      Serial.print(",");
      Serial.print(motor[JOINT2].present_position ,5);
      Serial.print(",");
      Serial.print(motor[JOINT3].present_position ,5);
      Serial.print(",");
      Serial.print(motor[JOINT4].present_position ,5);
      Serial.print(",");
      Serial.println(motor[END].present_position ,5);

      Serial.print(link[JOINT1].q_,5);
      Serial.print(",");
      Serial.print(link[JOINT2].q_,5);
      Serial.print(",");
      Serial.print(link[JOINT3].q_,5);
      Serial.print(",");
      Serial.print(link[JOINT4].q_,5);
      Serial.print(",");
      Serial.println(link[END].q_,5);

      moving = false;
      cnt = 0;
    }
    else
    {
      for (int num = BASE; num <= END; num++)
      {
        link[num].q_ = joint_tra(cnt, num);
      }
      cnt++;
    }
#ifdef SIMULATION
    sendJointDataToProcessing();
#endif

#ifdef DYNAMIXEL
      setJointDataToDynamixel();

      // if (cnt%5 == 0)
      //   getDynamixelPosition();
#endif
  }
}

/*******************************************************************************
* Send Joint Data to Processing
*******************************************************************************/
void sendJointDataToProcessing()
{
  Serial.print(link[JOINT1].q_,5);
  Serial.print(",");
  Serial.print(link[JOINT2].q_,5);
  Serial.print(",");
  Serial.print(link[JOINT3].q_,5);
  Serial.print(",");
  Serial.print(link[JOINT4].q_,5);
  Serial.print(",");
  Serial.println(link[END].q_,5);

  float angle[LINK_NUM];
  getLinkAngle(angle);
}

/*******************************************************************************
* Get Data From Processing
*******************************************************************************/
void getDataFromProcessing(bool &comm)
{
  String simulator = "";

  if (Serial.available())
  {
    simulator = Serial.readString();
    Serial.println(simulator);

    if (simulator == "ready")
    {
      initMotorTorque(true);
      getDynamixelPosition();

      comm = true;
    }
    else if (simulator == "stop")
    {
      initMotorTorque(false);

      comm = false;
    }
    else if (simulator == "start")
    {
      start_prop[BASE].pos = 0.0;
      start_prop[BASE].vel = 0.0;
      start_prop[BASE].acc = 0.0;

      end_prop[BASE].pos   = 0.0 * DEG2RAD;
      end_prop[BASE].vel   = 0.0 * DEG2RAD;
      end_prop[BASE].acc   = 0.0 * DEG2RAD;

      start_prop[JOINT1].pos = motor[JOINT1].present_position;
      start_prop[JOINT1].vel = 0.0;
      start_prop[JOINT1].acc = 0.0;

      end_prop[JOINT1].pos   = 0.0 * DEG2RAD;
      end_prop[JOINT1].vel   = 0.0 * DEG2RAD;
      end_prop[JOINT1].acc   = 0.0 * DEG2RAD;

      start_prop[JOINT2].pos = motor[JOINT2].present_position;
      start_prop[JOINT2].vel = 0.0;
      start_prop[JOINT2].acc = 0.0;

      end_prop[JOINT2].pos   = 60.0 * DEG2RAD;
      end_prop[JOINT2].vel   = 0.0 * DEG2RAD;
      end_prop[JOINT2].acc   = 0.0 * DEG2RAD;

      start_prop[JOINT3].pos = motor[JOINT3].present_position;
      start_prop[JOINT3].vel = 0.0;
      start_prop[JOINT3].acc = 0.0;

      end_prop[JOINT3].pos   = -20.0 * DEG2RAD;
      end_prop[JOINT3].vel   = 0.0 * DEG2RAD;
      end_prop[JOINT3].acc   = 0.0 * DEG2RAD;

      start_prop[JOINT4].pos = motor[JOINT4].present_position;
      start_prop[JOINT4].vel = 0.0;
      start_prop[JOINT4].acc = 0.0;

      end_prop[JOINT4].pos   = -40.0 * DEG2RAD;
      end_prop[JOINT4].vel   = 0.0 * DEG2RAD;
      end_prop[JOINT4].acc   = 0.0 * DEG2RAD;

      start_prop[END].pos = motor[END].present_position;
      start_prop[END].vel = 0.0;
      start_prop[END].acc = 0.0;

      end_prop[END].pos   = motor[END].present_position;
      end_prop[END].vel   = 0.0 * DEG2RAD;
      end_prop[END].acc   = 0.0 * DEG2RAD;

      joint_tra = trajectory->minimumJerk(start_prop,
                                          end_prop,
                                          LINK_NUM,
                                          control_period,
                                          mov_time);

      moving = true;
    }
    else
    {
      comm = comm;
    }
  }
}

/*******************************************************************************
* Initialization Timer
*******************************************************************************/
void initTimer()
{
  control_timer.stop();
  control_timer.setPeriod(CONTROL_RATE);
  control_timer.attachInterrupt(handler_control);
  control_timer.start();
}

/*******************************************************************************
* Get Dynamixel Position (rad)
*******************************************************************************/
void getDynamixelPosition()
{
  motor_driver->readPosition(motor);
}

/*******************************************************************************
* Get Link Position (rad)
*******************************************************************************/
void getLinkAngle(float* angle)
{
  for (int num = BASE; num <= END; num++)
  {
    angle[num] = link[num].q_;
    motor[num].present_position = angle[num];
  }
}

void setFK(open_manipulator::Link* link, int8_t me)
{
  kinematics->forward(link, me);
}

/*******************************************************************************
* Inverse Kinematics
*******************************************************************************/
void setIK(open_manipulator::Link* link, uint8_t to, open_manipulator::Pose goal_pose)
{
    // open_manipulator::Pose goal_pose;
    // goal_pose.position    << 0.179, 0.000, 0.201;   // (0, 20, -30, 30, 0)
    // goal_pose.orientation << 0.940, 0.000, -0.342,
    //                           0.000, 1.000, 0.000,
    //                           0.342, 0.000, 0.940;

    // kinematics->sr_inverse(link, END, goal_pose);

    kinematics->sr_inverse(link, to, goal_pose);
}

/*******************************************************************************
* Write Dynamixel Position (rad)
*******************************************************************************/
void setJointDataToDynamixel()
{
  int32_t joint_value[LINK_NUM] = {0, };

  for (int num = BASE; num <= END; num++)
  {
    joint_value[num] = motor_driver->convertRadian2Value(link[num].q_);
  }
  motor_driver->jointControl(joint_value);
}

/*******************************************************************************
* Set Gripper status
*******************************************************************************/
void setGripperDataToDynamixel(bool onoff)
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
void initLinkAndMotor()
{
  link[BASE].name_                      = "Base";
  link[BASE].mother_                    = -1;
  link[BASE].sibling_                   = -1;
  link[BASE].child_                     = 1;
  link[BASE].mass_                      = 1.0;
  link[BASE].p_                         = Eigen::Vector3f::Zero();
  link[BASE].R_                         = Eigen::Matrix3f::Identity(3,3);
  link[BASE].q_                         = 0.0;
  link[BASE].dq_                        = 0.0;
  link[BASE].ddq_                       = 0.0;
  link[BASE].a_                         = Eigen::Vector3f::Zero();
  link[BASE].b_                         = Eigen::Vector3f::Zero();
  link[BASE].v_                         = Eigen::Vector3f::Zero();
  link[BASE].w_                         = Eigen::Vector3f::Zero();

  motor[BASE].name                      = link[BASE].name_;
  motor[BASE].id                        = 0;
  motor[BASE].goal_position             = 0.0;
  motor[BASE].present_position          = 0.0;

  link[JOINT1].name_                    = "Joint1";
  link[JOINT1].mother_                  = 0;
  link[JOINT1].sibling_                 = -1;
  link[JOINT1].child_                   = 2;
  link[JOINT1].mass_                    = 1.0;
  link[JOINT1].p_                       = Eigen::Vector3f::Zero();
  link[JOINT1].R_                       = Eigen::Matrix3f::Identity(3,3);
  link[JOINT1].q_                       = 0.0;
  link[JOINT1].dq_                      = 0.0;
  link[JOINT1].ddq_                     = 0.0;
  link[JOINT1].a_                       << 0, 0, 1;
  link[JOINT1].b_                       << 0.012, 0, 0.036;
  link[JOINT1].v_                       = Eigen::Vector3f::Zero();
  link[JOINT1].w_                       = Eigen::Vector3f::Zero();

  motor[JOINT1].name                    = link[JOINT1].name_;
  motor[JOINT1].id                      = 1;
  motor[JOINT1].goal_position           = 0.0;
  motor[JOINT1].present_position        = 0.0;

  link[JOINT2].name_                    = "Joint2";
  link[JOINT2].mother_                  = 1;
  link[JOINT2].sibling_                 = -1;
  link[JOINT2].child_                   = 3;
  link[JOINT2].mass_                    = 1.0;
  link[JOINT2].p_                       = Eigen::Vector3f::Zero();
  link[JOINT2].R_                       = Eigen::Matrix3f::Identity(3,3);
  link[JOINT2].q_                       = 0.0;
  link[JOINT2].dq_                      = 0.0;
  link[JOINT2].ddq_                     = 0.0;
  link[JOINT2].a_                       << 0, -1, 0;
  link[JOINT2].b_                       << 0, 0, 0.040;
  link[JOINT2].v_                       = Eigen::Vector3f::Zero();
  link[JOINT2].w_                       = Eigen::Vector3f::Zero();

  motor[JOINT2].name                    = link[JOINT2].name_;
  motor[JOINT2].id                      = 2;
  motor[JOINT2].goal_position           = 0.0;
  motor[JOINT2].present_position        = 0.0;


  link[JOINT3].name_                    = "Joint3";
  link[JOINT3].mother_                  = 2;
  link[JOINT3].sibling_                 = -1;
  link[JOINT3].child_                   = 4;
  link[JOINT3].mass_                    = 1.0;
  link[JOINT3].p_                       = Eigen::Vector3f::Zero();
  link[JOINT3].R_                       = Eigen::Matrix3f::Identity(3,3);
  link[JOINT3].q_                       = 0.0;
  link[JOINT3].dq_                      = 0.0;
  link[JOINT3].ddq_                     = 0.0;
  link[JOINT3].a_                       << 0, -1, 0;
  link[JOINT3].b_                       << 0.022, 0, 0.122;
  link[JOINT3].v_                       = Eigen::Vector3f::Zero();
  link[JOINT3].w_                       = Eigen::Vector3f::Zero();

  motor[JOINT3].name                    = link[JOINT3].name_;
  motor[JOINT3].id                      = 3;
  motor[JOINT3].goal_position           = 0.0;
  motor[JOINT3].present_position        = 0.0;

  link[JOINT4].name_                    = "Joint4";
  link[JOINT4].mother_                  = 3;
  link[JOINT4].sibling_                 = -1;
  link[JOINT4].child_                   = 5;
  link[JOINT4].mass_                    = 1.0;
  link[JOINT4].p_                       = Eigen::Vector3f::Zero();
  link[JOINT4].R_                       = Eigen::Matrix3f::Identity(3,3);
  link[JOINT4].q_                       = 0.0;
  link[JOINT4].dq_                      = 0.0;
  link[JOINT4].ddq_                     = 0.0;
  link[JOINT4].a_                       << 0, -1, 0;
  link[JOINT4].b_                       << 0.124, 0, 0;
  link[JOINT4].v_                       = Eigen::Vector3f::Zero();
  link[JOINT4].w_                       = Eigen::Vector3f::Zero();

  motor[JOINT4].name                    = link[JOINT4].name_;
  motor[JOINT4].id                      = 4;
  motor[JOINT4].goal_position           = 0.0;
  motor[JOINT4].present_position        = 0.0;

  link[END].name_                       = "Gripper";
  link[END].mother_                     = 4;
  link[END].sibling_                    = -1;
  link[END].child_                      = -1;
  link[END].mass_                       = 1.0;
  link[END].p_                          = Eigen::Vector3f::Zero();
  link[END].R_                          = Eigen::Matrix3f::Identity(3,3);
  link[END].q_                          = 0.0;
  link[END].dq_                         = 0.0;
  link[END].ddq_                        = 0.0;
  link[END].a_                          = Eigen::Vector3f::Zero();
  link[END].b_                          << 0.070, 0, 0;
  link[END].v_                          = Eigen::Vector3f::Zero();
  link[END].w_                          = Eigen::Vector3f::Zero();

  motor[END].name                       = link[END].name_;
  motor[END].id                         = 5;
  motor[END].goal_position              = 0.0;
  motor[END].present_position           = 0.0;
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
* Initialization Motor Driver Library
*******************************************************************************/
void initMotorDriver(bool torque)
{
  motor_driver = new open_manipulator::MotorDriver(PROTOCOL_VERSION, BAUE_RATE);

  if (motor_driver->init(motor, JOINT_NUM+GRIP_NUM))
    initMotorTorque(torque);
  else
    return;
}

void initMotorTorque(bool onoff)
{
  motor_driver->setTorque(onoff);
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
