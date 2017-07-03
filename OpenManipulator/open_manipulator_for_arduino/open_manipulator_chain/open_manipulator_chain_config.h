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

#ifndef OPEN_MANIPULATOR_CHAIN_CONFIG_H_
#define OPEN_MANIPULATOR_CHAIN_CONFIG_H_

#include "kinematics.h"
#include "trajectory.h"
#include "motor_driver.h"
#include "debug.h"

#define CONTROL_RATE        8000
#define SERIAL_RATE         57600
#define BAUE_RATE           1000000

#define PROTOCOL_VERSION    2.0

#define JOINT_NUM           4
#define GRIP_NUM            1
#define LINK_NUM            6

#define BASE    0
#define JOINT1  1
#define JOINT2  2
#define JOINT3  3
#define JOINT4  4
#define END     5

const float grip_on  = 1.3;
const float grip_off = 0.0;

float mov_time       = 2.0;
const float control_period = 0.008;

bool moving = false;
bool comm   = false;

Eigen::MatrixXf joint_tra;

HardwareTimer timer(TIMER_CH1);

open_manipulator::Motor        motor[JOINT_NUM+GRIP_NUM];
open_manipulator::Link         link[LINK_NUM];
open_manipulator::Kinematics*  kinematics;
open_manipulator::MotorDriver* motor_driver;
open_manipulator::Property     start_prop[JOINT_NUM];
open_manipulator::Property     end_prop[JOINT_NUM];
open_manipulator::Trajectory*  trajectory;

void sendJointDataToProcessing(bool onoff);
void getDataFromProcessing(bool &comm);
void initTimer();
void setTimer(bool onoff);
void getDynamixelPosition();
void getLinkAngle(float* angle, uint8_t from, uint8_t to);
void writeDynamixelPosition(float* angle);
void setFK(float* angle);
void setGripper(bool onoff);
void initLink();
void initKinematics();
void initTrajectory();
void initMotor();
void initMotorDriver(bool torque);
void establishContactToProcessing();

#endif // OPEN_MANIPULATOR_CHAIN_CONFIG_H_
