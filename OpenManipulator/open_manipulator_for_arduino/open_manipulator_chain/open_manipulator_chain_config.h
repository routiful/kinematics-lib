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
#include "motor_driver.h"
#include "debug.h"

#define COMMUNICATION_RATE  300
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
#define GRIPPER 5

static float grip_on  = -10.0;
static float grip_off = -45.0;

float joint_angle[6] = {0.0, 0.0, 0.0*DEG2RAD, 0.0*DEG2RAD, 0.0*DEG2RAD, 0.0};
float gripper_pos    = GRIPPER_OFF;

open_manipulator::Link         link[LINK_NUM];
open_manipulator::Kinematics*  kinematics;
open_manipulator::MotorDriver* motor;

#endif // OPEN_MANIPULATOR_CHAIN_CONFIG_H_
