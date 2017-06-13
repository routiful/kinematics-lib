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

#ifndef OPEN_MANIPULATOR_JOINT_DRIVER_H_
#define OPEN_MANIPULATOR_JOINT_DRIVER_H_

#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series)
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_POSITION         132

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_POSITION             4
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_POSITION          4

#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define VALUE_OF_MAX_RADIAN_POSITION    4095
#define VALUE_OF_MIN_RADIAN_POSITION    0
#define VALUE_OF_ZERO_RADIAN_POSITION   2048

#define MIN_RADIAN                     -3.14
#define MAX_RADIAN                      3.14

namespace open_manipulator
{
class MotorDriver
{
 public:
  MotorDriver(int8_t motor_num, float protocol_version, uint32_t baud_rate);
  ~MotorDriver();

  bool init(void);
  void closeDynamixel(void);
  bool setTorque(bool onoff);

  uint16_t* readPosition();
  bool motorControl(uint32_t *set_joint_value);

  uint16_t* convertRadian2Value(float* radian);
  float* convertValue2Radian(uint32_t* value);

 private:
  int8_t motor_num_;
  uint32_t baud_rate_;
  float  protocol_version_;

  uint16_t present_position_value[];
  float present_position_radian[];

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWritePosition_;
  dynamixel::GroupSyncRead *groupSyncReadEncoder_;
};
}

#endif // OPEN_MANIPULATOR_JOINT_DRIVER_H_
