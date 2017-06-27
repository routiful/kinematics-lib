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

#include "motor_driver.h"
using namespace open_manipulator;

MotorDriver::MotorDriver(int8_t motor_num, float protocol_version, uint32_t baud_rate)
{
  motor_num_        = motor_num;
  joint_num_        = motor_num_ - 1;
  grip_num_         = motor_num_ - joint_num_;
  grip_id_          = motor_num;

  protocol_version_ = protocol_version;
  baud_rate_        = baud_rate;
}

MotorDriver::~MotorDriver()
{
  close();
}

bool MotorDriver::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

  // Open port
  if (portHandler_->openPort())
  {
    ERROR_PRINT("Port is opened");
  }
  else
  {
    ERROR_PRINT("Port couldn't be opened");

    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baud_rate_))
  {
    ERROR_PRINT("Baudrate is set");
  }
  else
  {
    ERROR_PRINT("Baudrate couldn't be set");

    return false;
  }


  groupSyncWriteTorque_   = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_TORQUE_ENABLE,    LEN_X_TORQUE_ENABLE);
  groupSyncWritePosition_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_POSITION,    LEN_X_GOAL_POSITION);

  groupSyncReadPosition_  = new dynamixel::GroupSyncRead (portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  // Enable Dynamixel Torque
  setTorque(true);

  return true;
}


void MotorDriver::close(void)
{
  // Disable Dynamixel Torque
  setTorque(false);

  // Close port
  portHandler_->closePort();
}

bool MotorDriver::setTorque(uint8_t onoff)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  uint8_t param_torque[4];

  for (int id = 0; id < motor_num_; id++)
  {
    param_torque[0] = DXL_LOBYTE(DXL_LOWORD(onoff));
    param_torque[1] = DXL_HIBYTE(DXL_LOWORD(onoff));
    param_torque[2] = DXL_LOBYTE(DXL_HIWORD(onoff));
    param_torque[3] = DXL_HIBYTE(DXL_HIWORD(onoff));

    dxl_addparam_result_ = groupSyncWriteTorque_->addParam(id, (uint8_t*)&param_torque);
    if (dxl_addparam_result_ != true)
      return false;
  }

  dxl_comm_result_ = groupSyncWriteTorque_->txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWriteTorque_->clearParam();
  return true;
}

bool MotorDriver::jointControl(uint32_t* value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  uint8_t param_goal_position[4];

  for (int id = 0; id < joint_num_; id++)
  {
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(value[id]));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(value[id]));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(value[id]));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(value[id]));

    dxl_addparam_result_ = groupSyncWritePosition_->addParam(id, (uint8_t*)&param_goal_position);
    if (dxl_addparam_result_ != true)
      return false;
  }

  dxl_comm_result_ = groupSyncWritePosition_->txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWritePosition_->clearParam();
  return true;
}

bool MotorDriver::gripControl(uint32_t* value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  uint8_t param_goal_position[4];

  for (int id = 0; id < grip_num_; id++)
  {
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(value[id]));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(value[id]));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(value[id]));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(value[id]));

    dxl_addparam_result_ = groupSyncWritePosition_->addParam(grip_id_, (uint8_t*)&param_goal_position);
    if (dxl_addparam_result_ != true)
      return false;
  }

  dxl_comm_result_ = groupSyncWritePosition_->txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWritePosition_->clearParam();
  return true;
}

uint32_t* MotorDriver::readPosition()
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  for (int id = 1; id <= motor_num_; id++)
  {
    // Set parameter
    dxl_addparam_result = groupSyncReadPosition_->addParam(id);
    if (dxl_addparam_result != true)
      return false;
  }

  // Syncread present position
  dxl_comm_result = groupSyncReadPosition_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    packetHandler_->printTxRxResult(dxl_comm_result);

  for (int id = 1; id <= motor_num_; id++)
  {
    // Check if groupSyncRead data of Dynamixels are available
    dxl_getdata_result = groupSyncReadPosition_->isAvailable(id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true)
      return false;

    // Get data
    present_position_value[id-1] = groupSyncReadPosition_->getData(id,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  }

  groupSyncReadPosition_->clearParam();
  return present_position_value;
}

uint32_t* MotorDriver::convertRadian2Value(float* radian)
{
  for (int id = 1; id <= motor_num_; id++)
  {
    if (radian[id-1] > 0)
    {
      if (VALUE_OF_MAX_RADIAN_POSITION <= VALUE_OF_ZERO_RADIAN_POSITION)
        present_position_value[id-1] =  VALUE_OF_MAX_RADIAN_POSITION;

      present_position_value[id-1] = (radian[id-1] * (VALUE_OF_MAX_RADIAN_POSITION - VALUE_OF_ZERO_RADIAN_POSITION) / MAX_RADIAN)
                  + VALUE_OF_ZERO_RADIAN_POSITION;
    }
    else if (radian[id-1] < 0)
    {
      if (VALUE_OF_MIN_RADIAN_POSITION >= VALUE_OF_ZERO_RADIAN_POSITION)
        present_position_value[id-1] =  VALUE_OF_MIN_RADIAN_POSITION;
      present_position_value[id-1] = (radian[id-1] * (VALUE_OF_MIN_RADIAN_POSITION - VALUE_OF_ZERO_RADIAN_POSITION) / MIN_RADIAN)
                  + VALUE_OF_ZERO_RADIAN_POSITION;
    }
    else
    {
      present_position_value[id-1] = VALUE_OF_ZERO_RADIAN_POSITION;
    }

    // if (present_position_value[id-1] > VALUE_OF_MAX_RADIAN_POSITION)
    //   present_position_value[id-1] =  VALUE_OF_MAX_RADIAN_POSITION;
    // else if (present_position_value[id-1] < VALUE_OF_MIN_RADIAN_POSITION)
    //   present_position_value[id-1] =  VALUE_OF_MIN_RADIAN_POSITION;
  }

  return present_position_value;
}

float* MotorDriver::convertValue2Radian(uint32_t* value)
{
  for (int id = 1; id <= motor_num_; id++)
  {
    if (value[id-1] > VALUE_OF_ZERO_RADIAN_POSITION)
    {
      if (MAX_RADIAN <= 0)
        present_position_radian[id-1] =  MAX_RADIAN;

      present_position_radian[id-1] = (float) (value[id-1] - VALUE_OF_ZERO_RADIAN_POSITION) * MAX_RADIAN
                 / (float) (VALUE_OF_MAX_RADIAN_POSITION - VALUE_OF_ZERO_RADIAN_POSITION);
    }
    else if (value[id-1] < VALUE_OF_ZERO_RADIAN_POSITION)
    {
      if (MIN_RADIAN >= 0)
        present_position_radian[id-1] =  MIN_RADIAN;

      present_position_radian[id-1] = (float) (value[id-1] - VALUE_OF_ZERO_RADIAN_POSITION) * MIN_RADIAN
                 / (float) (VALUE_OF_MIN_RADIAN_POSITION - VALUE_OF_ZERO_RADIAN_POSITION);
    }

  //  if (present_position_radian[id-1] > MAX_RADIAN)
  //    present_position_radian[id-1] =  MAX_RADIAN;
  //  else if (present_position_radian[id-1] < MIN_RADIAN)
  //    present_position_radian[id-1] =  MIN_RADIAN;
  }

  return present_position_radian;
}
