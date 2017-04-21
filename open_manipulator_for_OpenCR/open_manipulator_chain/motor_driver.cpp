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
  protocol_version_ = protocol_version;
  baud_rate_        = baud_rate;
}

MotorDriver::~MotorDriver()
{
  closeDynamixel();
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

  // Enable Dynamixel Torque
  setTorque(true);

  groupSyncWritePosition_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION);
  groupSyncReadEncoder_   = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  return true;
}

bool MotorDriver::setTorque(bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  for (int id = 1; id <= motor_num_; id++)
  {
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler_->printTxRxResult(dxl_comm_result);
    }
    else if(dxl_error != 0)
    {
      packetHandler_->printRxPacketError(dxl_error);
    }
  }
}

void MotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(false);

  // Close port
  portHandler_->closePort();
}

uint16_t* MotorDriver::readPosition()
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  for (int id = 1; id <= motor_num_; id++)
  {
    // Set parameter
    dxl_addparam_result = groupSyncReadEncoder_->addParam(id);
    if (dxl_addparam_result != true)
      return false;
  }

  // Syncread present position
  dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    packetHandler_->printTxRxResult(dxl_comm_result);

  for (int id = 1; id <= motor_num_; id++)
  {
    // Check if groupSyncRead data of Dynamixels are available
    dxl_getdata_result = groupSyncReadEncoder_->isAvailable(id, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true)
      return false;

    // Get data
    present_position_value[id-1] = groupSyncReadEncoder_->getData(id,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  }

  groupSyncReadEncoder_->clearParam();
  return present_position_value;
}

bool MotorDriver::motorControl(uint16_t* set_joint_value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  uint8_t param_goal_position[4];

  for (int id = 1; id <= motor_num_; id++)
  {
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(set_joint_value[id-1]));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(set_joint_value[id-1]));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(set_joint_value[id-1]));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(set_joint_value[id-1]));

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

uint16_t* MotorDriver::convertRadian2Value(float* radian)
{
  for (int id = 1; id <= motor_num_; id++)
  {
    if (radian[id-1] > 0)
    {
      if (VALUE_OF_MAX_RADIAN_POSITION <= VALUE_OF_ZERO_RADIAN_POSITION)
      {
        present_position_value[id-1] =  VALUE_OF_MAX_RADIAN_POSITION;
      }

      present_position_value[id-1] = (radian[id-1] * (VALUE_OF_MAX_RADIAN_POSITION - VALUE_OF_ZERO_RADIAN_POSITION) / MAX_RADIAN)
                  + VALUE_OF_ZERO_RADIAN_POSITION;
    }
    else if (radian[id-1] < 0)
    {
      if (VALUE_OF_MIN_RADIAN_POSITION >= VALUE_OF_ZERO_RADIAN_POSITION)
      {
        present_position_value[id-1] =  VALUE_OF_MIN_RADIAN_POSITION;
      }

      present_position_value[id-1] = (radian[id-1] * (VALUE_OF_MIN_RADIAN_POSITION - VALUE_OF_ZERO_RADIAN_POSITION) / MIN_RADIAN)
                  + VALUE_OF_ZERO_RADIAN_POSITION;
    }
    else
    {
      present_position_value[id-1] = VALUE_OF_ZERO_RADIAN_POSITION;
    }

    if (present_position_value[id-1] > VALUE_OF_MAX_RADIAN_POSITION)
    {
      present_position_value[id-1] =  VALUE_OF_MAX_RADIAN_POSITION;
    }
    else if (present_position_value[id-1] < VALUE_OF_MIN_RADIAN_POSITION)
    {
      present_position_value[id-1] =  VALUE_OF_MIN_RADIAN_POSITION;
    }
  }

  return present_position_value;
}

float* MotorDriver::convertValue2Radian(uint16_t* value)
{
  for (int id = 1; id <= motor_num_; id++)
  {
    if (value[id-1] > VALUE_OF_ZERO_RADIAN_POSITION)
    {
      if (MAX_RADIAN <= 0)
      {
        present_position_radian[id-1] =  MAX_RADIAN;
      }

      present_position_radian[id-1] = (double) (value[id-1] - VALUE_OF_ZERO_RADIAN_POSITION) * MAX_RADIAN
                 / (double) (VALUE_OF_MAX_RADIAN_POSITION - VALUE_OF_ZERO_RADIAN_POSITION);
    }
    else if (value[id-1] < VALUE_OF_ZERO_RADIAN_POSITION)
    {
      if (MIN_RADIAN >= 0)
      {
        present_position_radian[id-1] =  MIN_RADIAN;
      }

      present_position_radian[id-1] = (double) (value[id-1] - VALUE_OF_ZERO_RADIAN_POSITION) * MIN_RADIAN
                 / (double) (VALUE_OF_MIN_RADIAN_POSITION - VALUE_OF_ZERO_RADIAN_POSITION);
    }

  //  if (present_position_radian[id-1] > MAX_RADIAN)
  //    present_position_radian[id-1] =  MAX_RADIAN;
  //  else if (present_position_radian[id-1] < MIN_RADIAN)
  //    present_position_radian[id-1] =  MIN_RADIAN;
  }

  return present_position_radian;
}
