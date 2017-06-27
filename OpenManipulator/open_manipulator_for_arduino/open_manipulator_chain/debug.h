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

#ifndef DEBUG_H_
#define DEBUG_H_

#include <Arduino.h>
#include <Eigen.h>

void showLedStatus();
void updateRxTxLed();
void print_mt3f(const Eigen::Matrix3f& m);
void print_vt3f(const Eigen::Vector3f& v);

#endif // DEBUG_H_
