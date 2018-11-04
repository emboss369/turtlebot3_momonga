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

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#ifndef TURTLEBOT3_WAFFLE_H_
#define TURTLEBOT3_WAFFLE_H_

#define NAME                             "Waffle or Waffle Pi"

#define WHEEL_RADIUS                     0.0595          // meter
#define WHEEL_SEPARATION                 0.290           // meter (BURGER : 0.160, WAFFLE : 0.287)
#define TURNING_RADIUS                   0.145           // meter (BURGER : 0.080, WAFFLE : 0.1435)
#define ROBOT_RADIUS                     0.220           // meter (BURGER : 0.105, WAFFLE : 0.220)
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define MAX_LINEAR_VELOCITY              0.50  // m/s   (BURGER : 0.22, WAFFLE : 0.25)
#define MAX_ANGULAR_VELOCITY             1.82   // rad/s (BURGER : 2.84, WAFFLE : 1.82)

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY

#endif  //TURTLEBOT3_WAFFLE_H_
