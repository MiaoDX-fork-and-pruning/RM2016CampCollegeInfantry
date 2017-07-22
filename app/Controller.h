/**
 * Copyright (c) 2011-2016, Mobangjack Ī��� (mobangjack@foxmail.com).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <stdint.h>
#include "mecanum.h"
#include "pid.h"

#define CHASSIS_POSITION_PID_DEFAULT PID_CREATE(0.4,0.000020,4.0,1024,1024,1024,512)
#define CHASSIS_SPEED_PID_DEFAULT 	 PID_CREATE(20.0,0.2,4.2,4096,4096,4096,4096)
#define GIMBALS_POSITION_PID_DEFAULT PID_CREATE(2,0,0,5,0,0,5)
#define GIMBALS_SPEED_PID_DEFAULT 	 PID_CREATE(300,0,0,4950,0,0,4950)

#define CHASSIS_RAMP_DEFAULT RAMP(100, RAMP_DIR_UP)
#define GIMBALS_RAMP_DEFAULT RAMP(100, RAMP_DIR_UP)

#define CHASSIS_RAMP_LONGER RAMP(420, RAMP_DIR_UP)

typedef enum
{
	WORKING_STATE_PREPARE = 0x00,
	WORKING_STATE_NORMAL = 0x01,
	WORKING_STATE_STOP = 0x03,
}WorkingState;

typedef struct
{
	int16_t m1;
	int16_t m2;
	int16_t m3;
	int16_t m4;
}ChassisMotorCurrent;

typedef struct
{
	int16_t yaw;
	int16_t pit;
}GimbalsMotorCurrent;

void ControlTask(void);
void PID_ResetAll(void);
void Controller_Reset(void);

extern WorkingState workingState;
extern WorkingState lastWorkingState;

extern Mecanum mecanumPosition;
extern Mecanum mecanumSpeed;
extern Mecanum mecanumCurrent;
extern ChassisMotorCurrent chassisMotorCurrent;
extern GimbalsMotorCurrent gimbalsMotorCurrent;


extern PID chassisSpeedPid1;
extern PID chassisSpeedPid2;
extern PID chassisSpeedPid3;
extern PID chassisSpeedPid4;

extern PID chassisPositionPid1;
extern PID chassisPositionPid2;
extern PID chassisPositionPid3;
extern PID chassisPositionPid4;


#endif

