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
 
#include "main.h"

#define USE_Mecanum_Synthesis 0
#define ControllerDEBUG	1
#define ControllerDEBUGNUM 800
uint32_t debug_tick = 0;
/*
#if ControllerDEBUG
#endif	
*/
WorkingState workingState = WORKING_STATE_PREPARE;
WorkingState lastWorkingState = WORKING_STATE_PREPARE;

Mecanum mecanumPosition;
Mecanum mecanumSpeed;
Mecanum mecanumCurrent;

PID chassisPositionXPid = CHASSIS_POSITION_PID_DEFAULT;
PID chassisPositionYPid = CHASSIS_POSITION_PID_DEFAULT;
PID chassisPositionZPid = CHASSIS_POSITION_PID_DEFAULT;
PID chassisSpeedXPid = CHASSIS_SPEED_PID_DEFAULT;
PID chassisSpeedYPid = CHASSIS_SPEED_PID_DEFAULT;
PID chassisSpeedZPid = CHASSIS_SPEED_PID_DEFAULT;


PID chassisPositionPid1 = CHASSIS_POSITION_PID_DEFAULT;
PID chassisPositionPid2 = CHASSIS_POSITION_PID_DEFAULT;
PID chassisPositionPid3 = CHASSIS_POSITION_PID_DEFAULT;
PID chassisPositionPid4 = CHASSIS_POSITION_PID_DEFAULT;
PID chassisSpeedPid1 = CHASSIS_SPEED_PID_DEFAULT;
PID chassisSpeedPid2 = CHASSIS_SPEED_PID_DEFAULT;
PID chassisSpeedPid3 = CHASSIS_SPEED_PID_DEFAULT;
PID chassisSpeedPid4 = CHASSIS_SPEED_PID_DEFAULT;


PID gimbalsPositionYawPID = GIMBALS_POSITION_PID_DEFAULT;
PID gimbalsPositionPitchPID = GIMBALS_POSITION_PID_DEFAULT;
PID gimbalsSpeedYawPID = GIMBALS_SPEED_PID_DEFAULT;
PID gimbalsSpeedPitchPID = GIMBALS_SPEED_PID_DEFAULT;

Ramp chassisRamp = CHASSIS_RAMP_DEFAULT;
Ramp gimbalsRamp = GIMBALS_RAMP_DEFAULT;

ChassisMotorCurrent chassisMotorCurrent;
GimbalsMotorCurrent gimbalsMotorCurrent;


int ALL_Encoder_IsOk()
{
	/*
	if(Encoder_IsOk(&CM1Encoder) && Encoder_IsOk(&CM2Encoder) && Encoder_IsOk(&CM3Encoder) && Encoder_IsOk(&CM4Encoder))
	{
		return 1;
	}
	*/
	if(Encoder_IsOk(&CM3Encoder) && Encoder_IsOk(&CM4Encoder))
	{
		return 1;
	}
	
	return 0;
}

void WorkingStateSM(void)
{
	lastWorkingState = workingState;
	if(inputMode == INPUT_MODE_NO)
	{
		workingState = WORKING_STATE_STOP;
		return;
	}
	switch(workingState)
	{
		case WORKING_STATE_PREPARE:
		{
			if(ALL_Encoder_IsOk())
			{
				workingState = WORKING_STATE_NORMAL;
			}
		}break;
		case WORKING_STATE_NORMAL:
		{
			if(ALL_Encoder_IsOk() == 0)
			{
				workingState = WORKING_STATE_PREPARE;
			}
		}break;
		case WORKING_STATE_STOP:
		{
			if(inputMode != INPUT_MODE_NO)
			{
				workingState = WORKING_STATE_PREPARE;
			}
		}break;
		default:
		{
			workingState = WORKING_STATE_STOP;
		}break;
	}
}

void ChassisPositionControl(void)
{
	/* Mecanum Position Synthesis */
	mecanumPosition.w1 = CM1Encoder.rad;
	mecanumPosition.w2 = CM2Encoder.rad;
	mecanumPosition.w3 = CM3Encoder.rad;
	mecanumPosition.w4 = CM4Encoder.rad;
	
#if USE_Mecanum_Synthesis
	
	Mecanum_Synthesis(&mecanumPosition);
	
	chassisPositionXPid.ref = chassisPositionTarget.x;
	chassisPositionYPid.ref = chassisPositionTarget.y;
	chassisPositionZPid.ref = chassisPositionTarget.z;
	
	chassisPositionXPid.fdb = mecanumPosition.x;
	chassisPositionYPid.fdb = mecanumPosition.y;
	chassisPositionZPid.fdb = mecanumPosition.z;
	
	chassisPositionXPid.Calc(&chassisPositionXPid);
	chassisPositionYPid.Calc(&chassisPositionYPid);
	chassisPositionZPid.Calc(&chassisPositionZPid);
	
	chassisSpeedTarget.x = chassisPositionXPid.output;
	chassisSpeedTarget.y = chassisPositionYPid.output;
	chassisSpeedTarget.z = chassisPositionZPid.output;

#else
	
	Mecanum_Decompose(&chassisPositionTarget); // Decompose to four wheels
	
	chassisPositionPid1.ref = chassisPositionTarget.w1;
	chassisPositionPid2.ref = chassisPositionTarget.w2;
	chassisPositionPid3.ref = chassisPositionTarget.w3;
	chassisPositionPid4.ref = chassisPositionTarget.w4;
	
	chassisPositionPid1.fdb = mecanumPosition.w1;
	chassisPositionPid2.fdb = mecanumPosition.w2;
	chassisPositionPid3.fdb = mecanumPosition.w3;
	chassisPositionPid4.fdb = mecanumPosition.w4;
	
	chassisPositionPid1.Calc(&chassisPositionPid1);
	chassisPositionPid2.Calc(&chassisPositionPid2);
	chassisPositionPid3.Calc(&chassisPositionPid3);
	chassisPositionPid4.Calc(&chassisPositionPid4);
	
	chassisSpeedTarget.w1 = chassisPositionPid1.output;
	chassisSpeedTarget.w2 = chassisPositionPid2.output;
	chassisSpeedTarget.w3 = chassisPositionPid3.output;
	chassisSpeedTarget.w4 = chassisPositionPid4.output;
	
#endif


#if ControllerDEBUG	
	if(debug_tick%ControllerDEBUGNUM == 0){
		#if USE_Mecanum_Synthesis // NOTE the dec or syn
			Mecanum_Decompose(&chassisSpeedTarget);
		#else
			Mecanum_Synthesis(&chassisSpeedTarget);
		#endif
		
		
		printf("chassisPositionTarget:\n");
		Mecanum_Debug(&chassisSpeedTarget);
		printf("mecanumPosition:\n");
		Mecanum_Debug(&mecanumSpeed);
		printf("chassisSpeedTarget:\n");
		Mecanum_Debug(&mecanumCurrent);
		
	}
#endif	


}

void ChassisSpeedControl(void)
{
	/* Mecanum Speed Synthesis */
	mecanumSpeed.w1 = CM1Encoder.rad_rate;
	mecanumSpeed.w2 = CM2Encoder.rad_rate;
	mecanumSpeed.w3 = CM3Encoder.rad_rate;
	mecanumSpeed.w4 = CM4Encoder.rad_rate;
		
#if USE_Mecanum_Synthesis
	
	Mecanum_Synthesis(&mecanumSpeed);
	
	chassisSpeedXPid.ref = chassisSpeedTarget.x;
	chassisSpeedYPid.ref = chassisSpeedTarget.y;
	chassisSpeedZPid.ref = chassisSpeedTarget.z;
	
	chassisSpeedXPid.fdb = mecanumSpeed.x;
	chassisSpeedYPid.fdb = mecanumSpeed.y;
	chassisSpeedZPid.fdb = mecanumSpeed.z;
	
	chassisSpeedXPid.Calc(&chassisSpeedXPid);
	chassisSpeedYPid.Calc(&chassisSpeedYPid);
	chassisSpeedZPid.Calc(&chassisSpeedZPid);
	
	mecanumCurrent.x = chassisSpeedXPid.output;
	mecanumCurrent.y = chassisSpeedYPid.output;
	mecanumCurrent.z = chassisSpeedZPid.output;
	
#else
	
	Mecanum_Decompose(&chassisSpeedTarget); // Decompose to four wheels
	
	chassisSpeedPid1.ref = chassisSpeedTarget.w1;
	chassisSpeedPid2.ref = chassisSpeedTarget.w2;
	chassisSpeedPid3.ref = chassisSpeedTarget.w3;
	chassisSpeedPid4.ref = chassisSpeedTarget.w4;
	
	chassisSpeedPid1.fdb = mecanumSpeed.w1;
	chassisSpeedPid2.fdb = mecanumSpeed.w2;
	chassisSpeedPid3.fdb = mecanumSpeed.w3;
	chassisSpeedPid4.fdb = mecanumSpeed.w4;
	
	chassisSpeedPid1.Calc(&chassisSpeedPid1);
	chassisSpeedPid2.Calc(&chassisSpeedPid2);
	chassisSpeedPid3.Calc(&chassisSpeedPid3);
	chassisSpeedPid4.Calc(&chassisSpeedPid4);
	
	mecanumCurrent.w1 = chassisSpeedPid1.output;
	mecanumCurrent.w2 = chassisSpeedPid2.output;
	mecanumCurrent.w3 = chassisSpeedPid3.output;
	mecanumCurrent.w4 = chassisSpeedPid4.output;

#endif


#if ControllerDEBUG	
	if(debug_tick%ControllerDEBUGNUM == 0){
		#if USE_Mecanum_Synthesis // NOTE the dec or syn
			Mecanum_Decompose(&mecanumCurrent);
		#else
			Mecanum_Synthesis(&mecanumCurrent);
		#endif

		printf("chassisSpeedTarget:\n");
		Mecanum_Debug(&chassisSpeedTarget);
		printf("mecanumSpeed:\n");
		Mecanum_Debug(&mecanumSpeed);
		printf("mecanumCurrent:\n");
		Mecanum_Debug(&mecanumCurrent);

	}
#endif	
	


}

void ChassisCurrentControl(void)
{

#if USE_Mecanum_Synthesis
	Mecanum_Decompose(&mecanumCurrent);
#endif
	
	/*
	chassisRamp.Calc(&chassisRamp);
	
	chassisMotorCurrent.m1 = mecanumCurrent.w1 * chassisRamp.output;
	chassisMotorCurrent.m2 = mecanumCurrent.w2 * chassisRamp.output;
	chassisMotorCurrent.m3 = mecanumCurrent.w3 * chassisRamp.output;
	chassisMotorCurrent.m4 = mecanumCurrent.w4 * chassisRamp.output;
	*/
	
	chassisMotorCurrent.m1 = mecanumCurrent.w1;
	chassisMotorCurrent.m2 = mecanumCurrent.w2;
	chassisMotorCurrent.m3 = mecanumCurrent.w3;
	chassisMotorCurrent.m4 = mecanumCurrent.w4;
	
	
	//printf("CCX:%d", chassisMotorCurrent.m1);
}

void GimbalsPositionControl(void)
{
	gimbalsPositionYawPID.ref = gimbalsPosition.yaw;
	gimbalsPositionPitchPID.ref = gimbalsPosition.pit;
	
	gimbalsPositionYawPID.fdb = GMYEncoder.rate;
	gimbalsPositionPitchPID.fdb = GMPEncoder.rate;
	
	gimbalsPositionYawPID.Calc(&gimbalsPositionYawPID);
	gimbalsPositionPitchPID.Calc(&gimbalsPositionPitchPID);
	
	gimbalsSpeed.yaw = gimbalsPositionYawPID.output;
	gimbalsSpeed.pit = gimbalsPositionPitchPID.output;
}

void GimbalsSpeedControl(void)
{
	gimbalsSpeedYawPID.ref = gimbalsSpeed.yaw;
	gimbalsSpeedPitchPID.ref = gimbalsSpeed.pit;
	
	gimbalsSpeedYawPID.fdb = GMYEncoder.rate;
	gimbalsSpeedPitchPID.fdb = GMPEncoder.rate;
	
	gimbalsSpeedYawPID.Calc(&gimbalsSpeedYawPID);
	gimbalsSpeedPitchPID.Calc(&gimbalsSpeedPitchPID);
	
	gimbalsCurrent.yaw = gimbalsSpeedYawPID.output;
	gimbalsCurrent.pit = gimbalsSpeedPitchPID.output;
}

void GimbalsCurrentControl(void)
{
	gimbalsRamp.Calc(&gimbalsRamp);
	gimbalsMotorCurrent.yaw = gimbalsCurrent.yaw * gimbalsRamp.output;
	gimbalsMotorCurrent.pit = gimbalsCurrent.pit * gimbalsRamp.output;
}

void PID_ResetAll(void)
{
	PID_Reset(&chassisPositionXPid);
	PID_Reset(&chassisPositionYPid);
	PID_Reset(&chassisPositionZPid);
	PID_Reset(&gimbalsPositionYawPID);
	PID_Reset(&gimbalsPositionPitchPID);
	
	PID_Reset(&chassisSpeedXPid);
	PID_Reset(&chassisSpeedYPid);
	PID_Reset(&chassisSpeedZPid);
	PID_Reset(&gimbalsSpeedYawPID);
	PID_Reset(&gimbalsSpeedPitchPID);
}

void Encoder_ResetAll(void)
{
	Encoder_Reset(&CM1Encoder);
	Encoder_Reset(&CM2Encoder);
	Encoder_Reset(&CM3Encoder);
	Encoder_Reset(&CM4Encoder);
	Encoder_Reset(&GMYEncoder);
	Encoder_Reset(&GMPEncoder);
}

void Ramp_ResetAll(void)
{
	Ramp_ResetCounter(&chassisRamp);
	Ramp_ResetCounter(&gimbalsRamp);
}

void Controller_Reset(void)
{
	Encoder_ResetAll();
	Ramp_ResetAll();
	PID_ResetAll();
}

void ChassisMotorCurrentTransmit(void)
{
	SetCMCurrent(CAN1, chassisMotorCurrent.m1, chassisMotorCurrent.m2, chassisMotorCurrent.m3, chassisMotorCurrent.m4);
}

void GimbalsMotorCurrentTransmit(void)
{
	//SetGMCurrent(CAN1, gimbalsMotorCurrent.yaw, gimbalsMotorCurrent.pit);
}

static uint32_t ms_tick = 0;
void ControlTask(void)
{
	ms_tick++;
	debug_tick++;
	
	WorkingStateSM();
	if(lastWorkingState == WORKING_STATE_STOP && workingState == WORKING_STATE_PREPARE)
	{
		printf("Going to reset the variables and prepare\n\n");
		ms_tick = 0;		
		debug_tick = 0;
		Controller_Reset();
	}
	else if(workingState == WORKING_STATE_NORMAL)
	{
		//printf("run");
		if(lastCtrlMode != ctrlMode)
		{
			debug_tick = 0;
			//Ramp_ResetAll();
			//PID_ResetAll();
		}

#if ControllerDEBUG		
		if(ms_tick % 420 == 0)
		{
			printf("ctrlMode:%d\n", ctrlMode);
		}
#endif		
		
		switch(ctrlMode)
		{
			
			case CTRL_MODE_POSITION:
			{
				if(ms_tick % 4 == 0)
				{
					ChassisPositionControl();
					ChassisSpeedControl();
					ChassisCurrentControl();
					GimbalsPositionControl();
					GimbalsSpeedControl();
					GimbalsCurrentControl();
					ChassisMotorCurrentTransmit();
					GimbalsMotorCurrentTransmit();
				}
			}break;
			case CTRL_MODE_SPEED:
			{
				if(ms_tick % 4  == 0)
				{
					ChassisSpeedControl();
					ChassisCurrentControl();
					GimbalsSpeedControl();
					GimbalsCurrentControl();
					ChassisMotorCurrentTransmit();
					GimbalsMotorCurrentTransmit();
				}
			}break;
			case CTRL_MODE_PROGRAM: // Set position by program
			{
				if(ms_tick % 4 == 0)
				{
					ChassisPositionControl();
					ChassisSpeedControl();
					ChassisCurrentControl();
					GimbalsPositionControl();
					GimbalsSpeedControl();
					GimbalsCurrentControl();
					ChassisMotorCurrentTransmit();
					GimbalsMotorCurrentTransmit();
				}
			}break;
			/*
			case CTRL_MODE_CURRENT:
			{
				if(ms_tick % 4  == 0)
				{
					ChassisCurrentControl();
					GimbalsCurrentControl();
					ChassisMotorCurrentTransmit();
					GimbalsMotorCurrentTransmit();
				}
			}break;
			*/
			default:
			{
			}break;
		}
	}
	
}

