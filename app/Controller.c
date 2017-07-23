/**
 * Copyright (c) 2011-2016, Mobangjack Äª°ï½Ü (mobangjack@foxmail.com).
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
#include "InputScaner.h"

#define USE_Mecanum_Synthesis 0 // pros and cons
#define ControllerDEBUG	1
#define ControllerDEBUGNUM 4200
uint32_t ms_tick = 0;
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

void ChassisPositionControl(int with_speed)
{
	
	Mecanum_Reset(&mecanumPosition);
	/* Mecanum Position Synthesis */

	/*

	mecanumPosition.w1 = CM1Encoder.rad;
	mecanumPosition.w2 = CM2Encoder.rad;
	mecanumPosition.w3 = CM3Encoder.rad;
	mecanumPosition.w4 = CM4Encoder.rad;
	
	mecanumPosition.w1 = CM1Encoder.round;
	mecanumPosition.w2 = CM2Encoder.round;
	mecanumPosition.w3 = CM3Encoder.round;
	mecanumPosition.w4 = CM4Encoder.round;	
	
	*/
	mecanumPosition.w1 = CM1Encoder.angle/5.0;
	mecanumPosition.w2 = CM2Encoder.angle/5.0;
	mecanumPosition.w3 = CM3Encoder.angle/5.0;
	mecanumPosition.w4 = CM4Encoder.angle/5.0;
	
	
if(with_speed == 0){
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
	
	
	Mecanum_Reset(&chassisSpeedTarget);		
	chassisSpeedTarget.x = chassisPositionXPid.output;
	chassisSpeedTarget.y = chassisPositionYPid.output;
	chassisSpeedTarget.z = chassisPositionZPid.output;

#else
	
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
}	
	



#if ControllerDEBUG	

/*
	if(debug_tick%20 == 0){
		if(abs(chassisPositionTarget.w3 - mecanumPosition.w3) > 5.0){
			printf("Target:%f,Present:%f\n", chassisPositionTarget.w3, mecanumPosition.w3);
		}
	}
*/



	if(debug_tick%ControllerDEBUGNUM == 0){
		
		/*
		#if USE_Mecanum_Synthesis // NOTE the dec or syn
			Mecanum_Decompose(&chassisSpeedTarget);
		#else
			Mecanum_Synthesis(&chassisSpeedTarget);
		#endif
		*/
		
		if(abs(chassisPositionTarget.w3 - mecanumPosition.w3) > 2.0){
			printf("Target:%f,Present:%f\n", chassisPositionTarget.w3, mecanumPosition.w3);
		}
		
		printf("chassisPositionTarget:\n");
		Mecanum_Debug(&chassisPositionTarget);
		printf("mecanumPosition:\n");
		Mecanum_Debug(&mecanumPosition);
		//printf("chassisSpeedTarget:\n");
		//Mecanum_Debug(&chassisSpeedTarget);
		
		printf("CM3PID:\n");
		printf("P:%f,\tI:%f,\tD:%f,\tOUT:%f\n", chassisPositionPid3.componentKp,chassisPositionPid3.componentKi, chassisPositionPid3.componentKd, chassisPositionPid3.output);
		printf("CM4PID:\n");
		printf("P:%f,\tI:%f,\tD:%f,\tOUT:%f\n", chassisPositionPid4.componentKp,chassisPositionPid4.componentKi, chassisPositionPid4.componentKd, chassisPositionPid4.output);
		
		printf("chassisPositionYPid:\n");		
		printf("P:%f,\tI:%f,\tD:%f,\tOUT:%f\n", chassisPositionYPid.componentKp,chassisPositionYPid.componentKi, chassisPositionYPid.componentKd, chassisPositionYPid.output);
		
	}
#endif	


}

void ChassisSpeedControl(void)
{
	Mecanum_Reset(&mecanumSpeed);
	
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
	
	Mecanum_Reset(&mecanumCurrent);
	
	mecanumCurrent.x = chassisSpeedXPid.output;
	mecanumCurrent.y = chassisSpeedYPid.output;
	mecanumCurrent.z = chassisSpeedZPid.output;
	
#else
	
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

/*
	if(debug_tick%80 == 0){
		if(abs(chassisSpeedTarget.w3 - mecanumSpeed.w3) > 1.0){
			printf("Target:%f,Present:%f\n", chassisSpeedTarget.w3, mecanumSpeed.w3);
		}
	}
*/
	if(debug_tick%ControllerDEBUGNUM == 0){
		
		
		printf("chassisSpeedTarget:\n");
		Mecanum_Debug(&chassisSpeedTarget);
		printf("mecanumSpeed:\n");
		Mecanum_Debug(&mecanumSpeed);
		
		/*
		printf("CM1Encoder:\n");
		Encoder_Debug(&CM1Encoder);
		printf("CM2Encoder:\n");
		Encoder_Debug(&CM2Encoder);
		printf("CM3Encoder:\n");
		Encoder_Debug(&CM3Encoder);
		printf("CM4Encoder:\n");
		Encoder_Debug(&CM4Encoder);
		*/

		printf("CM3PID speed:\n");
		printf("P:%f,\tI:%f,\tD:%f,\tOUT:%f\n", chassisSpeedPid3.componentKp,chassisSpeedPid3.componentKi, chassisSpeedPid3.componentKd, chassisSpeedPid3.output);
		printf("CM4PID speed:\n");
		printf("P:%f,\tI:%f,\tD:%f,\tOUT:%f\n", chassisSpeedPid4.componentKp,chassisSpeedPid4.componentKi, chassisSpeedPid4.componentKd, chassisSpeedPid4.output);
		

		printf("CM3Encoder:\n");
		Encoder_Debug(&CM3Encoder);

		
	}
#endif	
	


}

void ChassisCurrentControl(void)
{

#if USE_Mecanum_Synthesis
	Mecanum_Decompose(&mecanumCurrent);
#endif
	
	
	chassisRamp.Calc(&chassisRamp);
	
	chassisMotorCurrent.m1 = mecanumCurrent.w1 * chassisRamp.output;
	chassisMotorCurrent.m2 = mecanumCurrent.w2 * chassisRamp.output;
	chassisMotorCurrent.m3 = mecanumCurrent.w3 * chassisRamp.output;
	chassisMotorCurrent.m4 = mecanumCurrent.w4 * chassisRamp.output;
	
	/*	
	chassisMotorCurrent.m1 = mecanumCurrent.w1;
	chassisMotorCurrent.m2 = mecanumCurrent.w2;
	chassisMotorCurrent.m3 = mecanumCurrent.w3;
	chassisMotorCurrent.m4 = mecanumCurrent.w4;
	*/
	
	
	if(debug_tick%ControllerDEBUGNUM == 0){
	//	printf("mecanumCurrent:\n");
		//Mecanum_Debug(&mecanumCurrent);
		printf("CCX:%d\n", chassisMotorCurrent.m3);
	}
	


	
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
	
	PID_Reset(&chassisPositionPid1);
	PID_Reset(&chassisPositionPid2);
	PID_Reset(&chassisPositionPid3);
	PID_Reset(&chassisPositionPid4);
	
	PID_Reset(&chassisSpeedXPid);
	PID_Reset(&chassisSpeedYPid);
	PID_Reset(&chassisSpeedZPid);
	PID_Reset(&gimbalsSpeedYawPID);
	PID_Reset(&gimbalsSpeedPitchPID);
	
	PID_Reset(&chassisSpeedPid1);
	PID_Reset(&chassisSpeedPid2);
	PID_Reset(&chassisSpeedPid3);
	PID_Reset(&chassisSpeedPid4);
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

void Mecanum_ResetAll(void)
{
	Mecanum_Reset(&mecanumPosition);
	Mecanum_Reset(&mecanumSpeed);
	Mecanum_Reset(&mecanumCurrent);
	
	Mecanum_Reset(&chassisPositionTarget);
	Mecanum_Reset(&chassisSpeedTarget);	
}

void Controller_Reset(void)
{
	Encoder_ResetAll();
	Ramp_ResetAll();
	PID_ResetAll();
	Mecanum_ResetAll();

	ms_tick = 0;		
	debug_tick = 0;
}

void ChassisMotorCurrentTransmit(void)
{
	SetCMCurrent(CAN1, chassisMotorCurrent.m1, chassisMotorCurrent.m2, chassisMotorCurrent.m3, chassisMotorCurrent.m4);
}

void GimbalsMotorCurrentTransmit(void)
{
	//SetGMCurrent(CAN1, gimbalsMotorCurrent.yaw, gimbalsMotorCurrent.pit);
}


void ControlTask(void)
{
	ms_tick++;
	debug_tick++;
	
	InputTask(); // dbus
	
	WorkingStateSM();
	if(lastWorkingState == WORKING_STATE_STOP && workingState == WORKING_STATE_PREPARE)
	{
		printf("Going to reset the variables and prepare\n\n");
		
		Controller_Reset();
	}
	else if(workingState == WORKING_STATE_NORMAL)
	{
		//printf("run");
		if(lastCtrlMode != ctrlMode)
		{
			ms_tick = 0;		
			debug_tick = 0;
			//Ramp_ResetAll();
			//PID_ResetAll();
		}

#if ControllerDEBUG		
		if(ms_tick % 4200 == 0)
		{
			printf("inputMode:%d\n", inputMode);
			printf("ctrlMode:%d\n", ctrlMode);
		}
#endif		
		
		switch(ctrlMode)
		{
			
			case CTRL_MODE_POSITION:
			{
				if(ms_tick % 4 == 0)
				{
					ChassisPositionControl(0);
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
				if(ms_tick % 4 == 0 && MOVE_FLAG == 1)
				{
					ChassisPositionControl(0);
					ChassisSpeedControl();
					ChassisCurrentControl();
					//GimbalsPositionControl();
					//GimbalsSpeedControl();
					//GimbalsCurrentControl();
					ChassisMotorCurrentTransmit();
					//GimbalsMotorCurrentTransmit();
				}
				else{
					SetCMCurrent(CAN1, 0, 0, 0, 0);
					ChassisPositionControl(0); // Just to look at present values
					ChassisSpeedControl();
				}
			}break;
			case CTRL_MODE_PROGRAM_WITH_SPEED: // Set position by program with speed
			{
				if(ms_tick % 4 == 0 && MOVE_FLAG == 1)
				{
					ChassisPositionControl(1);
					ChassisSpeedControl();
					ChassisCurrentControl();
					ChassisMotorCurrentTransmit();
				}
				else{
					SetCMCurrent(CAN1, 0, 0, 0, 0);					
					ChassisPositionControl(1);
					ChassisSpeedControl();
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
			case CTRL_MODE_PROGRAM_WITH_CURRENT:
			{
				if(ms_tick % 4  == 0)
				{
					ChassisCurrentControl();
					ChassisMotorCurrentTransmit();
				}
			}break;			
			
			default:
			{
			}break;
		}
	}
	
}

