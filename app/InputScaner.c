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
#include "sdbus.h"

InputMode inputMode = INPUT_MODE_NO;
InputMode lastInputMode = INPUT_MODE_NO;
CtrlMode ctrlMode = CTRL_MODE_SPEED;
CtrlMode lastCtrlMode = CTRL_MODE_SPEED;
SwitchPanel switchPanel;
Keyboard keyboard;


//ChassisPosition chassisPositionTarget;
//ChassisSpeed chassisSpeedTarget;
Mecanum chassisPositionTarget;
Mecanum chassisSpeedTarget;




GimbalsPosition gimbalsPosition;
GimbalsSpeed gimbalsSpeed;
GimbalsCurrent gimbalsCurrent;

MAFilterF32* mafilter_x = NULL;
MAFilterF32* mafilter_y = NULL;
MAFilterF32* mafilter_z = NULL;

MAFilterF32* mafilter_x_m = NULL;
MAFilterF32* mafilter_y_m = NULL;
MAFilterF32* mafilter_z_m = NULL;

#define MAF_CALC_Mecanum_NUM 150 // The bigger, the longer time to get desired value
Ramp chassisRampPOS = CHASSIS_RAMP_LONGER;
#define InputScannerDEBUG 0


#define MOVE_RESET 1 // reset before move, relative move without memory

#define PROGRAM_MODE 0
#define PROGRAM_WITH_SPEED_MODE 0
#define PROGRAM_WITH_CURRENT_MODE 0 // failed, since we can not make sure what current is guaranteed to make the motor run :(

void GetInputMode(void)
{
	lastInputMode = inputMode;
	switch(dbus.rc.s2)
	{
		case SW_UP:
		{
			inputMode = INPUT_MODE_RC;
		}break;
		case SW_MD:
		{
			inputMode = INPUT_MODE_HC;
		}break;
		case SW_DN:
		{
			inputMode = INPUT_MODE_NO;
		}break;
		default:
		{
			inputMode = INPUT_MODE_NO;
		}break;
	}

if(inputMode == INPUT_MODE_NO){
#if PROGRAM_MODE
inputMode = INPUT_MODE_PROGRAM;
#endif	
#if PROGRAM_WITH_SPEED_MODE
inputMode = INPUT_MODE_PROGRAM_WITH_SPEED;
#endif	
#if PROGRAM_WITH_CURRENT_MODE
inputMode = INPUT_MODE_PROGRAM_WITH_CURRENT;
#endif	
	
	
}	

	
}

void GetCtrlMode(void)
{
	lastCtrlMode = ctrlMode;
	switch(dbus.rc.s1)
	{
		case SW_UP:
		{
			ctrlMode = CTRL_MODE_POSITION;
		}break;
		case SW_MD:
		{
			ctrlMode = CTRL_MODE_SPEED;
		}break;
		case SW_DN:
		{
			ctrlMode = CTRL_MODE_NO;//CTRL_MODE_CURRENT;
		}break;
		default:
		{
			ctrlMode = CTRL_MODE_NO;
		}break;
	}

if(ctrlMode == CTRL_MODE_NO){
#if PROGRAM_MODE
ctrlMode = CTRL_MODE_PROGRAM;
#endif		
#if PROGRAM_WITH_SPEED_MODE
ctrlMode = CTRL_MODE_PROGRAM_WITH_SPEED;
#endif	
#if PROGRAM_WITH_CURRENT_MODE
ctrlMode = CTRL_MODE_PROGRAM_WITH_CURRENT;
#endif
}	

	
}

void GetSwitchState(SwitchState* s, uint8_t v)
{
	s->lastState = s->thisState;
	s->thisState = v;
	s->action = GET_SWITCH_ACTION(s->lastState, s->thisState);
}

void GetKeyAction(KeyState* k, uint8_t v)
{
	k->lastState = k->thisState;
	k->thisState = v;
	k->action = GET_KEY_ACTION(k->lastState, k->thisState);
}

void GetSwitchPanel(void)
{
	GetSwitchState(&switchPanel.s1, dbus.rc.s1);
	GetSwitchState(&switchPanel.s2, dbus.rc.s2);
}

void GetKeyboard(void)
{
	GetKeyAction(&keyboard.A, dbus.hc.key.val&KEY_A);
	GetKeyAction(&keyboard.D, dbus.hc.key.val&KEY_D);
	GetKeyAction(&keyboard.W, dbus.hc.key.val&KEY_W);
	GetKeyAction(&keyboard.S, dbus.hc.key.val&KEY_S);
	GetKeyAction(&keyboard.Shift, dbus.hc.key.val&KEY_SHIFT);
	GetKeyAction(&keyboard.Ctrl, dbus.hc.key.val&KEY_CTRL);
	GetKeyAction(&keyboard.Q, dbus.hc.key.val&KEY_Q);
	GetKeyAction(&keyboard.E, dbus.hc.key.val&KEY_E);
}


int maf_i = 0;
void MAF_CALC_Mecanum(Mecanum* mecanumVar)
{
	if(mafilter_x_m == NULL) mafilter_x_m = MAFilterF32_Create(MAF_CALC_Mecanum_NUM);
	if(mafilter_y_m == NULL) mafilter_y_m = MAFilterF32_Create(MAF_CALC_Mecanum_NUM);
	if(mafilter_z_m == NULL) mafilter_z_m = MAFilterF32_Create(MAF_CALC_Mecanum_NUM);
	if(lastWorkingState == WORKING_STATE_STOP && workingState == WORKING_STATE_PREPARE)
	{
		MAFilterF32_Reset(mafilter_x);
		MAFilterF32_Reset(mafilter_y);
		MAFilterF32_Reset(mafilter_z);
	}

#if InputScannerDEBUG	
	maf_i ++;
	if(maf_i >= 10)
	{
		printf("MAF IN:{%f,%f,%f}\n", mecanumVar->x, mecanumVar->y, mecanumVar->z);
	}
#endif	
	
	mecanumVar->x = MAFilterF32_Calc(mafilter_x_m, mecanumVar->x);
	mecanumVar->y = MAFilterF32_Calc(mafilter_y_m, mecanumVar->y);
	mecanumVar->z = MAFilterF32_Calc(mafilter_z_m, mecanumVar->z);

#if InputScannerDEBUG	
	if(maf_i >= 10)
	{
		printf("MAF OUT:{%f,%f,%f}\n", mecanumVar->x, mecanumVar->y, mecanumVar->z);
		maf_i = 0;
	}
#endif		
}

int ramp_i = 0;
void RAMP_CALC_Mecanum(Mecanum* mecanumVar)
{
	
	if(lastWorkingState == WORKING_STATE_STOP && workingState == WORKING_STATE_PREPARE)
	{
		Ramp_ResetCounter(&chassisRampPOS);
	}
	
	chassisRampPOS.Calc(&chassisRampPOS);

#if InputScannerDEBUG		
	ramp_i ++;
	if(ramp_i >= 10)
	{
		printf("RAMP IN:{%f,%f,%f}\n", mecanumVar->x, mecanumVar->y, mecanumVar->z);
		printf("chassisRampPOS.output:%f\n", chassisRampPOS.output);
	}
#endif	
	
	mecanumVar->x = mecanumVar->x * chassisRampPOS.output;
	mecanumVar->y = mecanumVar->y * chassisRampPOS.output;
	mecanumVar->z = mecanumVar->z * chassisRampPOS.output;

#if InputScannerDEBUG		
	if(ramp_i >= 10)
	{
		printf("RAMP OUT:{%f,%f,%f}\n", mecanumVar->x, mecanumVar->y, mecanumVar->z);
		ramp_i = 0;
	}
#endif	
}



int SIMLIAR(float x, float y)
{
	return abs(x-y) < 8.0; // this can vary according to our precision
}

static uint32_t mg_tick = 0;
static uint32_t similar_mg_tick[4] = {0,0,0,0};
uint32_t MOVE_FLAG = 0;
void MoveGuard()
{
	mg_tick++;
	
#if PROGRAM_WITH_SPEED_MODE
	if(SIMLIAR(chassisPositionTarget.w1, mecanumPosition.w1)) similar_mg_tick[0] ++; else similar_mg_tick[0] = 0;
	if(SIMLIAR(chassisPositionTarget.w2, mecanumPosition.w2)) similar_mg_tick[1] ++; else similar_mg_tick[1] = 0;
	if(SIMLIAR(chassisPositionTarget.w3, mecanumPosition.w3)) similar_mg_tick[2] ++; else similar_mg_tick[2] = 0;
	if(SIMLIAR(chassisPositionTarget.w4, mecanumPosition.w4)) similar_mg_tick[3] ++; else similar_mg_tick[3] = 0;
	
	int i = 0;
	/*
	for(i = 0; i < 4; i++){	
		if(similar_mg_tick[i] > 0 && similar_mg_tick[i] < 42 && similar_mg_tick[i] %10 == 0){
				printf("SIMLIAR of %d motor at %d times!!\n", i+1, similar_mg_tick[i]);
		}
	}*/
	
	
	for(i = 0; i < 4; i++){	
		if(similar_mg_tick[i] == 42){
			printf("Going to stop motor %d!\n", i + 1);
			if(i == 0) chassisSpeedTarget.w1 = 0;
			if(i == 1) chassisSpeedTarget.w2 = 0;
			if(i == 2) chassisSpeedTarget.w3 = 0;
			if(i == 3) chassisSpeedTarget.w4 = 0;
		}
	}
	
	//if(fabs(chassisSpeedTarget.w1) < 1e-8 && fabs(chassisSpeedTarget.w2) < 1e-8 && fabs(chassisSpeedTarget.w3) < 1e-8 && fabs(chassisSpeedTarget.w4) < 1e-8)
	if(fabs(chassisSpeedTarget.w3) < 1e-8 && fabs(chassisSpeedTarget.w4) < 1e-8)
	{
		printf("Going to stop!\n");
		printf("chassisPositionTarget:\n");
		Mecanum_Debug(&chassisPositionTarget);
		
		printf("mecanumPosition:\n");
		Mecanum_Debug(&mecanumPosition);
		MOVE_FLAG = 0;				
		#if MOVE_RESET
				Controller_Reset();
				printf("Controller_Reset done!\n");
		#endif
	}
		
			
	
#else // PROGRAM_WITH_SPEED_MODE
	
	if(SIMLIAR(chassisPositionTarget.w1, mecanumPosition.w1) && SIMLIAR(chassisPositionTarget.w2, mecanumPosition.w2) && SIMLIAR(chassisPositionTarget.w3, mecanumPosition.w3) && SIMLIAR(chassisPositionTarget.w4, mecanumPosition.w4)){	
		similar_mg_tick[0] ++;		
	}
	else{
		similar_mg_tick[0] = 0;
	}
	if(similar_mg_tick[0] > 0 && similar_mg_tick[0] %10 == 0){
			printf("SIMLIAR %d times!!\n", similar_mg_tick[0]);
	}
	
	if(similar_mg_tick[0] >= 4200){
		
		printf("Going to stop!\n");
		printf("chassisPositionTarget:\n");
		Mecanum_Debug(&chassisPositionTarget);
		
		printf("mecanumPosition:\n");
		Mecanum_Debug(&mecanumPosition);
		
		
		MOVE_FLAG = 0;
		
#if MOVE_RESET
		Controller_Reset();
		printf("Controller_Reset done!\n");
#endif

	}
	
#endif	// PROGRAM_WITH_SPEED_MODE
	
	
	
}

int symbol(float x)
{
	return x > 1e-8? 1: (x < -1e-8? -1 :0);
}


void GetStickCtrlChassisPositionProgramWithSpeedSetSpeed(void)
{
	printf("Enter GetStickCtrlChassisPositionProgramWithSpeedSetSpeed\n");	
	
	int move_able_num = 0;	
	
	if(fabs(sdbus.xf) > 1e-8){
		move_able_num ++;
		chassisSpeedTarget.x = MAP(symbol(sdbus.xf)*0.8, -INPUT_CHASSIS_SPEED_MAX, INPUT_CHASSIS_SPEED_MAX, -INPUT_CHASSIS_SPEED_MAX, INPUT_CHASSIS_SPEED_MAX);
	}
	if(fabs(sdbus.xtr) > 1e-8){
		move_able_num ++;
		chassisSpeedTarget.y = MAP(symbol(sdbus.xtr)*0.8, -INPUT_CHASSIS_SPEED_MAX, INPUT_CHASSIS_SPEED_MAX, -INPUT_CHASSIS_SPEED_MAX, INPUT_CHASSIS_SPEED_MAX);
	}
	if(fabs(sdbus.xrr) > 1e-8){
		move_able_num ++;
		chassisSpeedTarget.z = MAP(symbol(sdbus.xrr)*0.8, -INPUT_CHASSIS_SPEED_MAX, INPUT_CHASSIS_SPEED_MAX, -INPUT_CHASSIS_SPEED_MAX, INPUT_CHASSIS_SPEED_MAX);
	}	
	
	
	if(move_able_num > 1){		
		printf("More than one possible move direction, please do not do this!");
		Mecanum_Reset(&chassisSpeedTarget);
	}

	printf("We got predefined chassisSpeedTarget:\n");		
	Mecanum_Decompose(&chassisSpeedTarget); // Decompose to four wheels		
	Mecanum_Debug(&chassisSpeedTarget);
	
}


void GetStickCtrlChassisPositionProgram(void)
{	
	
	if(fabs(sdbus.xf) > 1e-8 || fabs(sdbus.xtr) > 1e-8 || fabs(sdbus.xrr) > 1e-8){
		
#if MOVE_RESET
		Controller_Reset();
		printf("Controller_Reset done!\n");
#endif

#if PROGRAM_WITH_SPEED_MODE
	GetStickCtrlChassisPositionProgramWithSpeedSetSpeed();
#endif	
		
		chassisPositionTarget.x = MAP(sdbus.xf, -INPUT_CHASSIS_POSITION_MAX, INPUT_CHASSIS_POSITION_MAX, -INPUT_CHASSIS_POSITION_MAX, INPUT_CHASSIS_POSITION_MAX);
		chassisPositionTarget.y = MAP(sdbus.xtr, -INPUT_CHASSIS_POSITION_MAX, INPUT_CHASSIS_POSITION_MAX, -INPUT_CHASSIS_POSITION_MAX, INPUT_CHASSIS_POSITION_MAX);		
		chassisPositionTarget.z = MAP(sdbus.xrr, -INPUT_CHASSIS_POSITION_MAX, INPUT_CHASSIS_POSITION_MAX, -INPUT_CHASSIS_POSITION_MAX, INPUT_CHASSIS_POSITION_MAX);
		//chassisPositionTarget.z = mecanumPosition.z + MAP(sdbus.xrr+CH_MID, CH_MIN, CH_MAX, -INPUT_GIMBALS_POSITION_MAX, INPUT_GIMBALS_POSITION_MAX)*81.92;
		
		
		chassisPositionTarget.x *= (180.0/PI/5.0);
		chassisPositionTarget.y *= (180.0/PI/5.0);
		chassisPositionTarget.z *= (180.0/PI/5.0);
		
		
		SDBUS_Reset(&sdbus);
		
		mg_tick = 0;
		MOVE_FLAG = 1;
		
		Mecanum_Decompose(&chassisPositionTarget); // Decompose to four wheels		
		
		printf("Going to move\n");
		printf("chassisPositionTarget:\n");
		Mecanum_Debug(&chassisPositionTarget);
		
		printf("mecanumPosition:\n");
		Mecanum_Debug(&mecanumPosition);
		
	}
	

	if(MOVE_FLAG){
		MoveGuard();
	}	
}





void GetStickCtrlChassisPositionProgramWithSpeed(void)
{	
	


	
	
	
	
	GetStickCtrlChassisPositionProgram();
	
}	



void GetStickCtrlChassisPosition(void)
{
	//printf("<P:\n");
	chassisPositionTarget.x = MAP(dbus.rc.ch0, CH_MIN, CH_MAX, -INPUT_CHASSIS_POSITION_MAX, INPUT_CHASSIS_POSITION_MAX);
	chassisPositionTarget.y = MAP(dbus.rc.ch1, CH_MIN, CH_MAX, -INPUT_CHASSIS_POSITION_MAX, INPUT_CHASSIS_POSITION_MAX);
	
	chassisPositionTarget.z = MAP(dbus.rc.ch2, CH_MIN, CH_MAX, -INPUT_GIMBALS_POSITION_MAX, INPUT_GIMBALS_POSITION_MAX);
	
	//MAF_CALC_Mecanum(&chassisPositionTarget);
	
	
	//printf(":P>\n");
	Mecanum_Decompose(&chassisPositionTarget); // Decompose to four wheels
	
}

void GetStickCtrlChassisSpeed(void)
{
	chassisSpeedTarget.x = MAP(dbus.rc.ch0, CH_MIN, CH_MAX, -INPUT_CHASSIS_SPEED_MAX, INPUT_CHASSIS_SPEED_MAX);	
	chassisSpeedTarget.y = MAP(dbus.rc.ch1, CH_MIN, CH_MAX, -INPUT_CHASSIS_SPEED_MAX, INPUT_CHASSIS_SPEED_MAX);
	chassisSpeedTarget.z = MAP(dbus.rc.ch2, CH_MIN, CH_MAX, -INPUT_GIMBALS_SPEED_MAX, INPUT_GIMBALS_SPEED_MAX);
	
	// MAF_CALC_Mecanum(&chassisSpeedTarget);
	
	Mecanum_Decompose(&chassisSpeedTarget); // Decompose to four wheels
}

void GetStickCtrlChassisCurrent(void)
{
	//printf(".");
	mecanumCurrent.x = MAP(dbus.rc.ch0, CH_MIN, CH_MAX, -INPUT_CHASSIS_CURRENT_MAX, INPUT_CHASSIS_CURRENT_MAX);
	mecanumCurrent.y = MAP(dbus.rc.ch1, CH_MIN, CH_MAX, -INPUT_CHASSIS_CURRENT_MAX, INPUT_CHASSIS_CURRENT_MAX);
	mecanumCurrent.z = MAP(dbus.rc.ch2, CH_MIN, CH_MAX, -INPUT_GIMBALS_CURRENT_MAX, INPUT_GIMBALS_CURRENT_MAX);
	
	Mecanum_Decompose(&mecanumCurrent);
}

void GetStickCtrlGimbalsPosition(void)
{
	gimbalsPosition.yaw = MAP(dbus.rc.ch2, CH_MIN, CH_MAX, -INPUT_GIMBALS_POSITION_MAX, INPUT_GIMBALS_POSITION_MAX);
	gimbalsPosition.pit = MAP(dbus.rc.ch3, CH_MIN, CH_MAX, -INPUT_GIMBALS_POSITION_MAX, INPUT_GIMBALS_POSITION_MAX);
}

void GetStickCtrlGimbalsSpeed(void)
{
	gimbalsSpeed.yaw = MAP(dbus.rc.ch2, CH_MIN, CH_MAX, -INPUT_GIMBALS_SPEED_MAX, INPUT_GIMBALS_SPEED_MAX);
	gimbalsSpeed.pit = MAP(dbus.rc.ch3, CH_MIN, CH_MAX, -INPUT_GIMBALS_SPEED_MAX, INPUT_GIMBALS_SPEED_MAX);
}

void GetStickCtrlGimbalsCurrent(void)
{
	gimbalsCurrent.yaw = MAP(dbus.rc.ch2, CH_MIN, CH_MAX, -INPUT_GIMBALS_CURRENT_MAX, INPUT_GIMBALS_CURRENT_MAX);
	gimbalsCurrent.pit = MAP(dbus.rc.ch3, CH_MIN, CH_MAX, -INPUT_GIMBALS_CURRENT_MAX, INPUT_GIMBALS_CURRENT_MAX);
}

void GetKeyboardCtrlChassisPosition(void)
{
	/**************************************/
	/*            ___________             */
	/*           /___________\            */
	/*          /             \           */
	/*     ____/               \____      */
	/*                                    */
	/**************************************/
	uint8_t key = dbus.hc.key.val;
	float position = (key & KEY_SHIFT) ? INPUT_CHASSIS_POSITION_MAX : INPUT_CHASSIS_POSITION_MAX / 2.f;
	float position_x = (key & KEY_A) ? -position : ((key & KEY_D) ? position : 0);
	float position_y = (key & KEY_S) ? -position : ((key & KEY_W) ? position : 0);
	float position_z = 500*MAP(dbus.hc.mouse.x, MS_MIN, MS_MAX, -INPUT_GIMBALS_POSITION_MAX, INPUT_GIMBALS_POSITION_MAX);
	if(mafilter_x == NULL) mafilter_x = MAFilterF32_Create(8);
	if(mafilter_y == NULL) mafilter_y = MAFilterF32_Create(8);
	if(mafilter_z == NULL) mafilter_z = MAFilterF32_Create(6);
	if(lastInputMode == INPUT_MODE_RC)
	{
		MAFilterF32_Reset(mafilter_x);
		MAFilterF32_Reset(mafilter_y);
		MAFilterF32_Reset(mafilter_z);
	}
	mecanumCurrent.x = MAFilterF32_Calc(mafilter_x, position_x);
	mecanumCurrent.y = MAFilterF32_Calc(mafilter_y, position_y);
	mecanumCurrent.z = MAFilterF32_Calc(mafilter_z, position_z);
}

void GetKeyboardCtrlChassisSpeed(void)
{
	/**************************************/
	/*            ___________             */
	/*           /___________\            */
	/*          /             \           */
	/*     ____/               \____      */
	/*                                    */
	/**************************************/
	uint8_t key = dbus.hc.key.val;
	float speed = (key & KEY_SHIFT) ? INPUT_CHASSIS_SPEED_MAX : INPUT_CHASSIS_SPEED_MAX / 2.f;
	float speed_x = (key & KEY_A) ? -speed : ((key & KEY_D) ? speed : 0);
	float speed_y = (key & KEY_S) ? -speed : ((key & KEY_W) ? speed : 0);
	float speed_z = 500*MAP(dbus.hc.mouse.x, MS_MIN, MS_MAX, -INPUT_GIMBALS_SPEED_MAX, INPUT_GIMBALS_SPEED_MAX);
	if(mafilter_x == NULL) mafilter_x = MAFilterF32_Create(8);
	if(mafilter_y == NULL) mafilter_y = MAFilterF32_Create(8);
	if(mafilter_z == NULL) mafilter_z = MAFilterF32_Create(6);
	if(lastInputMode == INPUT_MODE_RC)
	{
		MAFilterF32_Reset(mafilter_x);
		MAFilterF32_Reset(mafilter_y);
		MAFilterF32_Reset(mafilter_z);
	}
	chassisSpeedTarget.x = MAFilterF32_Calc(mafilter_x, speed_x);
	chassisSpeedTarget.y = MAFilterF32_Calc(mafilter_y, speed_y);
	chassisSpeedTarget.z = MAFilterF32_Calc(mafilter_z, speed_z);
}

void GetKeyboardCtrlChassisCurrent(void)
{
	/**************************************/
	/*            ___________             */
	/*           /___________\            */
	/*          /             \           */
	/*     ____/               \____      */
	/*                                    */
	/**************************************/
	uint8_t key = dbus.hc.key.val;
	float current = (key & KEY_SHIFT) ? INPUT_CHASSIS_CURRENT_MAX : INPUT_CHASSIS_CURRENT_MAX / 2.f;
	float current_x = (key & KEY_A) ? -current : ((key & KEY_D) ? current : 0);
	float current_y = (key & KEY_S) ? -current : ((key & KEY_W) ? current : 0);
	float current_z = 500*MAP(dbus.hc.mouse.x, MS_MIN, MS_MAX, -INPUT_GIMBALS_CURRENT_MAX, INPUT_GIMBALS_CURRENT_MAX);
	if(mafilter_x == NULL) mafilter_x = MAFilterF32_Create(8);
	if(mafilter_y == NULL) mafilter_y = MAFilterF32_Create(8);
	if(mafilter_z == NULL) mafilter_z = MAFilterF32_Create(6);
	if(lastInputMode == INPUT_MODE_RC)
	{
		MAFilterF32_Reset(mafilter_x);
		MAFilterF32_Reset(mafilter_y);
		MAFilterF32_Reset(mafilter_z);
	}
	mecanumCurrent.x = MAFilterF32_Calc(mafilter_x, current_x);
	mecanumCurrent.y = MAFilterF32_Calc(mafilter_y, current_y);
	mecanumCurrent.z = MAFilterF32_Calc(mafilter_z, current_z);
}

void GetMouseCtrlGimbalsPosition(void)
{
	gimbalsPosition.yaw = MAP(dbus.hc.mouse.x, MS_MIN, MS_MAX, -INPUT_GIMBALS_POSITION_MAX, INPUT_GIMBALS_POSITION_MAX);
	gimbalsPosition.pit = MAP(dbus.hc.mouse.y, MS_MIN, MS_MAX, -INPUT_GIMBALS_POSITION_MAX, INPUT_GIMBALS_POSITION_MAX);
}

void GetMouseCtrlGimbalsSpeed(void)
{
	gimbalsSpeed.yaw = MAP(dbus.hc.mouse.x, MS_MIN, MS_MAX, -INPUT_GIMBALS_SPEED_MAX, INPUT_GIMBALS_SPEED_MAX);
	gimbalsSpeed.pit = MAP(dbus.hc.mouse.y, MS_MIN, MS_MAX, -INPUT_GIMBALS_SPEED_MAX, INPUT_GIMBALS_SPEED_MAX);
}

void GetMouseCtrlGimbalsCurrent(void)
{
	gimbalsCurrent.yaw = MAP(dbus.hc.mouse.x, MS_MIN, MS_MAX, -INPUT_GIMBALS_CURRENT_MAX, INPUT_GIMBALS_CURRENT_MAX);
	gimbalsCurrent.pit = MAP(dbus.hc.mouse.y, MS_MIN, MS_MAX, -INPUT_GIMBALS_CURRENT_MAX, INPUT_GIMBALS_CURRENT_MAX);
}

void InputTask(void)
{
	GetInputMode();	
	GetCtrlMode();
	//printf("%d", inputMode);
	//printf("%d", ctrlMode);
	if(inputMode == INPUT_MODE_RC)
	{		
		GetSwitchPanel();
		switch(ctrlMode)
		{
			case CTRL_MODE_POSITION:
			{
				GetStickCtrlChassisPosition();
				GetStickCtrlGimbalsPosition();
			}break;
			case CTRL_MODE_SPEED:
			{
				GetStickCtrlChassisSpeed();
				GetStickCtrlGimbalsSpeed();
			}break;
			case CTRL_MODE_PROGRAM:
			{
				GetStickCtrlChassisPositionProgram();
			}break;
			case CTRL_MODE_PROGRAM_WITH_SPEED:
			{
				GetStickCtrlChassisPositionProgramWithSpeed();
			}break;
			/*
			case CTRL_MODE_CURRENT:
			{
				GetStickCtrlChassisCurrent();
				GetStickCtrlGimbalsCurrent();
			}break;
			*/
			default:
			{
			}break;
		}
		
	}
	else if(inputMode == INPUT_MODE_HC)
	{
		GetKeyboard();
		switch(ctrlMode)
		{
			case CTRL_MODE_POSITION:
			{
				GetKeyboardCtrlChassisPosition();
				GetMouseCtrlGimbalsPosition();
			}break;
			case CTRL_MODE_SPEED:
			{
				GetKeyboardCtrlChassisSpeed();
				GetMouseCtrlGimbalsSpeed();
			}break;
			/*
			case CTRL_MODE_CURRENT:
			{
				GetKeyboardCtrlChassisCurrent();
				GetMouseCtrlGimbalsCurrent();
			}break;
			*/
			default:
			{
			}break;
		}
		
	}
	else if(inputMode == INPUT_MODE_PROGRAM){ // NO RC
		GetStickCtrlChassisPositionProgram();
	}
	else if(inputMode == INPUT_MODE_PROGRAM_WITH_SPEED){ // NO RC and with speed mode
		GetStickCtrlChassisPositionProgramWithSpeed();
	}
	else if (inputMode == INPUT_MODE_PROGRAM_WITH_CURRENT)
	{
		GetStickCtrlChassisCurrent();
	}
	
	
	
}

float MAP(float val, float min1, float max1, float min2, float max2)
{
	float cal = (val-min1)*(max2-min2)/(max1-min1)+min2;
	//printf("cal:%f", cal);
	if(abs(cal)>abs(max2)+0.001){
		//printf("Too large:%f", max2);
		return max2;
	}
		
	
	return cal;
}
