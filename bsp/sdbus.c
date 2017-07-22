/************************************************************************************
  File Name     :  sdbus.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  sdbus编码和解码，仅用到了解码
-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.6       7/8        yf   			    sdbus       编码解码	   
************************************************************************************/
#include "sdbus.h"
#include "Controller.h"
#include "main.h"


SDBUS sdbus={0,0,0};
SPID spid={0,0,0};
void SDBUS_Enc(const SDBUS* sdbus,unsigned char* sdbuf)//sdbus编码
{
    /*sdbuf[0] = (sdbus->PitchAngle > 0)+'0';
    sdbuf[1] =  abs(sdbus->PitchAngle)/10+'0';
		sdbuf[2] = abs(sdbus->PitchAngle)%10+'0';
		sdbuf[3] = (sdbus->YawAngle > 0)+'0';
		sdbuf[4] = abs(sdbus->YawAngle)/10+'0';
		sdbuf[5] = abs(sdbus->YawAngle)%10+'0';
		*/
}

void SDBUS_Dec(SDBUS* sdbus,const unsigned char* sdbuf)//sdbus解码
{
		sdbus->xf = (double)( (','-sdbuf[0])*((sdbuf[1]-'0')*100+(sdbuf[2]-'0')*10+(sdbuf[3]-'0')) );
		sdbus->xtr = (double)( (','-sdbuf[4])*((sdbuf[5]-'0')*100+(sdbuf[6]-'0')*10+(sdbuf[7]-'0')) );
		sdbus->xrr = (double)( (','-sdbuf[8])*((sdbuf[9]-'0')*100+(sdbuf[10]-'0')*10+(sdbuf[11]-'0')) );
		
		/*
		sdbus->w1 = RM3510_1.thisPosition+sdbus->xf+sdbus->xtr+sdbus->xrr;
		sdbus->w2 = -RM3510_2.thisPosition+sdbus->xf-sdbus->xtr-sdbus->xrr;
		sdbus->w3 = RM3510_3.thisPosition+sdbus->xf-sdbus->xtr+sdbus->xrr;
		sdbus->w4 = -RM3510_4.thisPosition+sdbus->xf+sdbus->xtr-sdbus->xrr;
		*/
		
		printf("%f,%f,%f", sdbus->xf, sdbus->xtr, sdbus->xrr);
}

void SPID_Dec(SPID* spid,const unsigned char* sdbuf)
{
	spid->kp = (float)( (','-sdbuf[0])*((sdbuf[1]-'0')*100+(sdbuf[2]-'0')*10+(sdbuf[3]-'0')) );
	spid->ki = (float)( (','-sdbuf[4])*((sdbuf[5]-'0')*100+(sdbuf[6]-'0')*10+(sdbuf[7]-'0'))*0.1 );
	spid->kd = (float)( (','-sdbuf[8])*((sdbuf[9]-'0')*100+(sdbuf[10]-'0')*10+(sdbuf[11]-'0'))*0.1 );
		
	printf("Going to set pids of speed:\n");
	
	printf("%f,%f,%f", spid->kp, spid->ki, spid->kd);
	
	PID_Set(&chassisSpeedPid1, spid->kp, spid->ki, spid->kd);
	PID_Set(&chassisSpeedPid2, spid->kp, spid->ki, spid->kd);
	PID_Set(&chassisSpeedPid3, spid->kp, spid->ki, spid->kd);
	PID_Set(&chassisSpeedPid4, spid->kp, spid->ki, spid->kd);
	
	
	printf("Set pids of speed done:\n");
	
	printf("chassisSpeedPid1 PID:%f,%f,%f", chassisSpeedPid1.kp, chassisSpeedPid1.ki, chassisSpeedPid1.kd);
	
}

void SPID_POS_Dec(SPID* spid,const unsigned char* sdbuf)
{
	spid->kp = (float)( (','-sdbuf[0])*((sdbuf[1]-'0')*100+(sdbuf[2]-'0')*10+(sdbuf[3]-'0'))*0.01 );
	spid->ki = (float)( (','-sdbuf[4])*((sdbuf[5]-'0')*100+(sdbuf[6]-'0')*10+(sdbuf[7]-'0'))*0.0001 );
	spid->kd = (float)( (','-sdbuf[8])*((sdbuf[9]-'0')*100+(sdbuf[10]-'0')*10+(sdbuf[11]-'0'))*0.01 );
		
	printf("Going to set pids of position:\n");
	
	printf("%f,%f,%f", spid->kp, spid->ki, spid->kd);
	
	PID_Set(&chassisPositionPid1, spid->kp, spid->ki, spid->kd);
	PID_Set(&chassisPositionPid2, spid->kp, spid->ki, spid->kd);
	PID_Set(&chassisPositionPid3, spid->kp, spid->ki, spid->kd);
	PID_Set(&chassisPositionPid4, spid->kp, spid->ki, spid->kd);
	
	
	printf("Set pids of position done:\n");
	
	PID_ResetAll();
	
	printf("PID_ResetAll done:\n");
	
	printf("chassisPositionPid1 PID:%f,%f,%f\n", chassisPositionPid1.kp, chassisPositionPid1.ki, chassisPositionPid1.kd);
	
}


void SDBUS_Reset(SDBUS* psdbus)
{
	memset(psdbus, 0, sizeof(SDBUS));
	//delay_ms(10);
}
