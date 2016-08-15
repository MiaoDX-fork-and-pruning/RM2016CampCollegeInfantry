#ifndef _IMU_H_
#define _IMU_H_

#include <stdint.h>

#define M_PI 3.1415926535f
	
void Init_Quaternion(void);
void IMU_getYawPitchRoll(volatile float * ypr); //������̬
void GetPitchYawGxGyGz(void);
extern int16_t MPU6050_FIFO[6][11];//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ

extern volatile float angle[3];
extern volatile float yaw_angle,pitch_angle,roll_angle; //ʹ�õ��ĽǶ�ֵ

#endif

