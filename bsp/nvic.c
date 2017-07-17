/************************************************************************************
  File Name     :  nvic.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  NVIC_Set���ڿ�����ø����ֵ����ȼ�����Ҫע��channel����ռ���ȼ���
									 �����ȼ������ã�
									 NVIC_Configuration����������Ҫ����nvic��ģ����������á�
									 

-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.1       6/28       yf   			   	null
2     1.2       6/29       gyf 					������nvic.c  ����NVIC_Set����
3     1.3       6/29       yf 					ע�ͼ�nvic.h	��nvic�и����ֵ������ƶ���main.h��	   
************************************************************************************/
#include "nvic.h"
void NVIC_Configuration(void){
  
}

void NVIC_Set(int Channel,int PreemptionPriority,int SubPriority,FunctionalState Cmd){
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = Channel;
	nvic.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;
	nvic.NVIC_IRQChannelSubPriority = SubPriority;
	nvic.NVIC_IRQChannelCmd = Cmd;
	NVIC_Init(&nvic);
}
