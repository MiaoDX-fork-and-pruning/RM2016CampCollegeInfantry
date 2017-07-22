/************************************************************************************
  File Name     :  usart3.c 
  cpu           :  STM32F405RGT6
  Create Date   :  2016/6/29
  Author        :  yf
  Description   :  usart3�����ã����ڵ��Է����ã��Լ����ں�rosͨ�Ų���
									 -----USART3_TX-----PD8-----
									 -----USART3_RX-----PD9-----
									 ����gpio��usart��ʼ����nvic��usart3���ú�����
									 ���ڵ��Եķ���char��int��string����
									 

-------------------------------Revision Histroy-----------------------------------
No   Version    Date     Revised By       Item       Description   
1     1.1       6/28       yf   			usart3���ú���	����gpio��usart��ʼ����nvic		
																				���ͺ���			����char��int��string
2     1.2       6/29       gyf 
3     1.3       6/29       yf 					  ע��		
4     1.6       7/8        
************************************************************************************/
#include "main.h"
#include "sdbus.h"
#include "nvic.h"

unsigned char USART_RX_BUF[USART_REC_LEN];


void USART3_Config(void)
{
    USART_InitTypeDef usart3;
		GPIO_InitTypeDef  gpio;
	   DMA_InitTypeDef   dma;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); 
	
		gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOD,&gpio);

		usart3.USART_BaudRate =  115200;
		usart3.USART_WordLength = USART_WordLength_8b;
		usart3.USART_StopBits = USART_StopBits_1;
		usart3.USART_Parity = USART_Parity_No;
		usart3.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART3,&usart3);

    //USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
		USART_Cmd(USART3,ENABLE);
    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
		
		NVIC_Set(USART3_Channel,USART3_PreemptionPriority,USART3_SubPriority,ENABLE);
		NVIC_Set(DMA1_Channel,DMA1_PreemptionPriority,DMA1_SubPriority,ENABLE);
		
		DMA_DeInit(DMA1_Stream1);
    dma.DMA_Channel= DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)USART_RX_BUF;
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = USART_REC_LEN;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_Mode_Normal;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1,&dma);

    DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);

		DMA_Cmd(DMA1_Stream1,ENABLE);
}

void DMA1_Stream1_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1))
    {
				DMA_Cmd(DMA1_Stream1, DISABLE);
				printf("DMA_GetCurrDataCounter(DMA1_Stream1):%d\n", DMA_GetCurrDataCounter(DMA1_Stream1));
			
        DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
        DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
				

			switch(dbus.rc.s1){
				case SW_MD:
				{
					SPID_POS_Dec(&spid, USART_RX_BUF);
				}break;
				case SW_DN:
				{
					SDBUS_Dec(&sdbus,USART_RX_BUF);	
				}break;
			}
			
				//SDBUS_Dec(&sdbus,USART_RX_BUF);	
				//SPID_Dec(&spid, USART_RX_BUF);
				//SPID_POS_Dec(&spid, USART_RX_BUF);
				
			
				DMA_Cmd(DMA1_Stream1, ENABLE);
			  printf("ok\n");
		}
}

int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
    USART_SendData(USART3, (uint8_t)ch);
    return ch;
}
