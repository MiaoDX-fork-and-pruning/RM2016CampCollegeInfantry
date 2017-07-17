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
 
#ifndef __USART3_H__
#define __USART3_H__

#include <stdint.h>

#define USART_REC_LEN 12
//extern unsigned char USART_RX_BUF[USART_REC_LEN];
void USART3_Config(void);

#define USART3_Channel USART3_IRQn
#define USART3_PreemptionPriority 3
#define USART3_SubPriority 3

#define DMA1_Channel DMA1_Stream1_IRQn
#define DMA1_PreemptionPriority 0
#define DMA1_SubPriority 2

#endif
