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
 
#ifndef __LED_H__
#define __LED_H__

void Led_Config(void);

#define LED_OFF()           GPIO_SetBits(GPIOB, GPIO_Pin_9)
#define LED_ON()            GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define LED_TOGGLE()        GPIO_ToggleBits(GPIOB, GPIO_Pin_9)

#endif
