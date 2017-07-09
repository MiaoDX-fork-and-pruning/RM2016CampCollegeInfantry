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

#define LED_GREEN_ON()      GPIO_ResetBits(GPIOF, GPIO_Pin_14)
#define LED_GREEN_OFF()     GPIO_SetBits(GPIOF, GPIO_Pin_14)
#define LED_GREEN_TOGGLE()      GPIO_ToggleBits(GPIOF, GPIO_Pin_14)

#define LED_RED_ON()            GPIO_ResetBits(GPIOE, GPIO_Pin_7)
#define LED_RED_OFF()           GPIO_SetBits(GPIOE, GPIO_Pin_7)
#define LED_RED_TOGGLE()        GPIO_ToggleBits(GPIOE, GPIO_Pin_7)

#define LED_ON()\
LED_GREEN_ON();\
LED_RED_ON()

#define LED_OFF()\
LED_GREEN_OFF();\
LED_RED_OFF()


#define LED_TOGGLE()\
GPIO_ToggleBits(GPIOF, GPIO_Pin_14);\
GPIO_ToggleBits(GPIOE, GPIO_Pin_7)


#endif
