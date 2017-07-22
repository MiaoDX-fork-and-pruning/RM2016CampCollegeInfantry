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

void Encoder_Process(Encoder* encoder, uint16_t value)
{
	static const uint32_t ecd_mod = ENCODER_VAL_MAX + 1;
	static const float ecd_to_angle_coeff = 360.f/(float)ecd_mod;
	static const float angle_to_rad_coeff = PI/180.f;
	static const float ecd_to_rad_coeff = ecd_to_angle_coeff*angle_to_rad_coeff;
	if(encoder->cnt < ENCODER_INIT_FRAME_COUNT)
	{
		//printf("enc++");
		encoder->bias = value;
		encoder->cnt++;
	}
	encoder->last_value = encoder->value;
	encoder->value = value;
	encoder->diff = encoder->value - encoder->last_value;
	if(encoder->diff > ENCODER_DIFF_MAX)
	{
		encoder->rate_raw = encoder->diff - ecd_mod;
		encoder->round--;
	}
	else if(encoder->diff < -ENCODER_DIFF_MAX)
	{
		encoder->rate_raw = encoder->diff + ecd_mod;
		encoder->round++;
	}
	else
	{
		encoder->rate_raw = encoder->diff;
	}
	encoder->angle = (encoder->value - encoder->bias) * ecd_to_angle_coeff + encoder->round * 360;
	encoder->rad = (encoder->value - encoder->bias) * ecd_to_rad_coeff + encoder->round * 2 * PI;
	if(encoder->rate_cnt < ENCODER_RATE_BUF_SIZE)
	{
		encoder->rate_buf[encoder->rate_ptr++] = encoder->rate_raw;
		encoder->rate_sum += encoder->rate_raw;
		encoder->rate_cnt++;
	}
	else
	{
		if(encoder->rate_ptr == encoder->rate_cnt)
		{
			encoder->rate_ptr = 0;
		}
		encoder->rate_sum += (encoder->rate_raw - encoder->rate_buf[encoder->rate_ptr]);
		encoder->rate_buf[encoder->rate_ptr++] = encoder->rate_raw;
	}
	encoder->rate = encoder->rate_sum / encoder->rate_cnt;
	encoder->angle_rate = encoder->rate * 1000 * ecd_to_angle_coeff;
	encoder->rad_rate = encoder->angle_rate * angle_to_rad_coeff;
}

uint8_t Encoder_IsOk(Encoder* encoder)
{
	//printf("{%d}", encoder->cnt);
	return encoder->cnt == ENCODER_INIT_FRAME_COUNT;
}

void Encoder_Reset(Encoder* encoder)
{
	memset(encoder, 0, sizeof(Encoder));
}


void Encoder_Debug(Encoder* encoder)
{
/*
	uint16_t value;
	uint16_t last_value;
	int32_t diff;
	uint16_t bias;
	int32_t round;
	float angle;
	float rad;
	int32_t rate_raw;
	int32_t rate_buf[ENCODER_RATE_BUF_SIZE];
	uint8_t rate_cnt;
	uint8_t rate_ptr;
	int32_t rate_sum;
	int32_t rate;
	float angle_rate;
	float rad_rate;
	uint32_t cnt;
*/
	printf("value:%d\t", encoder->value);
	printf("round:%d\t", encoder->round);
	printf("angle:%f\t", encoder->angle);
	printf("rate:%d\t", encoder->rate);
	printf("rad_rate:%f\t", encoder->rad_rate);
	printf("rad:%f\t", encoder->rad);
	printf("bias:%d\t", encoder->bias);
	printf("cnt:%d\n", encoder->cnt);

}
