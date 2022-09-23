#ifndef __M3508_H__
#define __M3508_H__
#include "can.h"
#include "pid.h"

typedef struct
{
	
	int16_t angle;
	int16_t speed;
	int32_t angle_sum;
	int16_t angle_last;
	
	PID_STU M3508_Speed_PID;
	PID_STU M3508_Angle_PID;
	int16_t	output;
}M3508_STU;

extern M3508_STU m3508[4];
void M3508_Return_Process(CAN_RxHeaderTypeDef* RxHeader, uint8_t Data_Rx[]);
void M3508_Speed_Mode(M3508_STU *M3508, int16_t Speed);
void M3508_Angle_PID(M3508_STU *M3508, int16_t Angle);
	
#endif



