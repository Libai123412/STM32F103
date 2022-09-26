#include "m3508.h"
#include "can.h"

M3508_STU m3508[4];

int16_t ABS(int16_t num)
{
	return num > 0 ? num:-num;
}

void M3508_Return_Process(CAN_RxHeaderTypeDef* RxHeader, uint8_t Data_Rx[])
{
	uint8_t num = RxHeader->StdId & 0x00F;
	m3508[num-1].angle = Data_Rx[0]<<8 | Data_Rx[1];
	m3508[num-1].speed = Data_Rx[2]<<8 | Data_Rx[3];
	
	int16_t angle_add_1, angle_add_2, angle_add_fin = 0;
	
	if(m3508[num-1].angle > m3508[num-1].angle_last)
	{
		angle_add_1 = m3508[num-1].angle - m3508[num-1].angle_last;
		angle_add_2 = -8191 + m3508[num-1].angle - m3508[num-1].angle_last;
	}
	else
	{
		angle_add_1 = m3508[num-1].angle - m3508[num-1].angle_last;
		angle_add_2 = 8191 - m3508[num-1].angle - m3508[num-1].angle_last;
	}
	
	if(ABS(angle_add_1) > ABS(angle_add_2)) angle_add_fin = angle_add_2;
	else angle_add_fin = angle_add_1;
	
	m3508[num-1].angle_sum += angle_add_fin;
	m3508[num-1].angle_last = m3508[num-1].angle;
}

void M3508_Speed_Mode(M3508_STU *M3508, int16_t Speed)
{
	PID_Calculate_Incremental(&(M3508->M3508_Speed_PID), M3508->speed, Speed);
	
	if(M3508->M3508_Speed_PID.Output>1000)M3508->M3508_Speed_PID.Output = 1000;
	if(M3508->M3508_Speed_PID.Output<-1000)M3508->M3508_Speed_PID.Output = -1000;
	
	M3508->output = M3508->M3508_Speed_PID.Output;
}

void M3508_Angle_PID(M3508_STU *M3508, int16_t Angle)//串级pid控制角度
{ 
	PID_Calculate_Positional(&(M3508->M3508_Angle_PID), M3508->angle_sum, Angle*8191/360);	
	if(M3508->M3508_Angle_PID.Output > 30000)M3508->M3508_Angle_PID.Output = 30000;
	if(M3508->M3508_Angle_PID.Output < -30000)M3508->M3508_Angle_PID.Output= -30000 ;
	PID_Calculate_Incremental(&(M3508->M3508_Speed_PID), M3508->speed, M3508->M3508_Angle_PID.Output);
	M3508->output = M3508->M3508_Speed_PID.Output;
	if(M3508->output > 1000)M3508->output = 1000;
	if(M3508->output < -1000)M3508->output = -1000;
	if((M3508->output < 50) &&(M3508->output > -50)) M3508->output = 0;
	
}

//void M3508_Angle_PID(M3508_STU *M3508, int16_t Angle)
//{
//	PID_Calculate_Positional(&(M3508->M3508_Angle_PID), M3508->angle_sum, Angle);
//	
//	if(M3508->M3508_Angle_PID.Output>30000)M3508->M3508_Angle_PID.Output = 30000;
//	if(M3508->M3508_Angle_PID.Output<-30000)M3508->M3508_Angle_PID.Output = -30000;
//	
//	M3508->output = M3508->M3508_Angle_PID.Output;
//}

