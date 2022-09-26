#include "pid.h"
#include "m3508.h"



void PID_Calculate_Incremental(PID_STU* PID, float Measure, float Target)
{
	PID->Measure = Measure;
	PID->Target = Target;
	PID->Error = Target - Measure;
	
	PID->p_out = PID->KP * (PID->Error - PID->LastError);
	PID->i_out = PID->KI * PID->Error;
	PID->d_out = PID->KD * (PID->Error - PID->LastError - (PID->LastError-PID->LastLastError));
	PID->Output += PID->p_out + PID->i_out  + PID->d_out;
	
	PID->LastLastError = PID->LastError;
	PID->LastError = PID->Error;
}

void PID_Calculate_Positional(PID_STU* PID, float Measure, float Target)
{
	PID->Measure = Measure;
	PID->Target = Target;
	PID->Error = Target - Measure;
	
	PID->p_out = PID->KP * PID->Error;
	PID->i_out = PID->KI * PID->LastLastError;
	PID->d_out = PID->KD * (PID->Error - PID->LastError);
	PID->Output = PID->p_out + PID->i_out  + PID->d_out;
	
	PID->LastLastError += PID->Error;
	PID->LastError = PID->Error;
}


void PID_init()//PID的具体值自己调一下每个3508不一样
{
	m3508[0].M3508_Speed_PID.KP = 0.47;
	m3508[0].M3508_Speed_PID.KI = 0.1;
	m3508[0].M3508_Speed_PID.KD = 0.5;
	m3508[0].M3508_Angle_PID.KP = 0;
	m3508[0].M3508_Angle_PID.KI = 0;
	m3508[0].M3508_Angle_PID.KD = 0;
	
}
	
