#ifndef __PID_H__
#define __PID_H__

typedef struct
{
	float KP;
	float KI;
	float KD;
	
	float Error;
	float LastError;
	float LastLastError;
	
	float Measure;
	float Target;
	
	float p_out;
	float i_out;
	float d_out;
	float Output;
}PID_STU;

void PID_Calculate_Incremental(PID_STU* PID, float Measure, float Target);
void PID_Calculate_Positional(PID_STU* PID, float Measure, float Target);
void PID_init(void);
#endif
