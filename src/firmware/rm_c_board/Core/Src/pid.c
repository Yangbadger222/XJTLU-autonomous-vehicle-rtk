//pid.c
//2023.11.9
//by gjx

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "stm32f4xx.h"

#define ABS(x)		((x>0)? x: -x) 

PID_TypeDef pid_pitch,pid_pithch_speed,pid_roll,pid_roll_speed,pid_yaw_speed;
extern int isMove;

/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(
	PID_TypeDef * pid, 
	PID_ID   id,
	uint16_t maxout,
	uint16_t intergral_limit,
	float deadband,
	uint16_t period,
	int16_t  max_err,
	int16_t  target,

	float 	kp, 
	float 	ki, 
	float 	kd)
{
	pid->id = id;		
	
	pid->ControlPeriod = period;             //没用到
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;
	
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	
	pid->output = 0;
}

/*中途更改参数设定--------------------------------------------------------------*/
static void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

/*pid计算-----------------------------------------------------------------------*/

	
static float pid_calculate(PID_TypeDef* pid, float measure)//, int16_t target)
{
//	uint32_t time,lasttime;
	
	pid->lasttime = pid->thistime;
	pid->thistime = HAL_GetTick();
	uint32_t elapsed = pid->thistime - pid->lasttime;
	if (elapsed == 0) {
		elapsed = 1;
	}
	if (elapsed > 50) {
		elapsed = 50;
	}
	pid->dtime = (uint8_t)elapsed;
	pid->measure = measure;
  //	pid->target = target;
		
//	pid->last_err  = pid->err;
//	pid->last_output = pid->output;
	
	pid->err = pid->target - pid->measure;
	
	//是否进入死区
	if((ABS(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		pid->iout += (pid->ki * pid->err - 3 * 0.6566 * (pid->calculate_output - pid->output)) * pid->dtime / 1000.0f;
		pid->dout =  pid->kd * (pid->err - pid->last_err) / (pid->dtime / 1000.0f); 
		
		//积分是否超出限制
		if(pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if(pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit;
		
		//pid理论输出和
		pid->calculate_output = pid->pout + pid->iout + pid->dout;
		
		//pid->output = pid->output*0.7f + pid->last_output*0.3f;  //滤波？
		
		if(pid->calculate_output>pid->MaxOutput)         
		{
			pid->output = pid->MaxOutput;
		}
		else if(pid->calculate_output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
		else{
			pid->output = pid->calculate_output;
		}
	
	}


	pid->last_err = pid->err;
	return pid->output;
}

/*pid结构体初始化，每一个pid参数需要调用一次-----------------------------------------------------*/
void pid_init(PID_TypeDef* pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}
