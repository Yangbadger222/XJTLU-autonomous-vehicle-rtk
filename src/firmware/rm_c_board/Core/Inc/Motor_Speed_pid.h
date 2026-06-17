#ifndef MOTOR_SPEED_PID_H
#define MOTOR_SPEED_PID_H
#include "headfile.h"

extern float set_spdL;
extern float set_spdR;
extern float set_spdL1;
extern float set_spdR1;
extern float Vcx;  
extern float Wc; 
extern int free_flag;
extern int brake_flag;


void Motor_Speed_Calc(void);
void Motor_Speed_pid_init(void);
void Set_free(void);
void Emergency_Stop_Output(void);
void Clear_Brake_State(void);
void Active_Brake_Output(void);
void Speed_set(void);
void speed_print(void);
void MOTORrpm2vw(float left_motor_speed,float right_motor_speed,float *vcx,float*w);
float low_pass_filter(float value, float fc, float Ts);
#endif
