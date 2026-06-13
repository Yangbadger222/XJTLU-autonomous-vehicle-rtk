#include "Joystick.h"

#define max_count 3
float MAX_rotate_Speed = 1;
float Max_speed[max_count] = {0.5, 1, 2, 3, 4, 5};
int speed_count = 0;

extern int control_mode; 
extern float Vcx;
extern float Wc;
extern int motor_ready;
extern int motor_shutdown;
extern int free_flag;

float MAX_Speed = 0;
int flagg = 0;
 
void Joystick_motor_start(void)
{
    // 检测是否按下 R2（键码 9）
    if (PS2_KEY == 9)
    {
        control_mode = 1; // 切换到串口控制模式
        led_blue_blink();
    }

    if (PS2_KEY == 10)
    {
        control_mode = 0; // 切换到手柄控制模式
        led_red_blink();
    }

    // 处理Y按钮，电机使能
    if(PS2_KEY == 13 && PS2_LY == 128 && PS2_RX == 127)  //Y������ң�� �̵���
	{

		motor_shutdown = 0; //电机使能
        led_green_start();
        motor_ready = 1;
        free_flag = 0;
	}

    // 处理X按钮关闭
    if (PS2_KEY == 16 || 
        ((PS2_LY == 255) && (PS2_LX == 255) && (PS2_RX == 255) && (PS2_RY == 255)) || 
        ((PS2_LY == 128) && (PS2_LX == 128) && (PS2_RX == 128) && (PS2_RY == 128))) // X按钮关闭
    {
        control_mode = 0;
        motor_shutdown = 1; //设置电机不使能
        motor_ready = 0;   // 电机不准备好输入
        free_flag = 1;     // 进入自由滑行模式
        led_red_start();
    }

    if (PS2_KEY == 14) // B按钮按下急停
    {
        control_mode = 0;
        motor_shutdown = 1; // 与 X 键一致：撤力失能
        motor_ready = 0;    // 电机不准备好输入
        free_flag = 1;      // 进入自由滑行模式
        Set_free();
        
        led_pink_blink();   // 闪烁粉色LED
        led_pink_start();
    }
}

void Joystick_v_set(void)
{
    if (PS2_KEY == 5)
    {
        if (!flagg)
        {
            if (speed_count < max_count - 1)
            {
                speed_count += 1;
            }
            flagg = 1;
        }
    }

    if (PS2_KEY == 7)
    {
        if (!flagg)
        {
            if (speed_count > 0)
            {
                speed_count -= 1;
            }
            flagg = 1;
        }
    }

    if (PS2_KEY == 0)
    {
        flagg = 0;
    }

    MAX_Speed = Max_speed[speed_count];
}

void Joystick_motor_control(void)
{
    PS2_Receive(); // 接收手柄数据
    Joystick_v_set();
    Joystick_motor_start();

    // 只有在手柄控制模式且motor_shutdown == 0下，才从手柄更新 Vcx 和 Wc
    if (control_mode == 0)
    {
        if (motor_ready == 1 && motor_shutdown == 0 )
        {
            // 更新 Vcx
            if (PS2_LY <= 128)
            {
                Vcx = (float)(MAX_Speed - 0) / (float)(0 - 128) * (PS2_LY - 128);
            }
            else
            {
                Vcx = (float)(MAX_Speed - 0) / (float)(0 - 127) * (PS2_LY - 128);
            }

            if (Vcx > MAX_Speed)
                Vcx = MAX_Speed;
            if (Vcx < -MAX_Speed)
                Vcx = -MAX_Speed;
        }

        if (motor_ready == 1 && motor_shutdown == 0)
        {
            // 更新 Wc
            if (PS2_RX <= 127)
            {
                Wc = (float)(MAX_rotate_Speed - 0) / (float)(0 - 127) * (PS2_RX - 127);
            }
            else if (PS2_RX > 127)
            {
                Wc = (float)(MAX_rotate_Speed - 0) / (float)(0 - 128) * (PS2_RX - 127);
            }

            if (Wc > MAX_rotate_Speed)
                Wc = MAX_rotate_Speed;
            if (Wc < -MAX_rotate_Speed)
                Wc = -MAX_rotate_Speed;  
        }
    }

}
