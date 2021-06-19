#include "bsp_countrol.h"

/*
*电机顺时针为正转，角度增加
*/
#define CAN_CONTROL  //const current control

#define ABS(x) ((x > 0) ? (x) : (-x))

int set_v, set_spd[4];
int key_cnt = 0;
uint8_t run_angle_flag = 0;
uint8_t run_angle_c = 0;      //需要转动的圈数
uint8_t run_angle_stop = 0;   //电机最终需要听到的值
uint8_t run_start_angle = 0;  //电机开始转动角度时的当前值
uint8_t run_start_get_angle = 0;
int16_t run_speed = 0;    //通过遥控器获取的电机运转的速度
int16_t run_speed_l = 0;  //通过遥控器获取的电机运转的速度
int16_t run_speed_r = 0;  //通过遥控器获取的电机运转的速度

uint8_t MOVE_DIR = MOVE_STOP;

extern CAN_HandleTypeDef hcan1_PID;
extern CAN_HandleTypeDef hcan2_PID;

const RC_ctrl_t *local_rc_ctrl2;

/*****************************************
        p.ecd;电机的转角[0,8190]
    p.speed_rpm;获取的转速
    p.given_current;获取的电流
    p.temperate;获取的温度
    p.last_ecd;
        
        POSITION_PID  位置式
        DELTA_PID 增量式
*******************************************/
void CAN_Handle_PID(motor_measure_t *p)
{
    //    PID_struct_init(&pid_omg, POSITION_PID, 20000, 20000,
    //                                    1.5f,    0.1f,    0.0f    );  //angular rate closeloop.
    if (PID_flag == 1)
    {
        for (int i = 0; i < 4; i++)
        {
            PID_struct_init(&pid_spd[i], POSITION_PID, 20000, 20000, 1.5f, 0.1f, 0.1f);  //4 motos angular rate closeloop.
        }

#if defined CAN_CONTROL

        for (int i = 0; i < 4; i++)
        {
            //pid_calc_angle(&pid_spd[i], p->ecd, 4045);//pid队列，实际速度，设定速度,使角度停在中间
            if (local_rc_ctrl2->rc.ch[3] == 0 || local_rc_ctrl2->rc.ch[3] == 1024)
            {
                for (i = 0; i < 4; i++)
                    pid_calc(&pid_spd[i], p->speed_rpm, 0);  //pid队列，实际速度，设定速度，速度设定为0
            }
            else
                pid_calc(&pid_spd[i], p->speed_rpm, set_spd[i]);  //pid队列，实际速度，设定速度
        }
        //            pid_calc(&pid_spd[0], p->speed_rpm, set_spd[0]);//pid队列，实际速度，设定速度
        //            pid_calc(&pid_spd[1], p->speed_rpm, -set_spd[1]);//pid队列，实际速度，设定速度
        //            pid_calc(&pid_spd[2], p->speed_rpm, set_spd[2]);//pid队列，实际速度，设定速度
        //            pid_calc(&pid_spd[3], p->speed_rpm, -set_spd[3]);//pid队列，实际速度，设定速度

        CAN_cmd_chassis(pid_spd[0].pos_out,
                        -pid_spd[1].pos_out,
                        pid_spd[2].pos_out,
                        -pid_spd[3].pos_out);
#endif
        set_spd[0] = set_spd[1] = set_spd[2] = set_spd[3] = key_cnt * 50;
    }
}

/****************************************
                    电机运行方向说明

前进、后退、停止：
                                M1:run_speed                    M2:-run_speed                    M3:run_speed                    M4:-run_speed

左移、右移：
                                M1:-run_speed                    M2:-run_speed                    M3:+run_speed                    M4:+run_speed

左旋转，右旋转：
                                M1:run_speed                    M2:run_speed                    M3:run_speed                    M4:run_speed

左上、右下：
                                M1:run_speed                    M2:0                                    M3:0                                    M4:-run_speed

右上、左下：
                                M1:0                                    M2:-run_speed                    M3:run_speed                    M4:0

左绕圆：
                                M1:run_speed_l                M2:-run_speed_r                M3:run_speed_l                M4:-run_speed_r
                                
右绕圆：
                                M1:-run_speed_l                M2:run_speed_r                M3:-run_speed_l                M4:run_speed_r
*****************************************/
void CAN_Handle_PID_angle_M1(motor_measure_t *p)  //电机1的角度调节系统
{
    if (PID_flag == 2)
    {
        //电机1 PID调节，实际速度，设定速度，速度通过遥控器获取
        if (MOVE_DIR == MOVE_STOP || MOVE_DIR == MOVE_FRONT || MOVE_DIR == MOVE_BACK)
            pid_calc(&pid_spd[0], p->speed_rpm, run_speed);  //前后移动
        else if (MOVE_DIR == MOVE_LEFT || MOVE_DIR == MOVE_RIGHT)
            pid_calc(&pid_spd[0], p->speed_rpm, -run_speed);  //左右移动
        else if (MOVE_DIR == MOVE_TURN_LEFT || MOVE_DIR == MOVE_TURN_RIGHT)
            pid_calc(&pid_spd[0], p->speed_rpm, run_speed);  //左右旋转
        else if (MOVE_DIR == MOVE_TOP_LEFT || MOVE_DIR == MOVE_LOW_RIGHT)
            pid_calc(&pid_spd[0], p->speed_rpm, run_speed);  //左上右下
        else if (MOVE_DIR == MOVE_TOP_RIGHT || MOVE_DIR == MOVE_LOW_LEFT)
            pid_calc(&pid_spd[0], p->speed_rpm, 0);  //右上左下
        else if (MOVE_DIR == MOVE_CYCLE_LEFT)
            pid_calc(&pid_spd[0], p->speed_rpm, run_speed_l);  //左绕圆
        else if (MOVE_DIR == MOVE_CYCLE_RIGHT)
            pid_calc(&pid_spd[0], p->speed_rpm, -run_speed_l);  //右绕圆

        CAN_cmd_chassis(pid_spd[0].pos_out, pid_spd[1].pos_out, pid_spd[2].pos_out, pid_spd[3].pos_out);
    }
}

void CAN_Handle_PID_angle_M2(motor_measure_t *p)  //电机2的角度调节系统
{
    if (PID_flag == 2)
    {
        if (MOVE_DIR == MOVE_STOP || MOVE_DIR == MOVE_FRONT || MOVE_DIR == MOVE_BACK)
            pid_calc(&pid_spd[1], p->speed_rpm, -run_speed);  //前后移动
        else if (MOVE_DIR == MOVE_LEFT || MOVE_DIR == MOVE_RIGHT)
            pid_calc(&pid_spd[1], p->speed_rpm, -run_speed);  //左右移动
        else if (MOVE_DIR == MOVE_TURN_LEFT || MOVE_DIR == MOVE_TURN_RIGHT)
            pid_calc(&pid_spd[1], p->speed_rpm, run_speed);  //左右旋转
        else if (MOVE_DIR == MOVE_TOP_LEFT || MOVE_DIR == MOVE_LOW_RIGHT)
            pid_calc(&pid_spd[1], p->speed_rpm, 0);  //左上右下
        else if (MOVE_DIR == MOVE_TOP_RIGHT || MOVE_DIR == MOVE_LOW_LEFT)
            pid_calc(&pid_spd[1], p->speed_rpm, -run_speed);  //右上左下
        else if (MOVE_DIR == MOVE_CYCLE_LEFT)
            pid_calc(&pid_spd[1], p->speed_rpm, -run_speed_r);  //左绕圆
        else if (MOVE_DIR == MOVE_CYCLE_RIGHT)
            pid_calc(&pid_spd[1], p->speed_rpm, run_speed_r);  //右绕圆

        CAN_cmd_chassis(pid_spd[0].pos_out, pid_spd[1].pos_out, pid_spd[2].pos_out, pid_spd[3].pos_out);
    }
}

void CAN_Handle_PID_angle_M3(motor_measure_t *p)  //电机3的角度调节系统
{
    if (PID_flag == 2)
    {
        if (MOVE_DIR == MOVE_STOP || MOVE_DIR == MOVE_FRONT || MOVE_DIR == MOVE_BACK)
            pid_calc(&pid_spd[2], p->speed_rpm, run_speed);  //前后移动
        else if (MOVE_DIR == MOVE_LEFT || MOVE_DIR == MOVE_RIGHT)
            pid_calc(&pid_spd[2], p->speed_rpm, run_speed);  //左右移动
        else if (MOVE_DIR == MOVE_TURN_LEFT || MOVE_DIR == MOVE_TURN_RIGHT)
            pid_calc(&pid_spd[2], p->speed_rpm, run_speed);  //左右旋转
        else if (MOVE_DIR == MOVE_TOP_LEFT || MOVE_DIR == MOVE_LOW_RIGHT)
            pid_calc(&pid_spd[2], p->speed_rpm, 0);  //左上右下
        else if (MOVE_DIR == MOVE_TOP_RIGHT || MOVE_DIR == MOVE_LOW_LEFT)
            pid_calc(&pid_spd[2], p->speed_rpm, run_speed);  //右上左下
        else if (MOVE_DIR == MOVE_CYCLE_LEFT)
            pid_calc(&pid_spd[2], p->speed_rpm, run_speed_l);  //左绕圆
        else if (MOVE_DIR == MOVE_CYCLE_RIGHT)
            pid_calc(&pid_spd[2], p->speed_rpm, -run_speed_l);  //右绕圆

        CAN_cmd_chassis(pid_spd[0].pos_out, pid_spd[1].pos_out, pid_spd[2].pos_out, pid_spd[3].pos_out);
    }
}

void CAN_Handle_PID_angle_M4(motor_measure_t *p)  //电机4的角度调节系统
{
    if (PID_flag == 2)
    {
        if (MOVE_DIR == MOVE_STOP || MOVE_DIR == MOVE_FRONT || MOVE_DIR == MOVE_BACK)
            pid_calc(&pid_spd[3], p->speed_rpm, -run_speed);  //前后移动
        else if (MOVE_DIR == MOVE_LEFT || MOVE_DIR == MOVE_RIGHT)
            pid_calc(&pid_spd[3], p->speed_rpm, run_speed);  //左右移动
        else if (MOVE_DIR == MOVE_TURN_LEFT || MOVE_DIR == MOVE_TURN_RIGHT)
            pid_calc(&pid_spd[3], p->speed_rpm, run_speed);  //左右旋转
        else if (MOVE_DIR == MOVE_TOP_LEFT || MOVE_DIR == MOVE_LOW_RIGHT)
            pid_calc(&pid_spd[3], p->speed_rpm, -run_speed);  //左上右下
        else if (MOVE_DIR == MOVE_TOP_RIGHT || MOVE_DIR == MOVE_LOW_LEFT)
            pid_calc(&pid_spd[3], p->speed_rpm, 0);  //右上左下
        else if (MOVE_DIR == MOVE_CYCLE_LEFT)
            pid_calc(&pid_spd[3], p->speed_rpm, -run_speed_r);  //左绕圆
        else if (MOVE_DIR == MOVE_CYCLE_RIGHT)
            pid_calc(&pid_spd[3], p->speed_rpm, run_speed_r);  //右绕圆

        CAN_cmd_chassis(pid_spd[0].pos_out, pid_spd[1].pos_out, pid_spd[2].pos_out, pid_spd[3].pos_out);
    }
}

void CAN_Handle_PID_angle(motor_measure_t *p)
{
    int i = 0;
    PID_struct_init(&pid_spd[0], POSITION_PID, 20000, 20000, 1.5f, 0.1f, 0.0f);  //电机1速度环
    PID_struct_init(&pid_spd[1], POSITION_PID, 20000, 20000, 1.5f, 0.1f, 0.0f);  //电机2速度环
    PID_struct_init(&pid_spd[2], POSITION_PID, 20000, 20000, 1.5f, 0.1f, 0.0f);  //电机3速度环
    PID_struct_init(&pid_spd[3], POSITION_PID, 20000, 20000, 1.5f, 0.1f, 0.0f);  //电机4速度环

    if (local_rc_ctrl2->rc.ch[1] == 0)
    {
        for (i = 0; i < 4; i++)
            pid_calc(&pid_spd[i], p->speed_rpm, 0);  //pid队列，实际速度，设定速度，速度设定为0
    }
    else
    {
        pid_calc(&pid_spd[0], p->speed_rpm, M3508_A[0]);  //电机1 PID调节，实际速度，设定速度，吧速度设定为2000
        pid_calc(&pid_spd[1], p->speed_rpm, M3508_A[1]);  //电机2 PID调节，实际速度，设定速度，吧速度设定为2000
        pid_calc(&pid_spd[2], p->speed_rpm, M3508_A[2]);  //电机3 PID调节，实际速度，设定速度，吧速度设定为2000
        pid_calc(&pid_spd[3], p->speed_rpm, M3508_A[3]);  //电机4 PID调节，实际速度，设定速度，吧速度设定为2000
    }

    CAN_cmd_chassis(pid_spd[0].pos_out,
                    pid_spd[1].pos_out,
                    pid_spd[2].pos_out,
                    pid_spd[3].pos_out);
}

/*******************************************
                            继电器控制函数

    参数设定：
    ID：
                1                            2                            3                            4
        ReLe_front        ReRi_front    ReLe_back            ReRi_back
        左侧前方             右侧前方        左侧后方             右侧后方    
            PB12                     PB13                    PB15                     PB14

    on_off:
            0                                1
         关闭                         打开
************************************************/

void ON_OFF_relay(uint8_t ID, uint8_t on_off)
{
    if (on_off == 0)
    {
        switch (ID)
        {
        case 1:
            HAL_GPIO_WritePin(ReLe_front_GPIO_Port, ReLe_front_Pin, GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(ReRi_front_GPIO_Port, ReRi_front_Pin, GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(ReLe_back_GPIO_Port, ReLe_back_Pin, GPIO_PIN_RESET);
            break;
        case 4:
            HAL_GPIO_WritePin(ReRi_back_GPIO_Port, ReRi_back_Pin, GPIO_PIN_RESET);
            break;
        default:
            break;
        }
    }
    else if (on_off == 1)
    {
        switch (ID)
        {
        case 1:
            HAL_GPIO_WritePin(ReLe_front_GPIO_Port, ReLe_front_Pin, GPIO_PIN_SET);
            break;
        case 2:
            HAL_GPIO_WritePin(ReRi_front_GPIO_Port, ReRi_front_Pin, GPIO_PIN_SET);
            break;
        case 3:
            HAL_GPIO_WritePin(ReLe_back_GPIO_Port, ReLe_back_Pin, GPIO_PIN_SET);
            break;
        case 4:
            HAL_GPIO_WritePin(ReRi_back_GPIO_Port, ReRi_back_Pin, GPIO_PIN_SET);
            break;
        default:
            break;
        }
    }
}

/*
    引脚对应表

    PWM1                relay1
    PWM2                relay2
*/
void PWM_relay(uint8_t ID, uint8_t on_off)  //PWM引脚做继电器
{
    if (on_off == 0)
    {
        switch (ID)
        {
        case 1:
            HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_RESET);
            break;
        default:
            break;
        }
    }
    else if (on_off == 1)
    {
        switch (ID)
        {
        case 1:
            HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_SET);
            break;
        case 2:
            HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_SET);
            break;
        default:
            break;
        }
    }
}

/**
     PWM对应舵机角度设置
     ID表示舵机编号,  up_pwm为夹球舵机
                                         down_pwm为俯仰舵机
         angle为舵机角度, 500的时候角度最小,2000的时候角度最大
     none
  */
void pwm_angle_set(uint8_t ID, uint16_t angle)
{
    switch (ID)
    {
    case 1:
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, angle);
        break;
    case 2:
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, angle);
        break;
    default:
        break;
    }
}
