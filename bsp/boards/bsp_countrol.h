#ifndef BSP_COUNTROL_H
#define BSP_COUNTROL_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "bsp_usart.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "pid.h"
#include "mytype.h"
#include "tim.h"

#define up_pwm   1
#define down_pwm 2

#define MOVE_STOP        0   //停止
#define MOVE_FRONT       1   //前进
#define MOVE_BACK        2   //后退
#define MOVE_LEFT        3   //左移
#define MOVE_RIGHT       4   //右移
#define MOVE_TOP_LEFT    5   //左上方
#define MOVE_TOP_RIGHT   6   //右上方
#define MOVE_LOW_LEFT    7   //左下方
#define MOVE_LOW_RIGHT   8   //右下方
#define MOVE_TURN_LEFT   9   //左旋转
#define MOVE_TURN_RIGHT  10  //右旋转
#define MOVE_CYCLE_LEFT  11  //左绕圆
#define MOVE_CYCLE_RIGHT 12  //右绕圆

extern const RC_ctrl_t *local_rc_ctrl2;
extern int set_v, set_spd[4];
extern int key_cnt;

extern uint8_t MOVE_DIR;  //小车要移动的方向

extern uint8_t run_angle_flag;
extern uint8_t run_angle_c;          //旋转角度时电机需要转过的圈数
extern uint8_t run_angle_stop;       //电机旋转后最后需要听到的角度
extern uint8_t run_start_angle;      //用来存储电机开始需要转动的角度
extern uint8_t run_start_get_angle;  //用于获取当前电机的转角
extern int16_t run_speed;            //通过遥控器获取键盘动作速度
extern int16_t run_speed_l;          //通过遥控器获取键盘动作速度
extern int16_t run_speed_r;          //通过遥控器获取键盘动作速度

extern void CAN_Handle_PID(motor_measure_t *p);
extern void CAN_Handle_PID_angle(motor_measure_t *p);
extern void get_total_angle(moto_measure_t *p);
extern void CAN_Handle_PID_angle_M1(motor_measure_t *p);  //电机1的角度调节系统
extern void CAN_Handle_PID_angle_M2(motor_measure_t *p);  //电机2的角度调节系统
extern void CAN_Handle_PID_angle_M3(motor_measure_t *p);  //电机1的角度调节系统
extern void CAN_Handle_PID_angle_M4(motor_measure_t *p);  //电机2的角度调节系统

extern void ON_OFF_relay(uint8_t ID, uint8_t on_off);   //继电器控制函数
extern void PWM_relay(uint8_t ID, uint8_t on_off);      //PWM引脚做继电器
extern void pwm_angle_set(uint8_t ID, uint16_t angle);  //舵机控制函数,最大值2000，最小500
#endif
