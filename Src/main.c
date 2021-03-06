/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
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
#include "bsp_countrol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//backgroud program
//????????????
uint8_t exit_flag = 0;
uint8_t rising_falling_flag;

uint8_t run_flag = 0;
uint8_t PID_flag = 0;

int16_t servo_up = 1500;  //1280
int16_t servo_down = 1340;

const RC_ctrl_t *local_rc_ctrl;

const motor_measure_t *motor_data;

/**
  * @brief          Toggle the red led, green led and blue led 
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ????????????????????????????????????
  * @param[in]      none
  * @retval         none
  */
void bsp_led_toggle(void)
{
    // HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
}

/**
  * @brief          exit callback function
  * @param[in]      GPIO_Pin:gpio pin
  * @retval         none
  */
/**
  * @brief          ??????????????????
  * @param[in]      GPIO_Pin:?????????
  * @retval         none
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_Pin)
    {
        if (exit_flag == 0)
        {
            exit_flag = 1;
            rising_falling_flag = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
        }
    }
}

void handle_KEY()
{
    if (exit_flag == 1)
    {
        exit_flag = 2;
        if (rising_falling_flag == GPIO_PIN_RESET)
        {
            //debouce
            //??????
            HAL_Delay(20);
            if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
            {
                HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
                exit_flag = 0;
                if (run_flag == 0)
                    run_flag = 1;
                else
                    run_flag = 0;
                key_cnt = local_rc_ctrl->rc.ch[3];

                while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
                    ;
            }
            else
            {
                exit_flag = 0;
            }
        }
        else if (rising_falling_flag == GPIO_PIN_SET)
        {
            //debouce
            //??????
            HAL_Delay(20);
            if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
            {
                HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
                exit_flag = 0;
            }
            else
            {
                exit_flag = 0;
            }
        }
    }
}

/***********************************************************
    local_rc_ctrl->rc.ch[0]?????????????????????0???-660???660????????????
    local_rc_ctrl->rc.ch[1]?????????????????????1???-660???660????????????
    local_rc_ctrl->rc.ch[2]?????????????????????2???-660???660????????????
    local_rc_ctrl->rc.ch[3]?????????????????????3???-660???660????????????
    local_rc_ctrl->rc.ch[4]?????????4???????????????660???????????????0???6021
  local_rc_ctrl->rc.s[0]???????????????S1?????????1,2,3
    local_rc_ctrl->rc.s[1]???????????????S2?????????1,2,3
  local_rc_ctrl->mouse.x???
    local_rc_ctrl->mouse.y???
    local_rc_ctrl->mouse.z???
    local_rc_ctrl->mouse.press_l???
    local_rc_ctrl->mouse.press_r???
  local_rc_ctrl->key.v???

***************************************************************/

void control_motor()  //???????????????????????????
{
    int16_t get_speed_fb = 0;  //?????????ch[3]??????
    int16_t get_speed_lr = 0;  //?????????ch[2]??????

    M3508_A[0] = local_rc_ctrl->rc.ch[1] * 30;  //????????????
    M3508_A[1] = local_rc_ctrl->rc.ch[1] * 30;
    M3508_A[2] = local_rc_ctrl->rc.ch[1] * 30;
    M3508_A[3] = local_rc_ctrl->rc.ch[1] * 30;

    C620_A[0] = local_rc_ctrl->rc.ch[3] * 10;
    C620_A[1] = local_rc_ctrl->rc.ch[3] * 10;  //??????????????????????????????
    C620_A[2] = local_rc_ctrl->rc.ch[3] * 10;
    C620_A[3] = local_rc_ctrl->rc.ch[3] * 10;

    get_speed_fb = (local_rc_ctrl->rc.ch[1]) * 8;
    get_speed_lr = (local_rc_ctrl->rc.ch[0]) * 8;

    if (local_rc_ctrl->rc.ch[4] < 0)
    {
        //        ON_OFF_relay(1, 1); ON_OFF_relay(2, 1);    //????????????1

        PWM_relay(1, 1);  //??????PWM?????????????????????
    }
    else if (local_rc_ctrl->rc.ch[4] > 0)
    {
        //        ON_OFF_relay(3, 1); ON_OFF_relay(4, 1);    //????????????2

        PWM_relay(2, 1);  //??????PWM?????????????????????
    }
    else
    {
        //        ON_OFF_relay(1, 0); ON_OFF_relay(2, 0); ON_OFF_relay(3, 0); ON_OFF_relay(4, 0);//???????????????1,2,3,4

        PWM_relay(1, 0);
        PWM_relay(2, 0);  //???????????????
    }

    if (local_rc_ctrl->rc.s[0] == 1 && local_rc_ctrl->rc.s[1] == 1)  //s1???s2??????1??????????????????
    {
        PID_flag = 2;  //????????????

        if (local_rc_ctrl->rc.ch[1] == 0 && local_rc_ctrl->rc.ch[0] == 0)
        {
            MOVE_DIR = MOVE_STOP;                     //????????????
            run_speed = local_rc_ctrl->rc.ch[1] * 8;  //???????????????????????????
        }
        else if (local_rc_ctrl->rc.ch[1] > 0 && local_rc_ctrl->rc.ch[0] == 0)
        {
            MOVE_DIR = MOVE_FRONT;                    //??????
            run_speed = local_rc_ctrl->rc.ch[1] * 8;  //???????????????????????????
        }
        else if (local_rc_ctrl->rc.ch[1] < 0 && local_rc_ctrl->rc.ch[0] == 0)
        {
            MOVE_DIR = MOVE_BACK;                     //??????
            run_speed = local_rc_ctrl->rc.ch[1] * 8;  //???????????????????????????
        }
        else if (local_rc_ctrl->rc.ch[1] == 0 && local_rc_ctrl->rc.ch[0] > 0)
        {
            MOVE_DIR = MOVE_RIGHT;                    //??????
            run_speed = local_rc_ctrl->rc.ch[0] * 8;  //???????????????????????????
        }
        else if (local_rc_ctrl->rc.ch[1] == 0 && local_rc_ctrl->rc.ch[0] < 0)
        {
            MOVE_DIR = MOVE_LEFT;                     //??????
            run_speed = local_rc_ctrl->rc.ch[0] * 8;  //???????????????????????????
        }
        else if (local_rc_ctrl->rc.ch[1] > 220 && local_rc_ctrl->rc.ch[0] < -220)
        {
            MOVE_DIR = MOVE_TOP_LEFT;                                                 //??????
            run_speed = get_speed_fb < -get_speed_lr ? get_speed_fb : -get_speed_lr;  //???????????????
        }
        else if (local_rc_ctrl->rc.ch[1] > 220 && local_rc_ctrl->rc.ch[0] > 220)
        {
            MOVE_DIR = MOVE_TOP_RIGHT;                                              //??????
            run_speed = get_speed_fb < get_speed_lr ? get_speed_fb : get_speed_lr;  //???????????????
        }
        else if (local_rc_ctrl->rc.ch[1] < -220 && local_rc_ctrl->rc.ch[0] < -220)
        {
            MOVE_DIR = MOVE_LOW_LEFT;                                               //??????
            run_speed = get_speed_fb < get_speed_lr ? get_speed_fb : get_speed_lr;  //???????????????
        }
        else if (local_rc_ctrl->rc.ch[1] < -220 && local_rc_ctrl->rc.ch[0] > 220)
        {
            MOVE_DIR = MOVE_LOW_RIGHT;                                                //??????
            run_speed = get_speed_fb < -get_speed_lr ? get_speed_fb : -get_speed_lr;  //???????????????
        }

        if (local_rc_ctrl->rc.ch[2] < 0 && local_rc_ctrl->rc.ch[3] == 0)
        {
            MOVE_DIR = MOVE_TURN_LEFT;  //??????
            run_speed = local_rc_ctrl->rc.ch[2] * 8;
        }
        else if (local_rc_ctrl->rc.ch[2] > 0 && local_rc_ctrl->rc.ch[3] == 0)
        {
            MOVE_DIR = MOVE_TURN_RIGHT;  //??????
            run_speed = local_rc_ctrl->rc.ch[2] * 8;
        }
        else if (local_rc_ctrl->rc.ch[3] > 0 && local_rc_ctrl->rc.ch[2] == 0)
        {
            MOVE_DIR = MOVE_CYCLE_LEFT;  //?????????
            run_speed_l = local_rc_ctrl->rc.ch[3] * 4;
            run_speed_r = local_rc_ctrl->rc.ch[3] * 8;
        }
        else if (local_rc_ctrl->rc.ch[3] < 0 && local_rc_ctrl->rc.ch[2] == 0)
        {
            MOVE_DIR = MOVE_CYCLE_RIGHT;  //?????????
            run_speed_l = local_rc_ctrl->rc.ch[3] * 8;
            run_speed_r = local_rc_ctrl->rc.ch[3] * 4;
        }
    }

    else if (local_rc_ctrl->rc.s[0] == 3 && local_rc_ctrl->rc.s[1] == 3)  //??????s1???s2??????3?????????PID??????
    {
        if (local_rc_ctrl->rc.ch[3] == 0 || local_rc_ctrl->rc.ch[3] == 1024)
            key_cnt = 0;
        else
            key_cnt = local_rc_ctrl->rc.ch[3] * 8;

        PID_flag = 1;
        //    CAN_cmd_chassis(M3508_A[0], M3508_A[1], M3508_A[2], M3508_A[3]);//????????????
        //   CAN_cmd_gimbal(0, 0, 0, 0);//??????????????????
        //    HAL_Delay(2);
    }

    else if (local_rc_ctrl->rc.s[0] == 2 && local_rc_ctrl->rc.s[1] == 1)
    {
        servo_up = 1500 - 2 * local_rc_ctrl->rc.ch[3];      //180???????? ???120??
        servo_down = 1340 - 1.2 * local_rc_ctrl->rc.ch[1];  //270???????? ???90??

        if (local_rc_ctrl->rc.ch[3] < 0 || local_rc_ctrl->rc.ch[1] < 0)
        {
            pwm_angle_set(up_pwm, 1500);    //??????????????????????????????
            pwm_angle_set(down_pwm, 1340);  //??????????????????????????????
        }
        else if (local_rc_ctrl->rc.ch[3] >= 0 || local_rc_ctrl->rc.ch[1] >= 0)
        {
            pwm_angle_set(up_pwm, servo_up > 600 ? servo_up : 600);        //?????????????????????
            pwm_angle_set(down_pwm, servo_down > 400 ? servo_down : 400);  //?????????????????????
                                                                           //        pwm_angle_set(up_pwm,1500);//?????????????????????????????????
                                                                           //        pwm_angle_set(down_pwm,1500);//?????????????????????????????????
        }
    }

    else
    {
        PID_flag = 0;
        CAN_cmd_chassis(0, 0, 0, 0);  //????????????
        HAL_Delay(2);
    }
}

/**
  * @brief          Period elapsed callback in non-blocking mode
  * @param[in]      htim TIM handle
  * @retval         none
  */
/**
  * @brief          ???????????????????????????
  * @param[in]      htim:???????????????
  * @retval         none
  */
uint16_t led_times = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  //???????????????1ms
{
    led_times++;
    if (led_times >= 100)
    {
        led_times = 0;
        bsp_led_toggle();  //?????????
    }
}

//        void print_motor(uint8_t rx_data[8])
//{

//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();
    MX_TIM8_Init();
    /* USER CODE BEGIN 2 */
    can_filter_init();
    remote_control_init();
    usart1_tx_dma_init();
    local_rc_ctrl = get_remote_control_point();
    local_rc_ctrl2 = get_remote_control_point();

    HAL_TIM_Base_Start(&htim8);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    //        __HAL_RCC_TIM1_CLK_DISABLE();//???????????????1
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        //    handle_KEY();
        //    pwm_angle_set(1,500);
        control_motor();
        //
        //    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 500);
        //    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 500);
        //      HAL_Delay(1000);

        //    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2000);
        //    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 2000);
        //      HAL_Delay(1000);
    }

    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage 
  */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the CPU, AHB and APB busses clocks 
  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks 
  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
