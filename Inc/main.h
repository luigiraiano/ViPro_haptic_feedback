/*
 * Luigi Raiano
 * ViPro Experiment Nucleo f446
 * FW version: v7
 * 2020-08-05
 *
 * Release notes
 * - v7 is the same as the v6 but it uses the mcu f446 instead of f401
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define SELECT_MOT_4_Pin GPIO_PIN_14
#define SELECT_MOT_4_GPIO_Port GPIOB
#define SELECT_MOT_1_Pin GPIO_PIN_8
#define SELECT_MOT_1_GPIO_Port GPIOA
#define SELECT_MOT_2_Pin GPIO_PIN_9
#define SELECT_MOT_2_GPIO_Port GPIOA
#define SELECT_MOT_3_Pin GPIO_PIN_10
#define SELECT_MOT_3_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

// 100 Hz interrupt for 100 Hz pwm wave
#define PRE 7
#define PERIOD 49999

#define DUTY_MAX PERIOD // -> VCC = 3.6V
#define DUTY_MIN (round((0.2/3.6)*DUTY_MAX)) // -> VCC = 0.2V
//#define DUTY_MIN 0

#define MAX_TORQUE 0.8 // Nm
//#define MAX_TORQUE 3 // N
#define MAX_POSTION 0.25 // m

#define N_ACTUATOR_PER_DOF 2
#define N_DOF_PROVIDED N_DOFS_MAX
#define N_ACTUATORS N_ACTUATOR_PER_DOF*N_DOF_PROVIDED
#define TWOS_COMPL_ARRAY_SIZE 8
#define UART_RX_BUFFER_SIZE (TWOS_COMPL_ARRAY_SIZE*N_DOF_PROVIDED + 1 + 1 + 1) //tau*N_DOFS_MAX + flag_stim_type + flag_dof_selected + flag_status
#define N_DOFS_MAX 4

//#define BAUDRATE 115200
#define BAUDRATE 921600

#define duty_max PERIOD // -> V_mot = Vcc V
#define duty_min (round((0/3.6)*duty_max)) // -> V_mot = 0.2 @ Vcc = 3.6 V


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
