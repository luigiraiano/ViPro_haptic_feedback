/*
 * Luigi Raiano
 * ViPro Experiment Nucleo f446
 * FW version: v7
 * 2020-08-05
 *
 * Release notes
 * - v7 is the same as the v6 but it uses the mcu f446 instead of f401
 *
 * NOTE 2020/10/14
 * - ho aggiunto la possibilità di settare il workspace non simmetrico, sia per x che per y
 * - quando l'input è joint space position, i motori 1 e 2 vengono scambiati per fare in modo body location e giunti robot siano coerenti (indice k e non i nel ciclo)

 * NOTE 2021/01/27
 * - ho aggiunto offset motori per fare in modo che Input = 0 -> V_mot = 0.2 V.
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

TIM_OC_InitTypeDef TimConfig;
TIM_OC_InitTypeDef sConfigOC;

uint8_t RxBuffer[UART_RX_BUFFER_SIZE];
uint8_t Flag = 0;

union UnionTypeDef{
	double data_double;
	uint8_t twos_complement_array_data[TWOS_COMPL_ARRAY_SIZE];
};

union UnionTypeDef Torque[N_DOF_PROVIDED];
union UnionTypeDef Position[N_DOF_PROVIDED];
union UnionTypeDef Joint_Position[N_DOF_PROVIDED];

//double MAX_JOINT_POSITION[N_DOF_PROVIDED][2] = {{-77, 53}, {-40, 55}, {-100, 100}, {-100, 100}}; // [deg] evaluated with respect the configuration of the robot in the home position
double MAX_JOINT_POSITION[N_DOF_PROVIDED][2] = {{-20, 45}, {-15, 40}, {-100, 100}, {-100, 100}}; // [deg] evaluated with respect the configuration of the robot in the home position
// MAX_JOINT_POSITION = [\delta q_{1,min}, \delta q_{1,max}
//						 \delta q_{2,min}, \delta q_{2,max}];
/***** New 2020/10/14 *****/
double MAX_TASKSPACE_POSITIONS[N_DOF_PROVIDED][2] = {{-0.25, 0.25}, {-0.25, 0.25}, {0, 0}, {0, 0}}; // [m]
/***** New 2020/10/14 *****/

uint32_t duty = 0;
uint32_t value = 0;

/***** New 2021/01/27 *****/
uint32_t duty_off = 0;
uint32_t value_off = 0;
/***** New 2021/01/27 *****/

uint16_t pins[N_DOFS_MAX];
GPIO_TypeDef* ports[N_DOFS_MAX];

struct Timer_info{
	TIM_HandleTypeDef handler;
	uint32_t CHANNEL;
};

uint8_t DOF_SELECTED = 0;

struct Timer_info timer[N_DOF_PROVIDED];

char TxStartMessage[] = "\r\nUSART2 successfully configured\n Ready...\r\n";

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);

void HAL_Delay(__IO uint32_t Delay);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart);
void pwm_init(TIM_HandleTypeDef *timer, uint32_t CHANNEL);
void change_pwm_duty(uint32_t duty_value, TIM_HandleTypeDef *timer, uint32_t CHANNEL);
void GPIO_MotorSelection_settings();
void timer_MotorControlling_settings();
//uint32_t Duty_Value_Linear_Mapping_v1(uint32_t input, uint32_t input_max, uint32_t duty_val_min, uint32_t duty_val_max);
uint32_t Duty_Value_Linear_Mapping_v1(double input, double input_max, double duty_val_min, double duty_val_max);


/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USART2_UART_Init();

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	HAL_Delay(10);

	for(int i=0; i<N_DOFS_MAX; i++)
	{
		HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

	HAL_Delay(100);

	// Set GPIO for motor selection pins in array
	GPIO_MotorSelection_settings();

	HAL_Delay(10);

	// Set timer for motor controlling in array
	timer_MotorControlling_settings();

	pwm_init(&htim1, TIM_CHANNEL_4);
	HAL_Delay(100);
	pwm_init(&htim2, TIM_CHANNEL_1);
	HAL_Delay(100);
	pwm_init(&htim3, TIM_CHANNEL_1);
	HAL_Delay(100);
	pwm_init(&htim4, TIM_CHANNEL_1);
	HAL_Delay(100);

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	HAL_UART_Receive_IT(&huart2, (uint8_t *)&RxBuffer, UART_RX_BUFFER_SIZE);

	while (1)
	{
	}
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 80;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB busses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = PRE;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = PERIOD;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = PRE;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = PERIOD;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = PRE;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = PERIOD;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = PRE;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = PERIOD;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = BAUDRATE;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin|SELECT_MOT_1_Pin|SELECT_MOT_2_Pin|SELECT_MOT_3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SELECT_MOT_4_GPIO_Port, SELECT_MOT_4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin SELECT_MOT_1_Pin SELECT_MOT_2_Pin SELECT_MOT_3_Pin */
	GPIO_InitStruct.Pin = LD2_Pin|SELECT_MOT_1_Pin|SELECT_MOT_2_Pin|SELECT_MOT_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : SELECT_MOT_4_Pin */
	GPIO_InitStruct.Pin = SELECT_MOT_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SELECT_MOT_4_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart){
	if(huart->Instance==USART2){
//		HAL_UART_Receive_IT(&huart2, (uint8_t *)&RxBuffer, UART_RX_BUFFER_SIZE);
		if (HAL_UART_Receive_IT(&huart2, (uint8_t *)&RxBuffer, UART_RX_BUFFER_SIZE) != HAL_OK)
		{
			Error_Handler();
		}

		DOF_SELECTED = RxBuffer[UART_RX_BUFFER_SIZE-2];

		// value = (uint32_t) ( ( (duty_max - duty_min)/current_max)*current_provided +  duty_min );

		int i,j = 0;
		if(RxBuffer[UART_RX_BUFFER_SIZE-1]==115)
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

			uint32_t duty_value[N_DOF_PROVIDED];

			if(RxBuffer[UART_RX_BUFFER_SIZE-3]==99)// torque feedback
			{
				for(int i=0; i<DOF_SELECTED; i++)
				{
					for(j = 0; j<TWOS_COMPL_ARRAY_SIZE; j++)
					{
						Torque[i].twos_complement_array_data[j] = RxBuffer[j + i*TWOS_COMPL_ARRAY_SIZE];
					}


//					duty_value[i] = (uint32_t) abs(Torque[i].data_double*(PERIOD/MAX_TORQUE));
					/****** NEW 2021/01/27 ******/
					duty_value[i] = Duty_Value_Linear_Mapping_v1(Torque[i].data_double, MAX_TORQUE, duty_min, duty_max);
					/****** NEW 2021/01/27 ******/


					if((Torque[i].data_double)>=0)
					{
						HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);
					}
					else
					{
						HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
					}

					change_pwm_duty(duty_value[i], &(timer[i].handler), timer[i].CHANNEL);
				}


				if(DOF_SELECTED<N_DOF_PROVIDED)
				{
					for(int i=(DOF_SELECTED); i<N_DOF_PROVIDED; i++)
					{
						change_pwm_duty(0, &(timer[i].handler), timer[i].CHANNEL);
					}
				}

			}

			else if(RxBuffer[UART_RX_BUFFER_SIZE-3]==112) // end-effector position feedback
			{
				for(int i=0; i<DOF_SELECTED; i++)
				{
					for(j = 0; j<TWOS_COMPL_ARRAY_SIZE; j++)
					{
						Position[i].twos_complement_array_data[j] = RxBuffer[j + i*TWOS_COMPL_ARRAY_SIZE];
					}

					/***** New 2020/10/14 *****/
//					duty_value[i] = (uint32_t) fabs(Position[i].data_double*(PERIOD/MAX_POSTION));
					/***** New 2020/10/14 *****/

					if((Position[i].data_double)>=0)
					{
						/***** New 2020/10/14 *****/
//						duty_value[i] = (uint32_t) fabs(Position[i].data_double*(PERIOD/MAX_TASKSPACE_POSITIONS[i][1]));
						/***** New 2020/10/14 *****/

						/****** NEW 2021/01/27 ******/
						duty_value[i] = Duty_Value_Linear_Mapping_v1(Position[i].data_double, MAX_TASKSPACE_POSITIONS[i][1], duty_min, duty_max);
						/****** NEW 2021/01/27 ******/

						HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);
					}
					else
					{
						/***** New 2020/10/14 *****/
//						duty_value[i] = (uint32_t) fabs(Position[i].data_double*(PERIOD/MAX_TASKSPACE_POSITIONS[i][0]));
						/***** New 2020/10/14 *****/

						/****** NEW 2021/01/27 ******/
						duty_value[i] = Duty_Value_Linear_Mapping_v1(Position[i].data_double, MAX_TASKSPACE_POSITIONS[i][0], duty_min, duty_max);
						/****** NEW 2021/01/27 ******/

						HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
					}

					change_pwm_duty(duty_value[i], &(timer[i].handler), timer[i].CHANNEL);
				}

				if(DOF_SELECTED<N_DOF_PROVIDED)
				{
					for(int i=(DOF_SELECTED); i<N_DOF_PROVIDED; i++)
					{
						change_pwm_duty(0, &(timer[i].handler), timer[i].CHANNEL);
					}
				}
			}

			else if(RxBuffer[UART_RX_BUFFER_SIZE-3]==113) // Joint Position feedback
			{

				/**************NEW 2020/10/14****************/ // inverti giunto 1 con giunto 2
				int k = DOF_SELECTED-1; // inverti giunto 1 e 2 in modo da renderli coerenti con quelli del robot (la coerenza è legata alla posizione più o meno prossimale sul soggetto)
				for(int i=0; i<DOF_SELECTED; i++) // joint1 is the first
				{
					for(j = 0; j<TWOS_COMPL_ARRAY_SIZE; j++)
					{
						Joint_Position[k].twos_complement_array_data[j] = RxBuffer[j + i*TWOS_COMPL_ARRAY_SIZE];
					}

					if((Joint_Position[k].data_double)>=0)
					{
//						duty_value[k] = (uint32_t) fabs(duty_min + Joint_Position[k].data_double*(PERIOD/MAX_JOINT_POSITION[i][1]));

						/****** NEW 2021/01/27 ******/
						duty_value[k] = Duty_Value_Linear_Mapping_v1(Joint_Position[k].data_double, MAX_JOINT_POSITION[k][1], duty_min, duty_max);
						/****** NEW 2021/01/27 ******/

						HAL_GPIO_WritePin(ports[k], pins[k], GPIO_PIN_SET);
					}
					else
					{
//						duty_value[k] = (uint32_t) fabs(duty_min + Joint_Position[k].data_double*(PERIOD/MAX_JOINT_POSITION[i][0]));

						/****** NEW 2021/01/27 ******/
						duty_value[k] = Duty_Value_Linear_Mapping_v1(Joint_Position[k].data_double, MAX_JOINT_POSITION[k][0], duty_min, duty_max);
						/****** NEW 2021/01/27 ******/

						HAL_GPIO_WritePin(ports[k], pins[k], GPIO_PIN_RESET);
					}

					change_pwm_duty(duty_value[k], &(timer[k].handler), timer[k].CHANNEL);

					k = k-1;
				}
				/**************NEW 2020/10/14****************/

				/*------------ OLD VERSION --------*/
				/*
				for(int i=0; i<DOF_SELECTED; i++) // joint1 is the first
//				for(int i=DOF_SELECTED-1; i>=0; i--) // reverse joint1 with joint2 (joint1 is the last one)
				{
					for(j = 0; j<TWOS_COMPL_ARRAY_SIZE; j++)
					{
						Joint_Position[i].twos_complement_array_data[j] = RxBuffer[j + i*TWOS_COMPL_ARRAY_SIZE];
					}

					if((Joint_Position[i].data_double)>=0)
					{
						duty_value[i] = (uint32_t) fabs(Joint_Position[i].data_double*(PERIOD/MAX_JOINT_POSITION[i][1]));
						HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);
					}
					else
					{
						duty_value[i] = (uint32_t) fabs(Joint_Position[i].data_double*(PERIOD/MAX_JOINT_POSITION[i][0]));
						HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
					}

					change_pwm_duty(duty_value[i], &(timer[i].handler), timer[i].CHANNEL);
				}
				*/
				/*------------ OLD VERSION --------*/


				if(DOF_SELECTED<N_DOF_PROVIDED)
				{
					for(int i=(DOF_SELECTED); i<N_DOF_PROVIDED; i++)
					{
						change_pwm_duty(0, &(timer[i].handler), timer[i].CHANNEL);
					}
				}

			}
		}

		else if(RxBuffer[UART_RX_BUFFER_SIZE-1]==116)
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

			for (i=0; i<N_DOF_PROVIDED; i++)
			{
				HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
				change_pwm_duty(0, &(timer[i].handler), timer[i].CHANNEL);
			}
		}
	}
}

void GPIO_MotorSelection_settings()
{
	pins[0] = SELECT_MOT_1_Pin;
	ports[0] = SELECT_MOT_1_GPIO_Port;

	pins[2] = SELECT_MOT_2_Pin;
	ports[2] = SELECT_MOT_2_GPIO_Port;

	pins[1] = SELECT_MOT_3_Pin;
	ports[1] = SELECT_MOT_3_GPIO_Port;

	pins[3] = SELECT_MOT_4_Pin;
	ports[3] = SELECT_MOT_4_GPIO_Port;
}

// NOTA: tim2 e tim3 sono invertiti nell'ordine dell'array perchè sulla board c'è un problema con tim2.
void timer_MotorControlling_settings()
{
	timer[0].handler = htim1;
	timer[0].CHANNEL = TIM_CHANNEL_4;

	timer[2].handler = htim2;
	timer[2].CHANNEL = TIM_CHANNEL_1;

	timer[1].handler = htim3;
	timer[1].CHANNEL = TIM_CHANNEL_1;

	timer[3].handler = htim4;
	timer[3].CHANNEL = TIM_CHANNEL_1;
}

void change_pwm_duty(uint32_t duty_value, TIM_HandleTypeDef *timer, uint32_t CHANNEL){
	/*-------CHANGE PWM DUTY------*/
	if(duty_value >= duty_max){
		duty_value = duty_max;
	}
	else if(duty_value <= duty_min){
		duty_value = duty_min;
	}

	__HAL_TIM_SET_COMPARE(timer, CHANNEL, duty_value);
}


void pwm_init(TIM_HandleTypeDef *timer, uint32_t CHANNEL)
{
	/*-------INITIALIZE AND START PWM------*/

	// Initialize and start pwm
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = duty_min;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(timer, &sConfigOC, CHANNEL) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(timer, CHANNEL) != HAL_OK){
		Error_Handler();
	}   // start pwm generation
}

uint32_t Duty_Value_Linear_Mapping_v1(double input, double input_max, double duty_val_min, double duty_val_max)
//uint32_t Duty_Value_Linear_Mapping_v1(uint32_t input, uint32_t input_max, uint32_t duty_val_min, uint32_t duty_val_max)
{
	uint32_t duty_val_out = 0;

	duty_val_out = (uint32_t) fabs( duty_val_min + input*( (duty_val_max - duty_val_min)/(input_max) ) );

	return duty_val_out;
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	while(1)
	{
	}
}

#ifdef  USE_FULL_ASSERT
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
