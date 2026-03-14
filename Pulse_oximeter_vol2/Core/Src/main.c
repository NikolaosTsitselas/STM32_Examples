/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "Filters.h"
#include "stdio.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

LOW_FilterTypeDef Filter_IR; // an xreiastei

LOW_FilterTypeDef Filter_R; // an xreiastei
//LOW_FilterTypeDef Filter_R_ADC;



//uint32_t duty_cycle_PIR=1279; //25% dc
//uint32_t pwm_PIR=0;

//flags
volatile uint8_t adc_triggered_1=0;
volatile uint8_t adc_triggered_2=0;
volatile uint8_t data_1_ready=0;
volatile uint8_t data_2_ready=0;
volatile uint8_t active_adc=0;
volatile uint8_t phase_shift_flag=0;











volatile uint32_t duty_cycle_CIR=384; //50% dc 639
uint32_t max_duty_cycle_CIR=1151; //90% dc
uint32_t min_duty_cycle_CIR=64; //5% dc

uint16_t duty_cycle_envelope=25; // 1:1 means 35=>35%

volatile uint32_t duty_cycle_CR=384; //50% dc 639
uint32_t max_duty_cycle_CR=1151; //90% dc 1151
uint32_t min_duty_cycle_CR=64; //5% dc
volatile float duty_cycle_CR_per=0;

// ADC and VALUES
uint16_t adc_val[4];

volatile uint16_t ir_val=0;
volatile uint16_t red_val=0;

volatile uint16_t ir_ac_val=0;
volatile uint16_t r_ac_val=0;

uint8_t trigger_step=13;

//Filter
volatile float ir_ac_filtered=0.0; //output of 5hz LPF
volatile float r_ac_filtered=0.0; // output of 5hz LPF


volatile float ir_ac_final=0.0; // final filtered
volatile float r_ac_final=0.0; // final filtered


volatile float r_dc_filtered=0.0;



//temp
volatile float voltage_r=0.0;
volatile float voltage_ir=0.0;


uint16_t low_threshold_adc=2500; //2.014 Vdc
uint16_t high_threshold_adc=2600; //2.054 Vdc


//uint32_t duty_cycle_PR=25; //25% dc
//uint32_t pwm_PR=0;


//For timer interrupt 100 steps=2ms period
volatile uint8_t steps_1=0;
volatile uint8_t steps_2=0;

volatile uint8_t current_steps_1=0;
volatile uint8_t current_steps_2=0;

//for testing purposes
//==========================
//frequency r_ac_filtered
uint16_t x1,x2,x3=0;
uint16_t ton,toff=0;
uint16_t state=0;
float period=0.0;
float frequency=0.0;
//==========================






//for SPO2 measurement

float SPO2=0.0;
uint8_t ready_flag=0;
char data[20]={'\0'};




//PA0=CIR
//PA1=CR
//PA2=PIR
//PA3=PR
//PA4=Red DC signal ADC
//PA5=IR DC signal ADC
//PA6=Red AC Signal ADC
//PA7=IR AC Signal ADC
//PB11=Red signal for CD4066B
//PB12=IR signal for CD4066B



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

//functions for "heavy" cpu usage to place inside while

void r_ac_frequency(){
	if(r_ac_filtered<1900 && state==0){ //detect first falling edge
		x1=HAL_GetTick();
		state=1;
	}

	if(r_ac_filtered>2000 && state==1){ //detect first rising edge
		x2=HAL_GetTick();
		ton=x2-x1;
		state=2;
	}

	if(r_ac_filtered<1900 && state==2){// detect second falling edge
		x3=HAL_GetTick();
		toff=x3-x2;
		period=(ton+toff)/1000.0f;
		frequency=1.0f/period;
		x1=x3;
		state=1;
	}
}










//Ready data for CIR
void CIR_data_ready(){
    if(data_1_ready==1){
        data_1_ready=0;
        ir_val=adc_val[1];
        ir_ac_val=adc_val[3];
        //adc_val[2] and [0] will be garbage

        //Increase Duty Cycle of CIR
		if(ir_val<low_threshold_adc && duty_cycle_CIR<max_duty_cycle_CIR){
			duty_cycle_CIR++;
		}

        //Decrease Duty Cycle of CIR
		if(ir_val>high_threshold_adc && duty_cycle_CIR>min_duty_cycle_CIR){
			duty_cycle_CIR--;
		}




        //filter for IR_AC
        ir_ac_filtered=LOW_PASS_4_ORDER(&Filter_IR,(uint16_t) ir_ac_val); // 5HZ LPF
		ir_ac_final=LOW_PASS_IR_AC(0.003768,0.003768,(uint16_t) ir_ac_filtered); // 0.003768 => 0.3 HZ LPF








    }
}

//Ready data for CR
void CR_data_ready(){
    if(data_2_ready==1){
        data_2_ready=0;
        red_val=adc_val[0];
        r_ac_val=adc_val[2];
        //adc_val[1] and [3] will be garbage


      //  r_dc_filtered=LOW_PASS_4_ORDER(&Filter_R_ADC,(uint16_t) red_val);


            //Increase CR Duty cycle
        	if(red_val<low_threshold_adc && duty_cycle_CR<max_duty_cycle_CR){
						duty_cycle_CR=duty_cycle_CR+1;
					}


                    //Decrease CR Duty cycle
					if(red_val>high_threshold_adc && duty_cycle_CR>min_duty_cycle_CR){
						duty_cycle_CR=duty_cycle_CR-1;
					}



        //Filters for R_AC

        r_ac_filtered=LOW_PASS_4_ORDER(&Filter_R,(uint16_t) r_ac_val); // 5HZ LPF
        r_ac_final=LOW_PASS_R_AC(0.003768,0.003768,(uint16_t) r_ac_filtered); // 0.003768 => 0.3 HZ LPF
        duty_cycle_CR_per=(duty_cycle_CR*100)/1280.0f;
        //frequency
      //  r_ac_frequency();

        ready_flag=0; // ready to measure the SPO2

    }
}







/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_ADCEx_Calibration_Start(&hadc1);


  //FILTER INIT
  LOW_PASS_4_ORDER_Init(&Filter_IR,0.0628f); // 5 Hz LPF
  LOW_PASS_4_ORDER_Init(&Filter_R,0.0628f);  // 5 Hz LPF
  //LOW_PASS_4_ORDER_Init(&Filter_R_ADC,0.2512f); //0.2 Hz LPF

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  //for while loop


	  current_steps_1=steps_1;
	  current_steps_2=steps_2;

	  //trigger_step to trigger once the adc

	  //PIR

	  if(current_steps_1<=duty_cycle_envelope){ // PIR starts
	      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,1);

	      //start PWM for CIR
	      if(current_steps_1<duty_cycle_envelope){
	          TIM2->CCR1=duty_cycle_CIR;
	      }

	      //Trigger the ADC and choose when to trigger
	      //remove trigger_step if you want to trigger in the whole pulse
	          if(current_steps_1==trigger_step && adc_triggered_1==0){
	             // HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_val,4);
	              adc_triggered_1=1;
	              active_adc=1;
	          }


	  //PIR Pulse for CD4066
	  if(current_steps_1>=5 && current_steps_1<=15){
	      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	  }
	  //PIR Pulse stops for CD4066
	  else{
	      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);

	  }

	  }

	  //Stop the PIR Pulse
	  else {
	      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,0);
	  }

	  // Stop CIR PWM 1 step before the PIR pulse ends
	  if(current_steps_1>=duty_cycle_envelope){
	      TIM2->CCR1=0;
	  }

	  //if steps_1 go above 100, reset
	  if(current_steps_1==99){
	      adc_triggered_1=0;
	  }



	  //PR

	      if(phase_shift_flag==1){ // if 50 steps passed
	                               // they are now in sync

	          if(current_steps_2<=duty_cycle_envelope){ // Start PR
	              HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,1);

	              //Start PWM CR
	              if(current_steps_2<duty_cycle_envelope){
	                  TIM2->CCR2=duty_cycle_CR;
	              }

	              //Start PR Pulse for CD4066
	              if(current_steps_2>=5 && current_steps_2<=15){
	                  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
	              }
	              //Stop PR Pulse for CD4066
	              else {
	                  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
	              }

	               //Trigger the ADC and choose when to trigger
	               //remove trigger_step if you want to trigger in the whole pulse
	              if(current_steps_2 == trigger_step && adc_triggered_2==0){
	                  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_val,4);
	                  adc_triggered_2=1;
	                  active_adc=2;
	              }

	          }

	          //Stop PR Pulse
	          else {
	              HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,0);
	          }
	          //Stop PWM CR Pulse 1 step before PR
	          if(current_steps_2>=duty_cycle_envelope){
	              TIM2->CCR2=0;
	          }

	          if(current_steps_2==99){
	              adc_triggered_2=0;

	          }


	      }

	      //insert the 2 functions
	     // CIR_data_ready();
	      CR_data_ready();


	      //to see how much percent of SpO2 in serial monitor
	      /*
	      if(ready_flag==1){
	    	  ready_flag=0;
	    	  SPO2= ((r_ac_final/red_val)/(ir_ac_final/ir_val));
	    	  sprintf(data,"SpO2=%.2lf",SPO2);
	    	  HAL_UART_Transmit_DMA(&huart2,(uint8_t*) data,sizeof(data));
	      }
	      */
	      //end while loop



	  }// while end


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1279;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PIR_Pin|PR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RED_Pin|IR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PIR_Pin PR_Pin */
  GPIO_InitStruct.Pin = PIR_Pin|PR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_Pin IR_Pin */
  GPIO_InitStruct.Pin = RED_Pin|IR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM2){

		steps_1++;

		if(steps_1==50){
		    phase_shift_flag=1;
		}

		if(phase_shift_flag==1){
		    steps_2++;
		}

		if(steps_1>=100) steps_1=0;
		if(steps_2>=100) steps_2=0;

	}
}

	void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){

		if(active_adc==1){
		        data_1_ready=1;
		    }

		    if(active_adc==2){
		        data_2_ready=1;
		    }



	}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
