/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "math.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define dt 0.0002
#define dt2 0.0001
#define dt10 0.02
#define pi 3.141592653589793
#define ppr 3724
#define RX_BUFFER_SIZE 18
#define TX_BUFFER_SIZE 12
#define R 0.075
#define L 0.2125
#define get_dma_data_length() huart6.hdmarx->Instance->NDTR
#define get_dma_total_size() huart6.RxXferSize
//#define tau 0.00048

volatile int32_t encoder_cnt_R = 0;
volatile int32_t encoder_cnt_L = 0;
volatile int32_t encoder_cnt_pre_R = 0;
volatile int32_t encoder_cnt_pre_L = 0;

volatile int16_t cnt_R = 0;
volatile int16_t cnt_L = 0;

volatile double angular_velocity_R = 0;
volatile double angular_velocity_L = 0;
volatile double angular_velocity_pre_R = 0;
volatile double angular_velocity_pre_L = 0;
volatile double theta_R = 0;
volatile double theta_L = 0;
volatile double delta_s = 0;
volatile double delta_theta = 0;

//volatile int g_cnt = 0;

volatile uint16_t Motor_CCR_R = 0;
volatile uint16_t Motor_CCR_L = 0;
volatile uint16_t timer = 0;
volatile uint16_t ctimer = 0;
volatile uint16_t ptimer = 0;
volatile uint16_t ktimer = 0;


volatile double rep_r = 0;
volatile double rep_l = 0;
volatile double rep_pre_r = 0;
volatile double rep_pre_l = 0;
volatile double Kps[2] = {80.5, 61.5};//{74, 71};68 58
volatile double Kis[2] = {71.24, 68.12};//{55, 57};45 62
volatile double Kas[2] = {0.00455, 0.00555};
volatile double Es[2] = {0, 0};
volatile double Eis[2] = {0, 0};
volatile double speed[2] = {0, 0};

volatile int16_t lin_vel;
volatile int16_t ang_vel;

volatile uint8_t lin_vel_L;
volatile uint8_t lin_vel_H;
volatile uint8_t ang_vel_L;
volatile uint8_t ang_vel_H;

volatile int16_t sop = 0x55;
volatile int16_t checksum;

volatile uint8_t checksum_H;
volatile uint8_t checksum_L;

volatile char packet[RX_BUFFER_SIZE];
//volatile uint8_t parsing[RX_BUFFER_SIZE];

volatile int16_t plin_vel;
volatile int16_t pang_vel;
volatile int16_t pcheck;




uint8_t rx_dma_buffer[RX_BUFFER_SIZE];
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint32_t rx_size = 0;
uint8_t tx_len;
uint8_t tx_buffer[TX_BUFFER_SIZE];

uint16_t _prevRxXferedCount = 0;
uint16_t _lastRxXferedCount = 0;


uint16_t old_pos = 0;
uint16_t pos;
uint16_t umm;
uint16_t umm2;
uint16_t umm3;

int16_t trans_x;
int16_t trans_y;
int16_t trans_yaw;

uint8_t trans_x_L;
uint8_t trans_x_H;
uint8_t trans_y_L;
uint8_t trans_y_H;
uint8_t trans_yaw_L;
uint8_t trans_yaw_H;
int16_t trans_checksum;

uint8_t trans_checksum_L;
uint8_t trans_checksum_H;
uint8_t tx_odom[12];
uint8_t dir = 0;
volatile double odometry[3];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void Uart_rx_dma_handler() {
        

  uint8_t cnt;
  uint8_t i;
  uint8_t temp[12];
  
  rx_size = 0;
  pos = get_dma_total_size() - get_dma_data_length();
  
  i = 0;
  
  while(1) {
    if((rx_dma_buffer[i] == rx_dma_buffer[i+1]) && (rx_dma_buffer[i] == 'U')) break;
    else i++;
  }
  
  if(pos != old_pos) {
    if(pos > old_pos) {
        //umm++;//test
        memcpy(&rx_buffer[old_pos], &rx_dma_buffer[(old_pos+i)%RX_BUFFER_SIZE], sizeof(uint8_t)*(pos - old_pos));
        rx_size += pos - old_pos;
    }else {
        //umm2++;//test
        memcpy(&rx_buffer[old_pos], &rx_dma_buffer[(old_pos+i)%RX_BUFFER_SIZE], sizeof(uint8_t)*(get_dma_total_size() - old_pos));
        rx_size += get_dma_total_size() - old_pos;
        if(pos > 0){
                memcpy(&rx_buffer[get_dma_total_size() - old_pos], &rx_dma_buffer[0], sizeof(uint8_t)*pos);
                rx_size += get_dma_total_size() - old_pos;
        }
    }
    old_pos = pos;
    if(rx_size > 0) {
      for(cnt = 0;cnt<rx_size;cnt++) {
        pcheck = (rx_buffer[6] << 8) | rx_buffer[7] & 0xff;
        if(pcheck == rx_buffer[0] + rx_buffer[1] + rx_buffer[2] + rx_buffer[3] + rx_buffer[4] + rx_buffer[5]) {
          plin_vel = (rx_buffer[2] << 8) | rx_buffer[3] & 0xff;
          pang_vel = (rx_buffer[4] << 8) | rx_buffer[5] & 0xff;
          
        }
      }
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(htim == &htim4) {
    //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
    timer++;
    
    if(timer ==2) {
      ctimer++;//////////
      ptimer++;
      
      timer = 0;
      
      rep_pre_r = cos(pi);
      
      //if(ptimer < 91000) ptimer++;
      
      //TIM4 CHN2 R CHN1 L
      //DIR PA5 L PA4 R
      //ADC IN14 L IN15 R
     
     if(ctimer == 100){
        
        encoder_cnt_pre_R = encoder_cnt_R;
        encoder_cnt_pre_L = encoder_cnt_L;
        encoder_cnt_R = TIM3->CNT;
        encoder_cnt_L = TIM2->CNT;
        
        if((pang_vel < 1000 & pang_vel > -1000) & (plin_vel < 1000 & plin_vel > -1000)) {
          rep_r = 0.001*(pang_vel*L + plin_vel)/R;
          rep_l = 0.001*(-pang_vel*L + plin_vel)/R;
        }
        
        cnt_R = encoder_cnt_R-encoder_cnt_pre_R;
        cnt_L = encoder_cnt_L-encoder_cnt_pre_L;
        
        if(encoder_cnt_R < 100 & encoder_cnt_pre_R > 65450 ) cnt_R += 65535;
        else if(encoder_cnt_pre_R < 100 & encoder_cnt_R > 65000) cnt_R -= 65535;
        if(encoder_cnt_L < 100 & encoder_cnt_pre_L > 65450 ) cnt_L += 65535;
        else if(encoder_cnt_pre_L < 100 & encoder_cnt_L > 65000) cnt_L -= 65535;
        
        
        theta_R = (double)(2*pi*cnt_R/ppr);
        theta_L = (double)(2*pi*cnt_L/ppr);
        angular_velocity_R = (double)theta_R/dt10;
        angular_velocity_L = (double)theta_L/dt10;
        

        
        
        Es[0] = (double)(rep_r - angular_velocity_R);
        Eis[0] += (double)Es[0];
        speed[0] = (double)Kps[0] * Es[0] + Kis[0] * Eis[0];
        
        if(speed[0] >= 4198) {
          Eis[0] -= (double)1/Kps[0]*(speed[0] - 4198);
          speed[0] = 4198;
        }
        else if(speed[0] <= -4198) {
          Eis[0] -= (double)1/Kps[0]*(speed[0] - (-4198));
          speed[0] = -4198;
        }
        
        Es[1] = (double)(rep_l - angular_velocity_L);
        Eis[1] += (double)Es[1];
        speed[1] = (double)Kps[1] * Es[1] + Kis[1] * Eis[1];
        if(speed[1] >= 4198) {
          Eis[1] -= (double)1/Kps[1]*(speed[1] - 4198);
          speed[1] = 4198;
        }
        else if(speed[1] <= -4198) {
          Eis[1] -= (double)1/Kps[1]*(speed[1] - (-4198));
          speed[1] = -4198;
        }
        
        if(speed[1] < 260 && speed[1] > -260)     speed[1] = 0;


        
        if(speed[1]<0) {  //哭率官柠 开规氢
          HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
          speed[1]=-speed[1];
          dir = 0;
        }
        else {
          HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);  
          //speed[1]=speed[1];
          dir = 1;
        }
        if(speed[0] < 0 ) {
          HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
          speed[0] = -speed[0];
        }
        else {
          HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
          //speed[0] = speed[0];
        }
        
        Motor_CCR_R = (uint16_t)speed[0];
        Motor_CCR_L = (uint16_t)speed[1];
        
        //TIM4->CCR1 = Motor_CCR_L;
        //TIM4->CCR2 = Motor_CCR_R;
        
        delta_s = (double)R*(theta_R + theta_L)/2.0;
        delta_theta = (double)R*(theta_R - theta_L)/(2.0*L);
        odometry[0] += delta_s * cos(odometry[2] + (delta_theta/2.0));
        odometry[1] += delta_s * sin(odometry[2] + (delta_theta/2.0));
        odometry[2] += delta_theta;
        if(odometry[2] < -1 * pi) { 
          odometry[2] += 2.0*pi;
        }
        else if(odometry[2] > pi) {
          odometry[2] -= 2.0*pi;
        }
        
        
        
        ctimer = 0;
        //rep_pre_r = rep_r;
        //rep_pre_l = rep_l;
        
      }
     if(ptimer == 1500) {
      
       trans_x = (int16_t)1000*odometry[0];
       trans_y = (int16_t)1000*odometry[1];
       trans_yaw = (int16_t)1000*odometry[2];

        trans_x_L = trans_x & 0xff;
        trans_x_H = trans_x >>8;
        trans_y_L = trans_y & 0xff;;
        trans_y_H = trans_y >>8;
        trans_yaw_L = trans_yaw & 0xff;;
        trans_yaw_H = trans_yaw >>8;
        trans_checksum = sop+sop+trans_x_L+trans_x_H+trans_y_L+trans_y_H+trans_yaw_L+trans_yaw_H;
        trans_checksum_L = trans_checksum & 0xff;
        trans_checksum_H = trans_checksum >>8;
        sprintf(tx_odom,"%c%c%c%c%c%c%c%c%c%c\n",sop,sop,trans_x_H,trans_x_L,
                  trans_y_H,trans_y_L,trans_yaw_H,trans_yaw_L,trans_checksum_H,trans_checksum_L);
        HAL_UART_Transmit(&huart6,tx_odom,sizeof(tx_odom), 10);
         
        ptimer = 0;
     }/////////
      
      
      
    }
    TIM4->CCR1 = Motor_CCR_L;
    TIM4->CCR2 = Motor_CCR_R;


    //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
   }
}


/*
void HAL_UART_CallBack(UART_HandleTypeDef *huart) {
  
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  tx_len = sizeof(tx_buffer) -1;
  HAL_UART_Transmit(&huart6, tx_buffer, tx_len, HAL_MAX_DELAY);

}*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_UART_Receive_DMA(&huart6, rx_dma_buffer, RX_BUFFER_SIZE);
  //HAL_UART_Receive_IT(&huart6,rx_dma_buffer,RX_BUFFER_SIZE);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);//R RESET 沥 SET 开
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*g_cnt = 0;
    
    
    pcheck = (rx_buffer[6] << 8) | rx_buffer[7] & 0xff;
    if(pcheck == rx_buffer[0] + rx_buffer[1] + rx_buffer[2] + rx_buffer[3] + rx_buffer[4] + rx_buffer[5]) {
      plin_vel = (rx_buffer[2] << 8) | rx_buffer[3] & 0xff;
      pang_vel = (rx_buffer[4] << 8) | rx_buffer[5] & 0xff;
    }
    */
    Uart_rx_dma_handler();

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(sampling_GPIO_Port, sampling_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : sampling_Pin */
  GPIO_InitStruct.Pin = sampling_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(sampling_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
