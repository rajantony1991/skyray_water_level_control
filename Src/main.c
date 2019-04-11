
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "eeprom.h"
#define LOW_VOLTAGE  180         //MACRO FOR LOW VOLTAGE LEVEL
#define OVER_VOLTAGE 250         //MACRO FOR HIGH VOLTAGE LEVEL

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
RTC_TimeTypeDef sTime,bTime;
RTC_DateTypeDef sDate;
uint16_t adc_raw[3],Phase_voltage;
uint8_t Adc_sample = 1;
uint32_t Avg_voltage = 0 ;
uint8_t  Over_tank_level  = 0 ;
int32_t  Restart_time , Dry_run_time ,Dry_run_count,Restart_time_count;
bool Motor_auto_start =false  , Motor_on = false ,Over_tank_water_flow = false,Dry_run_wait = true ,Dry_run_error = false,Power_interrupt_state = false;
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x1000,0x1001,0x1002,0x1003,0x1004,0x1005,0x1006,0x1008,0x1009,0x1010};
uint16_t VarDataTab[NB_OF_VAR]    = {1,2,3,4,5,6,7,8,9,10,11};
uint16_t VarValue = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

//HAL_GPIO_EXTI_Callback(POWER_INTERRUPT_Pin)
//{
//
//
//
//}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{


Adc_sample ++;
Avg_voltage += adc_raw[0];
if(Adc_sample>=10)
{
	Phase_voltage  = (Avg_voltage/Adc_sample)/10;
	Phase_voltage += 10;
	Avg_voltage    = 0 ;
	Adc_sample     = 0 ;
}

}

void Get_time_date()
{

HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);

}

void Buzzer_sound(uint8_t Pulse , uint32_t Delay)
{

for(uint8_t i = 0 ; i<Pulse;i++)
{
HAL_GPIO_WritePin(GPIOC,BUZZER_OUT_Pin,GPIO_PIN_SET);
HAL_Delay(Delay);
HAL_GPIO_WritePin(GPIOC,BUZZER_OUT_Pin,GPIO_PIN_RESET);
HAL_Delay(Delay);

}


}


int32_t Map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void motor_control_loop() // motor control function for turn on and off motor
{

   	if(Motor_on == false)
    	{
   	     Buzzer_sound(2,500);                                      //function call for buzzer sound
    	 HAL_GPIO_WritePin(GPIOB ,RELAY_1_Pin|RELAY_2_Pin,GPIO_PIN_SET);       // TRUN ON relay 2 //(START RELAY)
    	 HAL_GPIO_WritePin(GPIOC ,RELAY_3_Pin,GPIO_PIN_SET);       // TURN ON relay 3 //(START RELAY 1 PHASE )
    	 HAL_Delay(3000);                                          // MOTOR starting wait time 3 sec
    	 HAL_GPIO_WritePin(GPIOB ,RELAY_2_Pin,GPIO_PIN_RESET);     // TRUN OFF relay 2 //(START RELAY)
    	 HAL_GPIO_WritePin(GPIOC ,RELAY_3_Pin,GPIO_PIN_RESET);     // TRUN OFF relay 3 //(START RELAY 1 PHASE)
    	 Dry_run_time = Map(adc_raw[1],0,4095,60,240);               // Read tha dry run sense POT and mab tha value to minimum= 1 minutes to max 20min
    	 Motor_on = true ;                                         // UPDATE Variable to current state
    	 HAL_TIM_Base_Stop_IT(&htim1);                             //stop Timer 1 interrupt
    	 HAL_TIM_Base_Init(&htim1)  ;                               // setting values Basic Timer interrupt
    	 HAL_TIM_Base_Start_IT(&htim1);                            // enable Timer 1 interrupt to generate 1 sec delay

    	}

        if(Dry_run_wait != true)                                   // wait until dry run reset time
        {
         HAL_TIM_Base_Stop_IT(&htim1);                             //stop Timer 1 interrupt
         if(Over_tank_water_flow==false)                           // check over tank water flow status
         {

          HAL_GPIO_WritePin(GPIOB,RELAY_1_Pin,GPIO_PIN_RESET);    // stop the motor for dry run condtion
          Dry_run_error = true ;                                  // SET DRY RUN ERROR BIT  to high
          Motor_on = false ;                                      // update motor off state
          Restart_time = Map(adc_raw[2],0,4095,60,43200);           // get the dryrun_restart POT value and mab into time in minutes
          HAL_TIM_Base_Stop_IT(&htim16);                          // stop timer if run
          HAL_TIM_Base_Init(&htim16)  ;                           // inti timer-16
          HAL_TIM_Base_Start_IT(&htim16);                         // start timer16 isr
          Dry_run_wait = true ;                                   // clear dry run wait bit for next execution
          Buzzer_sound(5,500);                                    //fuction call for buzzer sound
         }



        }




}

void reset_default()
{
	Motor_on      = false ;
	Dry_run_error = false ;
	Dry_run_wait  = true  ;                                       // clear all variable to default value
	Motor_auto_start = true;
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim16);
	HAL_TIM_Base_Init(&htim1) ;
	HAL_TIM_Base_Init(&htim16);
	Restart_time_count = 0 ;
	Dry_run_count      = 0 ;
	 HAL_GPIO_WritePin(GPIOA,LED_SUMP_TANK_HIGH_Pin|LED_SUMP_TANK_LOW_Pin , GPIO_PIN_SET);


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) //ISR for LEVEL_sensor checking
{

	//  HAL_GPIO_TogglePin(GPIOB,LED_VOLTAGE_LOW_Pin);   // TEST DEBUG CODE
	if(htim->Instance == htim14.Instance)
	{
	  if((HAL_GPIO_ReadPin(GPIOF,OVER_LOW_INPUT_Pin)==GPIO_PIN_SET)&&(HAL_GPIO_ReadPin(GPIOF,OVER_HIGH_INPUT_Pin)==GPIO_PIN_SET))
	  {

		    Over_tank_level = 1 ;  //TANK level is empty
		    HAL_GPIO_WritePin(GPIOA,LED_OVER_TANK_LOW_Pin,GPIO_PIN_RESET);// Over tank low  led ON
		    HAL_GPIO_WritePin(GPIOA,LED_OVER_TANK_HIGH_Pin,GPIO_PIN_SET);// Over tank hIGH  led Off

	  }
	  if((HAL_GPIO_ReadPin(GPIOF,OVER_LOW_INPUT_Pin)==GPIO_PIN_RESET)&&(HAL_GPIO_ReadPin(GPIOF,OVER_HIGH_INPUT_Pin)==GPIO_PIN_SET))
	  {
		    Over_tank_level = 2 ;//TANK level is half empty
			HAL_GPIO_WritePin(GPIOA,LED_OVER_TANK_LOW_Pin,GPIO_PIN_SET);// Over tank low  led Off
			HAL_GPIO_WritePin(GPIOA,LED_OVER_TANK_HIGH_Pin,GPIO_PIN_SET);// Over tank hIGH  led Off

	  }
	  if((HAL_GPIO_ReadPin(GPIOF,OVER_LOW_INPUT_Pin)==GPIO_PIN_RESET)&&(HAL_GPIO_ReadPin(GPIOF,OVER_HIGH_INPUT_Pin)==GPIO_PIN_RESET))
	  {

		    Over_tank_level = 3 ;//TANK level is full
		    HAL_GPIO_WritePin(GPIOA,LED_OVER_TANK_HIGH_Pin,GPIO_PIN_RESET);// Over tank hIGH  led ON
		    HAL_GPIO_WritePin(GPIOA,LED_OVER_TANK_LOW_Pin,GPIO_PIN_SET);// Over tank low  led OFF

	  }
	  if((HAL_GPIO_ReadPin(GPIOF,OVER_LOW_INPUT_Pin)==GPIO_PIN_SET)&&(HAL_GPIO_ReadPin(GPIOF,OVER_HIGH_INPUT_Pin)==GPIO_PIN_RESET))
	  {
		  Over_tank_level = 10 ;// error sensor stage ;
		  HAL_GPIO_WritePin(GPIOA,LED_OVER_TANK_HIGH_Pin,GPIO_PIN_SET);// Over tank hIGH  led OFF
		  HAL_GPIO_WritePin(GPIOA,LED_OVER_TANK_LOW_Pin,GPIO_PIN_SET);// Over tank low  led OFF

	  }
	  if(Phase_voltage>=OVER_VOLTAGE) // voltage Level checking
	  {
             Over_tank_level = 4;// over votage state
		     HAL_GPIO_WritePin(GPIOB , LED_VOLTAGE_HIGH_Pin,GPIO_PIN_RESET);//over voltage led ON
		     HAL_GPIO_WritePin(GPIOB , LED_VOLTAGE_LOW_Pin ,GPIO_PIN_SET);  //low voltage led OFF

	  }
	  else if(Phase_voltage <=LOW_VOLTAGE)
	  {
		     Over_tank_level = 4;// low voltage state
		     HAL_GPIO_WritePin(GPIOB , LED_VOLTAGE_HIGH_Pin,GPIO_PIN_SET);//over voltage led ON
		     HAL_GPIO_WritePin(GPIOB , LED_VOLTAGE_LOW_Pin ,GPIO_PIN_RESET);  //low voltage led OFF

	  }
	  else
	  {

		     HAL_GPIO_WritePin(GPIOB , LED_VOLTAGE_HIGH_Pin,GPIO_PIN_SET);//over voltage led ON
		     HAL_GPIO_WritePin(GPIOB , LED_VOLTAGE_LOW_Pin ,GPIO_PIN_SET);  //low voltage led OFF

	  }
       if(Over_tank_water_flow == true)
       {
    	   HAL_GPIO_TogglePin (GPIOA , LED_SUMP_INLET_Pin);  // over tank inlet water flow status
       }
       else
       {
    	  HAL_GPIO_WritePin  (GPIOA,LED_SUMP_INLET_Pin,GPIO_PIN_SET);// LED TURN OFF when no water flow

       }

	}

	if(htim->Instance == htim1.Instance) // Timer 1 ISR Handler
	{

		HAL_GPIO_TogglePin(GPIOA,LED_SUMP_TANK_HIGH_Pin); // test code
		Dry_run_count++;                                  // increment dryrun count in each 1 min interrupt
		if(Dry_run_count>=Dry_run_time)
		{

			Dry_run_wait = false ; // clear dry run wait bit to next execution
			Dry_run_count = 0;     //reset dry run count variable for next time use

		}

	}

	if(htim->Instance == htim16.Instance)
	{
		HAL_GPIO_TogglePin(GPIOA,LED_SUMP_TANK_LOW_Pin); // test code
		Restart_time_count++;
		if(Restart_time_count>=Restart_time)
		{
			Dry_run_error = false ;
			Restart_time_count = 0 ;

		}

	}



}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == POWER_INTERRUPT_Pin )
  {

	  HAL_GPIO_TogglePin(GPIOA,LED_SUMP_TANK_LOW_Pin);
	  EE_WriteVariable(VirtAddVarTab[0] , (uint16_t)Dry_run_error);
	  EE_WriteVariable(VirtAddVarTab[1],  (uint16_t)Restart_time_count);
	  EE_WriteVariable(VirtAddVarTab[2],  (uint16_t)Restart_time);
	  EE_WriteVariable(VirtAddVarTab[3],  (uint16_t)Motor_auto_start);
  }
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_ADC_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc,(uint32_t *)adc_raw,3);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_GPIO_WritePin(GPIOA,LED_SUMP_TANK_HIGH_Pin|LED_SUMP_TANK_LOW_Pin , GPIO_PIN_SET);
  HAL_FLASH_Unlock();
  EE_Init();

//  /* read the last stored variables data*/

  EE_ReadVariable(VirtAddVarTab[0], &VarDataTab[0]); //read flash value
  EE_ReadVariable(VirtAddVarTab[1], &VarDataTab[1]); //
  EE_ReadVariable(VirtAddVarTab[2], &VarDataTab[2]); //
  EE_ReadVariable(VirtAddVarTab[3], &VarDataTab[3]); //
  if(VarDataTab[3] >= 1) //
  {
	Motor_auto_start = true;             // motor auto start is enabled

  }
  else
  {

  Motor_auto_start = false;             // motor auto start is disabled

  }
  if(VarDataTab[0] >= 1)              // check power failure state
 {
 Dry_run_error = true ;
 if((VarDataTab[1]>=1)&&(VarDataTab[1]<=43200)) //check data in correct value
 {
	 Restart_time_count = VarDataTab[1] ;       //restore the restart time from flash
	 Restart_time       = VarDataTab[2] ;       //restore the restart POT value
	 HAL_TIM_Base_Start_IT(&htim16);            // start timer16 isr to time remaining
 }

 }
 else
 {

  Dry_run_error = false ;                 // dry run not accrued

 }




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(HAL_GPIO_ReadPin(GPIOA,SW_RESET_Pin)==GPIO_PIN_RESET)       //reset switch
	  {

	  HAL_Delay(100);                                                 // wait for switch de-bounce
      if(HAL_GPIO_ReadPin(GPIOA,SW_RESET_Pin)==GPIO_PIN_RESET)        //read reset switch state
      {
    	HAL_GPIO_WritePin(GPIOB ,RELAY_1_Pin,GPIO_PIN_RESET);         // stop the motor for reset condition
    	HAL_GPIO_WritePin(GPIOB ,RELAY_2_Pin,GPIO_PIN_RESET);         // TRUN OFF relay 2 //(START RELAY)
    	HAL_GPIO_WritePin(GPIOC ,RELAY_3_Pin,GPIO_PIN_RESET);         // TRUN OFF relay 3 //(START RELAY 1 PHASE)
    	HAL_GPIO_WritePin(GPIOB ,LED_MOTOR_ON_Pin,GPIO_PIN_SET);          // TURN OFF  motor_on led
    	reset_default();                                              // call function to reset the variable to default value
    	Buzzer_sound(1,200);                                          // buzzer sound for 2 time on off
    	HAL_Delay(10000);                                             // wait for 10 sec delay
      }
	  }

      Get_time_date();
      if((sTime.Seconds%10) == 1) // CONTACT SENSOR checking for every 10 sec
      {
    		HAL_GPIO_WritePin(GPIOA ,OVER_TRIGGER_OUT_Pin,GPIO_PIN_SET);      // enable the trigger pin high
    	    HAL_Delay(500);                                                   // wait for 5 sec
    	    if(HAL_GPIO_ReadPin(GPIOB,OVER_SENSE_INPUT_Pin)== GPIO_PIN_RESET) // read the pin state
    	    {

    	      Over_tank_water_flow =  true ; // set high water flow is present

    	    }
    	    else //
    	    {

    	     Over_tank_water_flow  =  false ;// set low for water flow not detected

    	    }

    	    HAL_GPIO_WritePin(GPIOA ,OVER_TRIGGER_OUT_Pin,GPIO_PIN_RESET);//disable the trigger pin low


      }



    	if(Power_interrupt_state == false)
    	{
    	    if(sTime.Seconds>=10)
    	    {
    		HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
            Power_interrupt_state = true ;
    	    }

    	}



      if(Motor_on == true) //check Motor Status
        {

         HAL_GPIO_WritePin(GPIOB,LED_MOTOR_ON_Pin,GPIO_PIN_RESET);	// turn the Motor led ON

        }
      else
      {

        HAL_GPIO_WritePin(GPIOB,LED_MOTOR_ON_Pin,GPIO_PIN_SET);    // turn the motor led OFF

      }
      if(Dry_run_error == true)                                   // check dry run error flage
      {
    	HAL_GPIO_WritePin(GPIOB,LED_DRY_RUN_Pin,GPIO_PIN_RESET);  // turn the dry run error LED ON

      }
      else
      {

    	HAL_GPIO_WritePin(GPIOB,LED_DRY_RUN_Pin,GPIO_PIN_SET);  // turn the dry run error LED OFF
    	HAL_TIM_Base_Stop_IT(&htim16);
      }

  if(Dry_run_error == false) // check dry run flag if not true the normal execution state
    {
      switch( Over_tank_level )// mode of execution
      {

      case 1:// tank empty state
      {
    	  Motor_auto_start = true; // motor auto turn on this flage cleared when tank get full also used for power resume
    	  motor_control_loop();


      break;
      }
      case 2://tank half empty
      {
         if(Motor_auto_start == true)
         {
    	  motor_control_loop();
         }

      break;
      }
      case 3://tank full
      {

    	  if(Motor_on == true)
    	  {
    		  HAL_GPIO_WritePin (GPIOB,RELAY_1_Pin|RELAY_2_Pin,GPIO_PIN_RESET); //turn off relay1,2
    		  HAL_GPIO_WritePin (GPIOC,RELAY_3_Pin,GPIO_PIN_RESET);             //turn off relay 3
    		  HAL_Delay(100);
    		  reset_default();
    		  Buzzer_sound(3,500);


    	  }

      Motor_auto_start = false ;
      break;
      }
      case 4://high and low voltage error
      {
    	  if(Motor_on == true)
    	  {
    	  HAL_GPIO_WritePin (GPIOB,RELAY_1_Pin|RELAY_2_Pin,GPIO_PIN_RESET); //turn off relay1,2
      	  HAL_GPIO_WritePin (GPIOC,RELAY_3_Pin,GPIO_PIN_RESET);             //turn off relay 3
      	  Motor_on = false ;
          HAL_TIM_Base_Stop_IT(&htim1);
      	  HAL_TIM_Base_Stop_IT(&htim16);
      	  HAL_TIM_Base_Init(&htim1) ;
      	  HAL_TIM_Base_Init(&htim16);

    	  }
      break;
      }
      case 10:// sensor error stage Toggle OVER Tank high and  low LED ,buzzer
      {

    	  HAL_GPIO_TogglePin(GPIOA,LED_OVER_TANK_HIGH_Pin|LED_OVER_TANK_LOW_Pin);
    	  HAL_GPIO_TogglePin(GPIOC,BUZZER_OUT_Pin);
    	  HAL_GPIO_WritePin (GPIOB,RELAY_1_Pin|RELAY_2_Pin,GPIO_PIN_RESET);
    	  HAL_Delay(500);
          HAL_TIM_Base_Stop_IT(&htim1);
      	  HAL_TIM_Base_Stop_IT(&htim16);
      	  HAL_TIM_Base_Init(&htim1) ;
      	  HAL_TIM_Base_Init(&htim16);
          break;
      }


      }
} // dry run execution end condition


      if(Over_tank_level != 10)
      {

      HAL_GPIO_WritePin(GPIOC,BUZZER_OUT_Pin,GPIO_PIN_RESET);

      }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL10;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 40000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1001;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 500;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 30000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM14 init function */
static void MX_TIM14_Init(void)
{

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 40000;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 2001;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 40000;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1001;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BUZZER_OUT_Pin|RELAY_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_OVER_TANK_HIGH_Pin|LED_OVER_TANK_LOW_Pin|LED_SUMP_INLET_Pin|LED_SUMP_TANK_HIGH_Pin 
                          |LED_SUMP_TANK_LOW_Pin|SUMP_TRIGGER_OUT_Pin|OVER_TRIGGER_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELAY_2_Pin|RELAY_1_Pin|LED_MOTOR_ON_Pin|LED_DRY_RUN_Pin 
                          |LED_VOLTAGE_HIGH_Pin|LED_VOLTAGE_LOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUZZER_OUT_Pin RELAY_3_Pin */
  GPIO_InitStruct.Pin = BUZZER_OUT_Pin|RELAY_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_OVER_TANK_HIGH_Pin LED_OVER_TANK_LOW_Pin LED_SUMP_INLET_Pin LED_SUMP_TANK_HIGH_Pin 
                           LED_SUMP_TANK_LOW_Pin */
  GPIO_InitStruct.Pin = LED_OVER_TANK_HIGH_Pin|LED_OVER_TANK_LOW_Pin|LED_SUMP_INLET_Pin|LED_SUMP_TANK_HIGH_Pin 
                          |LED_SUMP_TANK_LOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_INTERRUPT_Pin */
  GPIO_InitStruct.Pin = POWER_INTERRUPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(POWER_INTERRUPT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_2_Pin RELAY_1_Pin */
  GPIO_InitStruct.Pin = RELAY_2_Pin|RELAY_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_MOTOR_ON_Pin LED_DRY_RUN_Pin LED_VOLTAGE_HIGH_Pin LED_VOLTAGE_LOW_Pin */
  GPIO_InitStruct.Pin = LED_MOTOR_ON_Pin|LED_DRY_RUN_Pin|LED_VOLTAGE_HIGH_Pin|LED_VOLTAGE_LOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_RESET_Pin SUMP_SENSE_INPUT_Pin SUMP_HIGH_INPUT_Pin SUMP_LOW_INPUT_Pin */
  GPIO_InitStruct.Pin = SW_RESET_Pin|SUMP_SENSE_INPUT_Pin|SUMP_HIGH_INPUT_Pin|SUMP_LOW_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SUMP_TRIGGER_OUT_Pin OVER_TRIGGER_OUT_Pin */
  GPIO_InitStruct.Pin = SUMP_TRIGGER_OUT_Pin|OVER_TRIGGER_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OVER_LOW_INPUT_Pin OVER_HIGH_INPUT_Pin */
  GPIO_InitStruct.Pin = OVER_LOW_INPUT_Pin|OVER_HIGH_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : OVER_SENSE_INPUT_Pin */
  GPIO_InitStruct.Pin = OVER_SENSE_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(OVER_SENSE_INPUT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

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
