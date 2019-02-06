/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_tx;
DMA_HandleTypeDef hdma_sdio_rx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__

  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 
//redirect printf
PUTCHAR_PROTOTYPE{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF); 
  return ch;
}

void PrintTime();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */
uint8_t fac_us;

void delay_init(uint8_t SYSCLK){
	#if SYSTEM_SUPPORT_OS
		uint32_t reload;
	#endif
    
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	fac_us=SYSCLK;
    
	#if SYSTEM_SUPPORT_OS
		reload=SYSCLK; 
		reload*=1000000/delay_ostickspersec; 
		fac_ms=1000/delay_ostickspersec;
		SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;
		SysTick->LOAD=reload;
		SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;
	#else
	#endif
}

void delay_us(uint32_t nus){
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD; 
	ticks=nus*fac_us; 
	told=SysTick->VAL; 
	while(1) {
		tnow=SysTick->VAL;
		if(tnow!=told) {
			if(tnow<told)tcnt+=told-tnow;
			else tcnt+=reload-tnow+told;
			told=tnow;
			if(tcnt>=ticks)break; 
		}
	};
}

RTC_DateTypeDef sDate;
RTC_TimeTypeDef sTime;
uint8_t second_tmp = 0;

void LED0_off(){HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);}
void LED1_off(){HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);}

void LED0_on(){HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);}
void LED1_on(){HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);}

uint8_t PMSensorBuffer[128];
uint8_t PMSensor[17];
int TOT = 0, PMSensorLen;

//DHT22/AM2303 Sensor

uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t sum, RH, TEMP;
int temp_low, temp_high, rh_low, rh_high;
char temp_char1[2], temp_char2, rh_char1[2], rh_char2;
uint8_t check = 0;
GPIO_InitTypeDef GPIO_InitStruct;

void set_gpio_output(void){
	
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
}

void set_gpio_input(void){

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
}


void DHT22_start(void){
	
	set_gpio_output(); 
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0); 
	delay_us (500);   
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);  
	delay_us(30);
	set_gpio_input();
	
}

void check_response (void){
	
	delay_us(40);
	if(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))){
		delay_us (80);
		if((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)))check = 1;
	}
	while((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)));
	
}

uint8_t read_data (void){

	uint8_t i = 0,j;
	for (j=0; j<8; j++) {
		while (!(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1))); 
		delay_us (40); 
		if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)) == 0)  {
			i &= ~(1 << (7 - j) ); 
		}
		else i |= (1 << (7 - j) ); 
		while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1)));  
	}
	return i;
}

void UpdateTemp(void){
	
	DHT22_start ();
	check_response ();
	Rh_byte1 = read_data ();
	Rh_byte2 = read_data ();
	Temp_byte1 = read_data ();
	Temp_byte2 = read_data ();
	sum = read_data();
	//if (sum == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2))
	//{
	TEMP = ((Temp_byte1<<8)|Temp_byte2);
	RH = ((Rh_byte1<<8)|Rh_byte2);
	//}
	HAL_Delay(500);
	temp_low = TEMP/10;
	temp_high = TEMP%10;
	
	rh_low = RH/10;
	rh_high = RH%10;
	PrintTime();
	printf("[DHT22_STATE] Success!\n");
	
	return ;
}


struct PMSensor{
	int PM1_0_CF, PM2_5_CF, PM10_CF, PM1_0, PM2_5, PM10;
}PMResult;

void ListenPMSensor(void){
	memset(PMSensorBuffer, 0, sizeof(PMSensorBuffer));
	memset(PMSensor, 0, sizeof(PMSensor));
	
	HAL_UART_Receive(&huart2, (uint8_t*)PMSensorBuffer, 128, 3200);
	HAL_Delay(800);
	
	//printf("", PMSensorBuffer);
	for(int i = 0; i <= 127; ++i){
		//printf("%c \n", PMSensorBuffer[i]);
		if(PMSensorBuffer[i] == 'B' && PMSensorBuffer[i+1] == 'M'){			
			for(PMSensorLen = 0; PMSensorLen <= 31; ++PMSensorLen){
				PMSensor[PMSensorLen] = PMSensorBuffer[i + PMSensorLen];
			}
			break;
		}
	}
	
	HAL_Delay(2000);
	return ;
}

void GetPMSensor(void){
	uint16_t Buffer_Len;
	Buffer_Len = (uint16_t)((PMSensor[2] << 8) | PMSensor[3]);
	if(Buffer_Len == 28) {
		
		PMResult.PM1_0_CF = (uint16_t)((PMSensor[4]<<8) | PMSensor[5]);
		PMResult.PM2_5_CF = (uint16_t)((PMSensor[6]<<8) | PMSensor[7]);
		PMResult.PM10_CF 	= (uint16_t)((PMSensor[8]<<8) | PMSensor[9]);
		PMResult.PM1_0 	= (uint16_t)((PMSensor[10]<<8) | PMSensor[11]);
		PMResult.PM2_5 	= (uint16_t)((PMSensor[12]<<8) | PMSensor[13]);
		PMResult.PM10 	= (uint16_t)((PMSensor[14]<<8) | PMSensor[15]);		
		
	}
	else if(Buffer_Len == 20){
		
		PMResult.PM1_0_CF = (uint16_t)((PMSensor[4]<<8) | PMSensor[5]);
		PMResult.PM2_5_CF = (uint16_t)((PMSensor[6]<<8) | PMSensor[7]);
		PMResult.PM10_CF 	= (uint16_t)((PMSensor[8]<<8) | PMSensor[9]);
		PMResult.PM1_0 	= (uint16_t)((PMSensor[10]<<8) | PMSensor[11]);
		PMResult.PM2_5 	= (uint16_t)((PMSensor[12]<<8) | PMSensor[13]);
		PMResult.PM10 	= (uint16_t)((PMSensor[14]<<8) | PMSensor[15]);
		
	}
	return ;

}

int CheckPMSensor(void){
	
	uint16_t 	Cal_CheckSum;
	uint16_t 	Buffer_CheckSum;
	uint16_t 	Buffer_Len;
	uint8_t 	i;
	
	int Result = 0;

	if((PMSensor[0] == 'B')&&(PMSensor[1] == 'M')){
		Buffer_Len = (uint16_t)((PMSensor[2] << 8) | PMSensor[3]);

		Buffer_CheckSum = (uint16_t)((PMSensor[Buffer_Len + 2] << 8) | PMSensor[Buffer_Len + 3]);

		Cal_CheckSum = 0;
		for(i=0;i<(Buffer_Len + 2);i++){
			Cal_CheckSum += PMSensor[i];
		}

		if(Cal_CheckSum == Buffer_CheckSum)
			Result = 1;
	}
	return Result;
}


//CO2 sensor
uint8_t CO2SensorBuffer[12];
uint8_t CO2Sensor[12];

int CO2SensorLen = 0, CO2SensorResult = 0;

void ListenCO2Sensor(void){
	memset(CO2SensorBuffer, 0, sizeof(CO2SensorBuffer));
	memset(CO2Sensor, 0, sizeof(CO2Sensor));
	
	uint8_t CO2Transmit[12] = {0x42, 0x4D, 0xE3, 0x00, 0x00, 0x01, 0x72};
	
	HAL_UART_Transmit(&huart3, CO2Transmit, 7, 0xFFFF);
	HAL_UART_Receive(&huart3, (uint8_t*)CO2SensorBuffer, 12, 800);
	HAL_Delay(800);
	
	for(int i = 0; i <= 11; ++i){
		//printf("%c", CO2SensorBuffer[i]);
		if(CO2SensorBuffer[i] == 'B' && CO2SensorBuffer[i+1] == 'M'){
			for(CO2SensorLen = 0; CO2SensorLen <= 11; ++CO2SensorLen){	
				CO2Sensor[CO2SensorLen] = CO2SensorBuffer[i + CO2SensorLen];
			}
			break;
		}
		HAL_Delay(500);
	}
	HAL_Delay(2000);
	return ;
}


int CheckCO2Sensor(void){
	
	uint16_t 	Cal_CheckSum;
	uint16_t 	Buffer_CheckSum;
	uint16_t 	Buffer_Len;
	uint8_t 	i;
	
	int Result = 0;

	if((CO2Sensor[0] == 'B')&&(CO2Sensor[1] == 'M')){
		
		Buffer_Len = (uint16_t)((CO2Sensor[2] << 8) | CO2Sensor[3]);
		//printf("LEN:%d \n",Buffer_Len);
		Buffer_CheckSum = (uint16_t)((CO2Sensor[Buffer_Len + 2] << 8) | CO2Sensor[Buffer_Len + 3]);
        
		Cal_CheckSum = 0;
		for(i=0;i<(Buffer_Len + 2);i++){
			Cal_CheckSum += CO2Sensor[i];
		}
		//printf("CALSUM:%d, BUFFSUM:%d \n", Cal_CheckSum, Buffer_CheckSum);
		if(Cal_CheckSum == Buffer_CheckSum)
			Result = 1;
	}
	return Result;
}

void GetCO2Sensor(void){
	CO2SensorResult = (uint16_t)((CO2Sensor[4]<<8) | CO2Sensor[5]);
	return ;
}

FATFS SDFatFs;
FIL MyFile;

void WriteFile(void){
	
	LED0_off();
	LED1_off();
	
	uint32_t byteswritten;
	char nowdate[21];
	sprintf(nowdate, "[20%02d-%02d-%02d %02d:%02d:%02d]\n",
				sDate.Year,
				sDate.Month,
				sDate.Date,
				sTime.Hours,
				sTime.Minutes,
				sTime.Seconds
	);
	char wtext[150] = " ";
	sprintf(wtext, "%sTemp : _%d.%d_ degree Celsius\nRH : _%d.%d_  %% \nCO2 : _%04d_ ppm\nPM2.5 : _%04d_ ug/m3\nPM10 : _%04d_ ug/m3\n", 
		nowdate,
		temp_low,
		temp_high,
		rh_low,
		rh_high,
		CO2SensorResult, 
		PMResult.PM2_5, 
		PMResult.PM10
	);
	char FileName[50] = " ";
	
	sprintf(FileName, "Sensor_20%02d-%02d-%02d-%02d-%02d_.txt", 
		sDate.Year,
		sDate.Month,
		sDate.Date,
		sTime.Hours,
		sTime.Minutes
	);
	
	printf("%s\nFileName:%s\n", wtext, FileName);
	if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) == FR_OK){
		PrintTime();
		printf("[SD_FATFS_STAT] Mount Successful!\n");
		if(f_open(&MyFile, (TCHAR const*)FileName, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK){
			PrintTime();
			printf("[SD_FATFS_STAT] Open File Successful!\n");
			FRESULT res = f_write(&MyFile, wtext, (unsigned int)strlen(wtext), (void *)&byteswritten);
			if((byteswritten == 0) || (res != FR_OK)){
				PrintTime();
				printf("[SD_FATFS_STAT] Write Failed!\n");
			}
			else{
				f_close(&MyFile);
				PrintTime();
				printf("[SD_FATFS_STAT] Write Successful!\n");
			}
		}
		else{
				PrintTime();
				printf("[SD_FATFS_STAT] Open File Failed!\n");
		}
	}else{
		PrintTime();
		printf("[SD_FATFS_STAT] Mount Failed!\n");
	}
	return ;

}

void PrintTime(void){
	
	printf("[20%02d-%02d-%02d %02d:%02d:%02d]",
		sDate.Year,
		sDate.Month,
		sDate.Date,
		sTime.Hours,
		sTime.Minutes,
		sTime.Seconds
	);
	return ;
	
}

void CheckState(void){
	
	ListenCO2Sensor();
	if(CheckCO2Sensor()){
		PrintTime();
		printf("[CO2_SENSOR_STATE] Success!\n");
		GetCO2Sensor();
	}
	
	else{
		LED0_off();
		PrintTime();
		printf("[CO2_SENSOR_STATE] Error!\n");
	}
	
	/*PM SENSOR READ*/
	HAL_Delay(100);
	ListenPMSensor();
	if(CheckPMSensor()){
		PrintTime();
		printf("[PM_SENSOR_STATE] Success!\n");
		GetPMSensor();
	}
	else{
		LED1_off();
		PrintTime();
		printf("[PM_SENSOR_STATE] Error!\n");
	}
	
	PrintTime();
	printf("\nTemp : %d.%d degree Celsius\nRH : %d.%d % \nCO2: %d ppm \nPM2.5: %d ug/m^3\nPM10: %d ug/m^3\n", 
		temp_low,
		temp_high,
		rh_low,
		rh_high,
		CO2SensorResult,	
		PMResult.PM2_5, 
		PMResult.PM10
	);
	UpdateTemp();
	LED0_on();
	LED1_on();
	HAL_Delay(100);
	LED0_off();
	LED1_off();
	HAL_Delay(100);
	LED0_on();
	LED1_on();
	//WriteFile(filename);
	HAL_Delay(1000);
	
	return ;
	
}

/* USER CODE END 0 */

int main(void){
	
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
  MX_SDIO_SD_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	delay_init(168);
  /* USER CODE END 2 */
	
	HAL_Delay(1000 * 60);
	
  while (1){
    
		/* USER CODE BEGIN 3 */
		TOT++;
		LED0_on();
		LED1_on();
		HAL_Delay(100);
		LED0_off();
		LED1_off();
		HAL_Delay(100);
		LED0_on();
		LED1_on();
		HAL_Delay(100);
		LED0_off();
		LED1_off();
		HAL_Delay(100);
		LED0_on();
		LED1_on();
		HAL_Delay(100);
		LED0_off();
		LED1_off();
		HAL_Delay(100);
		LED0_on();
		LED1_on();
		HAL_Delay(500);

		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		if(second_tmp != sTime.Seconds){
			second_tmp = sTime.Seconds;
			//printf("20%d%d-%d%d-%d%d\n", sDate.Year/10%10, sDate.Year%10, sDate.Month/10%10, sDate.Month%10, sDate.Date/10%10, sDate.Date%10);
			printf("[20%02d-%02d-%02d %02d:%02d:%02d] LOOP = %d\n",
				sDate.Year,
				sDate.Month,
				sDate.Date,
				sTime.Hours,
				sTime.Minutes,
				sTime.Seconds,
				TOT
			);
		}
		
		CheckState();
		if(TOT > 1) {
			WriteFile();
			LED0_on();
			LED1_on();
			HAL_Delay(100);
			LED0_off();
			LED1_off();
			HAL_Delay(100);
			LED0_on();
			LED1_on();
			HAL_Delay(100);
			LED0_off();
			LED1_off();
			HAL_Delay(100);
			HAL_Delay(1000 * 60);
		}
		//
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void){
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void) {

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

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
  if (HAL_RTC_Init(&hrtc) != HAL_OK){
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /**Initialize RTC and set the Time and Date 
  */
	/*
  sTime.Hours = 2;
  sTime.Minutes = 6;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_FEBRUARY;
  sDate.Date = 5;
  sDate.Year = 19;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
	*/
  /**Enable the WakeUp 
  */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK){
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void) {

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 12;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void) {

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)  {
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LED0_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED0_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line) { 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
