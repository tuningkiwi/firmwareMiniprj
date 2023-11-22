/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMP 0
#define HUMI 1

/*********SHT2(Temp,Humid sensor)command set**************/
#define SHT2x_ADDR 	(0x40<<1)
#define SHT2x_HOLD_MASTER_T		0xE3
#define SHT2x_HOLD_MASTER_RH	0xE5
#define SHT2x_NOHOLD_MASTER_T 0xF3
#define SHT2x_NOHOLD_MASTER_RH	0xF5
#define SHT2x_WRITE_USER_REG		0xE6
#define SHT2x_READ_USER_REG			0xE7
#define SHT2x_SOFT_RESET		0xFE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FILE __stdout;
uint8_t pData;
int piano = EOF;
int menu = EOF;
uint8_t direction = 'r';
//1번 shift led 
uint16_t curPin = GPIO_PIN_0;
uint16_t nextPin = GPIO_PIN_1;

//uint16_t adcData[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE* stream);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
float SHT20(int select);
void ShowMenu();
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	//2번 mood light
	uint16_t CCRVal = 0;
	//3번 피아노 pwm
	uint32_t pwmF;
	uint16_t scale[]={523,587,659,698,783,880,987,1046};//음계 
	//4번 STREET LAMP 
	uint16_t adcData[2];
	//5번 Temp/Humid 
	float temperature, humidity;
	
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2,&pData,sizeof(pData));
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adcData,1);

	ShowMenu();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//printf("iam here0\n\r");
		//printf("menu:%d\n\r",menu);
		HAL_Delay(1000);
		TIM4->EGR = TIM4->EGR | 0x01;
		while(menu == 1){//1: LED SHIFT
				printf("LED SHIFT START\n\r");
				printf("please choose direct : 'r'ight or 'l'eft >>");
				while(direction ==NULL){
					//printf("iam here1-NULL\n\r");
					HAL_Delay(1000);
				}		
				while( direction == 'r'){//right
					if(menu != 1) {break;}
					HAL_GPIO_WritePin(GPIOC,curPin,GPIO_PIN_SET);
					HAL_Delay(1000);
					HAL_GPIO_WritePin(GPIOC,curPin,GPIO_PIN_RESET);				
					curPin <<= 1;
					nextPin <<= 1;
			
					if(curPin == GPIO_PIN_5){
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
						curPin = GPIO_PIN_0;
						nextPin =GPIO_PIN_1;
					}		
				}
				while(direction == 'l'){//left 
					if(menu != 1 ){break;}
					HAL_GPIO_WritePin(GPIOC,curPin,GPIO_PIN_SET);
					HAL_Delay(1000);
					HAL_GPIO_WritePin(GPIOC,curPin,GPIO_PIN_RESET);				
					curPin >>= 1;
					nextPin >>= 1;
			
					if(nextPin == GPIO_PIN_1){
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
						HAL_Delay(1000);
						HAL_GPIO_WritePin(GPIOC,curPin,GPIO_PIN_RESET);	
						curPin = GPIO_PIN_4;
						nextPin =GPIO_PIN_5;
					}					
				}

		}
		
		while(menu == 2){//2: MOOD LIGHT
			printf("MOOD LIGHT START\n\r");
			TIM3->CCR1 = CCRVal ;
			CCRVal++;
			printf("ccrVal: %d\n\r",CCRVal);
			HAL_Delay(100);
		
			if(CCRVal == 30){
				while(CCRVal){
						CCRVal--;
						TIM3->CCR1 = CCRVal;
						HAL_Delay(100);
				}
			}			
		}
		
		while(menu == 3){//3: PIANO (PB7     ------> TIM4_CH2)
			//TIM4->EGR = TIM4->EGR | 0x01;
			printf("PIANO START\n\r");
			printf("Please enter a scale>>");
			
			while(piano == EOF){
				HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);	
				//printf("iam here3-EOF\n\r");
				HAL_Delay(1000);
				if(menu==0|| menu ==1 || menu ==2|| menu==4|| menu==5){
					break;
				}
			}	
			if(piano >= 0 && piano<8){	
					HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);				
					pwmF = 10000000/ scale[piano];
					TIM4->ARR = pwmF -1;
					TIM4->CCR2 = pwmF /2;	
					HAL_Delay(500);
					
			}			
			piano = EOF;
		}
		
		while(menu == 4){//4: STREET LAMP ADC(PA1),GPIO_LED (PC5)/ DMA 
			//printf("ADC:%d\n\r",adcData[0]);
			//HAL_Delay(1000);
			printf("STREET LAMP START\n\r");
			printf("Try to darken the surroundings\n\r");
			if(adcData[0]<3000){ //dark so lamp on	
				printf("ADC:%d\n\r",adcData[0]);				
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_Delay(1000);
			}	
			if(adcData[0] >3000){//light so lamp off
				printf("ADC:%d\n\r",adcData[0]);		
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
				HAL_Delay(1000);

			}		
		}
		
		while(menu == 5){//5: TEMP/HUMID  PB6(I2C1_SCL)PB9(I2C1_SDA)   
			printf("TEMP/HUMID START\n\r");
			temperature = SHT20(TEMP);
			humidity = SHT20(HUMI);
			HAL_Delay(500);
			printf("TEMP: %.2lf HUMI: %.2lf \r\n",temperature, humidity);
		
		}
		
		while(menu == 0){
			printf("program exit\n\r");
			return 0;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void ShowMenu(){
		printf("--------Mini Project-------------\n\r");		
		printf("1.LED SHIFT\n\r");
		printf("2.Mood Light\n\r");
		printf("3.piano\n\r");
		printf("4.streetLight\n\r");
		printf("5.temp/humid\n\r");
		printf("0.program exit\n\r");
		printf("please, select menu>>\n\r");
}

float SHT20(int select){//NO HOLD MASTER MODE 
	uint8_t I2CData[3];
	uint16_t SLAVER_ADDR = SHT2x_ADDR;
	float convData = 0.0;
	if(select == TEMP){
		I2CData[0] =SHT2x_NOHOLD_MASTER_T;
		//printf("i am here\n\r");
		//온도를 측정해라 명령을 송신한다
		
		if(HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)SLAVER_ADDR,(uint8_t*)I2CData,1,0xffff)==HAL_OK){
				//printf("Send Command SUCCESS!!\n\r");
				
		}else {
				printf("Send Command FAILED!!\n\r");
		}
		HAL_Delay(100);
		HAL_I2C_Master_Receive(&hi2c1,(uint16_t)SLAVER_ADDR,(uint8_t*)I2CData,2,0xffff);
		//I2CData[0],I2CData[1] 두개를 묶어서 한 데이터로 처리 
		uint16_t sensor = I2CData[0] << 8 | I2CData[1];
		convData = -46.85+(175.72/65536*(float)sensor);
		
	}else if(select == HUMI){
		I2CData[0] =SHT2x_NOHOLD_MASTER_RH;
		
		HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)SLAVER_ADDR,(uint8_t*)I2CData,1,0xffff);
		HAL_Delay(100);
		HAL_I2C_Master_Receive(&hi2c1,(uint16_t)SLAVER_ADDR,(uint8_t*)I2CData,2,0xffff);
		//I2CData[0],I2CData[1] 두개를 묶어서 한 데이터로 처리 
		uint16_t sensor = I2CData[0] << 8 | I2CData[1];
		convData = -6+(125.0/65536*(float)sensor);
		//convData = -6+125*((float)sensor/65536);
		
	}

	return convData;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2){
		//printf("Receive Success >> pData : %c\n\r",pData);
		//piano = pData - '1';// score 배열 접근 
		if(pData >= '0' && pData <='5'){//메뉴선택
			menu = pData- '0';
			
			//printf("menu : %d\n\r", menu);
		}else if(pData >='a' && pData <='g'){//피아노 음계 
			//printf("correct piano\n\r");
			switch(pData){
				case 'c': piano =0;break;//도
				case 'd': piano =1;break;
				case 'e': piano =2;break;
				case 'f': piano =3;break;
				case 'g': piano =4;break;
				case 'a': piano =5;break;
				case 'b': piano =6;break;//시				
			}
			
		}else if(pData == 'l' || pData =='r'){//shift 명령어 
			//printf("correct direction\n\r");
			direction =pData;
//			if(pData =='l'){
//				curPin = GPIO_PIN_4;
//				nextPin =GPIO_PIN_5;
//			}else if(pData =='r'){
//				curPin = GPIO_PIN_0;
//				nextPin =GPIO_PIN_1;			
//			}
			
		}

		ShowMenu();

	}
	HAL_UART_Receive_IT(&huart2,&pData,sizeof(pData));
}

int fputc(int ch, FILE* stream){
	HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,0xffff);
	return ch;
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
