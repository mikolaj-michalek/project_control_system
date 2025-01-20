/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMPXX80.h"
#include <stdio.h>
#include <string.h>
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef	struct{
	float 		dt;				// czas probkowania Tp=1 s
	float		upid; 			// sygnal pid (przed nasyceniem)
	float		usat;			// sygnal sterujacy nasycony
	float		Kp;				// parametr PID Kp
	float		Ki;				// parametr PID Ki
	float		Kd;				// parametr PID Kd
	float		up;				// skladowa sygnalu sterujacego (proporcjonalna)
	float		ud;   			// skladowa sygnalu sterujacego (rozniczkujaca)
	float		ui;	    		// skladowa sygnalu sterujacego (calkujaca)
	float		e;				// blad regulacji
	float		prev_integral;	// zmienne pomocnicze
	float		integral;
	float		prev_error;
	float		derivative;
	float 		usat_fan;
}PidStruct;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define COUNTOF(_BUFFER_) (sizeof(_BUFFER_) / sizeof(*(_BUFFER_)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float temperature_from_bmp280;
float temperature_from_user;

char data[16];
PidStruct Pid1 = {1.0,0,0,1.2,0.02,0,0,0,0,0,0,0,0,0,0};
int pwm_duty;
int pwm_duty_fan;
int temperature_from_uart;
int int_temp_to_send;
uint8_t received_from_user;
uint32_t AdcValue_from_potentiometer;
float max_range = 30.0;
float min_range = 23.0;
char lcd_temp[16];
char lcd_set_point[16];
uint8_t uart_temp[64];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Saturacja(void)
{
	if (Pid1.upid > 1.0)
	{
		Pid1.usat = 1.0;
	}
	else if (Pid1.upid < 0.0)
	{
		Pid1.usat = 0.0;
	}
	else
	{
		Pid1.usat = Pid1.upid;
	}

	if (Pid1.upid <= -1.0)
	{
		Pid1.usat_fan = 1.0;
	}
	else if((Pid1.upid > -1.0)&&(Pid1.upid<-0.4))
	{
		Pid1.usat_fan = (-1)*Pid1.upid;
	}
	else if(Pid1.upid >= -0.4)
	{
		Pid1.usat_fan = 0.0;
	}
}

void PIDcalc(float temp_measured, float reference)
{
	Pid1.e = reference-temp_measured;

	// proportional part
	Pid1.up = Pid1.Kp * Pid1.e;

	// integral part

	if ((Pid1.usat-Pid1.upid < 0)||(Pid1.usat_fan+Pid1.upid < 0))
	{
		Pid1.integral = Pid1.prev_integral;
	}
	else
	{
		Pid1.integral = Pid1.prev_integral + (Pid1.e + Pid1.prev_error);
		Pid1.prev_integral = Pid1.integral;
	}
	Pid1.ui = Pid1.Ki * Pid1.integral  * (Pid1.dt/2.0);


	// derivative part
	Pid1.derivative = (Pid1.e - Pid1.prev_error) / Pid1.dt;
	Pid1.prev_error = Pid1.e;
	Pid1.ud = Pid1.Kd * Pid1.derivative;

	// sum
	Pid1.upid = Pid1.up + Pid1.ui +Pid1.ud;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int is_int(char *str) {
    int i = 0, len = strlen(str);
    if (str[i] == '+' || str[i] == '-') i++;
    if (i == len) return 0;
    while (i < len) {
        if (isdigit(str[i])) {
            i++;
        } else {
            return 0;
        }
    }
    return 1;
}

float ADC_to_temperature(uint32_t adc_val, float max_val, float min_val)
{
	float temp_val;
	temp_val = adc_val * ((max_val-min_val)/4095.0) + min_val;

	return temp_val;
}
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_I2C4_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  BMP280_Init(&hspi1, BMP280_TEMPERATURE_16BIT, BMP280_STANDARD, BMP280_FORCEDMODE);
  HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start (&htim4, TIM_CHANNEL_1);
  HAL_UART_Receive_IT(&huart3, &received_from_user, 3);
  GPIO_PinState pin_state_user_button;
  GPIO_PinState prev_pin_state_user_button = GPIO_PIN_RESET;
  HAL_TIM_Base_Start_IT(&htim7);
  lcd_init();
  lcd_clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// ODCZYT DANYCH WEJSCIOWYCH
	temperature_from_bmp280 = BMP280_ReadTemperature();
	pin_state_user_button = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	AdcValue_from_potentiometer = HAL_ADC_GetValue(&hadc1);

	// SEKWENCJA PROGRAMU

		// REGULACJA
	PIDcalc(temperature_from_bmp280, temperature_from_user);
	Saturacja();
	pwm_duty = (int)(Pid1.usat * (999.0));
	pwm_duty_fan = (int)(Pid1.usat_fan * (999.0));

		// Zmienna temperature_from_uart
	if (pin_state_user_button == GPIO_PIN_SET && prev_pin_state_user_button == GPIO_PIN_RESET)
	{
		temperature_from_uart = !temperature_from_uart;
	}
	prev_pin_state_user_button = pin_state_user_button;

		// Setpoint value from potentiometer
	if (temperature_from_uart == 0)
	{
		temperature_from_user = ADC_to_temperature(AdcValue_from_potentiometer, max_range, min_range);
	}

	// WYSLANIE DANYCH WYJSCIOWYCH
	//int_temp_to_send = (int)(temperature_from_bmp280*1000);
	sprintf((char*)uart_temp, "\n%f", temperature_from_bmp280);
	HAL_UART_Transmit(&huart3, uart_temp, strlen((char*)uart_temp), 50);


	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_duty);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_duty_fan);

		// Led 1  - temperature_from_uart
	if (temperature_from_uart == 1)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	}



	HAL_Delay(1000);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(is_int(&received_from_user))
	{
		if (temperature_from_uart == 1)
		    {
				temperature_from_user = atoi(&received_from_user)/10.0;
		    }
	}
	HAL_UART_Receive_IT(&huart3, &received_from_user, 3);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// LCD
	sprintf(lcd_temp, "Odczyt: %.2f C", temperature_from_bmp280);
	sprintf(lcd_set_point, "Zadane: %.2f C", temperature_from_user);
	lcd_put_cur(0, 0);
	lcd_send_string(lcd_temp);
	lcd_put_cur(1, 0);
	lcd_send_string(lcd_set_point);
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
