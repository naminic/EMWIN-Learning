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
SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
/* USER CODE BEGIN PFP */
void LCD_clear(uint16_t data);
void STM3210E_LCD_Init(void);
void LcdWriteReg(uint16_t Data);
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
  MX_FSMC_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(SSD_RST_GPIO_Port,SSD_RST_Pin,0);
	HAL_Delay(100);
	HAL_GPIO_WritePin(SSD_RST_GPIO_Port,SSD_RST_Pin,1);
	HAL_Delay(100);
	
	STM3210E_LCD_Init();
	LCD_clear(0xf800);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		LCD_clear(0xf800);
  	HAL_Delay(1000);
  	LCD_clear(0x7e0);
   	HAL_Delay(1000);
		LCD_clear(0x1f);
		HAL_Delay(1000);
//	LcdWriteReg(0x0029);	
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SSD_RST_GPIO_Port, SSD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SSD_RST_Pin */
  GPIO_InitStruct.Pin = SSD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SSD_RST_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 0x02;
  Timing.AddressHoldTime = 0xa;
  Timing.DataSetupTime = 0xa;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 0;
  Timing.DataLatency = 0;
  Timing.AccessMode = FSMC_ACCESS_MODE_B;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */
#define U16 uint16_t
#define U32 uint32_t

U16 driverCode;

#define LCD_DATA_ADDRESS    *(volatile U16*) ((volatile U32)0x60020000) 
#define LCD_REG_ADDRESS     *(volatile U16*) ((volatile U32)0x60000000)	
	
static void LcdWriteReg(U16 Data) {
  LCD_REG_ADDRESS = Data;
	//Datareg = Data;
}

/********************************************************************
*
*       LcdWriteData
*
* Function description:
*   Writes a value to a display register
*/
static void LcdWriteData(U16 Data) {
  LCD_DATA_ADDRESS = Data;
}

U16 LCD_X_Read01_16(void) {
	
	return  LCD_DATA_ADDRESS;
}


static U16 rd_reg (U16 reg) {
  LcdWriteReg(reg);
	HAL_Delay(1);
  return LCD_X_Read01_16();
}



#define  HDP	479  /*长*/
#define  HT		531
#define  HPS	43
#define  LPS	8
#define  HPW	10

#define  VDP	271	 /*宽*/
#define  VT		288
#define  VPS	12
#define  FPS	4
#define  VPW	10


void LCD_clear(uint16_t data)
{
    unsigned int l=800,w;

	LcdWriteReg(0x002A);	
	LcdWriteData(0);	    
	LcdWriteData(0);
	LcdWriteData(HDP>>8);	    
	LcdWriteData(HDP&0x00ff);
  LcdWriteReg(0x002b);	
	LcdWriteData(0);	    
	LcdWriteData(0);
	LcdWriteData(VDP>>8);	    
	LcdWriteData(VDP&0x00ff);
	LcdWriteReg(0x002c);
	
	
	while(l--)
	{
	    for(w=0;w<480;w++)
		{    
          	LcdWriteData(data);
		}
	}
}

void STM3210E_LCD_Init(void){
	
	

	
	LcdWriteReg(0x00E2);	
	LcdWriteData(0x0023);
	// Set PLL with OSC = 10MHz (hardware)
    // Multiplier N = 35, VCO (>250MHz)= OSC*(N+1), VCO = 360MHz	   
	LcdWriteData(0x0002);
	// Divider M = 2, PLL = 360/(M+1) = 120MHz
	LcdWriteData(0x0004);
	// Validate M and N values

	LcdWriteReg(0x00E0);  // PLL enable
	LcdWriteData(0x0001);
	HAL_Delay(1);
	LcdWriteReg(0x00E0);
	LcdWriteData(0x0003);
	HAL_Delay(5);
	LcdWriteReg(0x0001);  // software reset
	HAL_Delay(5);
	LcdWriteReg(0x00E6);	//PLL setting for PCLK, depends on resolution
	
	
	//Set LSHIFT freq, i.e. the DCLK with PLL freq 120MHz set previously
	//Typical DCLK for AT070TN92 is 34MHz
	//34MHz = 120MHz*(LCDC_FPR+1)/2^20
	//LCDC_FPR = 300000 (0x0493E0)
	
	LcdWriteData(0x0004);
	LcdWriteData(0x0093);
	LcdWriteData(0x00e0);

	LcdWriteReg(0x00B0);	//LCD SPECIFICATION
	LcdWriteData(0x0000);
	LcdWriteData(0x0000);
	LcdWriteData((HDP>>8)&0X00FF);  //Set HDP
	LcdWriteData(HDP&0X00FF);
  LcdWriteData((VDP>>8)&0X00FF);  //Set VDP
	LcdWriteData(VDP&0X00FF);
  LcdWriteData(0x0000);

	LcdWriteReg(0x00B4);	//HSYNC
	LcdWriteData((HT>>8)&0X00FF);  //Set HT
	LcdWriteData(HT&0X00FF);
	LcdWriteData((HPS>>8)&0X00FF);  //Set HPS
	LcdWriteData(HPS&0X00FF);
	LcdWriteData(HPW);			   //Set HPW
	LcdWriteData((LPS>>8)&0X00FF);  //Set HPS
	LcdWriteData(LPS&0X00FF);
	LcdWriteData(0x0000);

	LcdWriteReg(0x00B6);	//VSYNC
	LcdWriteData((VT>>8)&0X00FF);   //Set VT
	LcdWriteData(VT&0X00FF);
	LcdWriteData((VPS>>8)&0X00FF);  //Set VPS
	LcdWriteData(VPS&0X00FF);
	LcdWriteData(VPW);			   //Set VPW
	LcdWriteData((FPS>>8)&0X00FF);  //Set FPS
	LcdWriteData(FPS&0X00FF);

	LcdWriteReg(0x00BA);
	LcdWriteData(0x000f);//0x000F);    //GPIO[3:0] out 1

	LcdWriteReg(0x00B8);
	LcdWriteData(0x000f);    //GPIO3=input, GPIO[2:0]=output
	LcdWriteData(0x0001);    //GPIO0 normal

	LcdWriteReg(0x0036); //rotation
	LcdWriteData(0x0000);


	LcdWriteReg(0x00F0); //pixel data interface
	LcdWriteData(0x0003);


	HAL_Delay(5);

	//LCD_clear(0x0000);
	LcdWriteReg(0x0029); //display on

	LcdWriteReg(0x00BE); //set PWM for B/L
	LcdWriteData(0x0006);
	//LcdWriteData(0x0008);
	LcdWriteData(0x0080);
	//LcdWriteData(0x00f0);
	
	LcdWriteData(0x0001);
	LcdWriteData(0x00f0);
	LcdWriteData(0x0000);
	LcdWriteData(0x0000);

	LcdWriteReg(0x00d0);//设置动态背光控制配置 
	LcdWriteData(0x000d);
	
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
