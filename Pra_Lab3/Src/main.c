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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ft5336.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stdio.h"
#include "stdlib.h"
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

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

LTDC_HandleTypeDef hltdc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */

#define WINDOWS_WITH		224
#define WINDOWS_HEIGHT		224


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
void MX_USART3_UART_Init(void);
static void MX_LTDC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void LCD_Config(void);
static void GetPosition(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//uint32_t classification_result =0 ;
static TS_StateTypeDef  TS_State;
float PxlNet2D[28][28]={0};
extern uint32_t classification_result;

int radius=2;
uint32_t idx=0;
uint32_t ysrc, xsrc, xstp, ystp, pxl;
uint32_t tmp;
uint32_t xpos, ypos;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint8_t status=0;
	int x;
	int y;
 
	int load=0;
	int a;
	int b;
	 uint32_t output1;
	uint8_t buff;
	char buff1[250];
	int i ,j;
	int write=0;
	int mode=0;
	
  /* USER CODE END 1 */
  

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_CRC_Init();
  MX_LTDC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_USART1_UART_Init();
  MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */

   LCD_Config(); 
   status=  BSP_TS_Init(480 ,272);
	 
	 
	 if (status != TS_OK)
  { 		
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 95, (uint8_t *)"ERROR", CENTER_MODE);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 80, (uint8_t *)"Touchscreen cannot be initialized", CENTER_MODE);
	}
	else{
		
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
   BSP_LCD_DisplayStringAt(0,170,(uint8_t *)"lcd test ok",LEFT_MODE);
  BSP_LCD_Clear(LCD_COLOR_BLACK);
		
	}
	 BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	 
	 BSP_LCD_DrawHLine(0,10,240)   ;
		BSP_LCD_DrawHLine(0,260,240)   ;
		BSP_LCD_DrawVLine(0,10,250);
		BSP_LCD_DrawVLine(240,10,250);
				BSP_LCD_DrawHLine(260,10,200)   ;
		BSP_LCD_DrawHLine(260,260,200)   ;
		BSP_LCD_DrawVLine(260,10,250);
		BSP_LCD_DrawVLine(460,10,250); 
	// BSP_LCD_DisplayStringAt(0,170,(uint8_t *)"lcd test ok",LEFT_MODE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

 // MX_X_CUBE_AI_Process();
    /* USER CODE BEGIN 3 */
		BSP_TS_GetState(&TS_State);
		if(TS_State.touchDetected)
   {
		x=TS_State.touchX[0];
		y=TS_State.touchY[0];
		if((x<240 && x>10)&& (y<250 && y>10))
		{	
			
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			BSP_LCD_FillCircle(x,y,2);
			a=x/8.21;    //8.21
			b=y/8.57;   //8.57
			PxlNet2D[b][a]=1.0;   // not very precise
			write=1;
		}
		else if((x<460 && x>260)&& (y<250 && y>10))
		{
			
			if(load ==0 && write==1)
			{ 
				for(i=0;i<28;i++)
				{
					for(j=0;j<28;j++)
					{
						if(PxlNet2D[i][j]==1.0)
						{
							BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
						BSP_LCD_DrawCircle(260+i*2, 10+ j*2,1);
						}
						
					}
					
					
					
				}
				
				
				
				
				
				
				
				
				MX_X_CUBE_AI_Process();
				
				
					if ( classification_result <= 9)
	   {
		sprintf((char*)buff1,"%lu",classification_result);
	   } else {
	  	if ( classification_result <= 36)
		 {
		 	sprintf((char*)buff1," %c",(classification_result-10)+65);
		 } else {
			sprintf((char*)buff1," %c",(classification_result-36)+97);
	 	}
 	}
				
				
				
			//	sprintf(  &buff1 ,"%5d",classification_result);
				
			//	buff=(unsigned char)buff1;
      	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
				BSP_LCD_DisplayStringAt(270,130,(uint8_t*)buff1,LEFT_MODE);
				
				load=1;
	write=0;
				HAL_Delay(50);
				
			}
			else if(load==1)
			{
		BSP_LCD_Clear(LCD_COLOR_BLACK);
				HAL_Delay(50);
				BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		BSP_LCD_DrawHLine(0,10,240)   ;
		BSP_LCD_DrawHLine(0,260,240)   ;
		BSP_LCD_DrawVLine(0,10,250);
		BSP_LCD_DrawVLine(240,10,250);
		BSP_LCD_DrawHLine(260,10,200)   ;
		BSP_LCD_DrawHLine(260,260,200)   ;
		BSP_LCD_DrawVLine(260,10,250);
		BSP_LCD_DrawVLine(460,10,250); 
				load=0;
				BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
				BSP_LCD_DisplayStringAt(270,130,(uint8_t*)buff1,LEFT_MODE);
				
				
				for(i=0;i<28;i++)
				{
					for(j=0;j<28;j++)
					{
						PxlNet2D[i][j]=0.0;
						
						
					}
					
				}
				
				mode=1;
				
			}
			
			
		}    
	}
		//	
	
	
	
	
		HAL_Delay(10);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 50;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 494;
  hltdc.Init.AccumulatedActiveH = 277;
  hltdc.Init.TotalWidth = 500;
  hltdc.Init.TotalHeigh = 279;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
static void LCD_Config(void)
{
  /* LCD Initialization */ 
  BSP_LCD_Init();
 
  /* LCD Initialization */ 
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
  BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+(BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4));
 
  /* Enable the LCD */ 
  BSP_LCD_DisplayOn(); 
  
  /* Select the LCD Background Layer  */
  BSP_LCD_SelectLayer(0);
 
  /* Clear the Background Layer */ 
  BSP_LCD_Clear(LCD_COLOR_BLACK);  
  
  /* Select the LCD Foreground Layer  */
  BSP_LCD_SelectLayer(1);
 
  /* Clear the Foreground Layer */ 
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  
  /* Configure the transparency for foreground and background :
     Increase the transparency */	
}


static void GetPosition(void)
{
static uint32_t x = 0, y = 0;
  static uint32_t color_heigh;
  static uint32_t color;
  static TS_StateTypeDef  TS_State;

  /* Heigh of color pen */
  color_heigh = 20;


  /* Get Touch screen position */
  BSP_TS_GetState(&TS_State);

  /* Read the coordinate */
  x = TS_State.touchX[0];
  y = TS_State.touchY[0];

 if ((TS_State.touchDetected) & ( x > ((BSP_LCD_GetXSize()/2)-(WINDOWS_WITH/2)-1 + radius)) & ( y > ((BSP_LCD_GetYSize()/2)-(WINDOWS_HEIGHT/2)-1 ) ) & ( x < (BSP_LCD_GetXSize()-((BSP_LCD_GetXSize()/2)-(WINDOWS_WITH/2)-1  + radius )) ) & ( y < (BSP_LCD_GetYSize()-((BSP_LCD_GetYSize()/2)-(WINDOWS_HEIGHT/2)-1 + radius )) ))
  {
    BSP_LCD_FillCircle((x), (y), 7);
    //BSP_LED_On(LED3);
   // BSP_LED_Off(LED4);
  }
  else if ((TS_State.touchDetected) & ( (x > BSP_LCD_GetXSize()-50) & ( y > (BSP_LCD_GetYSize() - 50) )) & ( x < BSP_LCD_GetXSize()-50+28 ) & ( y < (BSP_LCD_GetYSize()) ))
  {

	 //Get picture'
	 // BSP_LED_On(LED4);
	idx=0;
	pxl=0;
	//printf("Read pixel %d to %d\r\n",(BSP_LCD_GetYSize()/2)-(WINDOWS_HEIGHT/2)+radius,((BSP_LCD_GetYSize()/2)-(WINDOWS_HEIGHT/2))+WINDOWS_HEIGHT+2);
	for ( ysrc=(BSP_LCD_GetYSize()/2)-(WINDOWS_HEIGHT/2)+radius,ypos=0; ysrc<((BSP_LCD_GetYSize()/2)-(WINDOWS_HEIGHT/2))+WINDOWS_HEIGHT+2; ysrc+=8,ypos++)
	{
		for ( xsrc=(BSP_LCD_GetXSize()/2)-(WINDOWS_WITH/2),xpos=0; xsrc<((BSP_LCD_GetXSize()/2)-(WINDOWS_WITH/2))+WINDOWS_WITH; xsrc+=8,xpos++)
		{
			for ( ystp=0; ystp<8; ystp++)
			{
				for ( xstp=0; xstp<8; xstp++)
				{
					tmp = BSP_LCD_ReadPixel( xsrc+xstp,ysrc+ystp);
					pxl += (tmp&0xFF);
					//pxl =tmp;
				}
			}
			pxl /= 100;
			if (pxl>0) pxl=255;
			//printf("pixel %d\r\n",pxl);
			//pxl=(0xFF<<24)|(pxl<<16)|(pxl<<8)|pxl;
			PxlNet2D[ypos][xpos] = (float)pxl;
			BSP_LCD_DrawPixel(xsrc/10,ysrc/10+270,(0xFF<<24)|(pxl<<16)|(pxl<<8)|pxl);
			pxl=0;
		}
		//break;

	}
//	BSP_LED_Off(LED4);
//	printf("Call AI Process xpos %d ypos %d\r\n",xpos, ypos);

	MX_X_CUBE_AI_Process();
}

}

/* void process(void)
{
    aiValidationProcess();
    
	
	ai_i32 batch = 0;
	ai_float max_out;
	uint32_t i, ii,x,y,j;

	ai_float dense_2_out[AI_NETWORK_OUT_1_SIZE] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	
	
	memset(dense_2_out, 0, sizeof(ai_float)*AI_NETWORK_OUT_1_SIZE);
	pnum=&PxlNet2D[0][0];
	
		input.data  = AI_HANDLE_PTR((const ai_float *) pnum);
	output.data = AI_HANDLE_PTR(dense_2_out);
	batch = ai_network_run(network, &input, &output);
	if (batch != 1)
	{
	
		while(1);
	}
	
		max_out = dense_2_out[0];
	classification_result = 0;
	for (i = 1; i < AI_NETWORK_OUT_1_SIZE; i++)
	{
		if (dense_2_out[i] > max_out)
		{
			max_out = dense_2_out[i];
			classification_result = i;
      
		}
	}
}
	 */
	
    /* USER CODE END 1 */











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
