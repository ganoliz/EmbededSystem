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
#include "stm32746g_discovery_lcd.h"
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

DMA2D_HandleTypeDef hdma2d;

LTDC_HandleTypeDef hltdc;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t i=0;
uint8_t sp=0;
int z;
int  rad;
 static int mode=0;
uint8_t uart_buffer[7]={0};
static void LCD_Config(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int temp=0;
	int current_state=0;
	int pre_state=0;
	int count=0;
	int y =5;
	int a,b;
	
	int complete_flag=0;  //complete transfer flag
	int flag=0;   //input posedge detect
	int busy=0;
	int random;
  /* USER CODE END 1 */
  

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
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	LCD_Config(); 
 BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
// BSP_LCD_DisplayStringAt(0,0,(uint8_t *)"lcd test ok",LEFT_MODE); 
 HAL_GPIO_WritePin(GPIOI,GPIO_PIN_1,GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 while (1)
  { HAL_Delay(20);
		current_state= HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_11);
		if(pre_state==current_state)
		{
			temp=current_state;
			pre_state=current_state;

		}	
		else {
		     temp=pre_state;
		     pre_state=current_state;
				 if(temp==0)
					 flag=1;    //detected posedge
			
		     }
		if(temp==1 && flag==1)
		  {   if(mode==0)
						{mode=1; 
						HAL_UART_Abort(&huart1);
						}
				  else if(mode==1 && busy==0)
						mode=0;
					
					flag=0;
			}	 
		count=count+1;
		// HAL_UART_IRQHandler(&huart1);
			

			
			
			
			switch(mode){
			
			
						 
			case 0:   { 
									if(complete_flag==0)
									{	
										HAL_GPIO_WritePin(GPIOI,GPIO_PIN_1,GPIO_PIN_RESET);
										HAL_UART_Receive(&huart1,&uart_buffer[0],1,10);
                        i=uart_buffer[0];
			
								//		HAL_UART_RxCpltCallback(&huart1);
			
											
             								 count=0;
										if(i>0 ){
										// i=i-48;
												BSP_LCD_Clear(LCD_COLOR_WHITE);
											BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
											busy=1;
											
									//		BSP_LCD_DisplayStringAt(0,10,(uint8_t *)"start",LEFT_MODE);
											HAL_UART_Transmit(&huart1,(uint8_t *)"start",5,10);
											HAL_UART_Transmit(&huart1,(uint8_t *)"\r\n",2,10);
											complete_flag=1;
											//mode=2;
										}
                 }
									else if(complete_flag==1)
									{
											if(count>=20)
										{			
										if(i>48)     //48
										{			
											if(sp==0)
											{	
											BSP_LCD_DisplayStringAt(0,y,&i,LEFT_MODE);
											
													HAL_UART_Transmit(&huart1,&i,1,10);
													HAL_UART_Transmit(&huart1,(uint8_t *)"\r\n",2,10);
											HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_1);
											i=i-1;
											y=y+24;
												sp=1;
											}
											else if(sp==1)
											{
												HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_1); sp=0;
											}
										}	
										else	if(i<=48)
												{  
													
													complete_flag=0;
													count=0;
													HAL_UART_Transmit(&huart1,(uint8_t *)"happy embedded project final\r\n" ,30,10);
													BSP_LCD_DisplayStringAt(0,200,(uint8_t *)"happy embedded project final",LEFT_MODE);
													HAL_Delay(3000);
													BSP_LCD_Clear(LCD_COLOR_WHITE);	
													random=(rand() % 17) +5;
													for (z=0;z<random;z++)
													{
													if(z%5==0)	BSP_LCD_SetTextColor(LCD_COLOR_RED);
														if(z%5==1 )	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
														if(z%5==2)	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
														if(z%5==3)	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
														if(z%5==4)	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
														//if(z==5)	BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
														a=(rand() % 400) +50;
													b=(rand() % 200)  +50;
														rad=(rand() % 10)  +2;
													BSP_LCD_FillCircle(a,b,3*rad);
														HAL_Delay(1000);
													}
													HAL_Delay(1000);
													i=0;
													uart_buffer[0]=0;
													y=5;
													busy=0;
													break;
												}
									HAL_UART_Abort(&huart1);
												
									count=0;
											}
										
										
										
										
				//						HAL_UART_Transmit(&huart1,(uint8_t *)"End\r\n" ,5,5);
					//			    complete_flag=0;
										
									}		
								break;			
							 }		
			
							 
							 
						/*	 			case 1: {  if(count>=5)
				         {
									 HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_1);
									 count=0;
									BSP_LCD_Clear(LCD_COLOR_WHITE);	
                 }
								 
							  break; 
							}		
							 
							 */
/*			case 2:     
			{							
								if(count>=20)
								{			i=i-1;
									if(i>48)
									{			
									BSP_LCD_DisplayStringAt(0,60-i,(uint8_t *)&i,LEFT_MODE);
											HAL_UART_Transmit(&huart1,(uint8_t *)&i,1,5);
										HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_1);
									}	
									else	if(i<=48)
											{ HAL_UART_Abort  ( &huart1 ); 
												mode=1;
												complete_flag=1;
												count=0;
												break;
											}	
									count=0;
								}
								break;
			}                         */
      default: mode=0;
								 
				 }
						 
			
	/*	switch(mode){
			
			
			case 0: {  if(count>=5)
				         {
									 HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_1);
									 count=0;
                 }
							  break; 
							}								 
			case 1:   { 
									
									
									if(complete_flag==0)
						 {	
                    
										HAL_UART_Receive_IT(&huart1,(uint8_t *)&uart_buffer,1);
										HAL_UART_RxCpltCallback(&huart1);
									
										   //uart_buffer[0];
										//i=uart_buffer[0];
										
								//	HAL_UART_IRQHandler(&huart1);
									
									if(i>0 )
  								{						mode=2;
									BSP_LCD_DisplayStringAt(0,0,(uint8_t *)"Start",LEFT_MODE);
										
							//		HAL_UART_IRQHandler(&huart1);			
								  HAL_UART_Transmit_IT(&huart1,(uint8_t *)"Start\r\n" ,7);
								  HAL_UART_TxCpltCallback(&huart1);		
								  		
									
										                                                  //transfer mode should be ensure
									}	                                       // should  test if i!=0     
							}	
								else if(complete_flag==1)
								{
								//HAL_UART_IRQHandler(&huart1);	
									
					//				HAL_Delay(200);
								HAL_UART_Transmit_IT(&huart1,(uint8_t *)"End\r\n" ,5);
								HAL_UART_TxCpltCallback(&huart1);	
						//		BSP_LCD_DisplayStringAt(0,10,(uint8_t *)"End",LEFT_MODE);	
								complete_flag=0;
									
								}
								break;
								
						}
			case 2:
						{
	
							
							
							//	BSP_LCD_DisplayStringAt(0,6,(uint8_t *)"your number",LEFT_MODE);
								  if(count>=25)              //	revise to 5 for test
									{
								//		BSP_LCD_DisplayStringAt(0,20,(uint8_t *)"your number",LEFT_MODE);
							//		HAL_UART_IRQHandler(&huart1);	
									if(complete_flag==1 )
									{
										
										mode=1;
									}	
										
										i=i-1;
									HAL_UART_Transmit_IT(&huart1,(uint8_t *)&i ,1);
									HAL_UART_TxCpltCallback(&huart1);
									
								HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_1);	
										
								 

									count=0;		
								
									}	
							

				
			
			        break;
						}
			//default: mode=0;
					}
			
			
			
			
			   */
		}	
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 50;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PI1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PI11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

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
