  /**
  ******************************************************************************
  * @file    GUI_App.c
  * @author  MCD Application Team
  * @brief   Simple demo drawing "Hello world"  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright © 2018 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
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

#include "GUI_App.h"
#include "GUI.h"

#include "DIALOG.h"
#include "ft5336.h"
#include "stm32746g_discovery_ts.h"
extern  WM_HWIN CreateFramewin(void); 
  WM_HWIN hWin;
		static TS_StateTypeDef TS_State;
	GUI_PID_STATE ts;
 // int color=0;
// int x,y;


void GRAPHICS_MainTask(void) {

  /* 1- Create a FrameWin using GUIBuilder */
 hWin= CreateFramewin();
 
/* USER CODE BEGIN GRAPHICS_MainTask */
 /* User can implement his graphic application here */
  /* Hello Word example */
   /* GUI_Clear();
    GUI_SetColor(GUI_WHITE);
    GUI_SetFont(&GUI_Font32_1);
    GUI_DispStringAt("Hello world!", (LCD_GetXSize()-150)/2, (LCD_GetYSize()-20)/2);
   */
	WM_HWIN hWinOld;
		int status=0;

	
 	
          BSP_TS_Init(480,272);	//static TS_StateTypeDef  TS_State;
          ts.Layer=0;                                        	//  static GUI_PID_STATE ts;
   // color=0;
	
	GUI_SetColor(GUI_BLACK);
	
	
/* USER CODE END GRAPHICS_MainTask */
  while(1)
{
       BSP_TS_GetState(&TS_State);
	if(TS_State.touchDetected)
	{
		ts.Pressed=1;
		ts.x=TS_State.touchX[0];
		ts.y=TS_State.touchY[0];
	/*	if(ts.x>10 && ts.x< 389 && ts.y>0 && ts.y<242)
		{
			
		//	GUI_FillCircle(ts.x,ts.y,1);
			ts.x=x;
			ts.y=y;
			
			//  hWinOld= WM_SelectWindow(WM_GetClientWindow(hWin));
	//		 WM_SelectWindow(hWinOld);
	//		 GUI_SetColor(GUI_YELLOW);
		//	 GUI_SelectLayer(3);
			// GUI_FillCircle(x,y,1);
			
		}
		else if(ts.x>389 && y>=90 && y<120)
		{
			
			color=1;
		}
		else if(ts.x>389 && y>=120 && y<151)
		{
			color=2;
		}
		else if(ts.x>389 && y>=151 && y<182)
		{
			
			color=3;
		}
				else if(ts.x>389 && y>=182 && y<212)
		{
			
			color=0;
		}
				else if(ts.x>389 && y>=212 && y<270)
		{
			
			color=4;
		}
					 switch(color){
				 
				 case 0: GUI_SetColor(GUI_BLACK); break;
				 case 1: GUI_SetColor(GUI_RED); break;
				 case 2:GUI_SetColor(GUI_GREEN); break;
				 case 3:GUI_SetColor(GUI_BLUE); break;
				 case 4:GUI_SetColor(GUI_YELLOW); break;
			 }
		//			 hWinOld= WM_SelectWindow(WM_GetClientWindow(hWin));
    */
		
		GUI_TOUCH_StoreStateEx(&ts);
		GUI_Exec();
		
	}
	else
	{
   ts.Pressed=0;
GUI_TOUCH_StoreStateEx(&ts);
GUI_Exec();		

		
	}
	
}
}










/*************************** End of file ****************************/
