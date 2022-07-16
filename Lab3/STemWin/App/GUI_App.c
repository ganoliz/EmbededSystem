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

#include "fatfs.h"

extern  WM_HWIN CreateFramewin(void); 
  

void GRAPHICS_MainTask(void) {

  /* 1- Create a FrameWin using GUIBuilder */
 // CreateFramewin();
 
/* USER CODE BEGIN GRAPHICS_MainTask */
 /* User can implement his graphic application here */
  /* Hello Word example */
int status=0;
	static TS_StateTypeDef TS_State;
	GUI_PID_STATE ts;
		static uint8_t *acBuffer;
 FRESULT f_res;
 int size;
	UINT count;
		char SDPath[4];                   
  FATFS fs;                                                                                                        
  FIL file;                                                                                                                        
  UINT fnum;
	 int a=0;
	int pic_number=0;
	int x;
	int y;
	int stop_flag=0;
	int counter=0;
	 BSP_TS_Init(480,272);	//static TS_StateTypeDef  TS_State;
          ts.Layer=0;
	
	
	
	//if( FATFS_LinkDriver(&SD_Driver, SDPath)==0)
		
			f_res = f_mount(&SDFatFS,"",1);  //(TCHAR const*)SDPath
		if(f_res != FR_OK){
						while(1);
					}
		
		

   //    GUI_BMP_DrawScaled(acBuffer,0,0,1,2);
       
			
 	//  GUI_BMP_DrawScaled(acBuffer,0,0,1,3);
   
/* USER CODE END GRAPHICS_MainTask */
  while(1)
		
{   	
 BSP_TS_GetState(&TS_State);
	if(TS_State.touchDetected)
	{ if(stop_flag==0)
		stop_flag=1;
		else if(stop_flag==1)
			stop_flag=0;
	}
	
	if(counter>=12 && stop_flag !=1)
	{
		pic_number++;
		if(pic_number>4)
			pic_number=0;
		counter=0;
		GUI_Clear();
	}
	
	
	
	
  if(pic_number==0 )
	{f_res=	f_open(&SDFile,"logo_1.bmp",FA_READ);
	size = SDFile.obj.objsize;                   //file error
    GUI_HMEM hMem = GUI_ALLOC_AllocZero(size);
  acBuffer =(char *) GUI_ALLOC_h2p(hMem);
 f_res = f_read(&SDFile, acBuffer, size, &count );
	f_res=f_close(&SDFile);
 x=GUI_BMP_GetXSize(acBuffer);
   	GUI_BMP_DrawScaled(acBuffer,0,0,480,x);
	//	if(stop_flag==0)
	//	pic_number++;
		GUI_ALLOC_Free(hMem);
	}
	else if(pic_number==1)
	{f_res=	f_open(&SDFile,"logo_2.bmp",FA_READ);
	size = SDFile.obj.objsize;                   //file error
    GUI_HMEM hMem = GUI_ALLOC_AllocZero(size);
  acBuffer =(char *) GUI_ALLOC_h2p(hMem);
 f_res = f_read(&SDFile, acBuffer, size, &count );
	f_res=f_close(&SDFile);
 x=GUI_BMP_GetXSize(acBuffer);
   	GUI_BMP_DrawScaled(acBuffer,0,0,480,x);
	//	if(stop_flag==0)
	//	pic_number++;
		GUI_ALLOC_Free(hMem);
	}
	else if(pic_number==2)
	{f_res=	f_open(&SDFile,"ntust_1.bmp",FA_READ);
	size = SDFile.obj.objsize;                   //file error
    GUI_HMEM hMem = GUI_ALLOC_AllocZero(size);
  acBuffer =(char *) GUI_ALLOC_h2p(hMem);
 f_res = f_read(&SDFile, acBuffer, size, &count );
	f_res=f_close(&SDFile);

 x=GUI_BMP_GetXSize(acBuffer);
   	GUI_BMP_DrawScaled(acBuffer,0,0,480,x);
	//	if(stop_flag==0)
	//	pic_number++;
		GUI_ALLOC_Free(hMem);
	}
	else if(pic_number==3)
	{f_res=	f_open(&SDFile,"ntust_2.bmp",FA_READ);
	size = SDFile.obj.objsize;                   //file error
    GUI_HMEM hMem = GUI_ALLOC_AllocZero(size);
  acBuffer =(char *) GUI_ALLOC_h2p(hMem);
 f_res = f_read(&SDFile, acBuffer, size, &count );
	f_res=f_close(&SDFile);

 x=GUI_BMP_GetXSize(acBuffer);
   	GUI_BMP_DrawScaled(acBuffer,0,0,480,x);
	//	if(stop_flag==0)
	//	pic_number++;
		GUI_ALLOC_Free(hMem);
	}
	else if(pic_number==4)
	{f_res=	f_open(&SDFile,"ntust_3.bmp",FA_READ);
	size = SDFile.obj.objsize;                   //file error
    GUI_HMEM hMem = GUI_ALLOC_AllocZero(size);
  acBuffer =(char *) GUI_ALLOC_h2p(hMem);
 f_res = f_read(&SDFile, acBuffer, size, &count );
	f_res=f_close(&SDFile);

 x=GUI_BMP_GetXSize(acBuffer);
   	GUI_BMP_DrawScaled(acBuffer,0,0,480,x);
	//	if(stop_flag==0)
	//	pic_number=0;
		GUI_ALLOC_Free(hMem);
	}
	//GUI_ALLOC_Free(size);
 HAL_Delay(50);
	counter++;
	
   //   GUI_Delay(100);
}
}

/*************************** End of file ****************************/
