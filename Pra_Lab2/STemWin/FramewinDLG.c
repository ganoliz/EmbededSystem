/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.44                          *
*        Compiled Nov 10 2017, 08:53:57                              *
*        (c) 2017 Segger Microcontroller GmbH & Co. KG               *
*                                                                    *
**********************************************************************
*                                                                    *
*        Internet: www.segger.com  Support: support@segger.com       *
*                                                                    *
**********************************************************************
*/

// USER START (Optionally insert additional includes)
// USER END

#include "DIALOG.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define ID_FRAMEWIN_0 (GUI_ID_USER + 0x00)
#define ID_BUTTON_0 (GUI_ID_USER + 0x01)
#define ID_BUTTON_1 (GUI_ID_USER + 0x02)
#define ID_BUTTON_2 (GUI_ID_USER + 0x03)
#define ID_BUTTON_3 (GUI_ID_USER + 0x04)
#define ID_BUTTON_4 (GUI_ID_USER + 0x05)
#define ID_BUTTON_5 (GUI_ID_USER + 0x06)
#define ID_BUTTON_6 (GUI_ID_USER + 0x07)
#define ID_BUTTON_7 (GUI_ID_USER + 0x08)
#define ID_BUTTON_8 (GUI_ID_USER + 0x09)
#define ID_BUTTON_9 (GUI_ID_USER + 0x0A)
#define ID_BUTTON_10 (GUI_ID_USER + 0x0B)
#define ID_BUTTON_11 (GUI_ID_USER + 0x0C)
#define ID_BUTTON_12 (GUI_ID_USER + 0x0D)
#define ID_BUTTON_13 (GUI_ID_USER + 0x0E)
#define ID_BUTTON_14 (GUI_ID_USER + 0x0F)
#define ID_BUTTON_15 (GUI_ID_USER + 0x10)
#define ID_BUTTON_16 (GUI_ID_USER + 0x11)
#define ID_BUTTON_17 (GUI_ID_USER + 0x12)
#define ID_TEXT_0 (GUI_ID_USER + 0x13)
#define ID_EDIT_0 (GUI_ID_USER + 0x14)


// USER START (Optionally insert additional defines)
// USER END

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

// USER START (Optionally insert additional static data)
// USER END

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { FRAMEWIN_CreateIndirect, "Framewin", ID_FRAMEWIN_0, -1, -2, 480, 272, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button0", ID_BUTTON_0, 6, 92, 79, 30, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button1", ID_BUTTON_1, 85, 91, 87, 29, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button2", ID_BUTTON_2, 175, 93, 90, 27, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button3", ID_BUTTON_3, 7, 122, 83, 29, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button4", ID_BUTTON_4, 88, 120, 84, 32, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button5", ID_BUTTON_5, 173, 119, 90, 29, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button6", ID_BUTTON_6, 6, 153, 80, 30, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button7", ID_BUTTON_7, 90, 151, 84, 29, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button8", ID_BUTTON_8, 175, 152, 87, 30, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button9", ID_BUTTON_9, 4, 182, 82, 25, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button_dot", ID_BUTTON_10, 89, 182, 86, 26, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button00", ID_BUTTON_11, 176, 182, 82, 28, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button_plus", ID_BUTTON_12, 263, 95, 81, 30, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button_sub", ID_BUTTON_13, 264, 125, 86, 30, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button_mut", ID_BUTTON_14, 266, 156, 82, 27, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button_div", ID_BUTTON_15, 261, 184, 84, 24, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button_clr", ID_BUTTON_16, 349, 94, 79, 61, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button_equ", ID_BUTTON_17, 345, 155, 86, 53, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_0, 200, 216, 128, 27, 0, 0x64, 0 },
  { EDIT_CreateIndirect, "Edit", ID_EDIT_0, 106, 31, 223, 33, 0, 0x64, 0 },
  // USER START (Optionally insert additional widgets)
  // USER END
};

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

// USER START (Optionally insert additional static code)
// USER END

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN hItem;
  int     NCode;
  int     Id;
  // USER START (Optionally insert additional variables)
	
	int j=0;
	char buff[100]={'0'};
	
	
  // USER END

  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    //
    // Initialization of 'Button0'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0);
    BUTTON_SetText(hItem, "0");
    //
    // Initialization of 'Button1'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_1);
    BUTTON_SetText(hItem, "1");
    //
    // Initialization of 'Button2'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_2);
    BUTTON_SetText(hItem, "2");
    //
    // Initialization of 'Button3'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_3);
    BUTTON_SetText(hItem, "3");
    //
    // Initialization of 'Button4'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_4);
    BUTTON_SetText(hItem, "4");
    //
    // Initialization of 'Button5'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_5);
    BUTTON_SetText(hItem, "5");
    //
    // Initialization of 'Button6'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_6);
    BUTTON_SetText(hItem, "6");
    //
    // Initialization of 'Button7'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_7);
    BUTTON_SetText(hItem, "7");
    //
    // Initialization of 'Button8'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_8);
    BUTTON_SetText(hItem, "8");
    //
    // Initialization of 'Button9'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_9);
    BUTTON_SetText(hItem, "9");
    //
    // Initialization of 'Button_dot'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_10);
    BUTTON_SetText(hItem, ".");
    //
    // Initialization of 'Button00'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_11);
    BUTTON_SetText(hItem, "00");
    //
    // Initialization of 'Button_plus'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_12);
    BUTTON_SetText(hItem, "+");
    //
    // Initialization of 'Button_sub'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_13);
    BUTTON_SetText(hItem, "-");
    //
    // Initialization of 'Button_mut'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_14);
    BUTTON_SetText(hItem, "*");
    //
    // Initialization of 'Button_div'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_15);
    BUTTON_SetText(hItem, "/");
    //
    // Initialization of 'Button_clr'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_16);
    BUTTON_SetText(hItem, "Clear");
    //
    // Initialization of 'Button_equ'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_17);
    BUTTON_SetText(hItem, "=");
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
    TEXT_SetText(hItem, "NTUST");
    //
    // Initialization of 'Edit'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_0);
    EDIT_SetText(hItem, "0");
    // USER START (Optionally insert additional code for further widget initialization)
    // USER END
    break;
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {
    case ID_BUTTON_0: // Notifications sent by 'Button0'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
			
			      GUI_StoreKeyMsg(0,1);
			BUTTON_GetText(pMsg->hWinSrc,&buff[j],sizeof(buff));
		  j++;
			hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_0);
      EDIT_SetText(hItem, "0");
			
			
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
			
			
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_1: // Notifications sent by 'Button1'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
						hItem = WM_GetDialogItem(pMsg->hWin, ID_EDIT_0);
      EDIT_SetText(hItem, "1");
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_2: // Notifications sent by 'Button2'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_3: // Notifications sent by 'Button3'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_4: // Notifications sent by 'Button4'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_5: // Notifications sent by 'Button5'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_6: // Notifications sent by 'Button6'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_7: // Notifications sent by 'Button7'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
			
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_8: // Notifications sent by 'Button8'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_9: // Notifications sent by 'Button9'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_10: // Notifications sent by 'Button_dot'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_11: // Notifications sent by 'Button00'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_12: // Notifications sent by 'Button_plus'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_13: // Notifications sent by 'Button_sub'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_14: // Notifications sent by 'Button_mut'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_15: // Notifications sent by 'Button_div'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_16: // Notifications sent by 'Button_clr'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_17: // Notifications sent by 'Button_equ'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
			 
			 
			
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_EDIT_0: // Notifications sent by 'Edit'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    // USER START (Optionally insert additional code for further Ids)
    // USER END
    }
    break;
  // USER START (Optionally insert additional message handling)
  // USER END
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       CreateFramewin
*/
WM_HWIN CreateFramewin(void);
WM_HWIN CreateFramewin(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
  return hWin;
}

// USER START (Optionally insert additional public code)
// USER END

/*************************** End of file ****************************/
