/*********************************************************************
*               SEGGER MICROCONTROLLER GmbH & Co KG                  *
*       Solutions for real time microcontroller applications         *
**********************************************************************
*                                                                    *
*       (c) 1995 - 2012  SEGGER Microcontroller GmbH & Co KG         *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       embOS * Real time operating system for microcontrollers      *
*                                                                    *
*                                                                    *
*       Please note:                                                 *
*                                                                    *
*       Knowledge of this file may under no circumstances            *
*       be used to write a similar product or a real-time            *
*       operating system for in-house use.                           *
*                                                                    *
*       Thank you for your fairness !                                *
*                                                                    *
**********************************************************************
*                                                                    *
*       OS version: 3.86i                                            *
*                                                                    *
**********************************************************************

----------------------------------------------------------------------
File    : Main_OS_Q.c
Purpose : Sample program for embOS using queues
--------- END-OF-HEADER --------------------------------------------*/

#include "RTOS.h"

OS_STACKPTR int StackHP[128], StackLP[128];          /* Task stacks */
OS_TASK TCBHP, TCBLP;                        /* Task-control-blocks */

/********************************************************************/

static OS_Q _MyQ;
static char _MyQBuffer[100];
	
static void HPTask(void) {
  char* pData;
  while (1) {
    int Len;
    Len = OS_Q_GetPtr(&_MyQ, (void**)&pData);
    OS_Delay(10);
    if (Len) {  /* Evaluate Message ... */
      OS_SendString(pData);
      OS_Q_Purge(&_MyQ);
    }
  }
}
	
static void LPTask(void) {
  while (1) {
    OS_Q_Put(&_MyQ, "\nHello", 7);
    OS_Q_Put(&_MyQ, "\nWorld !", 9);
    OS_Delay (500);
  }
}
	
/*********************************************************************
*
*       main
*
*********************************************************************/

int main(void) {
  OS_IncDI();                   /* Initially disable interrupts     */
  OS_InitKern();                /* Initialize OS                    */
  OS_InitHW();                  /* Initialize Hardware for OS       */
  OS_Q_Create(&_MyQ, &_MyQBuffer, sizeof(_MyQBuffer));
  /* You need to create at least one task before calling OS_Start() */
  OS_CREATETASK(&TCBHP, "HP Task", HPTask, 100, StackHP);
  OS_CREATETASK(&TCBLP, "LP Task", LPTask,  50, StackLP);
  OS_DecRI();                   /* Enable interrupts before sending */
  OS_SendString("embOS OS_Q example");
  OS_SendString("\n\nDemonstrating message passing\n");
  OS_Start();                   /* Start multitasking               */
  return 0;
}
