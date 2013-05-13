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
File    : MeasureCST_Scope.c
Purpose : Performance test program for OS
          This benchmark uses the BSP.c module to set and clear a port
          pin. This allows measuring the context switching time with an oscilloscope.

          The context switch time is

            Time = (d - c) - (b - a)

             -----   --                   ---------------
                  | |  |                 |
                   -    -----------------
                  ^ ^  ^                 ^
                  a b  c                 d


          The time between c and d ist the context switch time, but
          note that the real context switching time is shorter, because the
          signal also contains the overhead of switching the LED on and
          off. The time of this overhead is also displayed on the oscilloscope
          as a small peak between a and b.
--------  END-OF-HEADER  ---------------------------------------------
*/



#include "RTOS.h"
#include "BSP.h"

static OS_STACKPTR int StackHP[128], StackLP[128];  // Task stacks
static OS_TASK TCBHP, TCBLP;                        // Task-control-blocks

/*********************************************************************
*
*       HPTask
*/
static void HPTask(void) {
  while (1) {
    OS_Suspend(NULL);   // Suspend high priority task
    BSP_ClrLED(0);      // Stop measurement
  }
}

/*********************************************************************
*
*       LPTask
*/
static void LPTask(void) {
  while (1) {
    OS_Delay(100);     // Syncronize to tick to avoid jitter
    //
    // Display measurement overhead
    //
    BSP_SetLED(0);
    BSP_ClrLED(0);
    //
    // Perform measurement
    //
    BSP_SetLED(0);     // Start measurement
    OS_Resume(&TCBHP); // Resume high priority task to force task switch
  }
}

/*********************************************************************
*
*       main
*/
int main(void) {
  OS_IncDI();                      // Initially disable interrupts
  OS_InitKern();                   // Initialize OS
  OS_InitHW();                     // Initialize Hardware for OS
  BSP_Init();                      // Initialize LED ports
  OS_CREATETASK(&TCBHP, "HP Task", HPTask, 100, StackHP);
  OS_CREATETASK(&TCBLP, "LP Task", LPTask,  99, StackLP);
  OS_Start();                      // Start multitasking
  return 0;
}
