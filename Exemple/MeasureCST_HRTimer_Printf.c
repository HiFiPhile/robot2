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
File    : MeasureCST_HRTimer_Printf.c
Purpose : Performance test program for OS
          This benchmark measures the context switching time and displays
          the result on CSpy's terminal I/O window. It runs with every
          IAR workbench that supports terminal I/O with printf.
--------  END-OF-HEADER  ---------------------------------------------
*/

#include "RTOS.h"
#include "stdio.h"

static OS_STACKPTR int StackHP[128], StackLP[128];  // Task stacks
static OS_TASK TCBHP, TCBLP;                        // Task-control-blocks
static OS_U32 _Time;                                // Timer value for context switching measurement

/*********************************************************************
*
*       HPTask
*/
static void HPTask(void) {
  while (1) {
    OS_Suspend(NULL);              // Suspend high priority task
    OS_Timing_End(&_Time);         // Stop measurement
  }
}

/*********************************************************************
*
*       LPTask
*/
static void LPTask(void) {
  OS_U32 MeasureOverhead;          // Time for Measure Overhead
  OS_U32 v;                        // Real context switching time

  //
  // Measure overhead for time measurement so we can take this into account by subtracting it
  //
  OS_Timing_Start(&MeasureOverhead);
  OS_Timing_End(&MeasureOverhead);
  //
  // Perform measurements in endless loop
  //
  while (1) {
    OS_Delay(100);                 // Syncronize to tick to avoid jitter
    OS_Timing_Start(&_Time);       // Start measurement
    OS_Resume(&TCBHP);             // Resume high priority task to force task switch
    v  = OS_Timing_GetCycles(&_Time);
    v -= OS_Timing_GetCycles(&MeasureOverhead); // Calculate real context switching time (w/o measurement overhead)
    v  = OS_ConvertCycles2us(1000 * v);         // Convert cycles to nano-seconds, increase time resolution
    printf("Context switch time: %lu.%.3lu usec\r\n", (v / 1000uL), (v % 1000uL));     // Print out result
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
  OS_CREATETASK(&TCBHP, "HP Task", HPTask, 100, StackHP);
  OS_CREATETASK(&TCBLP, "LP Task", LPTask,  99, StackLP);
  OS_Start();                      // Start multitasking
  return 0;
}

