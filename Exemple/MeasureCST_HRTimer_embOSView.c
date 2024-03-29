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
File    : MeasureCST_HRTimer_embOSView.c
Purpose : Performance test program for OS
          This benchmark measures the OS context switching time and
          displays the result in the terminal window of embOSView.
          It is completly generic and runs on every target that is
          configured for embOSView.
--------  END-OF-HEADER  ---------------------------------------------
*/

#include "RTOS.h"
#include "stdio.h"

static OS_STACKPTR int StackHP[128], StackLP[128];  // Task stacks
static OS_TASK TCBHP, TCBLP;                        // Task-control-blocks
static OS_U32 _Time;                                // Timer value for context switch measurement

/*********************************************************************
*
*       HPTask
*/
static void HPTask(void) {
  while (1) {
    OS_Suspend(NULL);                   // Suspend high priority task
    OS_Timing_End(&_Time);              // Stop measurement
  }
}

/*********************************************************************
*
*       LPTask
*/
static void LPTask(void) {
  char   acBuffer[100];                 // Output buffer
  OS_U32 MeasureOverhead;               // Time for Measure Overhead
  OS_U32 v;                             // Real context switching time

  //
  // Measure Overhead for time measurement so we can take this into account by subtracting it
  //
  OS_Timing_Start(&MeasureOverhead);
  OS_Timing_End(&MeasureOverhead);
  //
  // Perform measurements in endless loop
  //
  while (1) {
    OS_Delay(100);                    // Syncronize to tick to avoid jitter
    OS_Timing_Start(&_Time);          // Start measurement
    OS_Resume(&TCBHP);                // Resume high priority task to force task switch
    v  = OS_Timing_GetCycles(&_Time);
    v -= OS_Timing_GetCycles(&MeasureOverhead); // Calculate real context switching time (w/o measurement overhead)
    v  = OS_ConvertCycles2us(1000 * v);         // Convert cycles to nano-seconds, increase time resolution
    sprintf(acBuffer, "Context switch time: %u.%.3u usec\r", v / 1000, v % 1000); // Create result text
    OS_SendString(acBuffer);                                                      // Print out result
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

