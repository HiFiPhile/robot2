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
File    : ExtendTaskContext.c
Purpose : Sample program for demonstrating how to dynamically
          extend the task context.
          This example adds a global variable to the task context of
          certain tasks.
--------  END-OF-HEADER  ---------------------------------------------
*/

#include "RTOS.h"

OS_STACKPTR int StackHP[128], StackLP[128];          /* Task stacks */
OS_TASK TCBHP, TCBLP;                        /* Task-control-blocks */

int GlobalVar;

/*********************************************************************
*
*       _Restore
*       _Save
*
*  Function description
*    This function pair saves and restores an extended task context.
*    In this case, the extended task context consists of just a single
*    member, which is a global variable.
*/
typedef struct {
  int GlobalVar;
} CONTEXT_EXTENSION;


static void _Save(void * pStack) {
  CONTEXT_EXTENSION * p;
  p = ((CONTEXT_EXTENSION*)pStack) - (1 - OS_STACK_AT_BOTTOM);          // Create pointer to our structure
  //
  // Save all members of the structure
  //
  p->GlobalVar = GlobalVar;
}

static void _Restore(const void * pStack) {
  const CONTEXT_EXTENSION * p;
  p = ((const CONTEXT_EXTENSION *)pStack) - (1 - OS_STACK_AT_BOTTOM);   // Create pointer to our structure
  //
  // Restore all members of the structure
  //
  GlobalVar = p->GlobalVar;
}

/*********************************************************************
*
*       Global variable which holds the function pointers
*       to save and restore the task context.
*/
const OS_EXTEND_TASK_CONTEXT _SaveRestore = {
  _Save,
  _Restore
};

/********************************************************************/


/*********************************************************************
*
*       HPTask
*
*  Function description
*    During the execution of this function, the thread-specific
*    global variable has always the same value of 1.
*/
static void HPTask(void) {
  OS_ExtendTaskContext(&_SaveRestore);
  GlobalVar = 1;
  while (1) {
    OS_Delay (10);
  }
}

/*********************************************************************
*
*       LPTask
*
*  Function description
*    During the execution of this function, the thread-specific
*    global variable has always the same value of 2.
*/
static void LPTask(void) {
  OS_ExtendTaskContext(&_SaveRestore);
  GlobalVar = 2;
  while (1) {
    OS_Delay (50);
  }
}

/*********************************************************************
*
*       main
*/
int main(void) {
  OS_IncDI();                      /* Initially disable interrupts  */
  OS_InitKern();                   /* Initialize OS                 */
  OS_InitHW();                     /* Initialize Hardware for OS    */
  /* You need to create at least one task here !                    */
  OS_CREATETASK(&TCBHP, "HP Task", HPTask, 100, StackHP);
  OS_CREATETASK(&TCBLP, "LP Task", LPTask,  50, StackLP);
  OS_Start();                      /* Start multitasking            */
  return 0;
}

