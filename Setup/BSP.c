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
File    : BSP.c
Purpose : BSP for STM8S on the STM8/128_Eval board
--------  END-OF-HEADER  ---------------------------------------------
*/

#define BSP_C
#include "BSP.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/

/****** SFRs used for LED-Port **************************************/

#define PORTH_ODR  (*(volatile unsigned char*) (0x005023))
#define PORTH_IDR  (*(volatile unsigned char*) (0x005024))
#define PORTH_DDR  (*(volatile unsigned char*) (0x005025))
#define PORTH_CR1  (*(volatile unsigned char*) (0x005026))
#define PORTH_CR2  (*(volatile unsigned char*) (0x005027))

/****** Assign LEDs to Ports ****************************************/

#define LED0_BIT  (0)
#define LED1_BIT  (1)
#define LED2_BIT  (2)
#define LED3_BIT  (3)

#define LED_MASK_ALL ((1uL << LED0_BIT) | (1uL << LED1_BIT) | (1uL << LED2_BIT) | (1uL << LED3_BIT))

#define LED_PORT  PORTH_ODR

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/

/*********************************************************************
*
*       BSP_Init()
*/
void BSP_Init(void) {
  //
  // Initialize LED-Port
  //
  PORTH_DDR |= LED_MASK_ALL;    // Configure Ports as outputs
  PORTH_ODR &= ~(LED_MASK_ALL); // Initially switch off outputs
  PORTH_CR1 |= LED_MASK_ALL;    // Activate Push-Pull buffer
  PORTH_CR2 |= LED_MASK_ALL;    // Limit output slew rate
}

/*********************************************************************
*
*       LED switching routines
*       LEDs are switched on with high level on port lines
*/
void BSP_SetLED(int Index) {
  LED_PORT |= (1uL << (LED0_BIT + Index));  // Switch on LED
}

void BSP_ClrLED(int Index) {
  LED_PORT &= ~(1uL << (LED0_BIT + Index)); // Switch off LED
}

void BSP_ToggleLED(int Index) {
  LED_PORT ^= (1uL << (LED0_BIT + Index));  // Toggle LED
}

/****** End Of File *************************************************/
