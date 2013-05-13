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
File    : RTOSInit_STM8.c                    (for ST STM8 CPUs)
Purpose : Initializes and handles the hardware for embOS as far
          as required by embOS.
          Feel free to modify this file acc. to your
          target system.
--------  END-OF-HEADER  ---------------------------------------------
*/

#include "RTOS.h"

/*********************************************************************
*
*                    Configuration
*
**********************************************************************
*/

#ifndef   OS_FSYS
  #define OS_FSYS (16000000uL)  /* Standard setting is 16 MHz       */
#endif

#ifndef   OS_PCLK_DIVIDER
  #define OS_PCLK_DIVIDER  (1)
#endif

#ifndef   OS_PCLK_TIMER
  #define OS_PCLK_TIMER (OS_FSYS / OS_PCLK_DIVIDER)
#endif

/*********************************************************************
*
*       Configuration of communication to OSView
*/
#ifndef   OS_VIEW_ENABLE            // Global enable of communication
  #define OS_VIEW_ENABLE    (0)     // Default: on
#endif

/*********************************************************************
*
*       UART settings for OSView
*       If you do not want (or can not due to hardware limitations)
*       to dedicate a UART to OSView, define it to be -1
*       Currently UART1 and UART3 are supported and the standard
*       setup enables UART 1 per default
*/
#ifndef   OS_UART
  #define OS_UART (-1)
#endif

#ifndef   OS_BAUDRATE
  #define OS_BAUDRATE (38400)
#endif

#ifndef   OS_PCLK_UART
  #define OS_PCLK_UART (OS_FSYS / OS_PCLK_DIVIDER)
#endif

/******	End of configurable option **********************************/

// Only UART 1 and UART 3 were tested, other UARTs may require modification
#define OS_UART_USED (OS_VIEW_ENABLE && ((OS_UART == 1) || (OS_UART == 3)))

/*********************************************************************
*
*		Local defines
*
**********************************************************************
*/

/*********************************************************************
*
*		SFRs used in RTOSInit
*/

/****** Clock and peripheral clock control **************************/

#define CLK_BASE_ADDR      (0x50C0)
#define CLK_CKDIVR         (*(volatile OS_U8* ) (CLK_BASE_ADDR + 0x06))
#define CLK_PCKENR1        (*(volatile OS_U8* ) (CLK_BASE_ADDR + 0x07))
#define CLK_PCKENR2        (*(volatile OS_U8* ) (CLK_BASE_ADDR + 0x0A))

/****** Timer *******************************************************/

#define TIM3_BASE_ADDR     (0x5320)
#define OS_TIM_BASE_ADDR   TIM3_BASE_ADDR

/****** CAUTION !!! the address offsets depend on the selected timer */
#define OS_TIM_CR1         (*(volatile OS_U8* ) (OS_TIM_BASE_ADDR + 0x00))
#define OS_TIM_IER         (*(volatile OS_U8* ) (OS_TIM_BASE_ADDR + 0x01))
#define OS_TIM_SR1         (*(volatile OS_U8* ) (OS_TIM_BASE_ADDR + 0x02))

#define OS_TIM_CNTRH       (*(volatile OS_U8* ) (OS_TIM_BASE_ADDR + 0x08))
#define OS_TIM_CNTRL       (*(volatile OS_U8* ) (OS_TIM_BASE_ADDR + 0x09))
#define OS_TIM_PSCR        (*(volatile OS_U8* ) (OS_TIM_BASE_ADDR + 0x0A))
#define OS_TIM_ARRH        (*(volatile OS_U8* ) (OS_TIM_BASE_ADDR + 0x0B))
#define OS_TIM_ARRL        (*(volatile OS_U8* ) (OS_TIM_BASE_ADDR + 0x0C))

/****** Clock enable register for embOS timer ***********************/

#define OS_TIM_CLK_ENR     (CLK_PCKENR1)
#define OS_TIM_CLK_ENR_BIT (6)

/****** Interrupt priority register for embOS timer *****************/

#define ITC_SPR4           (*(volatile OS_U8* ) 0x07F73)
#define OS_TIM_SPRREG      (ITC_SPR4)
#define OS_TIM_SPR_BIT     (6)
#define OS_TIM_PRIO        (0uL)


#if OS_UART_USED
/****** UART1 and UART3 *********************************************/

#define UART1_BASE_ADDR    (0x005230)
#define UART3_BASE_ADDR    (0x005240)

#if   (OS_UART == 1)
  #define OS_UART_BASE_ADDR  (UART1_BASE_ADDR)
  #define OS_TX_ISR_VECTOR   19
  #define OS_RX_ISR_VECTOR   20
  #define OS_UART_CLKEN_BIT  2
#elif (OS_UART == 3)
  #define OS_UART_BASE_ADDR  (UART3_BASE_ADDR)
  #define OS_TX_ISR_VECTOR   22
  #define OS_RX_ISR_VECTOR   23
  #define OS_UART_CLKEN_BIT  3
#else
  #error "Selected UART not supported, Code has to be modified"
#endif

#define OS_UART_SR          (*(volatile OS_U8* ) (OS_UART_BASE_ADDR + 0x00))
#define OS_UART_DR          (*(volatile OS_U8* ) (OS_UART_BASE_ADDR + 0x01))
#define OS_UART_BRR1        (*(volatile OS_U8* ) (OS_UART_BASE_ADDR + 0x02))
#define OS_UART_BRR2        (*(volatile OS_U8* ) (OS_UART_BASE_ADDR + 0x03))
#define OS_UART_CR1         (*(volatile OS_U8* ) (OS_UART_BASE_ADDR + 0x04))
#define OS_UART_CR2         (*(volatile OS_U8* ) (OS_UART_BASE_ADDR + 0x05))
#define OS_UART_CR3         (*(volatile OS_U8* ) (OS_UART_BASE_ADDR + 0x06))
#endif  // OS_UART_USED

/*********************************************************************
*
*		    Local functions
*
**********************************************************************
*/


/*********************************************************************
*
*		    OS_IsrTickHandler()
*/
#pragma vector=17
static __interrupt void OS_IsrTickHandler(void) {
  //
  // Clear interrupt pending condition...
  //
  OS_TIM_SR1 &= ~(1uL << 0);
  OS_EnterInterrupt();
  OS_HandleTick();
  OS_LeaveInterrupt();
}

#if OS_UART_USED
/*********************************************************************
*
*		    OS_IsrTxHandler()
*/
#pragma vector=OS_TX_ISR_VECTOR
static __interrupt void OS_IsrTxHandler(void) {
  //
  // Clear interrupt pending condition...
  //
  OS_UART_CR2 &= ~(1 << 7);  // Disable Tx interrupt
  OS_EnterInterrupt();
  OS_OnTx();                 // Send next character, if any
  OS_LeaveInterrupt();
}

/*********************************************************************
*
*		    OS_IsrTxHandler()
*/
#pragma vector=OS_RX_ISR_VECTOR
static __interrupt void OS_IsrRxHandler(void) {
  OS_U8 Status;
  OS_U8 Data;

  OS_UART_CR2 &= ~(1 << 5);  // Disable Rx interrupt
  OS_EnterInterrupt();

  Status = OS_UART_SR;      // Read the Status register to clear error
  Data   = OS_UART_DR;      // Read the data register after reading status
  if ((Status & ((1 << 0) | (1 << 1) | (1 << 3))) == 0) {
    //
    // No Parity, framing or overrun error detected, process byte
    //
    OS_OnRx(Data);
  }
  OS_UART_CR2 |= (1 << 5);  // Enable Rx interrupt
  OS_LeaveInterrupt();
}
#endif // OS_UART_USED

/*********************************************************************
*
*		Global functions
*
**********************************************************************
*/

/*********************************************************************
*
*		    OS_InitHW()
*
*  Initialize the hardware required for the OS to run. This will work
*  on any target hardware, but may have to be tailored a little
*  (regarding the clock frequency). Of course the same holds true
*  if for some reason you choose to use an other timer.
*  Per default we use the Compare match timer channel 0, which is an up-counter
*/
#define OS_TIMER_RELOAD ((OS_PCLK_TIMER / 1000) - 1)
void OS_InitHW(void) {

  OS_IncDI();                  // Disable interrupts
  //
  // Setup the sytem clock, speed up clock
  //
  CLK_CKDIVR = (0 << 0) // b2..b0: CPU clock divider = 1, full speed
             | (0 << 3) // b5..b3: HSI clock divider = 1, full speed
             ;
  //
  // Enable the system clock for embOS timer
  //
  OS_TIM_CLK_ENR |=  (1uL << OS_TIM_CLK_ENR_BIT);   // Enable clock for timer
  //
  // Setup the timer
  //
  OS_TIM_CR1   = 0;     // Stop the counter
  OS_TIM_IER   = 0;     // Disable all timer interrupts
  OS_TIM_PSCR  = 0;     // Clear the prescaler, => divide by 1
  OS_TIM_CNTRH = 0;     // Clear the counter, upper byte first!
  OS_TIM_CNTRL = 0;
  OS_TIM_ARRH  = (OS_TIMER_RELOAD >> 8);  // Set the reload value, upper byte first!
  OS_TIM_ARRL  = (OS_TIMER_RELOAD & 0xFF);
  //
  // Start the timer and enable periodic interrupt
  //
  OS_TIM_CR1 |= (1uL << 0);    // Start the counter
  OS_TIM_IER =  (1uL << 0);    // Enable the timer update interrupt

  OS_COM_Init();               // Initialize UART for embOSView
  OS_DecRI();                  // Restore interrupt state
}

/*********************************************************************
*
*       Idle loop  (OS_Idle)
*
*  Please note:
*  This is basically the "core" of the idle loop.
*  This core loop can be changed, but:
*  The idle loop does not have a stack of its own, therefore no
*  functionality should be implemented that relies on the stack
*  to be preserved. However, a simple program loop can be programmed
*  (like toggeling an output or incrementing a counter)
*/
void OS_Idle(void) { // Idle loop: No task is ready to exec
  for (;;);          // Nothing to do ... wait for an interrupt
}

/*********************************************************************
*
*       Get time [cycles]
*
*  This routine is required for task-info via embOSView or high
*  resolution time measurement.
*  It returns the system time in clock cycles.
*/
OS_U32 OS_GetTime_Cycles(void) {
  OS_U32 Time;
  OS_U16 Cnt;
  OS_U8  CntL;

  Time = OS_GetTime32();
  //
  // Read the counter value, High byte first !
  //
  Cnt  = OS_TIM_CNTRH;
  CntL = OS_TIM_CNTRL;
  //
  // Check if timer interrupt pending ...
  //
  if ((OS_TIM_SR1 & (1uL << 0)) != 0) {  // Adjust result
    Cnt  = OS_TIM_CNTRH;
    CntL = OS_TIM_CNTRL;
    Time++;
  }
  Cnt = (Cnt << 8) | CntL;
  return (OS_TIMER_RELOAD * Time) + Cnt;
}

/*********************************************************************
*
*       OS_ConvertCycles2us
*
*  Convert Cycles into micro seconds. When using 20.0 MHz, this
*  means dividing by 20. If you have a different clock frequency,
*  you may have to modify this routine in order to get proper
*  diagonstics. Please note: Operation of theOS is not affected
*  by this routine. It is used for profiling and testing
*/
OS_U32 OS_ConvertCycles2us(OS_U32 Cycles) {
  return Cycles/(OS_PCLK_TIMER/1000000);
}

/*********************************************************************
*
*       Communication for embOSView   (UART 1 or 3)
*
**********************************************************************
*/

#if OS_UART_USED

#define OS_UART_BRR (OS_PCLK_UART / OS_BAUDRATE / 16)

/*********************************************************************
*
*       OS_COM_Init()
*/
void OS_COM_Init(void) {

  OS_IncDI();  // Disable interrupts, save previous state
  //
  // Enable clock for UART1 peripheral
  //
  CLK_PCKENR1 |= (1uL << OS_UART_CLKEN_BIT);
  //
  // Setup the GPIO for UART input / output pins if required
  //

  //
  // Setup the UART for asyncronous communication, data format 8N1
  //
  OS_UART_CR1  = (0 << 0)               // PIEN: Disable parity error interrupt
               | (0 << 1)               // PS:   Parity select, EVEN (don't care)
               | (0 << 2)               // PCEN: Parity control disabled
               | (0 << 3)               // WAKE: don't care
               | (0 << 4)               // M:    8bit data
               | (0 << 5)               // UARTD: UART enabled
               | (0 << 6)               // b6..b7: 9th data bit, not used
               ;
  OS_UART_CR2  =  0;                    // Disable Rx, Tx and interrupts
  OS_UART_CR3  = (0 << 4);              // Set 1 Stop bit
  OS_UART_BRR2 = (OS_UART_BRR >> 8);    // Program the baudrate generator
  OS_UART_BRR1 = (OS_UART_BRR & 0xFF);

  OS_UART_CR2  = (1 << 2)               // Enable the receiver
               | (1 << 3)               // Enable the Transmitter
               | (1 << 5)               // Enable the Rx interrupt
               ;

  OS_DecRI();                           // Restore interrupt state
}

/*********************************************************************
*
*       OS_COM_Send1()
*/
void OS_COM_Send1(OS_U8 c) {
  OS_UART_DR   = c;
  OS_UART_CR2 |= (1uL << 7);   // Enable the transmitter interrupt
}

#else  /* No communication routines */
  void OS_COM_Init(void) {}
  void OS_COM_Send1(OS_U8 c) {
    OS_USEPARA(c);           /* Avoid warning */
    OS_COM_ClearTxActive();  /* Let OS know that transmitter is not busy  */
  }
#endif /* (OS_UART_USED) */

/****** End of file *************************************************/
