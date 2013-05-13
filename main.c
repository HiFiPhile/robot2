/**
******************************************************************************
* @file    Project/main.c
* @author  MCD Application Team
* @version V2.1.0
* @date    18-November-2011
* @brief   Main program body
******************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "RTOS.h"
#include "route.h"
/* Private defines -----------------------------------------------------------*/
u16 LED_Status=0x00;
extern u8 bUS_L, bUS_C, bUS_R;
#define  I2C2SDA_SET_1       I2C2_PORT->ODR |=  I2C2SDA
#define  I2C2SDA_SET_0       I2C2_PORT->ODR &=~ I2C2SDA
#define  I2C2SCL_SET_1       I2C2_PORT->ODR |=  I2C2SCL
#define  I2C2SCL_SET_0       I2C2_PORT->ODR &=~ I2C2SCL
#define  I2C2SDA_INPUT_IN    I2C2_PORT->IDR  &  I2C2SDA
/* Private function prototypes -----------------------------------------------*/
void SYS_init( void );
void delay(u32 ms);
void Motor_Left_Set(s8 power);              //Set motor power,from -100 to 100
void Motor_Right_Set(s8 power);
void Direction_Get( void );                       //Get current direction from -7 to 8 center is 0
void Detection_Enable( void );                    //Enable the zone detection
//i2c
void I2C2_Int( void );
void I2C2_Start( void );
void I2C2_Stop( void );
void I2C2_Ack( void );
void I2C2_SAck( void );
void I2C2_Nack( void );
void I2C2_Byte_Tx( u8 ByteData );
u8   I2C2_Byte_Rx( void );
void I2C2_Tx( u8 SlvAddr,u8 Address,u8 Data);
u8 I2C2_Rx( u8 SlvAddr,u8 Address );
void I2C2_DIR_Set( unsigned char SDADIR );
void LED_SET( u16 Data );
void LED_ON( u16 Data );
void LED_OFF( u16 Data );
void delays(void);
volatile s8 Direction;
volatile u16 Distance_Center, Distance_Left, Distance_Right;
/* Private functions ---------------------------------------------------------*/

OS_STACKPTR int StackSyS[128], StackBmp[128], StackRot[128];          /* Task stacks */
OS_TASK pUS, pROUTE, pBUMP;                        /* Task-control-blocks */
OS_EVENT BP_EVT, US_L_EVT , US_C_EVT, US_R_EVT;

static void tUPDATEINFO( void );
static void tROUTE( void );
static void tBUMP( void );
/*********************************************************************
*
*       main
*
*********************************************************************/

int main(void)
{
    OS_IncDI();                      /* Initially disable interrupts  */
    SYS_init();
    OS_InitKern();                   /* Initialize OS                 */
    OS_InitHW();                     /* Initialize Hardware for OS    */
    OS_DecRI();
    /* You need to create at least one task here !                    */
    OS_EVENT_Create(&BP_EVT);
    OS_EVENT_Create(&US_L_EVT);
    OS_EVENT_Create(&US_C_EVT);
    OS_EVENT_Create(&US_R_EVT);
    OS_CREATETASK(&pUS, "Sysinfo", tUPDATEINFO, 50, StackSyS);
    OS_CREATETASK(&pROUTE, "Route", tROUTE,  100, StackRot);
    OS_CREATETASK(&pBUMP, "Bumper", tBUMP,  150, StackBmp);
    OS_Start();                      /* Start multitasking            */

    return 0;
}

static void tUPDATEINFO()
{
    u8 tmp;
    OS_TIMING ustime;
    while(1)
    {
        //Center
        GPIO_Init(US_PORT,(GPIO_Pin_TypeDef)Center, GPIO_MODE_OUT_PP_HIGH_FAST);
        OS_Delayus(20);
        GPIO_WriteLow(US_PORT,(GPIO_Pin_TypeDef)Center);
        GPIO_Init(US_PORT,(GPIO_Pin_TypeDef)Center, GPIO_MODE_IN_FL_NO_IT);
        Direction_Get();
        OS_Delayus(200);
        //us broken ?
        if(US_PORT->IDR & (GPIO_Pin_TypeDef)Center)
        {
            Distance_Center=500;
            goto endc;
        }
        bUS_C=0;
        GPIO_Init(US_PORT,(GPIO_Pin_TypeDef)Center, GPIO_MODE_IN_FL_IT);
        //wait the return signal
        if(OS_EVENT_WaitTimed(&US_C_EVT,1)==1)
        {
            Distance_Center=500;
            goto endc;
        }
        //count the length of return signal
        OS_Timing_Start(&ustime);
        tmp=OS_EVENT_WaitTimed(&US_C_EVT,30);
        OS_Timing_End(&ustime);
        if(tmp==1)
        {
            Distance_Center=500;
        }
        else
        {
            Distance_Center=(u16)(OS_Timing_Getus(&ustime))/56;
        }
    endc:
        GPIO_Init(US_PORT, (GPIO_Pin_TypeDef)Center, GPIO_MODE_OUT_PP_LOW_FAST);
        //Left
        GPIO_Init(US_PORT,(GPIO_Pin_TypeDef)Left, GPIO_MODE_OUT_PP_HIGH_FAST);
        OS_Delayus(20);
        GPIO_WriteLow(US_PORT,(GPIO_Pin_TypeDef)Left);
        GPIO_Init(US_PORT,(GPIO_Pin_TypeDef)Left, GPIO_MODE_IN_FL_NO_IT);
        Direction_Get();
        OS_Delayus(200);
        //us broken ?
        if(US_PORT->IDR & (GPIO_Pin_TypeDef)Left)
        {
            Distance_Left=500;
            goto endl;
        }
        bUS_L=0;
        GPIO_Init(US_PORT,(GPIO_Pin_TypeDef)Left, GPIO_MODE_IN_FL_IT);
        //wait the return signal
        if(OS_EVENT_WaitTimed(&US_L_EVT,1)==1)
        {
            Distance_Left=500;
            goto endl;
        }
        //count the length of return signal
        OS_Timing_Start(&ustime);
        tmp=OS_EVENT_WaitTimed(&US_L_EVT,30);
        OS_Timing_End(&ustime);
        if(tmp==1)
        {
            Distance_Left=500;
        }
        else
        {
            Distance_Left=(u16)(OS_Timing_Getus(&ustime))/56;
        }
    endl:
        GPIO_Init(US_PORT, (GPIO_Pin_TypeDef)Left, GPIO_MODE_OUT_PP_LOW_FAST);
        //Right
        GPIO_Init(US_PORT,(GPIO_Pin_TypeDef)Right, GPIO_MODE_OUT_PP_HIGH_FAST);
        OS_Delayus(20);
        GPIO_WriteLow(US_PORT,(GPIO_Pin_TypeDef)Right);
        GPIO_Init(US_PORT,(GPIO_Pin_TypeDef)Right, GPIO_MODE_IN_FL_NO_IT);
        Direction_Get();
        OS_Delayus(200);
        //us broken ?
        if(US_PORT->IDR & (GPIO_Pin_TypeDef)Right)
        {
            Distance_Right=500;
            goto endr;
        }
        bUS_R=0;
        GPIO_Init(US_PORT,(GPIO_Pin_TypeDef)Right, GPIO_MODE_IN_FL_IT);
        //wait the return signal
        if(OS_EVENT_WaitTimed(&US_R_EVT,1)==1)
        {
            Distance_Right=500;
            goto endr;
        }
        //count the length of return signal
        OS_Timing_Start(&ustime);
        tmp=OS_EVENT_WaitTimed(&US_R_EVT,30);
        OS_Timing_End(&ustime);
        if(tmp==1)
        {
            Distance_Right=500;
        }
        else
        {
            Distance_Right=(u16)(OS_Timing_Getus(&ustime))/56;
        }
    endr:
        GPIO_Init(US_PORT, (GPIO_Pin_TypeDef)Right, GPIO_MODE_OUT_PP_LOW_FAST);
        OS_Delay(10);
    }
}

static void tROUTE()
{
    while(START_PORT->IDR & START_PIN)OS_Delay(50);
    route();
}

static void tBUMP()
{
    while(1)
    {
        OS_EVENT_Wait(&BP_EVT); //等待BP_evt事件 期间该进程暂停 cpu执行其他线程
        OS_Suspend(&pROUTE);    //暂停route线程（程序内容有点冲突）
        while((GPIOD->IDR & GPIO_PIN_6)==0)
        {
            BUMP_Left();
        }
        while((GPIOD->IDR & GPIO_PIN_7)==0)
        {
            BUMP_Center();
        }
        while((GPIOD->IDR & GPIO_PIN_5)==0)
        {
            BUMP_Right();
        }
        OS_Resume(&pROUTE);     //恢复route线程
    }
}

_Pragma("optimize=none")
void delay(__IO u32 ms)
{
    u16 tmp;
    while(ms--)
    {
        tmp=2300;
        while(tmp--);
    }
}

void SYS_init()
{
    //Internal Clock 16MHz
    CLK_DeInit();
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
    GPIO_DeInit(GPIOD);
    //init IO for EN of the motors
    GPIO_Init(MOTOR_PORT,(GPIO_Pin_TypeDef)(MOTOR_LEFT_EN_PIN | MOTOR_RIGHT_EN_PIN), GPIO_MODE_OUT_PP_LOW_FAST);
    //
    TIM1_DeInit();
    /* Time Base configuration */
    TIM1_TimeBaseInit(0, TIM1_COUNTERMODE_UP, 799, 0);      //20kHz
    /* Channel 1, 2,3 and 4 Configuration in PWM mode */
    TIM1_OC1Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE,
                 400, TIM1_OCPOLARITY_HIGH, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                 TIM1_OCNIDLESTATE_RESET);
    TIM1_OC2Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE,
                 400, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                 TIM1_OCNIDLESTATE_RESET);
    TIM1_OC3Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE,
                 400, TIM1_OCPOLARITY_HIGH, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                 TIM1_OCNIDLESTATE_RESET);
    TIM1_OC4Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_ENABLE, 400, TIM1_OCPOLARITY_LOW,
                 TIM1_OCIDLESTATE_SET);
    /* TIM1 counter enable */
    TIM1_Cmd(ENABLE);
    /* TIM1 Main Output Enable */
    TIM1_CtrlPWMOutputs(ENABLE);
    /* SPI configuration */
    SPI_DeInit();
    /* Initialize SPI in SLAVE mode  with RX int*/
    GPIO_Init(GPIOC,(GPIO_Pin_TypeDef)(GPIO_PIN_6 | GPIO_PIN_7), GPIO_MODE_IN_PU_NO_IT);
    SPI_Init(SPI_FIRSTBIT_LSB, SPI_BAUDRATEPRESCALER_2, SPI_MODE_SLAVE, SPI_CLOCKPOLARITY_HIGH,
             SPI_CLOCKPHASE_2EDGE, SPI_DATADIRECTION_2LINES_RXONLY, SPI_NSS_HARD,(uint8_t)0x07);
    //SPI_ITConfig(SPI_IT_RXNE, ENABLE);
    SPI_Cmd(ENABLE);
    //init IO for ultrasonic
    GPIO_Init(US_PORT,(GPIO_Pin_TypeDef)(US_LEFT | US_CENTER | US_RIGHT), GPIO_MODE_OUT_PP_LOW_FAST);
    //init IO for ballon
    GPIO_Init(BALLON_PORT,BALLON_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
    //init IO for jack
    GPIO_Init(START_PORT,START_PIN, GPIO_MODE_IN_PU_NO_IT);
    //init IO for bumper
    GPIO_Init(BUMP_PORT,(GPIO_Pin_TypeDef)(BUMP_LEFT | BUMP_CENTER | BUMP_RIGHT), GPIO_MODE_IN_PU_IT);
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD,EXTI_SENSITIVITY_RISE_FALL);
}

void Motor_Left_Set(s8 power)
{
    u16 tmp;
    if(power>100)power=100;
    if(power<-100)power=-100;
    tmp=(u16)((power+100)*4);
    TIM1_OC1Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE,
                 tmp, TIM1_OCPOLARITY_HIGH, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                 TIM1_OCNIDLESTATE_RESET);
    TIM1_OC2Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE,
                 tmp, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                 TIM1_OCNIDLESTATE_RESET);
}

void Motor_Right_Set(s8 power)
{
    u16 tmp;
    if(power>100)power=100;
    if(power<-100)power=-100;
    tmp=(u16)((power+100)*4);
    TIM1_OC3Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE,
                 tmp, TIM1_OCPOLARITY_HIGH, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
                 TIM1_OCNIDLESTATE_RESET);
    TIM1_OC4Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_ENABLE, tmp, TIM1_OCPOLARITY_LOW,
                 TIM1_OCIDLESTATE_SET);
}

void Direction_Get()
{
    volatile  u8 tmp=0;
    //if code received
    if (SPI->SR & SPI_SR_RXNE)
    {
        tmp=SPI->DR;
        switch(tmp)
        {
        case 0x01:
        case 0x82:
        case 0x83:
            Direction=0;
            LED_SET(0x0180);
            break;
        case 0x03:
        case 0x17:
            Direction=1;
            LED_SET(0x0080);
            break;
        case 0x02:
        case 0x05:
        case 0x07:
            Direction=2;
            LED_SET(0x0040);
            break;
        case 0x06:
        case 0x0f:
            Direction=3;
            LED_SET(0x0020);
            break;
        case 0x04:
        case 0x0a:
        case 0x0e:
            Direction=4;
            LED_SET(0x0010);
            break;
        case 0x0c:
        case 0x1e:
            Direction=5;
            LED_SET(0x0008);
            break;
        case 0x08:
        case 0x14:
        case 0x1c:
            Direction=6;
            LED_SET(0x0004);
            break;
        case 0x18:
        case 0x3c:
            Direction=7;
            LED_SET(0x0002);
            break;
        case 0x10:
        case 0x28:
        case 0x38:
            Direction=8;
            LED_SET(0x0001);
            break;
        case 0x30:
        case 0x78:
            Direction=-7;
            LED_SET(0x8000);
            break;
        case 0x20:
        case 0x50:
        case 0x70:
            Direction=-6;
            LED_SET(0x4000);
            break;
        case 0x60:
        case 0xf0:
            Direction=-5;
            LED_SET(0x2000);
            break;
        case 0x40:
        case 0xa0:
        case 0xe0:
            Direction=-4;
            LED_SET(0x1000);
            break;
        case 0xc0:
        case 0xe1:
            Direction=-3;
            LED_SET(0x0800);
            break;
        case 0x80:
        case 0x41:
        case 0xc1:
            Direction=-2;
            LED_SET(0x0400);
            break;
        case 0x81:
        case 0xc3:
            Direction=-1;
            LED_SET(0x0200);
            break;
        default:
            Direction=0;
            break;
        }
    }
}

void Detection_Enable()
{
    //init IO for zone detection with rising edge int
    GPIO_Init(DETZ_PORT,DETZ_PIN, GPIO_MODE_IN_PU_IT);
}

void I2C2_Int(void)
{
    /* Configure SPI pins*/
    GPIO_Init(I2C2_PORT,(GPIO_Pin_TypeDef)(I2C2SDA | I2C2SCL), GPIO_MODE_OUT_PP_HIGH_FAST );
    //GPIO_WriteLow(I2C2_PORT,I2C2SCL);
}

void I2C2_Start( void )
{
    I2C2SCL_SET_1;
    delays();
    I2C2SDA_SET_1;
    delays();
    I2C2SDA_SET_0;
    delays();
}

void I2C2_Stop( void )
{
    I2C2SDA_SET_0;
    delays();
    I2C2SCL_SET_1;
    delays();
    I2C2SDA_SET_1;
    delays();
}

void I2C2_Ack( void )
{
    I2C2SCL_SET_0;
    delays();
    I2C2SCL_SET_1;
    delays();
    I2C2SCL_SET_0;
    delays();
}

void I2C2_SAck( void )
{
    I2C2SCL_SET_0;
    delays();
    I2C2SDA_SET_0;
    delays();
    I2C2SCL_SET_1;
    delays();
    I2C2SCL_SET_0;
    delays();
    I2C2SDA_SET_1;
    delays();
}

void I2C2_Nack( void )
{
    I2C2SDA_SET_1;
    delays();
    I2C2SCL_SET_0;
    delays();
    I2C2SCL_SET_1;
    delays();
    I2C2SCL_SET_0;
    delays();
}

void I2C2_DIR_Set( u8 SDADIR )
{
    if(SDADIR == 0)
    {
        GPIO_Init(I2C2_PORT,I2C2SDA , GPIO_MODE_OUT_PP_HIGH_FAST );
    }
    else
    {
        GPIO_Init(I2C2_PORT,I2C2SDA , GPIO_MODE_IN_PU_NO_IT );
    }
}

void I2C2_Byte_Tx( u8 ByteData )
{
    unsigned char count;
    for(count=0; count<8; count++)
    {
        I2C2SCL_SET_0;
        delays();
        if(ByteData&0x80)
        {
            I2C2SDA_SET_1;
        }
        else
        {
            I2C2SDA_SET_0;
        }
        delays();
        I2C2SCL_SET_1;
        delays();
        ByteData<<=1;
    }
}

u8 I2C2_Byte_Rx( void )
{
    unsigned char count,readbyte=0;
    I2C2SCL_SET_0;
    for(count=0; count<8; count++)
    {
        readbyte=readbyte<<1;
        I2C2SCL_SET_1;
        delays();
        if(I2C2SDA_INPUT_IN)
        {
            readbyte=readbyte|0x01;
        }
        else
        {
            readbyte=readbyte&0xfe;
        }
        delays();
        I2C2SCL_SET_0;
        delays();
    }
    I2C2SCL_SET_0;
    return readbyte;
}

void I2C2_Tx( u8 SlvAddr,u8 Address,u8 Data)
{
    I2C2_Start();
    I2C2_Byte_Tx(SlvAddr << 1);
    I2C2_Ack();
    I2C2_Byte_Tx(Address);
    I2C2_Ack();
    I2C2_Byte_Tx(Data);
    I2C2_Ack();
    I2C2_Stop();
}

u8 I2C2_Rx( u8 SlvAddr,u8 Address )
{
    unsigned char readdata;
    I2C2_Start();
    I2C2_Byte_Tx(SlvAddr << 1);
    I2C2_Ack();
    I2C2_Byte_Tx(Address);
    I2C2_Ack();
    I2C2_Start();
    I2C2_Byte_Tx((SlvAddr << 1) + 1);
    I2C2_Ack();
    I2C2_Byte_Tx(Address);
    I2C2_Ack();
    readdata=I2C2_Byte_Rx();
    I2C2_Nack();
    I2C2_Stop();
    return (readdata);
}

void LED_SET( u16 Data )
{
    I2C2_DIR_Set(0);
    I2C2_Start();
    I2C2_Byte_Tx(0x42);
    I2C2_Ack();
    I2C2_Byte_Tx((u8)~Data);
    I2C2_Ack();
    I2C2_Byte_Tx((u8)(~Data >> 8));
    I2C2_Ack();
    I2C2_Stop();
}

void LED_ON( u16 Data )
{
    LED_Status |= Data;
    LED_SET(LED_Status);
}

void LED_OFF( u16 Data )
{
    LED_Status &=~Data;
    LED_SET(LED_Status);
}

_Pragma("optimize=none")
void delays(void)
{
    //u8 count = 1;
    //while(count--);
}

#ifdef USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*   where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval : None
*/
void assert_failed(u8* file, u32 line)
{
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif