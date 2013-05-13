/**
******************************************************************************
* @file    Project/main.c
* @author  LI Zixun
* @version V1.0
* @date    01/31/2013
* Built with IAR EWSTM8 1.30
******************************************************************************
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "route.h"
/* Private defines -----------------------------------------------------------*/
u16 LED_Status=0x00;
#define  I2C2SDA_SET_1       I2C2_PORT->ODR |=  I2C2SDA
#define  I2C2SDA_SET_0       I2C2_PORT->ODR &=~ I2C2SDA
#define  I2C2SCL_SET_1       I2C2_PORT->ODR |=  I2C2SCL
#define  I2C2SCL_SET_0       I2C2_PORT->ODR &=~ I2C2SCL
#define  I2C2SDA_INPUT_IN    I2C2_PORT->IDR  &  I2C2SDA
/* Private function prototypes -----------------------------------------------*/
void SYS_init();
void delay(u32 ms);
void Motor_Left_Set(s8 power);              //Set motor power,from -100 to 100
void Motor_Right_Set(s8 power);
void Direction_Get();                       //Get current direction from -7 to 8 center is 0
u16  Distance_Get(US_Channel_def channel);  //Get distance from ultrasonic detector in cm
void Detection_Enable();                    //Enable the zone detection
//i2c
void I2C2_Int(void);
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
/* Private functions ---------------------------------------------------------*/
void main(void)
{
    SYS_init();
    I2C2_Int();
    enableInterrupts();
    /* Infinite loop wait the start signal*/
    while(START_PORT->IDR & START_PIN);
    while(1)
    {
      route();
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
    //TIM2 Timebase init for 1us
    TIM2_DeInit();
    TIM2_TimeBaseInit(TIM2_PRESCALER_16, 65535);
    TIM2_PrescalerConfig(TIM2_PRESCALER_16,TIM2_PSCRELOADMODE_IMMEDIATE);
    TIM2_ARRPreloadConfig(ENABLE);
    TIM3_DeInit();
    //init IO for ultrasonic
    GPIO_Init(US_PORT,(GPIO_Pin_TypeDef)(US_LEFT | US_CENTER | US_RIGHT), GPIO_MODE_OUT_PP_LOW_FAST);
    //init IO for ultrasonic
    GPIO_Init(BALLON_PORT,BALLON_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
    //init IO for zone detection
    //GPIO_Init(DETZ_PORT,DETZ_PIN, GPIO_MODE_IN_PU_NO_IT);
    //init IO for jack
    GPIO_Init(START_PORT,START_PIN, GPIO_MODE_IN_PU_NO_IT);
    //init IO for bumper
    GPIO_Init(BUMP_PORT,(GPIO_Pin_TypeDef)(BUMP_LEFT | BUMP_CENTER | BUMP_RIGHT), GPIO_MODE_IN_PU_IT);
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD,EXTI_SENSITIVITY_FALL_LOW);
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

_Pragma("optimize=none")
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

u16  Distance_Get(US_Channel_def channel)
{
    u16 tmp=200;
    disableInterrupts();
    GPIO_Init(US_PORT,(GPIO_Pin_TypeDef)channel, GPIO_MODE_OUT_PP_HIGH_FAST);
    TIM2_SetCounter(0x00);
    TIM2_Cmd(ENABLE);
    while(TIM2_GetCounter() < 0x10);
    GPIO_WriteLow(US_PORT,(GPIO_Pin_TypeDef)channel);
    Direction_Get();
    while(tmp--);
    GPIO_Init(US_PORT,(GPIO_Pin_TypeDef)channel, GPIO_MODE_IN_FL_NO_IT);
    //TIM2_Cmd(DISABLE);
    TIM2_SetCounter(0x00);
    while((US_PORT->IDR & (GPIO_Pin_TypeDef)channel)==0)
    {
        if(TIM2->CNTRH > 0x04)return(500);
    }
    TIM2_SetCounter(0x00);
    while(US_PORT->IDR & (GPIO_Pin_TypeDef)channel)
    {
        if(TIM2->CNTRH > 0x76)return(500);
    }
    TIM2_Cmd(DISABLE);
    GPIO_Init(US_PORT, (GPIO_Pin_TypeDef)channel, GPIO_MODE_OUT_PP_LOW_FAST);
    enableInterrupts();
    tmp=TIM2_GetCounter()/56;
    return(tmp);
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

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
