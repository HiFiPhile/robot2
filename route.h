/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

#define  MOTOR_PORT                 GPIOB
#define  MOTOR_LEFT_EN_PIN          GPIO_PIN_1
#define  MOTOR_RIGHT_EN_PIN         GPIO_PIN_3
#define  MOTOR_LEFT_ON              MOTOR_PORT->ODR |= MOTOR_LEFT_EN_PIN
#define  MOTOR_RIGHT_ON             MOTOR_PORT->ODR |= MOTOR_RIGHT_EN_PIN
#define  MOTOR_LEFT_OFF             MOTOR_PORT->ODR &=~MOTOR_LEFT_EN_PIN
#define  MOTOR_RIGHT_OFF            MOTOR_PORT->ODR &=~MOTOR_RIGHT_EN_PIN
#define  US_PORT                    GPIOD
#define  US_LEFT                    GPIO_PIN_0
#define  US_CENTER                  GPIO_PIN_2
#define  US_RIGHT                   GPIO_PIN_3
#define  DETZ_PORT                  GPIOD
#define  DETZ_PIN                   GPIO_PIN_4
#define  START_PORT                 GPIOB
#define  START_PIN                  GPIO_PIN_7
#define  BALLON_PORT                GPIOB
#define  BALLON_PIN                 GPIO_PIN_6
#define  BALLON_START               BALLON_PORT->ODR |= BALLON_PIN
#define  BALLON_STOP                BALLON_PORT->ODR &=~BALLON_PIN
#define  BUMP_PORT                  GPIOD
#define  BUMP_LEFT                  GPIO_PIN_6
#define  BUMP_CENTER                GPIO_PIN_7
#define  BUMP_RIGHT                 GPIO_PIN_5
#define  BUMP_L                     ~(BUMP_PORT->IDR & BUMP_LEFT)
#define  BUMP_C                     ~(BUMP_PORT->IDR & BUMP_CENTER)
#define  BUMP_R                     ~(BUMP_PORT->IDR & BUMP_RIGHT)
//Software I2C
#define  I2C2SDA                    GPIO_PIN_5
#define  I2C2SCL                    GPIO_PIN_6
#define  I2C2INT                    GPIO_PIN_4
#define  I2C2_PORT                  GPIOA

typedef enum
{
    Left    = (u8)(US_LEFT),
    Center  = (u8)(US_CENTER),
    Right   = (u8)(US_RIGHT),
}US_Channel_def;

void route(void);
void final(void);
void BUMP_Left(void);
void BUMP_Center(void);
void BUMP_Right(void);

