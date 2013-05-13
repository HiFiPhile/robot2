//STM8S105
#include "route.h"
#include "RTOS.h"

#define diffright 99
#define diffleft 100
#define Maniabilite_IR 60
#define Maniabilite_US_Front 80
#define Maniabilite_US_Side 0.8
#define Distance_US_Side 17
#define Delay_US 15
#define Distance_US_Front 30
extern volatile u16 Distance_Center, Distance_Left, Distance_Right;
extern volatile s8 Direction;                                 //Read-only
extern void delay(u32 ms);                    //delay in millsecond
extern void Motor_Left_Set(s8 power);              //Set motor power,from -100 to 100
extern void Motor_Right_Set(s8 power);
extern void Detection_Enable( void );                    //Enable the zone detection
extern void LED_SET( u16 Data );                    //Set the status of LEDs
extern void LED_ON( u16 Data );
extern void LED_OFF( u16 Data );

void route(void)
{
    MOTOR_LEFT_ON;
    MOTOR_RIGHT_ON;
    LED_SET(0x0000);
    u16 dleft, dright;
    Detection_Enable();
    u16 dcenter;
    while(1)
    {

        dcenter = Distance_Center;
        if (dcenter >= Distance_US_Front)                     // champs libre devant
        {
            if (Direction >= 0)
            {
                Motor_Right_Set(diffright - 35*Direction);
                Motor_Left_Set(diffleft);


            }
            if (Direction < 0)
            {
                Motor_Left_Set(diffleft + 35*Direction);
                Motor_Right_Set(diffright);
            }
        }
        else
        {

            if (dcenter < Distance_US_Front)        // tant qu'il y a un obstacle devant...
            {
                dleft = Distance_Center;
                dright = Distance_Right;


                if ((dleft < 30) && (dright > dleft))  //...et ?gauche...
                {
                    Motor_Left_Set(diffleft);          //...on tourne ?droite.
                    Motor_Right_Set(0);
                    while(dcenter<30)
                    {
                        dcenter=Distance_Center;
                        OS_Delay(15);
                    }
                }

                if ((dright < 30) && (dright < dleft))  //...et ?droite...
                {
                    Motor_Left_Set(0);                  //...on tourne ?gauche.
                    Motor_Right_Set(diffright);
                    while(dcenter<30)
                    {
                        dcenter=Distance_Center;
                        OS_Delay(15);
                    }
                }


                if ((dleft > 30) && (dright > 30))      // ...et rien sur les côtés...
                {
                    if (Direction >= 0 )                //...et la balise ?droite...
                    {
                        Motor_Left_Set(diffleft);       // on tourne ?droite
                        Motor_Right_Set(-50);
                        while(dcenter<30)
                        {
                            dcenter=Distance_Center;
                            OS_Delay(15);
                        }
                    }

                    else                           //...et la balise ?gauche...
                    {
                        Motor_Left_Set(-50);       // on tourne ?gauche
                        Motor_Right_Set(diffright);
                        while(dcenter<30)
                        {
                            dcenter=Distance_Center;
                            OS_Delay(15);
                        }
                    }
                }
            }
        }
        OS_Delay(15);


        dright = Distance_Right;          //Danger ?droite
        if (dright < Distance_US_Side)
        {
            Motor_Right_Set(diffright);        //On tourne...
            Motor_Left_Set(-80);
            while(dright<Distance_US_Side)          //...jusqu'?ce que le danger soit écart?
            {
                dright = Distance_Right;
                OS_Delay(15);
            }
            Motor_Left_Set(diffleft);
        }




        dleft = Distance_Left;            //Danger ?gauche
        if (dleft < Distance_US_Side)
        {
            Motor_Left_Set(diffleft);          //On tourne...
            Motor_Right_Set(-80);
            while(dleft<Distance_US_Side)           //...jusqu'?ce que le danger soit écart?
            {
                dleft = Distance_Left;
                OS_Delay(15);
            }
            Motor_Right_Set(diffright);
        }




        // ****************************************************************************************************************************
        // ****************************************************************************************************************************
        /*
        //Test tout

        pright = diffright;
        pleft = diffleft;

        // *************************************************
        // Direction par rapport ?la balise
        if (Direction > 0)
        {
        pright = pright - 20*Direction;
    }
        if (Direction < 0)
        {
        pleft = pleft + 20*Direction;
    }

        // *************************************************
        // direction par rapport au US
        dright = Distance_Get(Right);
        if (dright < 7)
        pleft = 0;
        pright = diffright;

        dleft = Distance_Get(Left);
        if (dleft < 7)
        pright = 0;
        pleft = diffleft;

        dcenter = Distance_Get(Center);
        if (dcenter < 20)
        {
        if (dleft < dright)
        pleft = diffleft;
        pright = 0;
        if (dleft > dright)
        pright = diffright;
        pleft = 0;
    }

        if (dright < 10 && dright>6 && dleft < 10 && dleft > 6 && dcenter> 40)
        {
        pleft = diffleft;
        pright = diffright;
    }

        // *************************************************

        Motor_Left_Set(pleft);
        Motor_Right_Set(pright);
        */
        // ****************************************************************************************************************************
        // ****************************************************************************************************************************
    }
}

void final(void)
{
    disableInterrupts();
    Motor_Left_Set(-diffleft);
    Motor_Right_Set(-diffright);
    delay(50);
    MOTOR_LEFT_OFF;
    MOTOR_RIGHT_OFF;
    LED_ON(0xffff);
    BALLON_START;
    delay(1000);
    BALLON_STOP;
    while(1)
    {

    }
}

void BUMP_Left(void)
{
    LED_ON(0xff00);
    Motor_Left_Set(-diffleft);
    Motor_Right_Set(-diffright);
    OS_Delay(100);
    Motor_Left_Set(diffleft);
    Motor_Right_Set(-diffright);
    OS_Delay(100);
    Motor_Left_Set(diffleft);
    Motor_Right_Set(diffright);
    OS_Delay(100);
}

void BUMP_Center(void)
{

}

void BUMP_Right(void)
{
    LED_ON(0x00ff);
    Motor_Left_Set(-diffleft);
    Motor_Right_Set(-diffright);
    OS_Delay(100);
    Motor_Left_Set(-diffleft);
    Motor_Right_Set(diffright);
    OS_Delay(100);
    Motor_Left_Set(diffleft);
    Motor_Right_Set(diffright);
    OS_Delay(100);
}