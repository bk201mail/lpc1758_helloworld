/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 */
#include <stdio.h>      // printf() / scanf()
#include "io.hpp"       // Board sensors
#include "utilities.h"  // delay_ms()
//#include "uart2.hpp"    //uart2
//#include "string.h"     //for string (strlen)
//#include "spi.hpp"     // for spi1
#include "stdlib.h"
//#include "I2C2.hpp"       //for I2C2 Master
//#include "uart0_min.h"
#include <FreeRTOS.h>
#include "semphr.h"
#include "task.h"
#include "eint.h"
#include "wireless.h"
#include "soft_timer.hpp"
#include "gpio.hpp"
#include "lpc_pwm.hpp"
#include "lpc_sys.h"
//#include "ultra_sonic.hpp"


/*Wireless define------------------------------*/
enum{
    car_addr=300,
    commander_addr=400,
};
/*Motor defines---------------------------------*/
enum{
    move_forward   =1,
    move_backward  =2,
    move_right     =3,
    move_left      =4,
    stop           =5,
    full_speed     =6,
    half_speed     =7,
    auto_pilot_on  =8,
    auto_pilot_off =9,

};
/*--------------------------------------------*/

/*--------------------------------------------*/
GPIO Int1_Front(P1_29);
GPIO Int2_Front(P1_28);
GPIO Int1_Back(P1_23);
GPIO Int2_Back(P1_22);
PWM front_Motor(PWM::pwm1, 1000);;
PWM back_Motor(PWM::pwm1, 1000);;
int watchdog=0;
int right_sensor_distance=0;
int left_sensor_distance=0;
void motor_init(void){
        //FRONT MOTOR----------------------------------------------
        //front_Motor(PWM::pwm1, 1000); //Using 1KHz frequency...
        //Int1_Front(P1_29);   //Interrupt-1 (Front_Motor)
        Int1_Front.setAsOutput(); //Set as output
        //Int2_Front(P1_28);   //Interrupt-2 (Front_Motor)
        Int2_Front.setAsOutput(); //Set as output
        //-----------------------------------------------------------

        //BACK MOTOR-------------------------------------------------
        //back_Motor (PWM::pwm2, 1000); //Using 1KHz frequency...
        //Int1_Back(P1_23);   //Interrupt-1 (Back_Motor)
        Int1_Back.setAsOutput(); //Set as output
        //Int2_Back(P1_22);   //Interrupt-2 (Back_Motor)
        Int2_Back.setAsOutput(); //Set as output
        //------------------------------------------------------------
}

void motor_speed(int front_motor, int back_motor){
    front_Motor.set(front_motor);     //running at 100%
    back_Motor.set(back_motor);     //running at 100%

}

/*Ultra Sonic sensor-------------------------------------------*/

unsigned int timerValueAtIsr;
unsigned int timerValueAtTrigger;
//unsigned int timer_micro_sec;
float distance;
//float delta;
void echo_fall_edge(void){
    //while(1){
    float timerDelta=0;
    timerValueAtIsr = portGET_RUN_TIME_COUNTER_VALUE();   //Time (in ns) when falling edge of Echo pin is detected.
    //printf("timerValueAtTrigger: %i\n", timerValueAtTrigger);
    //printf("timerValueAtIsr: %i\n",timerValueAtIsr);

    timerDelta = (timerValueAtIsr - timerValueAtTrigger);  //Difference between the time
    timerDelta = (timerDelta * 0.666);                    //Converting time from nanosecond to microsecond.

    //printf("timerDelta_us: %f\n", timerDelta);
    distance = timerDelta /29;                   //Distance in CM (value is from DataSheet)

    printf("timerDelta_CM: %f\n", distance);
    //}
    //return timerValueAtIsr;
    //delta = sys_get_high_res_timer_us() - timer_micro_sec;
    //printf("timer_micro_sec: %i\n", timer_micro_sec);
    //printf("Delta: %i\n", delta);
    delay_ms(1000);

}

void auto_mode_init(void){
    GPIO echo_pin(P2_7);     //Define P2_7 as Echo pin.
    echo_pin.setAsInput();  //Echo pin as input
    LPC_GPIO1->FIODIR |= (1 << 20);   //Trigger pin as output (P1_20)
    eint3_enable_port2(7, eint_falling_edge, echo_fall_edge); //interrupt on fall edge at P2_7
}

void auto_mode(void){
         //delay_ms(2000);

        //while(1){
            timerValueAtTrigger = portGET_RUN_TIME_COUNTER_VALUE(); //Time (in ns) when sensor is trigger.
            //timer_micro_sec = sys_get_high_res_timer_us();
            LPC_GPIO1->FIOCLR = (1 << 20);     // clear trigger
            delay_us(2);                      // delay for 10 us
            LPC_GPIO1->FIOSET = (1 << 20);   // set trigger
            delay_us(20);                   // delay for 10 us
            LPC_GPIO1->FIOCLR = (1 << 20); // clear trigger
            delay_ms(1000);
        //}
}

void auto_mode_stop_go(void){
    //Stop the car and back a little bit
    printf("Stop\n");
    Int1_Front.setLow();
    Int2_Front.setLow();
    Int1_Back.setLow();
    Int2_Back.setLow();
    delay_ms(2000);


    printf("BackWard\n");
    Int1_Front.setLow();
    Int2_Front.setLow();
    Int1_Back.setLow();
    Int2_Back.setHigh();
    delay_ms(3000);

    //Check right and left sensor
    if(right_sensor_distance>=6.001){        //move right and then forward
        printf("Forward_Right\n");
        Int1_Front.setLow();
        Int2_Front.setHigh();
        Int1_Back.setHigh();
        Int2_Back.setLow();
        delay_ms(3000);

        printf("Forward\n");
        Int1_Front.setLow();
        Int2_Front.setLow();
        Int1_Back.setHigh();
        Int2_Back.setLow();
        return;
    }
    else if(left_sensor_distance >=6.001) { //move left and then forward

        printf("Forward_Left\n");
        Int1_Front.setHigh();
        Int2_Front.setLow();
        Int1_Back.setHigh();
        Int2_Back.setLow();
        delay_ms(3000);

        printf("Forward\n");
        Int1_Front.setLow();
        Int2_Front.setLow();
        Int1_Back.setHigh();
        Int2_Back.setLow();
        return;

    }
    else //go back a little and try again
        if(watchdog==3){         //At-least Try 3 attempts before giving up...
            //Stop the car and wait for user to take control.
            LE.on(1);                                          //Turn on LED 1 to indicated user's help.
            printf("Deadlock can't get out, Need user help\n");
            return;
        }
        else{
            printf("%i Try\n",watchdog);
            watchdog=watchdog+1;
            delay_ms(3000);
            auto_mode_stop_go();
        }

}

void auto_mode_go(void){
    //keep going forward
    printf("Forward\n");
    Int1_Front.setLow();
    Int2_Front.setLow();
    Int1_Back.setHigh();
    Int2_Back.setLow();
    //delay_ms(1000);
}





int main(void)
{
    //------Sending Logic------------------------------------------------------
    if(0){
            AS.init();                //Accelerometer initialization

            puts("Wireless  Test");
            char cmd=0;
            int car_control=0;
            int stop_car= 0;
            int full_speed_car=0;
            int half_speed_car=0;
            int dir=0;
            const char max_hops=1;
            mesh_set_node_address(commander_addr);
            while(1){
                int acc_val=AS.p_l_status();     //read Accelerometer landscape register
                int ornt   =(acc_val & 0x01);      //Read the orientation bit[0]

                if(ornt==1){                    //Disable the direction when board is flipped (means it is in auto pilot mode)
                    dir=4;
                }
                else{
                    dir=(acc_val & 0x07) >> 1;  //Read the direction bits[3:1]
                }

                stop_car= SW.getSwitch(1);
                full_speed_car=SW.getSwitch(2);
                half_speed_car=SW.getSwitch(3);
                if(stop_car or full_speed_car or half_speed_car){
                    car_control=1;
                }
                else{
                    car_control=0;
                }


                //Motor Control---------------------------------------------------------
                if(dir==0 && !car_control){
                    puts("Move Forward");
                    cmd=move_forward;
                    wireless_send(car_addr,mesh_pkt_nack, &cmd,1,max_hops);
                    delay_ms(1000);
                }

                else if(dir==1 && !car_control){
                    puts("Move Backward");
                    cmd=move_backward;
                    wireless_send(car_addr,mesh_pkt_nack, &cmd,1,max_hops);
                    delay_ms(1000);
                }

                else if(dir==3 && !car_control){
                    puts("Move Right");
                    cmd=move_right;
                    wireless_send(car_addr,mesh_pkt_nack, &cmd,1,max_hops);
                    delay_ms(1000);
                }

                else if(dir==2 && !car_control){
                    puts("Move Left");
                    cmd=move_left;
                    wireless_send(car_addr,mesh_pkt_nack, &cmd,1,max_hops);
                    delay_ms(1000);
                }

                else if(ornt==1 && !car_control){
                    puts("Auto Pilot Mode On");
                    cmd=auto_pilot_on;
                    wireless_send(car_addr,mesh_pkt_nack, &cmd,1,max_hops);
                    delay_ms(1000);
                }

                else if(ornt==0 && !car_control){
                    puts("Auto Pilot Mode Off");
                    cmd=auto_pilot_off;
                    wireless_send(car_addr,mesh_pkt_nack, &cmd,1,max_hops);
                    delay_ms(1000);
                }


                else if(SW.getSwitch(1)){
                    puts("Stop");
                    cmd=stop;
                    wireless_send(car_addr,mesh_pkt_nack, &cmd,1,max_hops);
                    delay_ms(1000);
                }

                else if(SW.getSwitch(2)){
                    puts("Full Speed");
                    cmd=full_speed;
                    wireless_send(car_addr,mesh_pkt_nack, &cmd,1,max_hops);
                    delay_ms(1000);
                }

                else if(SW.getSwitch(3)){
                    puts("Half Speed");
                    cmd=half_speed;
                    wireless_send(car_addr,mesh_pkt_nack, &cmd,1,max_hops);
                    delay_ms(1000);
                }

                else{
                    puts("Default stop");
                    cmd=stop;
                    wireless_send(car_addr,mesh_pkt_nack, &cmd,1,max_hops);
                    delay_ms(1000);

                }

            }
    }

   //Revecing logic----------------------------------------------------------------
    if(0){

        motor_init();            //Motor initialization
        motor_speed(100,100);   //Set motor , by default set to 100%
        auto_mode_init();      //Ultra Sonic sensor initialization
        mesh_set_node_address(car_addr);
        while(1){
            mesh_packet_t pkt;
            //printf("Pkt: %i\n",pkt);
            if(wireless_get_rx_pkt(&pkt,1000)){
                char command=pkt.data[0];
                switch (command){
                    case move_forward:
                        printf("Forward\n");
                        Int1_Front.setLow();
                        Int2_Front.setLow();
                        Int1_Back.setHigh();
                        Int2_Back.setLow();
                        delay_ms(1000);
                        break;
                    case move_backward:
                        printf("BackWard\n");
                        Int1_Front.setLow();
                        Int2_Front.setLow();
                        Int1_Back.setLow();
                        Int2_Back.setHigh();
                        delay_ms(1000);

                        break;
                    case move_right:
                        printf("Forward_Right\n");
                        Int1_Front.setLow();
                        Int2_Front.setHigh();
                        Int1_Back.setHigh();
                        Int2_Back.setLow();
                        delay_ms(1000);

                        break;
                    case move_left:
                        printf("Forward_Left\n");
                        Int1_Front.setHigh();
                        Int2_Front.setLow();
                        Int1_Back.setHigh();
                        Int2_Back.setLow();
                        delay_ms(1000);

                        break;
                    case stop:
                        //Stop the car---------------------
                        printf("Stop\n");
                        Int1_Front.setLow();
                        Int2_Front.setLow();
                        Int1_Back.setLow();
                        Int2_Back.setLow();
                        delay_ms(1000);
                        break;

                    case auto_pilot_on:
                        printf("Auto Pilot Mode On\n");
                        auto_mode();
                        delay_ms(100);
                        if(distance<=2.001){
                            watchdog=0;
                            if(LE.getValues()){
                                printf("Deadlock found!!.Please Switch to user mode\n");

                                while(command==!auto_pilot_on){
                                    LE.off(1);
                                    break;
                                }

                                break;
                            }
                            printf("Stopping the car\n");
                            auto_mode_stop_go();
                }
                        else{
                            auto_mode_go();
                        }
                        delay_ms(1000);
                        break;

                    case full_speed:
                        printf("Setting Half speed\n");
                        motor_speed(100,100); //running at 100%
                        delay_ms(1000);
                        break;
                    case half_speed:
                        printf("Setting Full speed\n");
                        motor_speed(50,50);  //running at half speed 50%
                        delay_ms(1000);
                        break;
                    default:
                        printf("Error: Invalid Command!\n");
                        break;
                        //------------------------------------
                }
            }

        }

    }





    //===Ultra_sonic Sensor====================================================

    if(1){
         delay_ms(2000);
         /*
        //LPC_PINCON->PINSEL3 |=  (0x03 << 6);  //enable CAP1.1
        //LPC_SC->PCONP |= 1 << 2; //Power up TimerCounter1
        LPC_TIM1->TCR |= 1<<1;  //enable timer reset
        LPC_TIM1->PR = 0x0;    //count every pclk tick
        //LPC_TIM1->CCR |= (1<<4);  //falling edge interrupt
        LPC_TIM1->TCR |=1<<0; //enable timer

        LPC_GPIO1->FIODIR &= ~(1 << 19);  //Echo as input
        LPC_GPIO1->FIODIR |= (1 << 20);   //Trigger as output
        */

        //portRESET_TIMER_FOR_RUN_TIME_STATS();
        //delay_us(2);
        GPIO pp(P2_7);    //Echo as input
        pp.setAsInput();
        LPC_GPIO1->FIODIR |= (1 << 20);   //Trigger as output
        eint3_enable_port2(7, eint_falling_edge, echo_fall_edge);
        //eint3_enable_port2(7, eint_rising_edge, foo2);
        printf("US test\n");
        while(1){

        timerValueAtTrigger = portGET_RUN_TIME_COUNTER_VALUE();
        //timer_micro_sec = sys_get_high_res_timer_us();
        //printf("timer_micro_sec_inside: %i\n", timer_micro_sec);
        LPC_GPIO1->FIOCLR = (1 << 20); // clear trigger
        delay_us(2); // delay for 10 us
        LPC_GPIO1->FIOSET = (1 << 20); // set trigger
        delay_us(20); // delay for 10 us
        LPC_GPIO1->FIOCLR = (1 << 20); // clear trigger


        //unsigned int
        //printf("timerValueAtTrigger: %i\n", timerValueAtTrigger);
        //delay_ms(1000);
        //unsigned int timerDelta=0;
        //timerDelta = (timerValueAtIsr - timerValueAtTrigger);
        //printf("timerDelta: %i\n", timerDelta);
        //unsigned  int deltaInMicroSec = timerDelta * 0.666;

        /*
        unsigned int timeAtTrigger = LPC_TIM1->TC;
        LPC_TIM1->CCR |= (1<<4);  //falling edge interrupt
        unsigned int timeAfterTrigger = LPC_TIM1->TC;
        LPC_TIM1->CCR |= (0<<4);  //falling edge interrupt

        unsigned int delta = timeAfterTrigger - timeAtTrigger;
        delta = delta * 0.666; //in Us
        delta = delta - 750;
        delta =delta/2;
        delta = delta/73.746;
        */
        //printf("Distance_inch: %i\n", deltaInMicroSec);
         delay_ms(1000);
        //printf("dead\n");
        //delay_ms(1000);

        }
    }
    return -1;
}



