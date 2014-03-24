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
#include "uart2.hpp"    //uart2
#include "string.h"     //for string (strlen)
#include "spi.hpp"     // for spi1
#include "stdlib.h"
#include "I2C2.hpp"       //for I2C2 Master
#include "uart0_min.h"
#include <FreeRTOS.h>
#include "semphr.h"
#include "task.h"
#include "eint.h"
#include "wireless.h"


extern volatile uint32_t SlaveState;
extern char slave_buffer[4];


SemaphoreHandle_t guard;

void task1(void* p1)
{
    while(1) {

        xSemaphoreGiveFromISR(guard,NULL);
        //uart0_puts("Hiiiiiiiii!! I am in Task1");
        //vTaskDelay(1000);
    }
}

void task2(void* p2)
{
    while(1) {
        if(xSemaphoreTake(guard,portMAX_DELAY)){
            puts("Task 2: I got the key\n");
        }//
        //uart0_puts("Byeeeeeeeeeeee from Task2");
        //vTaskDelay(1000);
    }
}
/*Wireless stuff------------------------------*/
enum{
    lights_addr=100,
    commander_addr=200,
};
enum{
    lights_on =1,
    lights_off=2,
};
/*-------------------------------------------*/

int main(void)
{
    //===Wireless Loop back Test (Hope it will work!!)===============================
    if(0){
        puts("Wireless Loopback Test");
        char cmd=0;
        const char max_hops=0;
        mesh_set_node_address(commander_addr);
        while(1){

            //---Sending logic------------------------------------------
            puts("Lights on!");
            cmd=lights_on;
            wireless_send(lights_addr,mesh_pkt_nack, &cmd,1,max_hops);
            delay_ms(1000);

            //puts("Lights off");
            //cmd=lights_off;
            //wireless_send(lights_addr,mesh_pkt_nack, &cmd,1,max_hops);
            //delay_ms(1000);
        //}

        //---Receiving logic-----------------------------------------
        mesh_set_node_address(lights_addr);
        //while(1){
            //delay_ms(1000);
            //delay_ms(1000);
            mesh_packet_t pkt;
            printf("Pkt: %i\n",pkt);
            if(wireless_get_rx_pkt(&pkt,1000)){
                puts("Rx pkt!");
                char command=pkt.data[0];
                switch (command){
                    case lights_on:
                        //LE.on(1);
                        break;
                    case lights_off:
                       // LE.off(1);
                        break;
                    default:
                        printf("Error: Invalid Command!\n");
                        break;
                }
            }

        }


    }


    //===Proj========================================================================
    if(0){
        int cmd;
        while(1){
            //cmd=command_received();
            if(cmd==1 ){
                //pwm=right;
                //right indicator
            }
            else if(cmd==2){
                //pwd==left;
                //left indicator
            }
            else if(cmd==3){
                //pwd==forward;
                if(cmd==5) {
                    //front_light=1;
                }
                else{
                    //front_light=0;
                }
            }
            else if(cmd==4){
                //pwd==back;
            }
            else {
                //pwd=stop;
                if(cmd==5) {
                    //front_light=1;
                }
                else{
                    //front_light=0;
                }
            }
        }

    }

    //Accelerometer Test====================================================
    if(0){
        while(1){
            int tilt_x = AS.getX();
            int tilt_y = AS.getY();
            int tilt_z = AS.getZ();
            printf("X: %i, Y: %i, Z: %i\n",tilt_x,tilt_y,tilt_z);
            delay_ms(1000);
        }
    }




    //===Task homework======================================================
    if(1){

        vSemaphoreCreateBinary(guard);

        //rit_setup_callback(isr_foo,1000);
        xTaskCreate(task1, "task1", 1024, 0, 1, 0);
        //xTaskCreate(task2, "task2", 1024, 0, 1, 0);
        vTaskStartScheduler();
    }



    //===I2C2==============================================================
        if(0){
            I2C2::getInstance().init(400);

            for (int i = 0; i < 4; i++ ){
                    printf("Init slave_buffer %i : %X\n",i, slave_buffer[i]);

            }

            //Slave---------------------------------------------------------
            if(1){
                printf("----I2C Slave----\n");

                while(SlaveState!=SlaveIdle){

                    //printf("I2C State Status: %X\n", LPC_I2C2->I2CONSET);
                    //delay_ms(100);
                }
                delay_ms(1000);

                for ( int i = 0; i < 4; i++ )
                {
                    printf("slave_buffer %i : %X\n",i,slave_buffer[i]);
                }


                int a = slave_buffer[0];
                int count = slave_buffer[1];
                LE.on(a);
                delay_ms(count* 1000);
                LE.off(a);

            }



            //Master------------------------------------------------------------------
            if(0){
                int dev_stat;
                printf("----I2C Master----\n");
                dev_stat=I2C2::getInstance().checkDeviceResponse(0x38);
                printf("Acceleration status: %i\n", dev_stat);
                dev_stat=I2C2::getInstance().checkDeviceResponse(0x77);
                printf("I2C slave status: %i\n", dev_stat);
                LPC_PINCON->PINSEL2 |=  (0x00 << 15);  //select p1_15 SW3
                LPC_GPIO1->FIODIR    =  (0x00 << 15);  //set as input
                while(1){
                    if(LPC_GPIO1->FIOPIN & (1<<15)){
                        printf("Switch 3 pressed\n");
                        const char my_dev_addr = 0x77; // device address
                        const char my_dev_reg  = 0x02; // register address
                        const char my_dev_data = 0x04; // data
                        I2C2::getInstance().writeReg(my_dev_addr, my_dev_reg, my_dev_data);
                    }

                    delay_ms(100);
                }
            }

        }

       //===SPI1=====================================================================
        if(0){
            //FLASH mem-----------------------
            //INIT FLASH
            spi1_init();

            LPC_GPIO0->FIOCLR = (1<<6);
            //LPC_SSEL1->FIOSET= (1<<10);

            spi1_ExchangeByte(0x9F);
            char rd_byte0=spi1_ExchangeByte(0x00);
            char rd_byte1=spi1_ExchangeByte(0x00);
            char rd_byte2=spi1_ExchangeByte(0x00);
            char rd_byte3=spi1_ExchangeByte(0x00);
            printf("Manufacture ID: %X\n", rd_byte0);
            printf("Device ID1: %X\n",rd_byte1);
            printf("Device ID2:%X\n",rd_byte2);
            printf("Extended Device info string length: %X\n",rd_byte3);

            //desslect flash mem
            //LPC_GPIO0->FIOSET = (1<<6);

            //CS enabel
            //LPC_GPIO0->FIOCLR = (1<<6);

            spi1_ExchangeByte(0xD2);
            char rd_fat32=spi1_ExchangeByte(0x00);
            printf("Its not working damit: %i\n", rd_fat32);
            //spi1_ExchangeByte(0x0D);

            //spi1_ExchangeByte(0x33);
            //spi1_ExchangeByte(0x44);
            //spi1_ExchangeByte(0x55);
            //spi1_ExchangeByte(0x66);

            //char rd_fat32=spi1_ExchangeByte(0x00);
            //char rd_byte1=spi1_ExchangeByte(0x00);
            //char rd_byte3=spi1_ExchangeByte(0x00);

            //int fd;

            //printf("byte per sector: %X\n", rd_fat32);
            //rd_fat32=spi1_ExchangeByte(0x00);
            //printf("byte per sector2: %X\n", rd_fat32);

            int i=0;
            for(i=0;i<12;i++){
                printf("%i byte: %i\n", i, rd_fat32);
                rd_fat32=spi1_ExchangeByte(0x00);
            }


            //printf("2nd byte: %X\n",rd_byte1);

            //CS disable
            LPC_GPIO0->FIOSET = (1<<6);
        }

        //===UART2==============================================================
        if(0){
            //printf("UART2 Testing");
            int len;
            char a[100];
            char c;
            char c_string[100];

            uart2_init(9600);
            delay_ms(1000);
            while(0){
                //Sending
                scanf("%s", a);
                strcat(a,"~");
                printf("Sending: \t%s\n", a);
                uart2_putstring(a);


                //Receiving
                c= uart2_getchar();
                while(c!='~'){
                    len=strlen(c_string);
                    c_string[len] =c;
                    c_string[len+1]='\0';
                    c= uart2_getchar();
                }
                printf("I Received>>: \t%s\n", c_string);

                len=strlen(c_string);
                for(int i=0;i<len;i++){
                    c_string[i]='\0';
                }

            }
        }

    if(0){
        while(1){
            printf("Temperature is: %f\n", TS.getFarenheit());
            printf("  Light sensor: %u%%\n\n", LS.getPercentValue());
            delay_ms(1000);
        }
    }

    return -1;
}
