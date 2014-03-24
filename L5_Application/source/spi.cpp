/*
 * spi.cpp
 *
 *  Created on: Feb 22, 2014
 *      Author: Deepak
 */

#include "io.hpp"       // Board sensors
#include "sys_config.h" //get CPU clock (48000000)

void spi1_init()
{
    LPC_SC->PCONP |= (1 << 10);     // SPI1 Power Enable
    LPC_SC->PCLKSEL0 &= ~(3 << 20); // Clear clock Bits
    LPC_SC->PCLKSEL0 |=  (1 << 20); // CLK / 1


    //select flash mem
    LPC_PINCON->PINSEL0 |=  (0x0 << 10); //select p0_6 flash chip select
    LPC_GPIO0->FIODIR =(0x1<<6);  //set as output
    // Select (P0_7, P0_8, P0_9) MISO, MOSI, and SCK pin-select functionality
    //LPC_PINCON->PINSEL0 &= ~( (3 << 14) | (3 << 16) | (3 << 18) );
    //LPC_PINCON->PINSEL0 |=  ( (2 << 14) | (2 << 16) | (2 << 18) );

    LPC_SSP1->CR0 = 7;          // 8-bit mode
    LPC_SSP1->CR1 = (1 << 1);   // Enable SSP as Master
    LPC_SSP1->CPSR = 8;         // SCK speed = CPU / 8
}

char spi1_ExchangeByte(char out)
{
    LPC_SSP1->DR = out;
    while(LPC_SSP1->SR & (1 << 4)); // Wait until SSP is busy
    return LPC_SSP1->DR;
}



