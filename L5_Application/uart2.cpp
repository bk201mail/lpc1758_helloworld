/*
 * uart2.cpp
 */

#include "io.hpp"       // Board sensors
#include "sys_config.h" //get CPU clock (48000000)

void uart2_init(unsigned long baudrate)
    {
        int CPU_CLOCK;
        unsigned long divider;
        CPU_CLOCK = sys_get_cpu_clock();


        LPC_SC->PCONP |= (1 << 24);       // Enable power to UART2
        LPC_SC->PCLKSEL1 &= ~(3 << 16);
        LPC_SC->PCLKSEL1 |=  (1 << 16);   // UART clock = CPU / 1

        //Selecting Pins: P2_8 and P2_9
        LPC_PINCON->PINSEL4 &= ~(0xF << 16); // Clear values
        LPC_PINCON->PINSEL4 |= (0xA << 16);  // Set values for UART2 Rx/Tx

        LPC_UART2->FCR |=0x01; //enable FIFO

        LPC_UART2->LCR = (1<<7);  // select DLAB (0x83)

        //BaudRate = PClk / (16 * (256 * DLL + DLM) * (1 + DAV/MV)))
        //LPC_UART2->FDR = 0x21;//0x10;   //set DivaddVal/mulVal
        LPC_UART2->DLM = 0;
        divider = CPU_CLOCK/((16 * baudrate) + 0.5);   //divider val calculation
        LPC_UART2->DLL = divider / 256;

        LPC_UART2->LCR = 0x03;    //disable DLAB, 8 bit, 1 stop, no parity

    }


char uart2_putchar (char out)
{
    LPC_UART2->THR = out;
    while(!(LPC_UART2->LSR & (1 << 6)));
    return 1;
}


int uart2_getchar (void)
{

    while (!(LPC_UART2->LSR & 0x01));
    return(LPC_UART2->RBR);
}



void uart2_putstring (const char *string)
{
    char outstring;

    while ((outstring = *string)) {
        uart2_putchar(outstring);
        string++;
    }
}



