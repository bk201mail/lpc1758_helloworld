/*
 * uart2.hpp
 *
 *  Created on: Feb 19, 2014
 *      Author: Deepak
 */

#ifndef UART2_HPP_
#define UART2_HPP_

//Uart2 Initialization.
void uart2_init (unsigned long baudrate);

//Output data uart2.
char uart2_putchar (char out);

//Get data uart2
int uart2_getchar (void);

//Output string on uart2.
void uart2_putstring (const char *string);


#endif /* UART2_HPP_ */
