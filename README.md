# NRF24L01(+) driver for STM32 microcontrollers
## Introduction
This is a highly portable driver written in C for the NRF24L01(+) radio transceiver.

## Library usage
To successfully run the driver, copy the .h and .c files in the appropriate
directory of your project and proceed with the following steps:
*  Change the HAL include in the header file according to your STM32 series.
   For example, in case of STM32F4 series:
```C
#include "stm32f4xx_hal.h"
```
*  Initialize a free running TIM counting microseconds and pass its
   pointer to the NRF24_begin function.
*  Initialize a SPI peripheral and two GPIO outputs for CE and CSN pins and
   supply the required pointers to the NRF24_begin function.
*  You can enable debug functions by uncommenting the NRF24_DEBUG_ENABLE
   define in the header file and initializing a UART peripheral.
```C
//#define NRF24_DEBUG_ENABLE
```
