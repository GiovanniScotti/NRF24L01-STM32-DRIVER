///////////////////////////////////////////////////////////////////////////////
// NRF24L01/NRF24L01+ DRIVER FOR STM32 MICROCONTROLLERS
//
// This is an STM32 device driver library for the NRF24L01 Nordic Radio
// transceiver using STM HAL libraries.
//
// HOW TO USE THIS DRIVER:
//	- Change the HAL include in the header file according to your STM32 series.
//  - Initialize a free running TIM counting microseconds and pass its
//    pointer to the NRF24_begin function.
//  - Initialize a SPI peripheral and two GPIO outputs (CE and CSN) and
//	  supply the required pointers to the NRF24_begin function.
//  - You can enable debug functions by uncommenting the NRF24_DEBUG_ENABLE
//	  define in the header file and initializing a UART peripheral.
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License version 3 as
//  published by the Free Software Foundation.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// WARNING: Change this according to your STM32 microcontroller series.
#include "stm32f4xx_hal.h"
//////////////////////////

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

// Activates debug functions.
//#define NRF24_DEBUG_ENABLE

///////////////////////////////////////////////////////////////////////////////
// NRF24 DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Macro helpers.
#define _BV(x)    (1<<(x))
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define _BOOL(x)  (((x)>0) ? 1:0)

// Memory map.
#define REG_CONFIG      	0x00
#define REG_EN_AA       	0x01
#define REG_EN_RXADDR   	0x02
#define REG_SETUP_AW    	0x03
#define REG_SETUP_RETR  	0x04
#define REG_RF_CH       	0x05
#define REG_RF_SETUP    	0x06
#define REG_STATUS      	0x07
#define REG_OBSERVE_TX  	0x08
#define REG_CD          	0x09
#define REG_RX_ADDR_P0		0x0A
#define REG_RX_ADDR_P1  	0x0B
#define REG_RX_ADDR_P2  	0x0C
#define REG_RX_ADDR_P3  	0x0D
#define REG_RX_ADDR_P4  	0x0E
#define REG_RX_ADDR_P5  	0x0F
#define REG_TX_ADDR     	0x10
#define REG_RX_PW_P0    	0x11
#define REG_RX_PW_P1    	0x12
#define REG_RX_PW_P2    	0x13
#define REG_RX_PW_P3    	0x14
#define REG_RX_PW_P4    	0x15
#define REG_RX_PW_P5    	0x16
#define REG_FIFO_STATUS 	0x17
#define REG_DYNPD	    	0x1C
#define REG_FEATURE	    	0x1D

// Bit mnemonics.
#define MASK_RX_DR  		6
#define MASK_TX_DS  		5
#define MASK_MAX_RT 		4
#define BIT_EN_CRC      	3
#define BIT_CRCO        	2
#define BIT_PWR_UP      	1
#define BIT_PRIM_RX     	0
#define BIT_ENAA_P5     	5
#define BIT_ENAA_P4     	4
#define BIT_ENAA_P3     	3
#define BIT_ENAA_P2     	2
#define BIT_ENAA_P1     	1
#define BIT_ENAA_P0     	0
#define BIT_ERX_P5      	5
#define BIT_ERX_P4      	4
#define BIT_ERX_P3      	3
#define BIT_ERX_P2      	2
#define BIT_ERX_P1      	1
#define BIT_ERX_P0      	0
#define BIT_AW          	0
#define BIT_ARD         	4
#define BIT_ARC         	0
#define BIT_PLL_LOCK    	4
#define BIT_RF_DR       	3
#define BIT_RF_PWR      	6
#define BIT_RX_DR       	6
#define BIT_TX_DS       	5
#define BIT_MAX_RT      	4
#define BIT_RX_P_NO     	1
#define BIT_TX_FULL     	0
#define BIT_PLOS_CNT    	4
#define BIT_ARC_CNT     	0
#define BIT_TX_REUSE    	6
#define BIT_FIFO_FULL   	5
#define BIT_TX_EMPTY    	4
#define BIT_RX_FULL     	1
#define BIT_RX_EMPTY    	0
#define BIT_DPL_P5	    	5
#define BIT_DPL_P4	    	4
#define BIT_DPL_P3	    	3
#define BIT_DPL_P2	    	2
#define BIT_DPL_P1	    	1
#define BIT_DPL_P0	    	0
#define BIT_EN_DPL	    	2
#define BIT_EN_ACK_PAY  	1
#define BIT_EN_DYN_ACK  	0

// Instruction mnemonics.
#define CMD_R_REGISTER    	0x00
#define CMD_W_REGISTER    	0x20
#define CMD_REGISTER_MASK 	0x1F
#define CMD_ACTIVATE      	0x50
#define CMD_R_RX_PL_WID   	0x60
#define CMD_R_RX_PAYLOAD  	0x61
#define CMD_W_TX_PAYLOAD  	0xA0
#define CMD_W_ACK_PAYLOAD 	0xA8
#define CMD_FLUSH_TX      	0xE1
#define CMD_FLUSH_RX      	0xE2
#define CMD_REUSE_TX_PL   	0xE3
#define CMD_NOP           	0xFF

// Non-P omissions.
#define LNA_HCURR   		0

// P model memory map.
#define REG_RPD         	0x09

// P model bit mnemonics.
#define RF_DR_LOW   		5
#define RF_DR_HIGH  		3
#define RF_PWR_LOW  		1
#define RF_PWR_HIGH 		2


///////////////////////////////////////////////////////////////////////////////
// TYPES DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Power amplifier function, NRF24_setPALevel().
typedef enum {
	RF24_PA_m18dB = 0,
	RF24_PA_m12dB,
	RF24_PA_m6dB,
	RF24_PA_0dB,
	RF24_PA_ERROR
}rf24_pa_dbm_e ;

// NRF24_setDataRate() input.
typedef enum {
	RF24_1MBPS = 0,
	RF24_2MBPS,
	RF24_250KBPS
}rf24_datarate_e;

// NRF24_setCRCLength() input.
typedef enum {
	RF24_CRC_DISABLED = 0,
	RF24_CRC_8,
	RF24_CRC_16
}rf24_crclength_e;

// Pipe address registers.
static const uint8_t NRF24_ADDR_REGS[7] = {
	REG_RX_ADDR_P0,
	REG_RX_ADDR_P1,
	REG_RX_ADDR_P2,
	REG_RX_ADDR_P3,
	REG_RX_ADDR_P4,
	REG_RX_ADDR_P5,
	REG_TX_ADDR
};

// RX_PW_Px registers addresses.
static const uint8_t RF24_RX_PW_PIPE[6] = {
	REG_RX_PW_P0,
	REG_RX_PW_P1,
	REG_RX_PW_P2,
	REG_RX_PW_P3,
	REG_RX_PW_P4,
	REG_RX_PW_P5
};


///////////////////////////////////////////////////////////////////////////////
// FUNCTIONS PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

// Begin function.
void NRF24_begin(GPIO_TypeDef *nrf24PORT, uint16_t nrfCSN_Pin,
	uint16_t nrfCE_Pin, SPI_HandleTypeDef *nrfSPI, TIM_HandleTypeDef *nrfTIM);

// Microseconds delay function.
void NRF24_DelayMicroSeconds(uint32_t uSec);

// Chip select.
void NRF24_csn(int state);

// Chip enable.
void NRF24_ce(int state);

// Reads single byte from register.
uint8_t NRF24_read_register(uint8_t reg);

// Reads multiple bytes register.
void NRF24_read_registerN(uint8_t reg, uint8_t *buf, uint8_t len);

// Writes single byte register.
void NRF24_write_register(uint8_t reg, uint8_t value);

// Writes multiple bytes register.
void NRF24_write_registerN(uint8_t reg, const uint8_t *buf, uint8_t len);

// Writes payload to transmit.
void NRF24_write_payload(const void *buf, uint8_t len);

// Reads received payload.
void NRF24_read_payload(void *buf, uint8_t len);

// Flushes TX buffer.
void NRF24_flush_tx(void);

// Flushes RX buffer.
void NRF24_flush_rx(void);

// Gets status register value.
uint8_t NRF24_get_status(void);

// Listens to open pipes for reading (must be preceded by
// NRF24_openReadingPipe() call first).
void NRF24_startListening(void);

// Stops listening (required before any write operation).
void NRF24_stopListening(void);

// Transmits data, returns true if successfully sent.
bool NRF24_write(const void *buf, uint8_t len, uint32_t timeout_ms);

// Checks for available data to read.
bool NRF24_available(void);

// Reads received data.
bool NRF24_read(void *buf, uint8_t len);

// Opens TX pipe for writing (cannot perform this while listening).
void NRF24_openWritingPipe(uint64_t address);

// Opens RX pipe for reading.
void NRF24_openReadingPipe(uint8_t number, uint64_t address);

// Sets transmit retries and delay between them.
void NRF24_setRetries(uint8_t delay, uint8_t count);

// Sets RF channel frequency.
void NRF24_setChannel(uint8_t channel);

// Sets payload size.
void NRF24_setPayloadSize(uint8_t size);

// Gets payload size.
uint8_t NRF24_getPayloadSize(void);

// Gets dynamic payload size of the latest packet received.
uint8_t NRF24_getDynamicPayloadSize(void);

// Enables payload on acknowledge packet.
void NRF24_enableAckPayload(void);

// Enables dynamic payload.
void NRF24_enableDynamicPayloads(void);

// Disables dynamic payload.
void NRF24_disableDynamicPayloads(void);

// Checks if module is NRF24L01+ or not.
bool NRF24_isNRF_Plus(void);

// Sets Auto Ack for all pipes.
void NRF24_setAutoAck(bool enable);

// Sets Auto Ack for a specific pipe.
void NRF24_setAutoAckPipe(uint8_t pipe, bool enable);

// Sets TX power level.
void NRF24_setPALevel(rf24_pa_dbm_e level);

// Gets TX power level.
rf24_pa_dbm_e NRF24_getPALevel(void);

// Sets data rate.
bool NRF24_setDataRate(rf24_datarate_e speed);

// Gets data rate.
rf24_datarate_e NRF24_getDataRate(void);

// Sets CRC length (disabled, 8-bits or 16-bits).
void NRF24_setCRCLength(rf24_crclength_e length);

// Gets CRC length.
rf24_crclength_e NRF24_getCRCLength(void);

// Disables CRC.
void NRF24_disableCRC(void);

// Powers up the radio.
void NRF24_powerUp(void);

// Powers down the radio.
void NRF24_powerDown(void);

// Checks if data are available and on which pipe.
// To be used for multiple RX pipes.
bool NRF24_availablePipe(uint8_t *pipe_num);

// Starts write for IRQ mode.
void NRF24_startWrite(const void *buf, uint8_t len);

// Writes acknowledge payload.
void NRF24_writeAckPayload(uint8_t pipe, const void *buf, uint8_t len);

// Checks if an Ack payload is available.
bool NRF24_isAckPayloadAvailable(void);

// Checks interrupt flags.
void NRF24_whatHappened(bool *tx_ok, bool *tx_fail, bool *rx_ready);

// Tests if there is a carrier on the previous listening period
// (useful for checking interferences).
bool NRF24_testCarrier(void);

// Tests if a signal carrier exists (=> -64dB), only for NRF24L01+.
bool NRF24_testRPD(void) ;

// Resets status.
void NRF24_resetStatus(void);

// Executes activate command.
void NRF24_ACTIVATE_cmd(void);

// Gets Ack payload size.
uint8_t NRF24_GetAckPayloadSize(void);


///////////////////////////////////////////////////////////////////////////////
// DEBUG FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

#ifdef NRF24_DEBUG_ENABLE

// Initializes debug functions.
void nrf24_DebugUART_Init(UART_HandleTypeDef *nrf24Uart);

// Prints radio settings.
void printRadioSettings(void);

// Prints radio status.
void printStatusReg(void);

// Prints radio configuration.
void printConfigReg(void);

// Prints FIFO status.
void printFIFOstatus(void);

#endif
