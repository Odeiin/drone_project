#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"

#include "drone_err.h"

//GPIO pins
#define MISO_pin 19
#define MOSI_pin 23
#define SCK_pin 18
#define CSN_pin 14
#define CE_pin 4

// have to add more when needed
// NRF24L01 command set
#define CMD_R_REG 0x00 // OR'd with 0b 000A AAAA where A is the reg address
#define CMD_W_REG 0x20 // OR'd with 0b 000A AAAA where A is the reg address
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define CMD_NOP 0xFF

#define MAX_PACKET_SIZE 32

// radio addresses are stored as an array of 5 uint8_t
typedef uint8_t NRF_addr_t[5];

// channel is an 8 bit value but must be less than 127 or 0x7F
typedef uint8_t NRF_channel_t;

// based on state diagram in NRF24L01 documentation (figure 4)
typedef enum {
	powerDown,
	standby,
	TXmode,
	RXmode
} radioState_t;

typedef struct{
	spi_device_handle_t SPI;
	radioState_t state;
} NRF_handle_t;

// creates SPI config settings and returns the SPI handle
spi_device_handle_t SPI_init();

/** uses SPI config settings, transmits a tx buffer, rx buffer will be modified
 * 	for NRF24L01, the tx_buffer should be 2 bytes, the first byte being the command and the second being data 
 * */ 
void SPI_transmit(spi_device_handle_t SPI, uint8_t *tx_buffer, uint8_t *rx_buffer, uint8_t length, uint8_t rxlength);

// initialises an radio and returns its handle
drone_err_t NRF_init(NRF_handle_t *radio, NRF_addr_t rxAddr, NRF_addr_t txAddr, NRF_channel_t RF_CH);

// transitions the radio to power down state
drone_err_t NRF_enter_power_down(NRF_handle_t *radio);

// transitions the radio to standby state
drone_err_t NRF_enter_standby(NRF_handle_t *radio);

// supposed to wait for 130us afterwards before it can send data
drone_err_t NRF_enter_RXmode(NRF_handle_t *radio);

// supposed to wait for 130us afterwards before it can send data
drone_err_t NRF_enter_TXmode(NRF_handle_t *radio);

// true if fifo empty, false otherwise
bool TX_Fifo_Empty(NRF_handle_t *radio);

// true if fifo full, false otherwise
bool TX_Fifo_Full(NRF_handle_t *radio);