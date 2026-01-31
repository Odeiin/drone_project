#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"

#include "control_protocol.h"
#include "joysticks.h"
#include "drone_err.h"

//GPIO pins
#define MISO_pin 19
#define MOSI_pin 23
#define SCK_pin 18
#define CSN_pin 14
#define CE_pin 4
#define IRQ_pin 22

// have to add more when needed
// NRF24L01 command set
#define CMD_R_REG 0x00 // OR'd with 0b 000A AAAA where A is the reg address
#define CMD_W_REG 0x20 // OR'd with 0b 000A AAAA where A is the reg address
#define CMD_R_RX_PAYLOAD 0x61
#define CMD_W_TX_PAYLOAD 0xA0
#define CMD_FLUSH_TX 0xA0
#define CMD_FLUSH_TX 0xA0
#define CMD_NOP 0xFF

#define MAX_PACKET_SIZE 32
#define PACKET_SIZE sizeof(ControlData_t)

// radio addresses are stored as an array of 5 uint8_t
typedef uint8_t NRF_addr_t[5];

// radio packet is stored as 32Bytes
// typedef uint8_t NRF_packet_t[32];

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
void SPI_transmit(spi_device_handle_t SPI, const void *txBuffer, void *rxBuffer, size_t length_Bytes, size_t rxLength_Bytes);

// initialises an radio and returns its handle
drone_err_t NRF_init(NRF_handle_t *radio, NRF_addr_t rxAddr, NRF_addr_t txAddr, NRF_channel_t RF_CH, size_t packetLength);

// transitions the radio to power down state
drone_err_t NRF_enter_power_down(NRF_handle_t *radio);

// transitions the radio to standby state
drone_err_t NRF_enter_standby(NRF_handle_t *radio);

drone_err_t NRF_enter_RXmode(NRF_handle_t *radio);

drone_err_t NRF_enter_TXmode(NRF_handle_t *radio);

// creates 10us pulse for sending data in TXmode, busy waits
drone_err_t NRF_pulse_TXmode(NRF_handle_t *radio);

// true if fifo empty, false otherwise
bool NRF_TX_Fifo_Empty(NRF_handle_t *radio);

// true if fifo full, false otherwise
bool NRF_TX_Fifo_Full(NRF_handle_t *radio);

// true if fifo empty, false otherwise
bool NRF_RX_Fifo_Empty(NRF_handle_t *radio);

// true if fifo full, false otherwise
bool NRF_RX_Fifo_Full(NRF_handle_t *radio);

// packetLength is in bytes
drone_err_t NRF_push_packet(NRF_handle_t *radio, const uint8_t *packet, size_t packetLength);

// the receiver should be expecting a standard packet size so I thought it made sense to pass in a packet size 
drone_err_t NRF_read_Fifo(NRF_handle_t *radio, uint8_t *packet, size_t packetLength);
