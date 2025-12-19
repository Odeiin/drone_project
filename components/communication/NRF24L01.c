#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"

#include "NRF24L01.h"
#include "drone_err.h"
#include "esp_err.h"

// initialises a SPI communication and returns the handle
spi_device_handle_t SPI_init() {
	spi_device_interface_config_t devconfig = {
    .clock_speed_hz = 2000000, // 2Mhz, kinda arbitrary choice for the time being
    .mode = 0,                 // CPOL + CPHA mode 0
    .spics_io_num = CSN_pin,
    .queue_size = 1,
  };

  spi_bus_config_t busconfig = {
    .miso_io_num = MISO_pin, 
    .mosi_io_num = MOSI_pin,
    .sclk_io_num = SCK_pin,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 64, // set 0 for default 4092 bytes 
  };

  // these asserts happen during initialisation so my current thought is that if these fail its ok for an abort
	// spi3_host is one of the general purpose SPI, i believe use it for vspi, DMA disabled
  esp_err_t err = spi_bus_initialize(SPI3_HOST, &busconfig, SPI_DMA_DISABLED); 
  assert(err == ESP_OK);

  spi_device_handle_t SPI;
  err = spi_bus_add_device(SPI3_HOST, &devconfig, &SPI);
  assert(err == ESP_OK); 

	return SPI;
}

// transmits an SPI buffer, the receiver buffer is modified by this function, length is how many bits shifted out of MOSI
// rxlength is how many bits are copied into the rx buffer
void SPI_transmit(spi_device_handle_t SPI, uint8_t *txBuffer, uint8_t *rxBuffer, uint8_t length, uint8_t rxLength) {
  spi_transaction_t trans;
  memset(&trans, 0, sizeof(trans));

  trans.length = length;
  trans.rxlength = rxLength;
  trans.tx_buffer = txBuffer; // in buffer
  trans.rx_buffer = rxBuffer; // out buffer
	// buffers modified
  esp_err_t err = spi_device_transmit(SPI, &trans);
  if (err != ESP_OK) { // only other error is invalid params
    assert(false);
  }
}

// initialises a NRF handle with given addresses and channel, moves radio to standby state
drone_err_t NRF_init(NRF_handle_t *radio, NRF_addr_t rxAddr, NRF_addr_t txAddr, NRF_channel_t RF_CH) {
  if (RF_CH > 127) {
    return NRF_INVALID_CHANNEL;
  }

  radio.SPI = SPI_init();
  drone_err_t err = NRF_enter_standby(&radio);
  if (err) {
    return err;
  }
  radio.state = standby;

  // setting initial values
  uint8_t txBuffer[1 + 5];
  uint8_t rxBuffer[1 + 5];

  // the reset values in the documentation says it resets -> 2Mbps but mine appears to reset -> 1Mbps so
  // this might just not be necessary
  // (set to 250Kbps if too unreliable)
  // setting Data Rate to 1Mbps, [RF_DR_LOW, RF_DR_HIGH] = 00
  txBuffer[0] = CMD_R_REG | 0x06; // read RF_SETUP
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio.SPI, txBuffer, rxBuffer, 16, 16);
  txBuffer[0] = CMD_W_REG | 0x06; // write to RF_SETUP
  txBuffer[1] = rxBuffer[1] & 0xD7; // [RF_DR_LOW, RF_DR_HIGH] = 00
  SPI_transmit(radio.SPI, txBuffer, rxBuffer, 16, 0);

  // setting RX_ADDR_P1
  txBuffer[0] = CMD_W_REG | 0x0B; // write to RX_ADDR_P1
  memcpy(&txBuffer[1], rxAddr, 5);
  SPI_transmit(radio.SPI, txBuffer, rxBuffer, 48, 0);

  // setting TX_ADDR
  txBuffer[0] = CMD_W_REG | 0x10; // write to TX_ADDR
  memcpy(&txBuffer[1], txAddr, 5);
  SPI_transmit(radio.SPI, txBuffer, rxBuffer, 48, 0);

  // setting RF_CH
  txBuffer[0] = CMD_W_REG | 0x05; // write to RF_CH
  txBuffer[1] = RF_CH;
  SPI_transmit(radio.SPI, txBuffer, rxBuffer, 16, 0);

  return radio;
}

// maybe return ints for error codes
drone_err_t NRF_enter_power_down(NRF_handle_t *radio) {
  if (radio->state == powerDown) {
    return DRONE_OK;
  }
  if (radio->state != standby) { // not recommended transition
    return NRF_ILLEGAL_TRANSITION; 
  }
  // gpio_set_level(CE_pin, 0); shouldnt matter

  uint8_t txBuffer[2];
  uint8_t rxBuffer[2];

  txBuffer[0] = CMD_R_REG | 0x00; // read config
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 16, 16);

  txBuffer[0] = CMD_W_REG | 0x00; // write to config (ORing just to be explicit im not stupid i swear)
  txBuffer[1] = rxBuffer[1] & 0xFD; // config PWR_UP bit -> 0
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 16, 0);

  radio->state = powerDown;
  return DRONE_OK;
}


drone_err_t NRF_enter_standby(NRF_handle_t *radio) {
  if (radio->state == standby) {
    return DRONE_OK;
  }
  gpio_set_level(CE_pin, 0); // necessary

  uint8_t txBuffer[2];
  uint8_t rxBuffer[2];

  txBuffer[0] = CMD_R_REG | 0x00; // read config
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 16, 16);

  txBuffer[0] = CMD_W_REG | 0x00; // write to config
  txBuffer[1] = rxBuffer[1] | 0x02; // config PWR_UP bit -> 1
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 16, 0);

  // if (radio->state == powerDown) {
  //   vTaskDelay(pdMS_TO_TICKS(2));
  // }

  radio->state = standby;
  return DRONE_OK;
}

// supposed to wait for 130us afterwards before it can send data
drone_err_t NRF_enter_RXmode(NRF_handle_t *radio) {
  if (radio->state == RXmode) {
    return DRONE_OK;
  }
  if (radio->state != standby) {
    return NRF_ILLEGAL_TRANSITION;
  }
  gpio_set_level(CE_pin, 1); // required for transition

  uint8_t txBuffer[2];
  uint8_t rxBuffer[2];

  txBuffer[0] = CMD_R_REG | 0x00; // read config
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 16, 16);

  txBuffer[0] = CMD_W_REG | 0x00; // write to config
  txBuffer[1] = rxBuffer[1] | 0x01; // config PRIM_RX bit -> 1
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 16, 0);

  radio->state = RXmode;
  return DRONE_OK;
}

// supposed to wait for 130us afterwards before it can send data
drone_err_t NRF_enter_TXmode(NRF_handle_t *radio) {
  if (radio->state == TXmode) {
    return DRONE_OK;
  }
  if (radio->state != standby) {
    return NRF_ILLEGAL_TRANSITION;
  }
  if (TX_Fifo_Empty(radio)) {
    return NRF_EMPTY_TXFIFO;
  }
  gpio_set_level(CE_pin, 1); // required for transition

  uint8_t txBuffer[2];
  uint8_t rxBuffer[2];

  txBuffer[0] = CMD_R_REG | 0x00; // read config
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 16, 16);

  txBuffer[0] = CMD_W_REG | 0x00; // write to config
  txBuffer[1] = rxBuffer[1] & 0xFE; // config PRIM_RX bit -> 0
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 16, 0);

  radio->state = TXmode;
  return DRONE_OK;
}

// true if fifo empty, false otherwise
bool TX_Fifo_Empty(NRF_handle_t *radio) {
  uint8_t txBuffer[2];
  uint8_t rxBuffer[2];

  txBuffer[0] = CMD_R_REG | 0x17; // read FIFO_STATUS
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 16, 16);

  if (rxBuffer[1] & 0x10 == 0x10) {
    return true;
  }
  return false;
}

// true if fifo full, false otherwise
bool TX_Fifo_Full(NRF_handle_t *radio) {
  uint8_t txBuffer[2];
  uint8_t rxBuffer[2];

  txBuffer[0] = CMD_R_REG | 0x17; // read FIFO_STATUS
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 16, 16);

  if (rxBuffer[1] & 0x20 == 0x20) {
    return true;
  }
  return false;
}

// packetLength is in bytes
drone_err_t NRF_push_packet(NRF_handle_t *radio, const uint8_t *packet, uint8_t packetLength) {
  if (packetLength > MAX_PACKET_SIZE || packetLength == 0) {
    return NRF_INVALID_PACKET_LEN;
  }
  if (TX_Fifo_Full(radio)) {
    return NRF_FULL_TXFIFO;
  }

  uint8_t txBuffer[MAX_PACKET_SIZE + 1];
  //uint8_t rxBuffer[2];

  uint8_t SPILength = (packetLength + 1) * 8;
  txBuffer[0] = W_TX_PAYLOAD; // write to TX fifo
  memcpy(&txBuffer[1], packet, packetLength);
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, SPILength, 0);

  return DRONE_OK;
}


drone_err_t NRF_read_Fifo(NRF_handle_t *radio, uint8_t *packetLocation) {

  if rxfifo is empty {
    return
  }

  need to fill with NOPS to read all the data!  ! !  ! 
  uint8_t txBuffer[1];
  uint8_t rxBuffer[MAX_PACKET_SIZE];

  txBuffer[0] = W_TX_PAYLOAD; // write to TX fifo
  memcpy(&txBuffer[1], packet, packetLength);
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, SPILength, 0);

  return DRONE_OK;
}