#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"

#include "control_protocol.h"
#include "NRF24L01.h"
#include "drone_err.h"
#include "NRF_err.h"
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
// lengths are now bytes
void SPI_transmit(spi_device_handle_t SPI, const void *txBuffer, void *rxBuffer, size_t length_Bytes, size_t rxLength_Bytes) {
  spi_transaction_t trans;
  memset(&trans, 0, sizeof(trans));

  trans.length = (length_Bytes * 8);
  trans.rxlength = (rxLength_Bytes * 8);
  trans.tx_buffer = txBuffer; // in buffer
  trans.rx_buffer = rxBuffer; // out buffer
	// buffers modified
  esp_err_t err = spi_device_transmit(SPI, &trans);
  if (err != ESP_OK) { // only other error is invalid params
    assert(false);
  }
}

// refactor later
// initialises a NRF handle with given addresses and channel, moves radio to standby state
drone_err_t NRF_init(NRF_handle_t *radio, NRF_addr_t rxAddr, NRF_addr_t txAddr, NRF_channel_t RF_CH, size_t packetLength) {
  if (RF_CH > 127) {
    return NRF_INVALID_CHANNEL;
  }
  if (packetLength > MAX_PACKET_SIZE || packetLength == 0) {
    return NRF_INVALID_PACKET_LEN;
  }
  radio->state = powerDown;
  radio->SPI = SPI_init();
  drone_err_t err = NRF_enter_standby(radio);
  if (err) {
    return err;
  }

  // setting initial values
  uint8_t txBuffer[1 + 5];
  uint8_t rxBuffer[1 + 5];

  // the reset values in the documentation says it resets -> 2Mbps but mine appears to reset -> 1Mbps so
  // this might just not be necessary
  // (set to 250Kbps if too unreliable)
  // setting Data Rate to 1Mbps, [RF_DR_LOW, RF_DR_HIGH] = 00
  txBuffer[0] = CMD_R_REG | 0x06; // read RF_SETUP
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 2, 2);
  //1Mbps
  // txBuffer[0] = CMD_W_REG | 0x06; // write to RF_SETUP
  // txBuffer[1] = rxBuffer[1] & 0xD7; // [RF_DR_LOW, RF_DR_HIGH] = 00
  // SPI_transmit(radio->SPI, txBuffer, NULL, 2, 0);
  // 250Kbps
  txBuffer[0] = CMD_W_REG | 0x06; // write to RF_SETUP
  txBuffer[1] = rxBuffer[1] | 0x20; // [RF_DR_LOW, RF_DR_HIGH] = 10, RF_DR_HIGH is dont care
  // setting RF_PWR to -18dBm, attempting to make comms more reliable
  txBuffer[1] = txBuffer[1] & 0xF9;
  SPI_transmit(radio->SPI, txBuffer, NULL, 2, 0);

  // setting RX_ADDR_P0
  txBuffer[0] = CMD_W_REG | 0x0A; // write to RX_ADDR_P0
  memcpy(&txBuffer[1], rxAddr, 5);
  SPI_transmit(radio->SPI, txBuffer, NULL, 6, 0);

  // setting TX_ADDR
  txBuffer[0] = CMD_W_REG | 0x10; // write to TX_ADDR
  memcpy(&txBuffer[1], txAddr, 5);
  SPI_transmit(radio->SPI, txBuffer, NULL, 6, 0);

  // setting RF_CH
  txBuffer[0] = CMD_W_REG | 0x05; // write to RF_CH
  txBuffer[1] = RF_CH;
  SPI_transmit(radio->SPI, txBuffer, NULL, 2, 0);

  // setting RX_PW_P0
  txBuffer[0] = CMD_W_REG | 0x11; // write to RX_PW_P0
  txBuffer[1] = packetLength;
  SPI_transmit(radio->SPI, txBuffer, NULL, 2, 0);

  // setting EN_RXADDR
  txBuffer[0] = CMD_W_REG | 0x02; // write to EN_RXADDR
  txBuffer[1] = 0x01;             // Enable data pipe 0
  SPI_transmit(radio->SPI, txBuffer, NULL, 2, 0);

  // I may change this back in future
  // disable auto-ack on all pipes
  txBuffer[0] = CMD_W_REG | 0x01;
  txBuffer[1] = 0x00;
  SPI_transmit(radio->SPI, txBuffer, NULL, 2, 0);

  txBuffer[0] = CMD_R_REG | 0x00; // read config
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 2, 2);
  txBuffer[0] = CMD_W_REG | 0x00;
  txBuffer[1] = rxBuffer[1] | 0x04; // CRCO = 1
  SPI_transmit(radio->SPI, txBuffer, NULL, 2, 0);

  return DRONE_OK;
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
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 2, 2);

  txBuffer[0] = CMD_W_REG | 0x00; // write to config (ORing just to be explicit im not stupid i swear)
  txBuffer[1] = rxBuffer[1] & 0xFD; // config PWR_UP bit -> 0
  SPI_transmit(radio->SPI, txBuffer, NULL, 2, 0);

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
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 2, 2);

  txBuffer[0] = CMD_W_REG | 0x00; // write to config
  txBuffer[1] = rxBuffer[1] | 0x02; // config PWR_UP bit -> 1
  SPI_transmit(radio->SPI, txBuffer, NULL, 2, 0);

  // Tpd2stby = 1.5ms, not sure if this is necessary
  // 3ms to be safe
  vTaskDelay(pdMS_TO_TICKS(3));

  radio->state = standby;
  return DRONE_OK;
}


drone_err_t NRF_enter_RXmode(NRF_handle_t *radio) {
  if (radio->state == RXmode) {
    return DRONE_OK;
  }
  if (radio->state != standby) {
    return NRF_ILLEGAL_TRANSITION;
  }

  uint8_t txBuffer[2];
  uint8_t rxBuffer[2];

  txBuffer[0] = CMD_R_REG | 0x00; // read config
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 2, 2);

  txBuffer[0] = CMD_W_REG | 0x00; // write to config
  txBuffer[1] = rxBuffer[1] | 0x01; // config PRIM_RX bit -> 1
  SPI_transmit(radio->SPI, txBuffer, NULL, 2, 0);

  gpio_set_level(CE_pin, 1); // required for transition

  radio->state = RXmode;
  return DRONE_OK;
}


drone_err_t NRF_enter_TXmode(NRF_handle_t *radio) {
  if (radio->state == TXmode) {
    return DRONE_OK;
  }
  if (radio->state != standby) {
    return NRF_ILLEGAL_TRANSITION;
  }
  if (NRF_TX_Fifo_Empty(radio)) {
    return NRF_EMPTY_TXFIFO;
  }

  uint8_t txBuffer[2];
  uint8_t rxBuffer[2];

  txBuffer[0] = CMD_R_REG | 0x00; // read config
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 2, 2);

  txBuffer[0] = CMD_W_REG | 0x00; // write to config
  txBuffer[1] = rxBuffer[1] & 0xFE; // config PRIM_RX bit -> 0
  SPI_transmit(radio->SPI, txBuffer, NULL, 2, 0);

  gpio_set_level(CE_pin, 1); // required for transition
  //esp_rom_delay_us(150);
  
  radio->state = TXmode;
  return DRONE_OK;
}

// creates 10us pulse for sending data in TXmode, busy waits
drone_err_t NRF_pulse_TXmode(NRF_handle_t *radio) {
  if (radio->state != standby) {
    return NRF_ILLEGAL_TRANSITION;
  }
  if (NRF_TX_Fifo_Empty(radio)) {
    return NRF_EMPTY_TXFIFO;
  }

  uint8_t txBuffer[2];
  uint8_t rxBuffer[2];

  txBuffer[0] = CMD_R_REG | 0x00; // read config
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 2, 2);
  printf("config1 %02X, %02X\n", rxBuffer[0], rxBuffer[1]);

  txBuffer[0] = CMD_W_REG | 0x00; // write to config
  txBuffer[1] = rxBuffer[1] & 0xFE; // config PRIM_RX bit -> 0
  SPI_transmit(radio->SPI, txBuffer, NULL, 2, 0);

  gpio_set_level(CE_pin, 1); // required for transition
  esp_rom_delay_us(11); // must be atleast 10us
  gpio_set_level(CE_pin, 0);
  //esp_rom_delay_us(150);

  txBuffer[0] = CMD_R_REG | 0x00; // read config
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 2, 2);
  printf("config3 %02X, %02X\n", rxBuffer[0], rxBuffer[1]);

  return DRONE_OK;
}

// true if fifo empty, false otherwise
bool NRF_TX_Fifo_Empty(NRF_handle_t *radio) {
  uint8_t txBuffer[2];
  uint8_t rxBuffer[2];

  txBuffer[0] = CMD_R_REG | 0x17; // read FIFO_STATUS
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 2, 2);
  
  //printf("%X, %X", rxBuffer[0], rxBuffer[1]);

  if ((rxBuffer[1] & 0x10) == 0x10) { // TX_EMPTY == 1
    return true;
  }
  return false;
}

// true if fifo full, false otherwise
bool NRF_TX_Fifo_Full(NRF_handle_t *radio) {
  uint8_t txBuffer[2];
  uint8_t rxBuffer[2];

  txBuffer[0] = CMD_R_REG | 0x17; // read FIFO_STATUS
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 2, 2);

  if ((rxBuffer[1] & 0x20) == 0x20) { // TX_FULL == 1
    return true;
  }
  return false;
}

// true if fifo empty, false otherwise
bool NRF_RX_Fifo_Empty(NRF_handle_t *radio) {
  uint8_t txBuffer[2];
  uint8_t rxBuffer[2];

  txBuffer[0] = CMD_R_REG | 0x17; // read FIFO_STATUS
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 2, 2);

  if ((rxBuffer[1] & 0x01) == 0x01) { // RX_EMPTY == 1
    return true;
  }
  return false;
}

// true if fifo full, false otherwise
bool NRF_RX_Fifo_Full(NRF_handle_t *radio) {
  uint8_t txBuffer[2];
  uint8_t rxBuffer[2];

  txBuffer[0] = CMD_R_REG | 0x17; // read FIFO_STATUS
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 2, 2);

  if ((rxBuffer[1] & 0x02) == 0x02) { // RX_FULL == 1
    return true;
  }
  return false;
}


drone_err_t NRF_clear_MAX_RT(NRF_handle_t *radio) {
  uint8_t txBuffer[2];
  uint8_t rxBuffer[2];

  txBuffer[0] = CMD_R_REG | 0x07; // read STATUS
  txBuffer[1] = CMD_NOP;
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, 2, 2);

  txBuffer[0] = CMD_W_REG | 0x07; // write to STATUS
  txBuffer[1] = rxBuffer[1] | 0x10; // MAX_RT bit -> 1
  SPI_transmit(radio->SPI, txBuffer, NULL, 2, 0);

  return DRONE_OK;
}

// packetLength is in bytes
drone_err_t NRF_push_packet(NRF_handle_t *radio, const uint8_t *packet, size_t packetLength) {
  if (packetLength > MAX_PACKET_SIZE || packetLength == 0) {
    return NRF_INVALID_PACKET_LEN;
  }
  if (NRF_TX_Fifo_Full(radio)) {
    return NRF_FULL_TXFIFO;
  }

  uint8_t txBuffer[MAX_PACKET_SIZE + 1];
  //uint8_t rxBuffer[2];

  txBuffer[0] = CMD_W_TX_PAYLOAD; // write to TX fifo
  memcpy(&txBuffer[1], packet, packetLength);
  SPI_transmit(radio->SPI, txBuffer, NULL, (packetLength + 1), 0);

  return DRONE_OK;
}

// maybe just make do max size then caller can handle 
// the receiver should be expecting a standard packet size so I thought it made sense to pass in a packet size 
drone_err_t NRF_read_Fifo(NRF_handle_t *radio, uint8_t *packet, size_t packetLength) {
  if (packetLength > MAX_PACKET_SIZE || packetLength == 0) {
    return NRF_INVALID_PACKET_LEN;
  }
  if (radio->state != RXmode) {
    return NRF_INCORRECT_STATE;  
  }
  if (NRF_RX_Fifo_Empty(radio)) {
    return NRF_EMPTY_RXFIFO;  
  }

  uint8_t txBuffer[MAX_PACKET_SIZE + 1];
  uint8_t rxBuffer[MAX_PACKET_SIZE + 1];

  txBuffer[0] = CMD_R_RX_PAYLOAD; 
  memset(&txBuffer[1], CMD_NOP, packetLength);
  SPI_transmit(radio->SPI, txBuffer, rxBuffer, (1 + packetLength), (1 + packetLength));

  memcpy(packet, &rxBuffer[1], packetLength);

  return DRONE_OK;
}


drone_err_t NRF_flush_TX(NRF_handle_t *radio) {
  if (radio->state != TXmode) {
    return NRF_INCORRECT_STATE;
  }

  uint8_t txBuffer[1];
  //uint8_t rxBuffer[1];

  txBuffer[0] = CMD_FLUSH_TX; // flush TX
  SPI_transmit(radio->SPI, txBuffer, NULL, 1, 0);

  return DRONE_OK;
}
