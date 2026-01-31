// not a very good way to do this but some tests for copying and pasting

// ------------ nrf24l01 ---------------------


  NRF_addr_t txAddr = {0x1A, 0x1A, 0x1A, 0x1A, 0x1A};
  NRF_addr_t rxAddr = {0x50, 0x50, 0x50, 0x50, 0x50};
  NRF_channel_t channel = 50;
  NRF_handle_t radio;  
  drone_err_t err = NRF_init(&radio, rxAddr, txAddr, channel);
  assert(err == DRONE_OK);

  for (;;) {
    uint8_t txBuffer[1 + 5];
    uint8_t rxBuffer[1 + 5];

    // read config
    txBuffer[0] = CMD_R_REG | 0x00;
    txBuffer[1] = CMD_NOP;
    SPI_transmit(radio.SPI, txBuffer, rxBuffer, 2, 2);
    printf("%02X\n", rxBuffer[1]);

    txBuffer[0] = CMD_W_REG | 0x00;
    txBuffer[1] = rxBuffer[1] & 0xFD; 
    SPI_transmit(radio.SPI, txBuffer, NULL, 2, 0);

    // read config
    txBuffer[0] = CMD_R_REG | 0x00;
    txBuffer[1] = CMD_NOP;
    SPI_transmit(radio.SPI, txBuffer, rxBuffer, 2, 2);
    printf("%02X\n", rxBuffer[1]);
  }



// ------------ nrf24l01 ---------------------
