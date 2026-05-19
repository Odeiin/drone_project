#pragma once
#include "drone_err.h"

#define NRF_ERR_BASE            0x1000

#define SPI_ERROR               (NRF_ERR_BASE + 1)
#define NRF_INVALID_CHANNEL     (NRF_ERR_BASE + 2)
#define NRF_ILLEGAL_TRANSITION  (NRF_ERR_BASE + 3) // not really illegal but not recommended
#define NRF_EMPTY_TXFIFO        (NRF_ERR_BASE + 4)
#define NRF_FULL_TXFIFO         (NRF_ERR_BASE + 5)
#define NRF_EMPTY_RXFIFO        (NRF_ERR_BASE + 6)
#define NRF_FULL_RXFIFO         (NRF_ERR_BASE + 7)
#define NRF_INVALID_PACKET_LEN  (NRF_ERR_BASE + 8)
#define NRF_INCORRECT_STATE     (NRF_ERR_BASE + 9)
#define NRF_TX_TIMEOUT          (NRF_ERR_BASE + 10)
#define NRF_MAX_RT              (NRF_ERR_BASE + 11)