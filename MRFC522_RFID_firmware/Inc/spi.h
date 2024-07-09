#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include "stm32f4xx.h"

void spi_init(void);
int8_t spi_transmit(uint8_t *data, uint32_t size);
int8_t spi_receive(uint8_t *data, uint32_t size);
int8_t spi_receive_byte(void);

#endif /* SPI_H_ */
