#ifndef __TRANSFER_HANDLER_H_
#define __TRANSFER_HANDLER_H_

#include "custom_board.h"
#include <stdint.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"

typedef enum
{
		OUTPUT,
    INPUT,
    INPUT_PULLUP,
    INPUT_PULLDOWN
} pin_mode_t;

#define digitalWrite(_pin, _value) 		nrf_gpio_pin_write(_pin, _value)
#define digitalRead(_pin) 						nrf_gpio_pin_read(_pin)
void pinMode(int _pin, pin_mode_t _mode);

#define delay(_ms) nrf_delay_ms(_ms)


#define DEBUG

#ifdef DEBUG
	#include "nrf_log.h"
	#define Debug(__info,...) NRF_LOG_INFO("Debug: " __info,##__VA_ARGS__)//printf()
#else
	#define Debug(__info,...)  
#endif

#ifdef USE_SPI

	void spi_init(void);
	void spi_transfer(const uint8_t *p_tx_buffer, uint8_t tx_length, uint8_t *p_rx_buffer, uint8_t rx_length);

#endif

#ifdef USE_IIC

	void twi_init(void);
	void twi_readRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len);
	void twi_writeRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len);

#endif

#endif
