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

#define LOW 0
#define HIGH 1

#define digitalWrite(_pin, _value) 		nrf_gpio_pin_write(_pin, _value)
#define digitalRead(_pin) 						nrf_gpio_pin_read(_pin)
void pinMode(int _pin, pin_mode_t _mode);

#define delay(_ms) 										nrf_delay_ms(_ms)
#define delayMicroseconds(_us) 				nrf_delay_us(_us)


#define DEBUG

#ifdef DEBUG
	#include "nrf_log.h"
	#define Debug(__info,...) NRF_LOG_INFO(__info,##__VA_ARGS__)//printf()
#else
	#define Debug(__info,...)  
#endif

#ifdef USE_SPI

	void spi_init(void);
	void spi_transfer(const uint8_t *p_tx_buffer, uint8_t tx_length, uint8_t *p_rx_buffer, uint8_t rx_length); //rx length begins from the first byte!

#endif

#ifdef USE_IIC

	void iic_init(void);
	void iic_send(uint8_t addr, uint8_t *buffer, uint8_t len, bool no_stop);
	void iic_read(uint8_t addr, uint8_t *buffer, uint8_t len);

#endif

#endif
