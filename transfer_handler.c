#include "transfer_handler.h"

void pinMode(int _pin, pin_mode_t _mode)
{
		switch(_mode)
		{
			case OUTPUT:
				nrf_gpio_cfg_output(_pin);
				break;
			case INPUT:
				nrf_gpio_cfg_input(_pin, NRF_GPIO_PIN_NOPULL);
				break;
			case INPUT_PULLUP:
				nrf_gpio_cfg_input(_pin, NRF_GPIO_PIN_PULLUP);
				break;
			case INPUT_PULLDOWN:
				nrf_gpio_cfg_input(_pin, NRF_GPIO_PIN_PULLDOWN);
				break;	
		}
}

#ifdef USE_SPI

#include "nrf_drv_spi.h"
#include <string.h>

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done = false;
static uint8_t spi_buf[255];

static void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
    spi_xfer_done = true;
}

static nrf_drv_spi_config_t spi_config = {                                                            \
    .sck_pin      = SPI_PIN_SCK,                \
    .mosi_pin     = SPI_PIN_MOSI,                \
    .miso_pin     = SPI_PIN_MISO,                \
    .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,                \
    .irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,         \
    .orc          = 0xFF,                                    \
    .frequency    = NRF_DRV_SPI_FREQ_8M,                     \
    .mode         = NRF_DRV_SPI_MODE_0,                      \
    .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         \
};

void spi_init(void)
{

		APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
	
}

void spi_transfer(const uint8_t *p_tx_buffer, uint8_t tx_length, uint8_t *p_rx_buffer, uint8_t rx_length)
{
		memcpy(spi_buf, p_tx_buffer, tx_length);
		spi_xfer_done = false;
		nrf_drv_spi_transfer(&spi, spi_buf, tx_length, spi_buf, tx_length + rx_length);
		while(!spi_xfer_done) __WFE();
		memcpy(p_rx_buffer, (spi_buf + tx_length), rx_length);
}


#endif

////#include "nrf_pwr_mgmt.h"
//#include "custom_board.h"

///* Indicates if operation on TWI has ended. */
//volatile bool m_xfer_done = false;

///* TWI instance. */
//const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

//static uint8_t iic_sendbuf[20];
//static uint8_t iic_recvbuf[20];

//ret_code_t ble_data_send(uint8_t* sendbuf, uint16_t llength);

///**
// * @brief TWI events handler.
// */
//void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
//{
//	ret_code_t ret;
//	switch (p_event->type)
//	{
//		case NRF_DRV_TWI_EVT_DONE:
//			m_xfer_done = true;
//			break;
//		default:
//			
////			llength = sprintf(twi_err_sendbuf, "TWI ERR Addr:%x, Err:%d", p_event->xfer_desc.address, p_event->type);
////			do{
////				ret = ble_data_send((uint8_t*)twi_err_sendbuf, llength);
////			} while (ret != NRF_SUCCESS) ;
//		
//			NRF_LOG_DEBUG("IIC ERROR,%x,%d", p_event->xfer_desc.address, p_event->type);
//			NRF_LOG_FLUSH();
//		
//			m_xfer_done = true;
//			
//			break;
//	}
//}

//void twi_init(void)
//{
//	ret_code_t err_code;

//	const nrf_drv_twi_config_t twi_afe_config = {
//		.scl = BOARD_SCL_PIN,
//		.sda = BOARD_SDA_PIN,
//		.frequency = NRF_DRV_TWI_FREQ_400K,
//		.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
//		.clear_bus_init = true
//	};

//	err_code = nrf_drv_twi_init(&m_twi, &twi_afe_config, twi_handler, NULL);
//	APP_ERROR_CHECK(err_code);

//	nrf_drv_twi_enable(&m_twi);
//}

//void twi_readRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len){
//	
//	ret_code_t err_code;
//	
//	iic_sendbuf[0] = reg;
//	
//	m_xfer_done = false;
//	err_code = nrf_drv_twi_tx(&m_twi, addr, iic_sendbuf, 1, false);
//	APP_ERROR_CHECK(err_code);
//	while (m_xfer_done == false) ;//nrf_pwr_mgmt_run();
//	
//	m_xfer_done = false;
//	err_code = nrf_drv_twi_rx(&m_twi, addr, iic_recvbuf, len);
//	APP_ERROR_CHECK(err_code);
//	while (m_xfer_done == false) ;//nrf_pwr_mgmt_run();
//	
//	memcpy(buffer, iic_recvbuf, len);

//}

//void twi_writeRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len){
//	
//	ret_code_t err_code;
//	
//	iic_sendbuf[0] = reg;
//	memcpy(iic_sendbuf + 1, buffer, len);
//	
//	m_xfer_done = false;
//	err_code = nrf_drv_twi_tx(&m_twi, addr, iic_sendbuf, len + 1, false);
//	APP_ERROR_CHECK(err_code);
//	while (m_xfer_done == false) ;//nrf_pwr_mgmt_run();

//}

