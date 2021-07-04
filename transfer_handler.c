#include "transfer_handler.h"
#include "nrf_pwr_mgmt.h"

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

#define SPI_INSTANCE  1 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done = false;
//static uint8_t spi_buf[255];

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
    .mode         = NRF_DRV_SPI_MODE_3,                      \
    .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         \
};

void spi_init(void)
{

    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
	
}

void spi_transfer(const uint8_t *p_tx_buffer, uint8_t tx_length, uint8_t *p_rx_buffer, uint8_t rx_length)
{
    //memcpy(spi_buf, p_tx_buffer, tx_length);
    spi_xfer_done = false;
    nrf_drv_spi_transfer(&spi, p_tx_buffer, tx_length, p_rx_buffer, rx_length);
    while(!spi_xfer_done) nrf_pwr_mgmt_run();
    //memcpy(p_rx_buffer, (spi_buf + tx_length), rx_length);
	
}

void spi_send(const uint8_t *p_tx_buffer, uint8_t tx_length)
{
    Debug("SPI Send %d", tx_length);
    spi_xfer_done = false;
    nrf_drv_spi_transfer(&spi, p_tx_buffer, tx_length > 255 ? 255 : tx_length, NULL, 0);
    while(!spi_xfer_done) nrf_pwr_mgmt_run();
    
    if(tx_length > 255)
        spi_send(p_tx_buffer + 255, tx_length - 255);
    
}


#endif

#ifdef USE_IIC

#include "nrf_drv_twi.h"

bool iic_initialized = false;

/* Indicates if operation on TWI has ended. */
static volatile bool twi_xfer_done = false;
#define TWI_INSTANCE_ID     0
/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

//static uint8_t iic_sendbuf[20];
//static uint8_t iic_recvbuf[20];

///**
// * @brief TWI events handler.
// */
static void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
{
//	ret_code_t ret;
	switch (p_event->type)
	{
		case NRF_DRV_TWI_EVT_DONE:
			twi_xfer_done = true;
			break;
		default:
			
//			llength = sprintf(twi_err_sendbuf, "TWI ERR Addr:%x, Err:%d", p_event->xfer_desc.address, p_event->type);
//			do{
//				ret = ble_data_send((uint8_t*)twi_err_sendbuf, llength);
//			} while (ret != NRF_SUCCESS) ;
		
			Debug("IIC ERROR,%x,%d", p_event->xfer_desc.address, p_event->type);
		
			twi_xfer_done = true;
			
			break;
	}
}

void iic_init(void)
{
	ret_code_t err_code;
	
	if(!iic_initialized)
	{
		const nrf_drv_twi_config_t twi_afe_config = {
				.scl = IIC_SCL,
				.sda = IIC_SDA,
				.frequency = NRF_DRV_TWI_FREQ_400K,
				.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
				.clear_bus_init = true
			};

			err_code = nrf_drv_twi_init(&m_twi, &twi_afe_config, twi_handler, NULL);
			APP_ERROR_CHECK(err_code);

			nrf_drv_twi_enable(&m_twi);
	}
	
	iic_initialized = true;
	
}

void iic_send(uint8_t addr, uint8_t *buffer, uint8_t len, bool no_stop)
{
	
	ret_code_t err_code;
	twi_xfer_done = false;
	err_code = nrf_drv_twi_tx(&m_twi, addr, buffer, len, no_stop);
	APP_ERROR_CHECK(err_code);
	while (twi_xfer_done == false) nrf_pwr_mgmt_run();

}

void iic_read(uint8_t addr, uint8_t *buffer, uint8_t len)
{
	
	ret_code_t err_code;
	twi_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, addr, buffer, len);
	APP_ERROR_CHECK(err_code);
	while (twi_xfer_done == false) nrf_pwr_mgmt_run();

}

#endif


#ifdef USE_UART

#include "nrf_uart.h"
#include "app_uart.h"

static uint8_t     rx_buf[128];                                                  
static uint8_t     tx_buf[128]; 

void uart_event_handle(app_uart_evt_t *p_event);

void uart_init(void)
{
    ret_code_t err_code;
    app_uart_comm_params_t const bmd101_comm_params =
	{
			.rx_pin_no    = BMD101_TX,
			.tx_pin_no    = UART_PIN_DISCONNECTED,
			.rts_pin_no   = UART_PIN_DISCONNECTED,
			.cts_pin_no   = UART_PIN_DISCONNECTED,
			.flow_control = APP_UART_FLOW_CONTROL_DISABLED,
			.use_parity   = false,
			.baud_rate    = NRF_UART_BAUDRATE_57600
	};
	
	app_uart_buffers_t buffers = {
	
			.rx_buf      = rx_buf,                                                            
			.rx_buf_size = sizeof (rx_buf),                                                
			.tx_buf      = tx_buf,                 
			.tx_buf_size = sizeof (tx_buf)
	
	};
	
	err_code = app_uart_init(&bmd101_comm_params, &buffers, uart_event_handle, APP_IRQ_PRIORITY_LOWEST);
	
	APP_ERROR_CHECK(err_code);


}

#endif


