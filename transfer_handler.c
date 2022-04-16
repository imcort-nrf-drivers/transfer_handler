#include "transfer_handler.h"

#ifdef SOFTDEVICE_PRESENT 
#include "nrf_pwr_mgmt.h"
#endif

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

#ifdef USE_INTERRUPT

#include "nrf_drv_gpiote.h"

bool gpiote_init = false;

void attachInterrupt(int _pin, void* _func, int_mode_t _mode)
{
    ret_code_t err_code;
    
    if(!gpiote_init)
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }
    
    nrf_drv_gpiote_in_config_t in_config = 
    {                                                   \
        .sense = NRF_GPIOTE_POLARITY_HITOLO,            \
        .pull = NRF_GPIO_PIN_NOPULL,                    \
        .is_watcher = false,                            \
        .hi_accuracy = true,                         \
        .skip_gpio_setup = false,                       \
    };  
    
    switch(_mode)
    {
        case LOW:
        case FALLING:
            in_config.sense = NRF_GPIOTE_POLARITY_HITOLO;
            in_config.pull = NRF_GPIO_PIN_PULLUP;
            break;
        case RISING:
            in_config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
            in_config.pull = NRF_GPIO_PIN_PULLDOWN;
            break;
        case CHANGE:
            in_config.sense = NRF_GPIOTE_POLARITY_TOGGLE;
            break;
    
    }
    
    err_code = nrf_drv_gpiote_in_init(_pin, &in_config, _func);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(_pin, true);
}
#endif

#ifdef USE_PWM

#include "app_pwm.h"

APP_PWM_INSTANCE(PWM1,1);

void pwm_init(int pin)
{
    ret_code_t err_code;
    /* 2-channel PWM, 200Hz, output on DK LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(5000L, pin);

    /* Switch the polarity of the second channel. */
    pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;

    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM1, &pwm1_cfg, NULL);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);

}

void pwm_set(uint16_t duty)
{
    
    app_pwm_channel_duty_set(&PWM1, 0, duty);

}

#endif

#ifdef USE_SPI

#include "nrf_drv_spi.h"
#include <string.h>

#define SPI_INSTANCE  1 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done = false;
//static uint8_t spi_buf[255];

bool spi_initialized = false;

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
    if(!spi_initialized)
	{
        APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
    }
    
    spi_initialized = true;
	
}

void spi_transfer(const uint8_t *p_tx_buffer, uint8_t tx_length, uint8_t *p_rx_buffer, uint8_t rx_length)
{
    //memcpy(spi_buf, p_tx_buffer, tx_length);
    spi_xfer_done = false;
    nrf_drv_spi_transfer(&spi, p_tx_buffer, tx_length, p_rx_buffer, rx_length);
    while(!spi_xfer_done) 
    #ifdef SOFTDEVICE_PRESENT 
        nrf_pwr_mgmt_run();
    #else
        __WFE();
    #endif 
    //memcpy(p_rx_buffer, (spi_buf + tx_length), rx_length);
	
}

void spi_send(const uint8_t *p_tx_buffer, int32_t tx_length)
{

    int32_t shift = 0;
    
    while (tx_length > 0)
    {
        
        spi_xfer_done = false;
        nrf_drv_spi_transfer(&spi, p_tx_buffer + shift, tx_length > 200 ? 200 : tx_length, NULL, 0);
        while(!spi_xfer_done) 
        #ifdef SOFTDEVICE_PRESENT 
            nrf_pwr_mgmt_run();
        #else
            __WFE();
        #endif 
        tx_length = tx_length - 200;
        shift = shift + 200;
        
    }
    
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
				.frequency = NRF_DRV_TWI_FREQ_100K,
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
	while (twi_xfer_done == false) 
    #ifdef SOFTDEVICE_PRESENT 
        nrf_pwr_mgmt_run();
    #else
        __WFE();
    #endif    
    

}

void iic_read(uint8_t addr, uint8_t *buffer, uint8_t len)
{
	
	ret_code_t err_code;
	twi_xfer_done = false;
	err_code = nrf_drv_twi_rx(&m_twi, addr, buffer, len);
	APP_ERROR_CHECK(err_code);
	while (twi_xfer_done == false) 
    #ifdef SOFTDEVICE_PRESENT 
        nrf_pwr_mgmt_run();
    #else
        __WFE();
    #endif 

}

#endif


#ifdef USE_UART

#define UART_BUF_SIZE 128   

bool uart_initialized = false;

void uart_event_handle(app_uart_evt_t * p_event);

void uart_init(void)
{
    
    if(!uart_initialized)
    {
        ret_code_t err_code;
        app_uart_comm_params_t const uart_comm_params =
        {
                .rx_pin_no    = UART_SENSOR_TX,
                .tx_pin_no    = UART_SENSOR_RX,
                .rts_pin_no   = UART_PIN_DISCONNECTED,
                .cts_pin_no   = UART_PIN_DISCONNECTED,
                .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
                .use_parity   = false,
                .baud_rate    = NRF_UART_BAUDRATE_9600
        };
        
        APP_UART_FIFO_INIT(&uart_comm_params,
                   UART_BUF_SIZE,
                   UART_BUF_SIZE,
                   uart_event_handle,
                   APP_IRQ_PRIORITY_LOWEST,
                   err_code);
        
        APP_ERROR_CHECK(err_code);
        
        Debug("UART INIT OK");
        
    }
    
    uart_initialized = true;
    
}

#endif


