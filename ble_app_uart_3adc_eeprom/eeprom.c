#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/* Common addresses definition for EEPROM. */
#define EEPROM_ADDR_S          (0xACU >> 1)			//read memory,address small than 512bit
#define EEPROM_ADDR_B          (0xAEU >> 1)			//read memory,address big than 512bit

#define SCL_PIN 1
#define SDA_PIN 2
#define WC_PIN  3 

/* Indicates if operation on TWI has ended. */
static volatile bool twi_rx_done = false;
static volatile bool twi_tx_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);

/**
 * @brief Function for erasing EEPROM data.
 */
void ERASE_EEPROM_data()
{//131072
	int i=0;
	uint8_t EEPROM_addr;
	uint8_t reg_byte_H=0x00;
	bool max_bit=false;
    ret_code_t err_code;
    uint8_t reg[255];
	memset(reg, 0xFF, 255);
	
	nrf_gpio_pin_clear(WC_PIN);//unlock eeprom
	for(i=0;i<512;i++)
	{
		twi_tx_done=false;
		reg[0]=reg_byte_H;
		reg[1]=0x00;

		/*determine device_address by max bit */
		if(max_bit)
			EEPROM_addr=EEPROM_ADDR_B;
		else
			EEPROM_addr=EEPROM_ADDR_S;
		
		/* Writing to pointer byte. */
		err_code = nrf_drv_twi_tx(&m_twi, EEPROM_addr, reg, 255, false);
		APP_ERROR_CHECK(err_code);
		while (twi_tx_done == false);
		if(reg_byte_H==0xFF)//upper_counter for eeprom_address
		{
			max_bit=true;
			reg_byte_H=0x00;
		}
		else
			reg_byte_H+=0x01;
		
		nrf_delay_ms(5);
	}
	nrf_gpio_pin_set(WC_PIN);//lock eeprom
}

/**
 * @brief Function for writing EEPROM data.
 *
 * @param[in] reg_byte_H    EEPROM reg address high byte.
 * @param[in] reg_byte_L    EEPROM reg address low byte.
 * @param[in] adc_H         ADC value high byte.
 * @param[in] adc_L         ADC value low byte.
 * @param[in] max_bit       EEPROM reg address max bit, if EEPROM address more then 512Kb, max_bit=ture.
 */
void write_EEPROM_data(uint8_t reg_byte_H, uint8_t reg_byte_L, uint8_t adc_H, uint8_t adc_L, bool max_bit)
{
	uint8_t EEPROM_addr;
    ret_code_t err_code;
	twi_tx_done=false;
    uint8_t reg[4] = {reg_byte_H, reg_byte_L, adc_H, adc_L};
	
	nrf_gpio_pin_clear(WC_PIN);//unlock eeprom
	/*determine device_address by max bit */
	if(max_bit)
		EEPROM_addr=EEPROM_ADDR_B;
	else
		EEPROM_addr=EEPROM_ADDR_S;
	
    /* Writing to pointer byte. */
    err_code = nrf_drv_twi_tx(&m_twi, EEPROM_addr, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
	while (twi_tx_done == false);
	nrf_gpio_pin_clear(WC_PIN);//lock eeprom
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
				//NRF_LOG_INFO("Rx done!\r\n");
				twi_rx_done = true;
            }
			
			if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX)
            {
				//NRF_LOG_INFO("Tx done!\r\n");
				twi_tx_done = true;
            }
            break;
        default:
            break;
    }
}

/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

	nrf_gpio_cfg_output(WC_PIN);  //set Write_Control output
	
    const nrf_drv_twi_config_t twi_eeprom_config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_eeprom_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief Function for reading data from EEPROM.
 *
 * @param[in]  reg_byte_H    EEPROM reg address high byte.
 * @param[in]  reg_byte_L    EEPROM reg address low byte.
 * @param[in]  max_bit       EEPROM reg address max bit, if EEPROM address more then 512byte, max_bit=ture.
 * @param[out] data          Read ADC value from EEPROM, then return the data_array.
 */
void read_EEPROM_data(uint8_t reg_byte_H, uint8_t reg_byte_L, bool max_bit, uint8_t *data)
{
	int i=0;
	uint8_t EEPROM_addr;
	ret_code_t err_code;
    twi_rx_done = false;
    uint8_t reg[2] = {reg_byte_H, reg_byte_L};

	/*determine device_address by max bit */
	if(max_bit)
		EEPROM_addr=EEPROM_ADDR_B;
	else
		EEPROM_addr=EEPROM_ADDR_S;

    /* Writing to pointer byte. */
    err_code = nrf_drv_twi_tx(&m_twi, EEPROM_addr, reg, sizeof(reg), true);//no_stop
    APP_ERROR_CHECK(err_code);
	nrf_delay_us(500);
	/* reading at pointer byte. */
	err_code = nrf_drv_twi_rx(&m_twi, EEPROM_addr, &data[0], 253);
	APP_ERROR_CHECK(err_code);
	while (twi_rx_done == false);

	/*for(i=0;i<253;i++)//printf page data
	{
		printf(" %d ",data[i]);
		nrf_delay_ms(5);
	}*/
}

/** @} */
