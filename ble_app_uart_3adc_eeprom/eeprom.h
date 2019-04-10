#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

void ERASE_EEPROM_data();
void write_EEPROM_data(uint8_t reg_most_byte, uint8_t reg_least_byte, uint8_t adc_H, uint8_t adc_L, bool max_bit);
void twi_init (void);
void read_EEPROM_data(uint8_t reg_most_byte, uint8_t reg_least_byte, bool max_bit, uint8_t *data);
