#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
#include "nrf_drv_adc.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "eeprom.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

#define BUTTON_INPUT                    12    // Digital pin 12
int bt_count=1;

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        while (app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while (app_uart_put('\r') != NRF_SUCCESS);
    while (app_uart_put('\n') != NRF_SUCCESS);
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);

}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' i.e '\r\n' (hex 0x0D) or if the string has reached a length of
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    //advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;//have timeout
	advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;//have not timeout

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


bool adc_start_flag=false;
/**
 * @brief pin interrupt function
 */
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	adc_start_flag = ~adc_start_flag;
	if(adc_start_flag)
	{
		printf("adc_start\n");
	}
	nrf_delay_ms(500);
}


/**
 *@brief Function for making the ADC start a battery level conversion.
 */
 uint16_t adc_start(int adc_number)
{
    uint32_t err_code;
    uint8_t times = 0 ;
    uint16_t adc_result = 0;
	
    // Configure ADC
	if(adc_number==5)
	{
		NRF_ADC->CONFIG     = (ADC_CONFIG_RES_10bit                        << ADC_CONFIG_RES_Pos)     |						//10bit adc_conversion
							  (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)  |					//simulate pin input 1/3v
							  (ADC_CONFIG_REFSEL_VBG    << ADC_CONFIG_REFSEL_Pos)  |										//refer 1.2 voltage
							  (ADC_CONFIG_PSEL_AnalogInput5                  << ADC_CONFIG_PSEL_Pos)    |					//select adc 
							  (ADC_CONFIG_EXTREFSEL_None                  << ADC_CONFIG_EXTREFSEL_Pos);						//without external refer voltage
	}
	else if(adc_number==6)
	{
		NRF_ADC->CONFIG     = (ADC_CONFIG_RES_10bit                        << ADC_CONFIG_RES_Pos)     |						//10bit adc_conversion
							  (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)  |					//simulate pin input 1/3v
							  (ADC_CONFIG_REFSEL_VBG    << ADC_CONFIG_REFSEL_Pos)  |										//refer 1.2 voltage
							  (ADC_CONFIG_PSEL_AnalogInput6                  << ADC_CONFIG_PSEL_Pos)    |					//select adc 
							  (ADC_CONFIG_EXTREFSEL_None                  << ADC_CONFIG_EXTREFSEL_Pos);						//without external refer voltage
	}
	else if(adc_number==7)
	{
		NRF_ADC->CONFIG     = (ADC_CONFIG_RES_10bit                        << ADC_CONFIG_RES_Pos)     |						//10bit adc_conversion
							  (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos)  |					//simulate pin input 1/3v
							  (ADC_CONFIG_REFSEL_VBG    << ADC_CONFIG_REFSEL_Pos)  |										//refer 1.2 voltage
							  (ADC_CONFIG_PSEL_AnalogInput7                  << ADC_CONFIG_PSEL_Pos)    |					//select adc 
							  (ADC_CONFIG_EXTREFSEL_None                  << ADC_CONFIG_EXTREFSEL_Pos);						//without external refer voltage
	}
	else
		return 0;
	
	NRF_ADC->INTENSET   = ADC_INTENSET_END_Msk;
    NRF_ADC->EVENTS_END = 0;
    NRF_ADC->ENABLE     = ADC_ENABLE_ENABLE_Enabled;
 
    // Enable ADC interrupt
    err_code = sd_nvic_ClearPendingIRQ(ADC_IRQn);
    APP_ERROR_CHECK(err_code);
 
    err_code = sd_nvic_SetPriority(ADC_IRQn, 3);
    APP_ERROR_CHECK(err_code);
                     
  	err_code =sd_nvic_DisableIRQ(ADC_IRQn);
   // err_code = sd_nvic_EnableIRQ(ADC_IRQn);
    APP_ERROR_CHECK(err_code);
 
    NRF_ADC->EVENTS_END  = 0;    // Stop any running conversions.
    NRF_ADC->TASKS_START = 1;
	while (NRF_ADC->EVENTS_END == 0 || times >200 )
    {
		times ++ ;
	}     
	if(times > 200)
	{
		return 0;
	}
	else
	{
		NRF_ADC->EVENTS_END     = 0;
		adc_result              = NRF_ADC->RESULT;
		NRF_ADC->TASKS_STOP     = 1;
	}
    return adc_result;   
}

/**
 * @brief set adc_pin nopull input. 
 */
static void adc_pin_config(void)
{
	nrf_gpio_cfg_input(4, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(5, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(6, NRF_GPIO_PIN_NOPULL);
}

/**
 * @brief Function for configuring: BUTTON_INPUT pin for input,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;

	//nrf_gpio_cfg_sense_input(BUTTON_INPUT,NRF_GPIO_PIN_PULLDOWN,NRF_GPIO_PIN_SENSE_HIGH);  //if want to wake up on deep sleep mode
    err_code = nrf_drv_gpiote_in_init(BUTTON_INPUT, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BUTTON_INPUT, true);
}

/**
 * @brief Function button change active mode.
 */
void button_active(void)
{
	int i=0;
	
	if(nrf_gpio_pin_read(12))
	{
		nrf_gpio_pin_clear(8);
		for(i=0;i<2000;i++)
		{
			if(nrf_gpio_pin_read(12))
			{
				bt_count++;
				if(bt_count==5)
					bt_count=1;
				
				nrf_gpio_pin_clear(14);
				nrf_gpio_pin_clear(13);
				nrf_gpio_pin_clear(10);
				nrf_gpio_pin_clear(7);
				
				switch (bt_count)
				{
					case 1:
						nrf_gpio_pin_set(14);break;
					case 2:
						nrf_gpio_pin_set(13);break;
					case 3:
						nrf_gpio_pin_set(10);break;
					case 4:
						nrf_gpio_pin_set(7);break;
				}
				nrf_delay_ms(500);
				i=0;
			}
			nrf_delay_ms(1);
		}
		nrf_gpio_pin_set(8);
	}
}

void led_gpio_init(void)
{
	nrf_gpio_cfg_output(14);//standby mode
	nrf_gpio_cfg_output(13);//earse mode
	nrf_gpio_cfg_output(10);//write mode
	nrf_gpio_cfg_output(7); //read mode
	nrf_gpio_cfg_output(8); //work led
	
	nrf_gpio_pin_set(14);
	nrf_gpio_pin_clear(13);
	nrf_gpio_pin_clear(10);
	nrf_gpio_pin_clear(7);
	nrf_gpio_pin_set(8);
}

void write_mode(void)
{
	uint8_t adc_string[50]="";
	uint32_t adc_value=0;
	uint8_t adc_H,adc_L;
	uint8_t reg_byte_H=0x00,reg_byte_L=0x00;
	bool max_bit=false;
	
	max_bit=false;reg_byte_H=0x00,reg_byte_L=0x00;
	while(1)
	{
		if(nrf_gpio_pin_read(12))
			break;
		
		//////////////////////////////////////////////////
		adc_value=adc_start(5);//manually trigger adc
		adc_H=adc_value/256;
		adc_L=adc_value%256;
		printf("ADC5:%d \r\n",adc_value);
		sprintf(adc_string,"1:%d\n",adc_value);
		write_EEPROM_data(reg_byte_H,reg_byte_L,adc_H,adc_L,max_bit);//write data to eeprom
		if(reg_byte_H==0xFF)//upper_counter for eeprom_address
		{
			if(max_bit==true)
				break;
			max_bit=true;
			reg_byte_H=0x00;
			reg_byte_L=0x00;
		}
		else
		{
			if(reg_byte_L==0xFE)
			{
				reg_byte_H+=0x01;
				reg_byte_L=0x00;
			}
			else
			{
				reg_byte_L+=0x02;
			}
		}
		nrf_delay_ms(5);
		//////////////////////////////////////////////////
		adc_value=adc_start(6);//manually trigger adc
		adc_H=adc_value/256;
		adc_L=adc_value%256;
		printf("ADC6:%d \r\n",adc_value);
		sprintf(adc_string,"%s2:%d\n",adc_string,adc_value);
		write_EEPROM_data(reg_byte_H,reg_byte_L,adc_H,adc_L,max_bit);//write data to eeprom
		if(reg_byte_H==0xFF)//upper_counter for eeprom_address
		{
			if(max_bit==true)
				break;
			max_bit=true;
			reg_byte_H=0x00;
			reg_byte_L=0x00;
		}
		else
		{
			if(reg_byte_L==0xFE)
			{
				reg_byte_H+=0x01;
				reg_byte_L=0x00;
			}
			else
			{
				reg_byte_L+=0x02;
			}
		}
		nrf_delay_ms(5);
		//////////////////////////////////////////////////
		adc_value=adc_start(7);//manually trigger adc
		adc_H=adc_value/256;
		adc_L=adc_value%256;
		printf("ADC7:%d \r\n",adc_value);
		sprintf(adc_string,"%s3:%d\n",adc_string,adc_value);
		write_EEPROM_data(reg_byte_H,reg_byte_L,adc_H,adc_L,max_bit);//write data to eeprom
		if(reg_byte_H==0xFF)//upper_counter for eeprom_address
		{
			if(max_bit==true)
				break;
			max_bit=true;
			reg_byte_H=0x00;
			reg_byte_L=0x00;
		}
		else
		{
			if(reg_byte_L==0xFE)
			{
				reg_byte_H+=0x01;
				reg_byte_L=0x00;
			}
			else
			{
				reg_byte_L+=0x02;
			}
		}
		nrf_delay_ms(5);
		//////////////////////////////////////////////////
		ble_nus_string_send(&m_nus, adc_string, 20);//send data through ble uart
		printf("%s\r\n",adc_string);			
		//nrf_delay_ms(50);
    }
}

void read_mode(void)
{
	int i;
	uint8_t reg_byte_H=0x00;
	bool max_bit=false;
	uint8_t m_data[255]; //Buffer for read EEPROM data. 
	
	max_bit=false;reg_byte_H=0x00;
	
	while(1)
	{
		if(nrf_gpio_pin_read(12))
			break;
		
		read_EEPROM_data(reg_byte_H,0x00,max_bit,m_data);//read eeprom data in specific register
		for(i=0;i<12;i++)//send eeprom_data, max_size of ble_send_data is 20byte 
		{
			ble_nus_string_send(&m_nus, &m_data[i*20], 20);//send data through ble uart
			nrf_delay_ms(10);
		}
		if(reg_byte_H==0xFF)//upper_counter for eeprom_address
		{
			if(max_bit==true)
				break;
			max_bit=true;
			reg_byte_H=0x00;
		}
		else
		{
			reg_byte_H+=0x01;
		}
	}
}

/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
	led_gpio_init();//led_pin config
	adc_pin_config();//adc_pin config
	nrf_gpio_cfg_input(12,NRF_GPIO_PIN_PULLDOWN);  //button config p0.12
	//gpio_init();//p0.12 input interrupt initial
	
	// eeprom init
	twi_init();//i2c init
	
    // Initialize ble.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    uart_init();
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    printf("\r\nUART Start!\r\n");
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

	/*max_bit=false;reg_byte_H=0x00,reg_byte_L=0x00;
	while(1)
	{
		read_EEPROM_data(0x7a,0x00,max_bit,m_data);//read eeprom data in specific register
		printf("\r\n\r\n0x00\r\n\r\n");
		nrf_delay_ms(500);
		read_EEPROM_data(0x80,0x00,max_bit,m_data);//read eeprom data in specific register
		printf("\r\n\r\n0x55\r\n\r\n");
		nrf_delay_ms(500);
		read_EEPROM_data(0x88,0x00,max_bit,m_data);//read eeprom data in specific register
		printf("\r\n\r\n0xaa\r\n\r\n");
		nrf_delay_ms(500);
	}*/	
	
	while(1)
	{
		button_active();
		switch(bt_count)
		{
			case 1:
				printf("\r\nStandby mode\r\n");
				nrf_gpio_pin_set(14);
				power_manage();//low power state
				break;
			case 2:
				printf("\r\nwErase mode\r\n");
				ERASE_EEPROM_data();
				nrf_gpio_pin_clear(13);
				bt_count=0;
				break;
			case 3:
				printf("\r\nWrite mode\r\n");
				write_mode();
				nrf_gpio_pin_clear(10);
				bt_count=0;
				break;
			case 4:
				printf("\r\nRead mode\r\n");
				read_mode();
				nrf_gpio_pin_clear(7);
				bt_count=0;
				break;
		}
		nrf_delay_ms(100);
	}
}
