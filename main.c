/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This example can be a central for up to 8 peripherals.
 * The peripheral is called ble_app_blinky and can be found in the ble_peripheral
 * folder.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_lbs_c.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "ble_lbs.h"
#include "nrf_ble_qwr.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

static char const m_target_periph_name[] = "Nordic_Blinky_TMR1";                     /**< Name of the device to try to connect to. This name is searched for in the scanning report data. */

#define DEVICE_NAME                     "Nordic_Blinky_R1"                       /**< Name of advertising */
#define LINK_TOTAL                      NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + \
                                        NRF_SDH_BLE_CENTRAL_LINK_COUNT

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_ADV_TIMEOUT_IN_SECONDS      BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

// #define ADVERTISING_LED                 BSP_BOARD_LED_3                         /**< Is on when device is advertising. */
// #define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
// #define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
// #define LEDBUTTON_BUTTON                BSP_BUTTON_0

#define APP_BLE_CONN_CFG_TAG      1                                     /**< Tag that refers to the BLE stack configuration that is set with @ref sd_ble_cfg_set. The default tag is @ref APP_BLE_CONN_CFG_TAG. */
#define APP_BLE_OBSERVER_PRIO     3                                     /**< BLE observer priority of the application. There is no need to modify this value. */

// #define CENTRAL_SCANNING_LED      BSP_BOARD_LED_0
// #define CENTRAL_CONNECTED_LED     BSP_BOARD_LED_1
#define LEDBUTTON_LED             BSP_BOARD_LED_0                       /**< LED to indicate a change of state of the Button characteristic on the peer. */

#define LEDBUTTON_BUTTON          BSP_BUTTON_0                          /**< Button that writes to the LED characteristic of the peer. */
#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(50)                   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_LBS_C_ARRAY_DEF(m_lbs_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);           /**< LED button client instances. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */

BLE_LBS_DEF(m_lbs);
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT);                          /**< Context for the Queued Write module.*/
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

APP_TIMER_DEF(m_scan_timer);
APP_TIMER_DEF(m_adv_timer);


/**
 * LED1 : turn on/off when first Peripheral device is connected or is disconnected
 * LED2 : turn on/off when second Peripheral device is connected or is disconnected
 * LED3 : turn on/off when first Central device is connected or is disconnected
 * LED4 : turn on/off when second Central device is connected or is disconnected
 * Button1 : to send an event to first Peripheral device
 * Button2 : to send an event to second Peripheral device
 * Button3 : to send an event to first Central device
 * Button4 : to send an event to second Central device
 * Blink LED1 : when push and then release the Button1 on first Peripheral device
 * Blink LED2 : when push and then release the Button1 on second Peripheral device
 * Turn off LED3 : when push On-Button on first Central device
 * Turn on  LED3 : when push Off-Button on first Central device
 * Turn off LED4 : when push On-Button on second Central device
 * Turn on  LED4 : when push Off-Button on second Central device
*/ 
// Define BSP LEDs and Buttons
#define FIRST_PERI_CONNECTED            BSP_BOARD_LED_0
#define SECOND_PERI_CONNECTED           BSP_BOARD_LED_1
#define FIRST_CENT_CONNECTED            BSP_BOARD_LED_2
#define SECOND_CENT_CONNECTED           BSP_BOARD_LED_3
#define SEND_PERI_FIRST                 BSP_BUTTON_0
#define SEND_PERI_SECOND                BSP_BUTTON_1
#define SEND_CENT_FIRST                 BSP_BUTTON_2
#define SEND_CENT_SECOND                BSP_BUTTON_3

static uint8_t peri_1_handle = 0xff;
static uint8_t peri_2_handle = 0xff;
static uint8_t cent_1_handle = 0xff;
static uint8_t cent_2_handle = 0xff;


static ble_gap_addr_t m_peer_addr[NRF_SDH_BLE_TOTAL_LINK_COUNT] = {0};


/**@brief convert peer address byte array to string with ':' as deliminator */ 
static char peer_addr_to_string(const ble_gap_addr_t *peer_addr, char addr_str[])
{
    // char addr_str[BLE_GAP_ADDR_LEN*3];
    char *addr_ptr = &addr_str[0];

    addr_ptr += sprintf(addr_ptr, "%02X", (unsigned char) peer_addr->addr[0]);
    for (int i = 1; i < BLE_GAP_ADDR_LEN; i++) {
        addr_ptr += sprintf(addr_ptr, ":%02X", (unsigned char) peer_addr->addr[i]);
    }

    // return addr_ptr;
}


/**@brief print peer address byte array */
static void print_peer_addr(const ble_gap_addr_t *p_peer_addr)
{
    NRF_LOG_INFO("Connecting to target %02x:%02x:%02x:%02x:%02x:%02x",
            p_peer_addr->addr[0],
            p_peer_addr->addr[1],
            p_peer_addr->addr[2],
            p_peer_addr->addr[3],
            p_peer_addr->addr[4],
            p_peer_addr->addr[5]
            );
}


/**@brief add peer address to address list */
static void add_peer_addr(uint8_t conn_handle, const ble_gap_addr_t *p_peer_addr)
{
    memcpy(&m_peer_addr[conn_handle].addr, p_peer_addr->addr, sizeof(p_peer_addr->addr));
    // print_peer_addr(&m_peer_addr[conn_handle]);
    // print_peer_addr(p_peer_addr);
    NRF_LOG_INFO("Peer addres added to handle #: 0x%x", conn_handle);
}


/**@brief clear peer address from address list */
static void clear_peer_addr(uint8_t conn_handle)
{
    memset(&m_peer_addr[conn_handle], 0, sizeof(ble_gap_addr_t));
    NRF_LOG_INFO("Peer addres removed from handle #: 0x%x", conn_handle);
}


/**@brief initialize peer address list */
static void peer_addr_list_init()
{
    for (uint16_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++) {
        clear_peer_addr(i);
    }
}


/**@brief search list for the peer addres return false if found in list */
static bool is_new_peer(const ble_gap_addr_t *p_peer_addr)
{
    bool is_new = true;

    for (uint16_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++) {
        is_new = memcmp(m_peer_addr[i].addr, p_peer_addr->addr, sizeof(p_peer_addr->addr));
        if (is_new == 0) {
            return false;
        }
    }

    return is_new;
}


/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of an assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


static void scan_timer_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);

    if (peri_1_handle == 0xff) {
        bsp_board_led_invert(FIRST_PERI_CONNECTED);
    }
    if (peri_2_handle == 0xff) {
        bsp_board_led_invert(SECOND_PERI_CONNECTED);
    }
    
}


static void advertising_timer_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);

    if (cent_1_handle == 0xff) {
        bsp_board_led_invert(FIRST_CENT_CONNECTED);
    }

    if (cent_2_handle == 0xff) {
        bsp_board_led_invert(SECOND_CENT_CONNECTED);
    }
    
}


/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
{
    if (conn_handle == cent_1_handle) {
        // if (led_state) {
        //     bsp_board_led_off(FIRST_CENT_CONNECTED);
        //     NRF_LOG_INFO("Received LED ON from link 0x%x!", conn_handle);
        // }
        // else {
        //     bsp_board_led_on(FIRST_CENT_CONNECTED);
        //     NRF_LOG_INFO("Received LED OFF from link 0x%x!", conn_handle);
        // }

        NRF_LOG_INFO("Relay to Peripheral from link 0x%x!", peri_1_handle);
        ret_code_t err_code = ble_lbs_led_status_send(&m_lbs_c[peri_1_handle], led_state);
        if (err_code != NRF_SUCCESS && 
            err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
            err_code != NRF_ERROR_INVALID_STATE) {
            NRF_LOG_INFO("LBS write LED state %d", led_state);
        }
    }

    if (conn_handle == cent_2_handle) {
        // if (led_state) {
        //     bsp_board_led_off(SECOND_CENT_CONNECTED);
        //     NRF_LOG_INFO("Received LED ON from link 0x%x!", conn_handle);
        // }
        // else {
        //     bsp_board_led_on(SECOND_CENT_CONNECTED);
        //     NRF_LOG_INFO("Received LED OFF from link 0x%x!", conn_handle);
        // }

        NRF_LOG_INFO("Relay to Peripheral from link 0x%x!", peri_2_handle);
        ret_code_t err_code = ble_lbs_led_status_send(&m_lbs_c[peri_2_handle], led_state);
        if (err_code != NRF_SUCCESS && 
            err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
            err_code != NRF_ERROR_INVALID_STATE) {
            NRF_LOG_INFO("LBS write LED state %d", led_state);
        }
    }

}


/**@brief Function for initializing the LEDs.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);

    ret_code_t err_code;
    err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    NRF_LOG_INFO("Start scanning for device name %s.", (uint32_t)m_target_periph_name);
    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    // Turn on the LED to signal scanning.
    // bsp_board_led_on(CENTRAL_SCANNING_LED);
    // ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    
    ret = app_timer_start(m_scan_timer, APP_TIMER_TICKS(500), NULL);
    APP_ERROR_CHECK(ret);
}


static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    ble_gap_evt_adv_report_t const * p_adv = p_scan_evt->params.filter_match.p_adv_report;
    ble_gap_scan_params_t    const * p_scan_param = p_scan_evt->p_scan_params;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
        } break;

        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
        {
            bool new_peer = is_new_peer(&p_adv->peer_addr);
            if (!new_peer) {
                NRF_LOG_INFO("Duplicated connection!");
                sd_ble_gap_connect_cancel();

                // scan_start();
            }
        } break;

        // case NRF_BLE_SCAN_EVT_CONNECTED:
        // {
        //     ble_gap_evt_connected_t const *p_connected =
        //             p_scan_evt->params.connected.p_connected;
        //     // Scan is automatically stopped by the connection.
        //     NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
        //                 p_connected->peer_addr.addr[0],
        //                 p_connected->peer_addr.addr[1],
        //                 p_connected->peer_addr.addr[2],
        //                 p_connected->peer_addr.addr[3],
        //                 p_connected->peer_addr.addr[4],
        //                 p_connected->peer_addr.addr[5]
        //                 );
        // } break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
            NRF_LOG_INFO("Scan timed out.");
            scan_start();
        } break;
        
        default:
            break;
    }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    // bsp_board_led_on(ADVERTISING_LED);
    // bsp_indication_set(BSP_INDICATE_ADVERTISING);

    err_code = app_timer_start(m_adv_timer, APP_TIMER_TICKS(500), NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Handles events coming from the LED Button central module.
 *
 * @param[in] p_lbs_c     The instance of LBS_C that triggered the event.
 * @param[in] p_lbs_c_evt The LBS_C event.
 */
static void lbs_c_evt_handler(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt)
{
    switch (p_lbs_c_evt->evt_type)
    {
        case BLE_LBS_C_EVT_DISCOVERY_COMPLETE:
        {
            ret_code_t err_code;

            NRF_LOG_INFO("LED Button Service discovered on conn_handle 0x%x",
                         p_lbs_c_evt->conn_handle);

            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);

            // LED Button Service discovered. Enable notification of Button.
            err_code = ble_lbs_c_button_notif_enable(p_lbs_c);
            APP_ERROR_CHECK(err_code);
        } break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

        case BLE_LBS_C_EVT_BUTTON_NOTIFICATION:
        {
            // NRF_LOG_INFO("Link 0x%x, Button state changed on peer to 0x%x",
            //              p_lbs_c_evt->conn_handle,
            //              p_lbs_c_evt->params.button.button_state);

            // if (p_lbs_c_evt->params.button.button_state)
            // {
            //     bsp_board_led_on(LEDBUTTON_LED);
            // }
            // else
            // {
            //     bsp_board_led_off(LEDBUTTON_LED);
            // }

            uint8_t button_action = p_lbs_c_evt->params.button.button_state;

            NRF_LOG_INFO("lbs_c_evt handle 0x%x, cent_1_handle 0x%x, peri_1_handle 0x%x", 
                    p_lbs_c_evt->conn_handle, cent_1_handle, peri_1_handle);

            if (p_lbs_c_evt->conn_handle == peri_1_handle) {
                ret_code_t err_code = ble_lbs_on_button_change(cent_1_handle, &m_lbs, button_action);
                if (err_code != NRF_SUCCESS &&
                    err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                    err_code != NRF_ERROR_INVALID_STATE &&
                    err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
                    NRF_LOG_INFO("LBS write LED state %d", button_action);
                    APP_ERROR_CHECK(err_code);
                }
            }

            if (p_lbs_c_evt->conn_handle == peri_2_handle) {
                ret_code_t err_code = ble_lbs_on_button_change(cent_2_handle, &m_lbs, button_action);
                if (err_code != NRF_SUCCESS &&
                    err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                    err_code != NRF_ERROR_INVALID_STATE &&
                    err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
                    NRF_LOG_INFO("LBS write LED state %d", button_action);
                    APP_ERROR_CHECK(err_code);
                }
            }

        } break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for assigning new connection handle to available instance of QWR module.
 *
 * @param[in] conn_handle New connection handle.
 */
static void multi_qwr_conn_handle_assign(uint16_t conn_handle)
{
    for (uint32_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
    {
        if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            ret_code_t err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], conn_handle);
            APP_ERROR_CHECK(err_code);
            break;
        }
    }
}


/**@brief   Function for handling BLE events from the central application.
 *
 * @details This function parses scanning reports and initiates a connection to peripherals when a
 *          target UUID is found. It updates the status of LEDs used to report the central application
 *          activity.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_central_evt(ble_evt_t const * p_ble_evt)
{   
    ret_code_t err_code;

    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral is connected, initiate DB
        // discovery, update LEDs status, and resume scanning, if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            // add the peer address to list
            ble_gap_addr_t peer_addr = p_gap_evt->params.connected.peer_addr;
            add_peer_addr(p_gap_evt->conn_handle, &peer_addr);
            char addr_str[BLE_GAP_ADDR_LEN*3];
            peer_addr_to_string(&peer_addr, addr_str);

            int cnt = ble_conn_state_central_conn_count();

            NRF_LOG_INFO("Central #%d Connected 0x%x Peer %s, starting DB discovery.", 
                        cnt, conn_handle, nrf_log_push(addr_str));
            
            APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);

            err_code = ble_lbs_c_handles_assign(&m_lbs_c[p_gap_evt->conn_handle],
                                                p_gap_evt->conn_handle,
                                                NULL);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle],
                                              p_gap_evt->conn_handle);
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }

            // Assign connection handle to the QWR module.
            multi_qwr_conn_handle_assign(p_gap_evt->conn_handle);

            if (cnt == NRF_SDH_BLE_CENTRAL_LINK_COUNT) {
                if ((peri_1_handle == 0xff) && (peri_2_handle != 0xff)) {
                    peri_1_handle = p_gap_evt->conn_handle;
                    bsp_board_led_on(FIRST_PERI_CONNECTED);
                }

                if ((peri_1_handle != 0xff) && (peri_2_handle == 0xff)) {
                    peri_2_handle = p_gap_evt->conn_handle;
                    bsp_board_led_on(SECOND_PERI_CONNECTED);
                }
            
                // Stop scanning timer
                app_timer_stop(m_scan_timer);
            }
            else if (cnt < NRF_SDH_BLE_CENTRAL_LINK_COUNT) {
                if ((peri_1_handle == 0xff) && (peri_2_handle == 0xff)) {
                    peri_1_handle = p_gap_evt->conn_handle;
                    bsp_board_led_on(FIRST_PERI_CONNECTED);
                }

                // Resume scanning
                scan_start();
            }

        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer that disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {   // clear peer address from list
            clear_peer_addr(p_gap_evt->conn_handle);

            int cnt = ble_conn_state_central_conn_count();

            NRF_LOG_INFO("LBS CENTRAL link 0x%x disconnected (reason: 0x%x)",
                        p_gap_evt->conn_handle,
                        p_gap_evt->params.disconnected.reason);

            if (p_gap_evt->conn_handle == peri_1_handle) {
                bsp_board_led_off(FIRST_PERI_CONNECTED);
                peri_1_handle = 0xff;
            }
            if (p_gap_evt->conn_handle == peri_2_handle) {
                bsp_board_led_off(SECOND_PERI_CONNECTED);
                peri_2_handle = 0xff;
            }
            // if (cnt == 0) {
            //     err_code = app_button_disable();
            //     APP_ERROR_CHECK(err_code);
            // }

            if (cnt == (NRF_SDH_BLE_CENTRAL_LINK_COUNT - 1)) {
                // Scanning is not running when all connections are taken, and must therefore be started.                
                scan_start();
            }            

        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // Timeout for scanning is not specified, so only the connection requests can time out.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST.");
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT client timeout event.
            NRF_LOG_DEBUG("GATT client timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT server timeout event.
            NRF_LOG_DEBUG("GATT server timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief   Function for handling BLE events from peripheral applications.
 * @details Updates the status LEDs used to report the activity of the peripheral applications.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_peripheral_evt(ble_evt_t const * p_ble_evt)
{
    ret_code_t err_code;

    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    
    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral is connected, initiate DB
        // discovery, update LEDs status, and resume scanning, if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            // add the peer address to list
            ble_gap_addr_t peer_addr = p_gap_evt->params.connected.peer_addr;
            add_peer_addr(p_gap_evt->conn_handle, &peer_addr);
            char addr_str[BLE_GAP_ADDR_LEN*3];
            peer_addr_to_string(&peer_addr, addr_str);

            int cnt = ble_conn_state_peripheral_conn_count();

            NRF_LOG_INFO("Peripheral #%d Connected 0x%x Peer %s.", 
                        cnt, conn_handle, nrf_log_push(addr_str));

            // Assign connection handle to available instance of QWR module.
            multi_qwr_conn_handle_assign(p_gap_evt->conn_handle);

            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);

            if (cnt == NRF_SDH_BLE_PERIPHERAL_LINK_COUNT) {
                if ((cent_1_handle == 0xff) && (cent_2_handle != 0xff)) {
                    cent_1_handle = p_gap_evt->conn_handle;
                    bsp_board_led_on(FIRST_CENT_CONNECTED);
                }

                if ((cent_1_handle != 0xff) && (cent_2_handle == 0xff)) {
                    cent_2_handle = p_gap_evt->conn_handle;
                    bsp_board_led_on(SECOND_CENT_CONNECTED);
                }

                // turn off advertising timer     
                app_timer_stop(m_adv_timer);
            }
            else if (cnt < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT) {
                if ((cent_1_handle == 0xff) && (cent_2_handle == 0xff)) {
                    cent_1_handle = p_gap_evt->conn_handle;
                    bsp_board_led_on(FIRST_CENT_CONNECTED);
                }

                // Resume advertising
                advertising_start();
            }


        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer that disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {   // clear peer address from list
            clear_peer_addr(p_gap_evt->conn_handle);

            int cnt = ble_conn_state_peripheral_conn_count();

            NRF_LOG_INFO("LBS PERIPHERAL link 0x%x disconnected (reason: 0x%x)",
                        p_gap_evt->conn_handle,
                        p_gap_evt->params.disconnected.reason);

            if (p_gap_evt->conn_handle == cent_1_handle) {
                bsp_board_led_off(FIRST_CENT_CONNECTED);
                cent_1_handle = 0xff;
            }
            if (p_gap_evt->conn_handle == cent_2_handle) {
                bsp_board_led_off(SECOND_PERI_CONNECTED);
                cent_2_handle = 0xff;
            }
            // if (cnt == 0) {
            //     err_code = app_button_disable();
            //     APP_ERROR_CHECK(err_code);
            // }            

            if (cnt == (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT - 1)) {
                // Advertising is not running when all connections are taken, and must therefore be started.                
                advertising_start();
            }

        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        {
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
        
        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT client timeout event.
            NRF_LOG_DEBUG("GATT client timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT server timeout event.
            NRF_LOG_DEBUG("GATT server timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for checking whether a bluetooth stack event is an advertising timeout.
 *
 * @param[in] p_ble_evt Bluetooth stack event.
 */
static bool ble_evt_is_advertising_timeout(ble_evt_t const * p_ble_evt)
{
    return (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_SET_TERMINATED);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    uint16_t conn_handle = p_gap_evt->conn_handle;
    uint16_t gap_role    = ble_conn_state_role(conn_handle);
    
    if (gap_role == BLE_GAP_ROLE_PERIPH || ble_evt_is_advertising_timeout(p_ble_evt)) {
        on_ble_peripheral_evt(p_ble_evt);
    }
    else if (gap_role = BLE_GAP_ROLE_CENTRAL) {
        on_ble_central_evt(p_ble_evt);
    }
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief LED Button collector initialization. */
static void lbs_c_init(void)
{
    ret_code_t       err_code;
    ble_lbs_c_init_t lbs_c_init_obj;

    lbs_c_init_obj.evt_handler = lbs_c_evt_handler;

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_lbs_c_init(&m_lbs_c[i], &lbs_c_init_obj);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for writing to the LED characteristic of all connected clients.
 *
 * @details Based on whether the button is pressed or released, this function writes a high or low
 *          LED status to the server.
 *
 * @param[in] button_action The button action (press or release).
 *            Determines whether the LEDs of the servers are ON or OFF.
 *
 * @return If successful, NRF_SUCCESS is returned. Otherwise, returns the error code from @ref ble_lbs_led_status_send.
 */
static ret_code_t led_status_send_to_all(uint8_t button_action)
{
    ret_code_t err_code;

    for (uint32_t i = 0; i< NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_lbs_led_status_send(&m_lbs_c[i], button_action);
        if (err_code != NRF_SUCCESS &&
            err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
            err_code != NRF_ERROR_INVALID_STATE)
        {
            return err_code;
        }
    }
        return NRF_SUCCESS;
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press or release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case SEND_PERI_FIRST:
        {
            err_code = ble_lbs_led_status_send(&m_lbs_c[peri_1_handle], button_action);
            if (err_code != NRF_SUCCESS && 
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE) {
                NRF_LOG_INFO("LBS write LED state %d", button_action);
            }
        } break;
        
        case SEND_PERI_SECOND:
        {
            err_code = ble_lbs_led_status_send(&m_lbs_c[peri_2_handle], button_action);
            if (err_code != NRF_SUCCESS && 
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE) {
                NRF_LOG_INFO("LBS write LED state %d", button_action);
            }
        } break;
        
        case SEND_CENT_FIRST:
        {
            err_code = ble_lbs_on_button_change(cent_1_handle, &m_lbs, button_action);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE &&
                err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
                NRF_LOG_INFO("LBS write LED state %d", button_action);
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case SEND_CENT_SECOND:
        {
            err_code = ble_lbs_on_button_change(cent_2_handle, &m_lbs, button_action);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE &&
                err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
                NRF_LOG_INFO("LBS write LED state %d", button_action);
                APP_ERROR_CHECK(err_code);
            }            
        } break;
        
        // case LEDBUTTON_BUTTON:
        //     err_code = led_status_send_to_all(button_action);
        //     if (err_code == NRF_SUCCESS)
        //     {
        //         NRF_LOG_INFO("LBS write LED state %d", button_action);
        //     }
        //     break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

   // The array must be static because a pointer to it is saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        // {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler}
        {SEND_PERI_FIRST, false, BUTTON_PULL, button_event_handler},
        {SEND_PERI_SECOND, false, BUTTON_PULL, button_event_handler},
        {SEND_CENT_FIRST, false, BUTTON_PULL, button_event_handler},
        {SEND_CENT_SECOND, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    NRF_LOG_DEBUG("call to ble_lbs_on_db_disc_evt for instance %d and link 0x%x!",
                  p_evt->conn_handle,
                  p_evt->conn_handle);

    ble_lbs_on_db_disc_evt(&m_lbs_c[p_evt->conn_handle], p_evt);
}


/** @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details This function handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/** @brief Function for initializing the log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
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


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t           err_code;
    ble_advdata_t        advdata;
    ble_advdata_t        srdata;
    ble_gap_adv_params_t adv_params;


    ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    // Start advertising.
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.p_peer_addr   = NULL;
    adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
    adv_params.interval      = APP_ADV_INTERVAL;

    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_lbs_init_t     init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module instances.
    qwr_init.error_handler = nrf_qwr_error_handler;

    for (uint32_t i = 0; i < LINK_TOTAL; i++)
    {
        err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
        APP_ERROR_CHECK(err_code);
    }

    // Initialize LBS.
    init.led_write_handler = led_write_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);

    ble_conn_state_init();
}


/**@brief Function for handling a Connection Parameters error.
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
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = true;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // creating the timer for scan indicate
    err_code = app_timer_create(&m_scan_timer, APP_TIMER_MODE_REPEATED, scan_timer_handler);
    APP_ERROR_CHECK(err_code);

    // creating the timer for advertising indicate
    err_code = app_timer_create(&m_adv_timer, APP_TIMER_MODE_REPEATED, advertising_timer_handler);
    APP_ERROR_CHECK(err_code);

}


int main(void)
{
    // Initialize.
    log_init();
    timer_init();
    leds_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    gatt_init();

    gap_params_init();  //Add for Peripheral Role
    services_init();    //Add for Peripheral Role
    advertising_init(); //Add for Peripheral Role
    conn_params_init(); //Add for Peripheral Role

    db_discovery_init();
    lbs_c_init();
    ble_conn_state_init();
    scan_init();

    // Start execution.
    NRF_LOG_INFO("Multilink example started.");
    scan_start();

    advertising_start();    //Add for Peripheral Role

    for (;;)
    {
        idle_state_handle();
    }
}
