/**
* THIS SOFTWARE IS PROVIDED BY LEI SHI "AS IS"
* Use helper functions to only allow a signle connection to the same address
*/

#ifndef HELPER_H__
#define HELPER_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ble.h"
#include "ble_gap.h"
#include "app_util.h"
#include "sdk_errors.h"
#include "sdk_config.h"

#ifdef __cplusplus
extern "C" {
#endif


/**@brief convert peer address byte array to string with ':' as deliminator */
char peer_addr_to_string(const ble_gap_addr_t *peer_addr, char addr_str[]);


/**@brief print peer address byte array */
void print_peer_addr(const ble_gap_addr_t *p_peer_addr);

/**@brief print all connected addresses */
void print_conn_addr();


/**@brief add peer address to address list */
void add_conn_addr(uint8_t conn_handle, const ble_gap_addr_t *p_peer_addr);


/**@brief clear peer address from address list */
void clear_conn_addr(uint8_t conn_handle);


/**@brief initialize peer address list */
void conn_addr_list_init();


/**@brief Function for comparing two provided address. */
bool compare_conn_addr(ble_gap_addr_t const * p_peer_addr, ble_gap_addr_t const * p_addr);


/** @brief Function for searching the provided address with the addresses of the connected devices.*/
bool find_conn_addr(const ble_gap_addr_t *p_peer_addr);


#ifdef __cplusplus
}
#endif

#endif // HELPER_H__
