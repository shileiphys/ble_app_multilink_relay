#include "sdk_common.h"


#include "sdk_config.h"
#include <stdlib.h>

#include "helper.h"

#include <string.h>
#include "app_error.h"
#include "nrf_assert.h"
#include "sdk_macros.h"
#include "ble_advdata.h"


#include "nrf_log.h"


static ble_gap_addr_t m_conn_addr[NRF_SDH_BLE_TOTAL_LINK_COUNT] = {0};


/**@brief convert peer address byte array to string with ':' as deliminator */ 
char peer_addr_to_string(const ble_gap_addr_t *peer_addr, char addr_str[])
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
void print_peer_addr(const ble_gap_addr_t *p_peer_addr)
{
    NRF_LOG_INFO("Connected to target %02x:%02x:%02x:%02x:%02x:%02x",
            p_peer_addr->addr[0],
            p_peer_addr->addr[1],
            p_peer_addr->addr[2],
            p_peer_addr->addr[3],
            p_peer_addr->addr[4],
            p_peer_addr->addr[5]
            );
}


/**@brief print all connected addresses */
void print_conn_addr()
{
    for (uint8_t index = 0; index < NRF_SDH_BLE_TOTAL_LINK_COUNT; index++) {
        print_peer_addr(&m_conn_addr[index]);
    }
}

/**@brief add peer address to connected address list */
void add_conn_addr(uint8_t conn_handle, const ble_gap_addr_t *p_peer_addr)
{
    memcpy(&m_conn_addr[conn_handle].addr, p_peer_addr->addr, sizeof(p_peer_addr->addr));
    // print_peer_addr(&m_peer_addr[conn_handle]);
    // print_peer_addr(p_peer_addr);
    NRF_LOG_INFO("Peer addres added to handle #: 0x%x", conn_handle);
}


/**@brief clear peer address from connected address list */
void clear_conn_addr(uint8_t conn_handle)
{
    memset(&m_conn_addr[conn_handle], 0, sizeof(ble_gap_addr_t));
    NRF_LOG_INFO("Peer addres removed from handle #: 0x%x", conn_handle);
}


/**@brief initialize peer address list */
void conn_addr_list_init()
{
    for (uint16_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++) {
        clear_conn_addr(i);
    }
}


/**@brief Function for comparing two provided address.
 *
 * @details Use this function to compare two provided address.
 *
 *
 * @param[in]   p_peer_addr    peer address data to compare.
 * @param[in]   p_addr         Address to search for. The address length must correspond to @ref BLE_GAP_ADDR_LEN.
 *
 * @return   True if the provided address was found, false otherwise.
 */
bool compare_conn_addr(ble_gap_addr_t const * p_peer_addr, ble_gap_addr_t const * p_addr)
{
    // Compare addresses.
    if (memcmp(p_addr->addr, p_peer_addr->addr, sizeof(p_peer_addr->addr)) == 0)
    {
        return true;
    }

    return false;
}


/** @brief Function for searching the provided address with the addresses of the connected devices.
 *
 * @param[in] p_peer_addr      Pointer to the peer address.
 *
 * @retval True when the address matches with the addresses of the advertising devices. False otherwise.
 */
bool find_conn_addr(const ble_gap_addr_t *p_peer_addr)
{

    for (uint8_t index = 0; index < NRF_SDH_BLE_TOTAL_LINK_COUNT; index++)
    {
        // Search for address.
        if (compare_conn_addr(p_peer_addr, &m_conn_addr[index]))
        {
            return true;
        }
    }

    return false;

}