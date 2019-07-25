### LEDs
Default bsp has a single m_bsp_leds_tmr with one m_stable_state, thus can not run both led simultanously.
Use two timer m_scan_timer and m_adv_timer to control leds for scanning and advertising.


<info> app: Multilink example started.
<info> app: Start scanning for device name Nordic_Blinky.
<info> app: Connection 0x0 established, starting DB discovery.
<info> app: Central Connection Count = 1
<info> app: Start scanning for device name Nordic_Blinky.
<info> app: Connection 0x2 established, starting DB discovery.
<error> app: Fatal error
APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);
For central connection, handle start from 0; 
while for peripheral connection, handle starts after NRF_SDH_BLE_CENTRAL_LINK_COUNT (2),
Connection 0x2 (3rd connection) > NRF_SDH_BLE_CENTRAL_LINK_COUNT


<info> app: Multilink example started.
<info> app: Start scanning for device name Nordic_Blinky.
<info> app: Connection 0x2 established.
<info> app: Peripheral Connection Count = 1
<info> app: Connection 0x3 established.
<info> app: Peripheral Connection Count = 2
<info> app: LBS PERIPHERAL link 0x2 disconnected (reason: 0x13)
<info> app: Connection 0x2 established.
<info> app: Peripheral Connection Count = 2
<info> app: LBS PERIPHERAL link 0x2 disconnected (reason: 0x13)
<info> app: LBS PERIPHERAL link 0x3 disconnected (reason: 0x8)
<error> app: Fatal error

<info> app: Multilink example started.
<info> app: Start scanning for device name Nordic_Blinky.
<info> app: Central #1 Connected 0x0, starting DB discovery.
<info> app: Start scanning for device name Nordic_Blinky.
<info> app: LED Button Service discovered on conn_handle 0x0
<info> app: Peripheral #1 Connected 0x2.
<info> app: LBS PERIPHERAL link 0x2 disconnected (reason: 0x13)

<info> app: Multilink example started.
<info> app: Start scanning for device name Nordic_Blinky.
<info> app: Central #1 Connected 0x0, starting DB discovery.
<info> app: Start scanning for device name Nordic_Blinky.
<info> app: LED Button Service discovered on conn_handle 0x0
<info> app: Peripheral #1 Connected 0x2.
<info> app: Received LED ON from link 0x2!
<info> app: Relay to Peripheral from link 0x0!
<info> app: Received LED OFF from link 0x2!
<info> app: Relay to Peripheral from link 0x0!
<info> app: Received LED ON from link 0x2!
<info> app: Relay to Peripheral from link 0x0!
<info> app: Received LED OFF from link 0x2!
<info> app: Relay to Peripheral from link 0x0!

<info> app: Multilink example started.
<info> app: Start scanning for device name Nordic_Blinky_P.
<info> app: Central #1 Connected 0x0, starting DB discovery.
<info> app: Start scanning for device name Nordic_Blinky_P.
<info> app: Central #2 Connected 0x1, starting DB discovery.
<info> app: LED Button Service discovered on conn_handle 0x1
<info> app: LED Button Service discovered on conn_handle 0x0
<info> app: Peripheral #1 Connected 0x2.
<info> app: LBS PERIPHERAL link 0x2 disconnected (reason: 0x13)
<info> app: Peripheral #1 Connected 0x3.
<info> app: Received LED ON from link 0x3!
<info> app: Relay to Peripheral from link 0x0!
<info> app: Received LED OFF from link 0x3!
<info> app: Relay to Peripheral from link 0x0!
<info> app: Received LED ON from link 0x3!
<info> app: Relay to Peripheral from link 0x0!
<info> app: Received LED OFF from link 0x3!
<info> app: Relay to Peripheral from link 0x0!
<info> app: Received LED ON from link 0x3!
<info> app: Relay to Peripheral from link 0x0!
