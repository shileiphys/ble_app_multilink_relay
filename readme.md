### LEDs
Default bsp has a single m_bsp_leds_tmr with one m_stable_state, thus can not run both led simultanously.
Use two timer m_scan_timer and m_adv_timer to control leds for scanning and advertising.