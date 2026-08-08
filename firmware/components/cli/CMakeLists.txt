idf_component_register(
    SRCS "cli.c"
    INCLUDE_DIRS "include"
    REQUIRES
        console
        hackrf_usb
        subghz
        ir_transceiver
        ble_wifi_scanner
        driver
)
