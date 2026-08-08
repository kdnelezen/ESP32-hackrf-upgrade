idf_component_register(
    SRCS
        "main.c"
        "spectrum_analyzer.c"
        "signal_replayer.c"
    INCLUDE_DIRS "."
    REQUIRES
        hackrf_usb
        subghz
        ir_transceiver
        ble_wifi_scanner
        cli
        nvs_flash
        esp_wifi
        bt
        driver
        usb
)
