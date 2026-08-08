idf_component_register(
    SRCS "ble_wifi_scanner.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi bt nvs_flash esp_event
)
