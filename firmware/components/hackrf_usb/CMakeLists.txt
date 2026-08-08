idf_component_register(
    SRCS "hackrf_usb.c"
    INCLUDE_DIRS "include"
    REQUIRES usb driver esp_timer
)
