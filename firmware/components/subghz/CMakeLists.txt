idf_component_register(
    SRCS "subghz.c"
    INCLUDE_DIRS "include"
    REQUIRES hackrf_usb driver esp_timer
    PRIV_REQUIRES m
)
