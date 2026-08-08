/**
 * @file main.c
 * @brief ESP32 HackRF Upgrade – Main entry point
 *
 * Boots all subsystems and starts the interactive CLI.
 * Feature set inspired by the Flipper Zero:
 *   - Sub-GHz signal receive/transmit via HackRF
 *   - Spectrum analyzer
 *   - Signal capture and replay
 *   - IR transceiver
 *   - BLE and Wi-Fi scanning
 *   - Interactive UART CLI
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "hackrf_usb.h"
#include "subghz.h"
#include "ir_transceiver.h"
#include "ble_wifi_scanner.h"
#include "cli.h"
#include "spectrum_analyzer.h"
#include "signal_replayer.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 HackRF Upgrade starting...");

    /* Initialize NVS (needed by Wi-Fi / BT stacks) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Bring up the HackRF over USB */
    ESP_ERROR_CHECK(hackrf_usb_init());
    ESP_LOGI(TAG, "HackRF USB initialised");

    /* Sub-GHz subsystem */
    ESP_ERROR_CHECK(subghz_init());
    ESP_LOGI(TAG, "Sub-GHz subsystem ready");

    /* IR transceiver */
    ESP_ERROR_CHECK(ir_transceiver_init());
    ESP_LOGI(TAG, "IR transceiver ready");

    /* BLE / Wi-Fi scanner */
    ESP_ERROR_CHECK(ble_wifi_scanner_init());
    ESP_LOGI(TAG, "BLE/Wi-Fi scanner ready");

    /* Spectrum analyser and signal replayer share the HackRF handle */
    hackrf_device_t *dev = hackrf_usb_get_device();
    spectrum_analyzer_init(dev);
    signal_replayer_init(dev);

    /* Start the interactive CLI (runs in its own task) */
    cli_start();

    ESP_LOGI(TAG, "All subsystems running – type 'help' in the console");
}
