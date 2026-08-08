/**
 * @file ble_wifi_scanner.h
 * @brief BLE and Wi-Fi scanning – Flipper Zero-style wireless recon
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_SCANNER_MAX_RESULTS  64
#define WIFI_SCANNER_MAX_RESULTS 32

/** BLE advertisement record. */
typedef struct {
    uint8_t  addr[6];
    int8_t   rssi;
    char     name[32];
    uint8_t  adv_data[62];
    uint8_t  adv_data_len;
} ble_scan_result_t;

/** Wi-Fi AP record. */
typedef struct {
    char     ssid[33];
    uint8_t  bssid[6];
    int8_t   rssi;
    uint8_t  channel;
    uint8_t  auth_mode;   /**< esp_wifi auth mode enum */
} wifi_scan_result_t;

/** Callback for BLE results (called per advertisement). */
typedef void (*ble_scan_cb_t)(const ble_scan_result_t *res, void *ctx);

/** Callback for Wi-Fi results (called once scan is complete). */
typedef void (*wifi_scan_cb_t)(const wifi_scan_result_t *results,
                                uint16_t count, void *ctx);

/**
 * @brief Initialise BLE and Wi-Fi stacks.
 * @return ESP_OK on success.
 */
esp_err_t ble_wifi_scanner_init(void);

/**
 * @brief Start a BLE passive scan.
 * @param duration_ms  Scan window in milliseconds (0 = continuous).
 * @param cb           Callback invoked per advertisement.
 * @param user_ctx     Caller context.
 * @return ESP_OK on success.
 */
esp_err_t ble_scan_start(uint32_t duration_ms, ble_scan_cb_t cb, void *user_ctx);

/**
 * @brief Stop an ongoing BLE scan.
 */
void ble_scan_stop(void);

/**
 * @brief Perform a Wi-Fi AP scan.
 * @param cb       Callback invoked when scan is complete.
 * @param user_ctx Caller context.
 * @return ESP_OK on success.
 */
esp_err_t wifi_scan_all(wifi_scan_cb_t cb, void *user_ctx);

#ifdef __cplusplus
}
#endif
