/**
 * @file ble_wifi_scanner.c
 * @brief BLE and Wi-Fi scanner implementation using ESP-IDF stacks.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "ble_wifi_scanner.h"

static const char *TAG = "BLE_WIFI";

#define WIFI_SCAN_DONE_BIT BIT0

/* ── State ──────────────────────────────────────────────────────────────── */
static bool              s_inited       = false;
static ble_scan_cb_t     s_ble_cb       = NULL;
static void             *s_ble_ctx      = NULL;
static wifi_scan_cb_t    s_wifi_cb      = NULL;
static void             *s_wifi_ctx     = NULL;
static EventGroupHandle_t s_wifi_events = NULL;

/* ── BLE GAP callback ────────────────────────────────────────────────────── */
static void ble_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    if (event != ESP_GAP_BLE_SCAN_RESULT_EVT) return;
    if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) return;
    if (!s_ble_cb) return;

    ble_scan_result_t res = {0};
    memcpy(res.addr, param->scan_rst.bda, 6);
    res.rssi = param->scan_rst.rssi;

    /* Extract Local Name from AD */
    uint8_t adv_len;
    uint8_t *adv = esp_ble_resolve_adv_data(param->scan_rst.ble_adv,
                                              ESP_BLE_AD_TYPE_NAME_CMPL, &adv_len);
    if (adv && adv_len > 0 && adv_len < (int)sizeof(res.name)) {
        memcpy(res.name, adv, adv_len);
    }

    uint32_t raw_len = param->scan_rst.adv_data_len;
    if (raw_len > sizeof(res.adv_data)) raw_len = sizeof(res.adv_data);
    memcpy(res.adv_data, param->scan_rst.ble_adv, raw_len);
    res.adv_data_len = (uint8_t)raw_len;

    s_ble_cb(&res, s_ble_ctx);
}

/* ── Wi-Fi event handler ─────────────────────────────────────────────────── */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) {
        xEventGroupSetBits(s_wifi_events, WIFI_SCAN_DONE_BIT);
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

esp_err_t ble_wifi_scanner_init(void)
{
    if (s_inited) return ESP_OK;

    s_wifi_events = xEventGroupCreate();

    /* Wi-Fi init */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* BLE init */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(ble_gap_cb));

    s_inited = true;
    ESP_LOGI(TAG, "BLE + Wi-Fi scanner ready");
    return ESP_OK;
}

esp_err_t ble_scan_start(uint32_t duration_ms, ble_scan_cb_t cb, void *user_ctx)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    s_ble_cb  = cb;
    s_ble_ctx = user_ctx;

    esp_ble_scan_params_t scan_params = {
        .scan_type          = BLE_SCAN_TYPE_PASSIVE,
        .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval      = 0x50,
        .scan_window        = 0x30,
        .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE,
    };
    esp_err_t ret = esp_ble_gap_set_scan_params(&scan_params);
    if (ret != ESP_OK) return ret;

    uint32_t dur = duration_ms ? (duration_ms / 1000) : 0;
    return esp_ble_gap_start_scanning(dur);
}

void ble_scan_stop(void)
{
    if (s_inited) esp_ble_gap_stop_scanning();
}

esp_err_t wifi_scan_all(wifi_scan_cb_t cb, void *user_ctx)
{
    if (!s_inited) return ESP_ERR_INVALID_STATE;
    s_wifi_cb  = cb;
    s_wifi_ctx = user_ctx;

    xEventGroupClearBits(s_wifi_events, WIFI_SCAN_DONE_BIT);

    wifi_scan_config_t scan_cfg = {
        .ssid        = NULL,
        .bssid       = NULL,
        .channel     = 0,
        .show_hidden = true,
        .scan_type   = WIFI_SCAN_TYPE_ACTIVE,
    };
    esp_err_t ret = esp_wifi_scan_start(&scan_cfg, false);
    if (ret != ESP_OK) return ret;

    /* Block until scan completes (max 10 s) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_events, WIFI_SCAN_DONE_BIT,
                                            pdTRUE, pdFALSE, pdMS_TO_TICKS(10000));
    if (!(bits & WIFI_SCAN_DONE_BIT)) return ESP_ERR_TIMEOUT;

    uint16_t count = WIFI_SCANNER_MAX_RESULTS;
    wifi_ap_record_t ap_records[WIFI_SCANNER_MAX_RESULTS];
    esp_wifi_scan_get_ap_records(&count, ap_records);

    if (!cb) return ESP_OK;

    wifi_scan_result_t results[WIFI_SCANNER_MAX_RESULTS];
    for (uint16_t i = 0; i < count; i++) {
        memcpy(results[i].ssid,  ap_records[i].ssid, 33);
        memcpy(results[i].bssid, ap_records[i].bssid, 6);
        results[i].rssi      = ap_records[i].rssi;
        results[i].channel   = ap_records[i].primary;
        results[i].auth_mode = ap_records[i].authmode;
    }
    cb(results, count, user_ctx);
    return ESP_OK;
}
