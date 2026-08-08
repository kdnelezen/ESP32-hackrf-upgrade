/**
 * @file hackrf_usb.c
 * @brief HackRF One USB host driver implementation for ESP32-S2/S3
 *
 * Uses the ESP-IDF USB Host Library (usb_host) to enumerate and communicate
 * with a HackRF One (VID 0x1D50, PID 0x6089) over USB OTG.
 *
 * USB control requests are derived from the libhackrf source:
 *   https://github.com/greatscottgadgets/hackrf/tree/master/host/libhackrf
 */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "usb/usb_host.h"
#include "hackrf_usb.h"

static const char *TAG = "HACKRF_USB";

/* ── HackRF USB identifiers ────────────────────────────────────────────── */
#define HACKRF_VID         0x1D50
#define HACKRF_PID         0x6089
#define HACKRF_IFACE       0
#define HACKRF_EP_IN       0x81   /**< Bulk IN  – I/Q samples from radio */
#define HACKRF_EP_OUT      0x02   /**< Bulk OUT – I/Q samples to radio   */
#define HACKRF_BULK_SIZE   16384  /**< Bytes per bulk transfer           */

/* ── libhackrf vendor request codes ────────────────────────────────────── */
#define HACKRF_VENDOR_REQUEST_SET_TRANSCEIVER_MODE  1
#define HACKRF_VENDOR_REQUEST_SET_FREQ              11
#define HACKRF_VENDOR_REQUEST_AMP_ENABLE            12
#define HACKRF_VENDOR_REQUEST_BASEBAND_FILTER_BW    13
#define HACKRF_VENDOR_REQUEST_SET_LNA_GAIN          16
#define HACKRF_VENDOR_REQUEST_SET_VGA_GAIN          17
#define HACKRF_VENDOR_REQUEST_SET_TXVGA_GAIN        18
#define HACKRF_VENDOR_REQUEST_ANTENNA_ENABLE        19
#define HACKRF_VENDOR_REQUEST_SET_SAMPLE_RATE       24

#define TRANSCEIVER_MODE_OFF  0
#define TRANSCEIVER_MODE_RX   1
#define TRANSCEIVER_MODE_TX   2

/* ── Internal state ─────────────────────────────────────────────────────── */
struct hackrf_device {
    usb_host_client_handle_t client;
    usb_device_handle_t      dev_hdl;
    usb_transfer_t          *rx_xfer;
    hackrf_rx_cb_t           rx_cb;
    void                    *rx_ctx;
    bool                     rx_running;
    uint64_t                 freq_hz;
    uint32_t                 sample_rate;
};

static hackrf_device_t s_dev;   /* single-device singleton */
static bool            s_inited = false;

/* ── Forward declarations ───────────────────────────────────────────────── */
static void usb_host_task(void *arg);
static void rx_transfer_cb(usb_transfer_t *xfer);
static esp_err_t ctrl_out(hackrf_device_t *dev, uint8_t request,
                           uint16_t value, uint16_t index,
                           const uint8_t *data, uint16_t len);

/* ────────────────────────────────────────────────────────────────────────── */
/*  Public API                                                                */
/* ────────────────────────────────────────────────────────────────────────── */

esp_err_t hackrf_usb_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    memset(&s_dev, 0, sizeof(s_dev));

    /* Install USB host library */
    usb_host_config_t host_cfg = {
        .skip_phy_setup       = false,
        .intr_flags           = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t ret = usb_host_install(&host_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start USB host daemon task */
    xTaskCreate(usb_host_task, "usb_host", 4096, NULL, 10, NULL);

    /* Register client */
    usb_host_client_config_t client_cfg = {
        .is_synchronous    = false,
        .max_num_event_msg = 5,
        .async             = { .client_event_callback = NULL, .callback_arg = NULL },
    };
    ret = usb_host_client_register(&client_cfg, &s_dev.client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_client_register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Wait for HackRF to enumerate (blocking poll up to 5 s) */
    for (int i = 0; i < 50; i++) {
        usb_host_client_handle_events(s_dev.client, pdMS_TO_TICKS(100));
        /* Try to open device by VID/PID */
        uint8_t dev_addr_list[16];
        int num_devs = 0;
        usb_host_device_addr_list_fill(sizeof(dev_addr_list), dev_addr_list, &num_devs);
        for (int d = 0; d < num_devs; d++) {
            usb_device_handle_t hdl;
            if (usb_host_device_open(s_dev.client, dev_addr_list[d], &hdl) != ESP_OK) {
                continue;
            }
            const usb_device_desc_t *desc;
            usb_host_get_device_descriptor(hdl, &desc);
            if (desc->idVendor == HACKRF_VID && desc->idProduct == HACKRF_PID) {
                s_dev.dev_hdl = hdl;
                ESP_LOGI(TAG, "HackRF One found at address %d", dev_addr_list[d]);
                goto found;
            }
            usb_host_device_close(s_dev.client, hdl);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGE(TAG, "HackRF not found within timeout");
    return ESP_ERR_NOT_FOUND;

found:
    ret = usb_host_interface_claim(s_dev.client, s_dev.dev_hdl, HACKRF_IFACE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "interface claim failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Allocate RX transfer */
    usb_host_transfer_alloc(HACKRF_BULK_SIZE + sizeof(usb_transfer_t),
                             0, &s_dev.rx_xfer);
    s_dev.rx_xfer->device_handle  = s_dev.dev_hdl;
    s_dev.rx_xfer->bEndpointAddress = HACKRF_EP_IN;
    s_dev.rx_xfer->callback        = rx_transfer_cb;
    s_dev.rx_xfer->context         = &s_dev;
    s_dev.rx_xfer->num_bytes       = HACKRF_BULK_SIZE;

    s_inited = true;
    ESP_LOGI(TAG, "HackRF USB driver ready");
    return ESP_OK;
}

void hackrf_usb_deinit(void)
{
    if (!s_inited) return;
    hackrf_stop_rx(&s_dev);
    usb_host_transfer_free(s_dev.rx_xfer);
    usb_host_interface_release(s_dev.client, s_dev.dev_hdl, HACKRF_IFACE);
    usb_host_device_close(s_dev.client, s_dev.dev_hdl);
    usb_host_client_deregister(s_dev.client);
    usb_host_uninstall();
    s_inited = false;
}

hackrf_device_t *hackrf_usb_get_device(void)
{
    return s_inited ? &s_dev : NULL;
}

/* ── Configuration helpers ─────────────────────────────────────────────── */

esp_err_t hackrf_set_freq(hackrf_device_t *dev, uint64_t freq_hz)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (freq_hz < HACKRF_FREQ_MIN_HZ || freq_hz > HACKRF_FREQ_MAX_HZ) {
        ESP_LOGE(TAG, "Frequency out of range: %llu Hz", freq_hz);
        return ESP_ERR_INVALID_ARG;
    }

    /* libhackrf packs frequency as two 32-bit little-endian words:
     *   [0..3] MHz part,  [4..7] Hz remainder */
    uint8_t buf[8];
    uint32_t mhz = (uint32_t)(freq_hz / 1000000ULL);
    uint32_t hz  = (uint32_t)(freq_hz % 1000000ULL);
    buf[0] = (mhz >>  0) & 0xFF;
    buf[1] = (mhz >>  8) & 0xFF;
    buf[2] = (mhz >> 16) & 0xFF;
    buf[3] = (mhz >> 24) & 0xFF;
    buf[4] = (hz  >>  0) & 0xFF;
    buf[5] = (hz  >>  8) & 0xFF;
    buf[6] = (hz  >> 16) & 0xFF;
    buf[7] = (hz  >> 24) & 0xFF;

    esp_err_t ret = ctrl_out(dev, HACKRF_VENDOR_REQUEST_SET_FREQ, 0, 0, buf, sizeof(buf));
    if (ret == ESP_OK) {
        dev->freq_hz = freq_hz;
        ESP_LOGI(TAG, "Frequency set to %llu Hz", freq_hz);
    }
    return ret;
}

esp_err_t hackrf_set_sample_rate(hackrf_device_t *dev, uint32_t sample_rate)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    /* libhackrf packs as divider pair: freq_mhz / divider */
    uint8_t buf[8];
    uint32_t freq = sample_rate * 2;   /* double – then divide by 2 */
    uint32_t div  = 2;
    buf[0] = (freq >>  0) & 0xFF;
    buf[1] = (freq >>  8) & 0xFF;
    buf[2] = (freq >> 16) & 0xFF;
    buf[3] = (freq >> 24) & 0xFF;
    buf[4] = (div  >>  0) & 0xFF;
    buf[5] = (div  >>  8) & 0xFF;
    buf[6] = (div  >> 16) & 0xFF;
    buf[7] = (div  >> 24) & 0xFF;

    esp_err_t ret = ctrl_out(dev, HACKRF_VENDOR_REQUEST_SET_SAMPLE_RATE, 0, 0, buf, sizeof(buf));
    if (ret == ESP_OK) {
        dev->sample_rate = sample_rate;
        ESP_LOGI(TAG, "Sample rate set to %u sps", sample_rate);
    }
    return ret;
}

esp_err_t hackrf_set_lna_gain(hackrf_device_t *dev, uint8_t gain_db)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    gain_db &= ~0x07;   /* round down to multiple of 8 */
    if (gain_db > HACKRF_LNA_GAIN_MAX) gain_db = HACKRF_LNA_GAIN_MAX;
    uint8_t result;
    esp_err_t ret = ctrl_out(dev, HACKRF_VENDOR_REQUEST_SET_LNA_GAIN, 0, gain_db, &result, 1);
    if (ret == ESP_OK) ESP_LOGI(TAG, "LNA gain set to %u dB", gain_db);
    return ret;
}

esp_err_t hackrf_set_vga_gain(hackrf_device_t *dev, uint8_t gain_db)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    gain_db &= ~0x01;   /* round down to even */
    if (gain_db > HACKRF_VGA_GAIN_MAX) gain_db = HACKRF_VGA_GAIN_MAX;
    uint8_t result;
    esp_err_t ret = ctrl_out(dev, HACKRF_VENDOR_REQUEST_SET_VGA_GAIN, 0, gain_db, &result, 1);
    if (ret == ESP_OK) ESP_LOGI(TAG, "VGA gain set to %u dB", gain_db);
    return ret;
}

esp_err_t hackrf_set_txvga_gain(hackrf_device_t *dev, uint8_t gain_db)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (gain_db > HACKRF_TXVGA_GAIN_MAX) gain_db = HACKRF_TXVGA_GAIN_MAX;
    uint8_t result;
    esp_err_t ret = ctrl_out(dev, HACKRF_VENDOR_REQUEST_SET_TXVGA_GAIN, 0, gain_db, &result, 1);
    if (ret == ESP_OK) ESP_LOGI(TAG, "TXVGA gain set to %u dB", gain_db);
    return ret;
}

esp_err_t hackrf_set_amp_enable(hackrf_device_t *dev, bool enable)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    return ctrl_out(dev, HACKRF_VENDOR_REQUEST_AMP_ENABLE, 0, enable ? 1 : 0, NULL, 0);
}

esp_err_t hackrf_set_antenna_enable(hackrf_device_t *dev, bool enable)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    return ctrl_out(dev, HACKRF_VENDOR_REQUEST_ANTENNA_ENABLE, 0, enable ? 1 : 0, NULL, 0);
}

/* ── RX streaming ──────────────────────────────────────────────────────── */

esp_err_t hackrf_start_rx(hackrf_device_t *dev, hackrf_rx_cb_t cb, void *user_ctx)
{
    if (!dev || !cb) return ESP_ERR_INVALID_ARG;
    if (dev->rx_running) return ESP_ERR_INVALID_STATE;

    dev->rx_cb      = cb;
    dev->rx_ctx     = user_ctx;
    dev->rx_running = true;

    /* Put HackRF into RX mode */
    esp_err_t ret = ctrl_out(dev, HACKRF_VENDOR_REQUEST_SET_TRANSCEIVER_MODE,
                               0, TRANSCEIVER_MODE_RX, NULL, 0);
    if (ret != ESP_OK) return ret;

    /* Submit first bulk transfer */
    return usb_host_transfer_submit(dev->rx_xfer);
}

esp_err_t hackrf_stop_rx(hackrf_device_t *dev)
{
    if (!dev || !dev->rx_running) return ESP_OK;
    dev->rx_running = false;
    ctrl_out(dev, HACKRF_VENDOR_REQUEST_SET_TRANSCEIVER_MODE,
             0, TRANSCEIVER_MODE_OFF, NULL, 0);
    return ESP_OK;
}

/* ── TX ─────────────────────────────────────────────────────────────────── */

esp_err_t hackrf_transmit(hackrf_device_t *dev, const int8_t *buf, uint32_t len)
{
    if (!dev || !buf || len == 0) return ESP_ERR_INVALID_ARG;

    /* Switch to TX mode */
    esp_err_t ret = ctrl_out(dev, HACKRF_VENDOR_REQUEST_SET_TRANSCEIVER_MODE,
                               0, TRANSCEIVER_MODE_TX, NULL, 0);
    if (ret != ESP_OK) return ret;

    /* Allocate and submit bulk OUT transfer */
    usb_transfer_t *tx_xfer = NULL;
    usb_host_transfer_alloc(len, 0, &tx_xfer);
    tx_xfer->device_handle    = dev->dev_hdl;
    tx_xfer->bEndpointAddress = HACKRF_EP_OUT;
    tx_xfer->num_bytes        = len;
    memcpy(tx_xfer->data_buffer, buf, len);
    tx_xfer->callback = NULL;   /* synchronous */

    ret = usb_host_transfer_submit(tx_xfer);
    usb_host_transfer_free(tx_xfer);

    /* Return to RX mode (or off) */
    ctrl_out(dev, HACKRF_VENDOR_REQUEST_SET_TRANSCEIVER_MODE,
             0, TRANSCEIVER_MODE_OFF, NULL, 0);
    return ret;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Internal helpers                                                          */
/* ────────────────────────────────────────────────────────────────────────── */

static esp_err_t ctrl_out(hackrf_device_t *dev, uint8_t request,
                            uint16_t value, uint16_t index,
                            const uint8_t *data, uint16_t len)
{
    usb_transfer_t *ctrl = NULL;
    uint16_t total = sizeof(usb_setup_packet_t) + len;
    usb_host_transfer_alloc(total, 0, &ctrl);

    usb_setup_packet_t *setup = (usb_setup_packet_t *)ctrl->data_buffer;
    setup->bmRequestType = USB_BM_REQUEST_TYPE_DIR_OUT |
                           USB_BM_REQUEST_TYPE_TYPE_VENDOR |
                           USB_BM_REQUEST_TYPE_RECIP_DEVICE;
    setup->bRequest      = request;
    setup->wValue        = value;
    setup->wIndex        = index;
    setup->wLength       = len;

    if (data && len > 0) {
        memcpy(ctrl->data_buffer + sizeof(usb_setup_packet_t), data, len);
    }

    ctrl->device_handle    = dev->dev_hdl;
    ctrl->bEndpointAddress = 0;   /* control endpoint */
    ctrl->num_bytes        = total;
    ctrl->callback         = NULL;

    esp_err_t ret = usb_host_transfer_submit_control(dev->client, ctrl);
    usb_host_transfer_free(ctrl);
    return ret;
}

static void rx_transfer_cb(usb_transfer_t *xfer)
{
    hackrf_device_t *dev = (hackrf_device_t *)xfer->context;
    if (!dev->rx_running) return;

    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        if (dev->rx_cb) {
            int stop = dev->rx_cb((const int8_t *)xfer->data_buffer,
                                   xfer->actual_num_bytes, dev->rx_ctx);
            if (stop) {
                hackrf_stop_rx(dev);
                return;
            }
        }
        /* Re-submit for continuous streaming */
        usb_host_transfer_submit(xfer);
    } else {
        ESP_LOGW(TAG, "RX transfer error: %d", xfer->status);
        if (dev->rx_running) {
            usb_host_transfer_submit(xfer);
        }
    }
}

static void usb_host_task(void *arg)
{
    while (true) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            break;
        }
    }
    usb_host_uninstall();
    vTaskDelete(NULL);
}
