/**
 * @file signal_replayer.c
 * @brief Signal capture-and-replay implementation.
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hackrf_usb.h"
#include "signal_replayer.h"

static const char *TAG = "REPLAYER";

/* ── Library ────────────────────────────────────────────────────────────── */
typedef struct {
    uint64_t  freq_hz;
    uint32_t  sample_rate;
    int8_t   *buf;
    uint32_t  len;
    bool      valid;
} replay_entry_t;

static replay_entry_t s_lib[REPLAYER_MAX_SIGNALS];
static hackrf_device_t *s_dev = NULL;

/* ── Capture callback ───────────────────────────────────────────────────── */
typedef struct {
    int8_t   *buf;
    uint32_t  max;
    uint32_t  written;
} cap_ctx_t;

static int cap_rx_cb(const int8_t *buf, uint32_t len, void *user_ctx)
{
    cap_ctx_t *ctx = (cap_ctx_t *)user_ctx;
    uint32_t space = ctx->max - ctx->written;
    uint32_t copy  = (len < space) ? len : space;
    memcpy(ctx->buf + ctx->written, buf, copy);
    ctx->written += copy;
    return (ctx->written >= ctx->max) ? 1 : 0;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void signal_replayer_init(hackrf_device_t *dev)
{
    s_dev = dev;
    memset(s_lib, 0, sizeof(s_lib));
    ESP_LOGI(TAG, "Signal replayer initialised (%d slots)", REPLAYER_MAX_SIGNALS);
}

esp_err_t signal_replayer_capture(uint64_t freq_hz, uint32_t duration_ms, uint8_t slot)
{
    if (slot >= REPLAYER_MAX_SIGNALS) return ESP_ERR_INVALID_ARG;
    if (!s_dev) return ESP_ERR_INVALID_STATE;

    uint32_t sps   = HACKRF_SAMPLE_RATE_2M;
    uint32_t bytes = (uint32_t)((uint64_t)duration_ms * sps * 2 / 1000);
    if (bytes < 4096) bytes = 4096;

    /* Free old entry */
    if (s_lib[slot].valid && s_lib[slot].buf) {
        free(s_lib[slot].buf);
        s_lib[slot].buf   = NULL;
        s_lib[slot].valid = false;
    }

    int8_t *buf = (int8_t *)malloc(bytes);
    if (!buf) return ESP_ERR_NO_MEM;

    hackrf_set_freq(s_dev, freq_hz);
    hackrf_set_sample_rate(s_dev, sps);
    hackrf_set_lna_gain(s_dev, 32);
    hackrf_set_vga_gain(s_dev, 24);

    cap_ctx_t ctx = { .buf = buf, .max = bytes };
    hackrf_start_rx(s_dev, cap_rx_cb, &ctx);
    vTaskDelay(pdMS_TO_TICKS(duration_ms + 50));
    hackrf_stop_rx(s_dev);

    s_lib[slot].freq_hz     = freq_hz;
    s_lib[slot].sample_rate = sps;
    s_lib[slot].buf         = buf;
    s_lib[slot].len         = ctx.written;
    s_lib[slot].valid       = true;

    ESP_LOGI(TAG, "Slot %u captured %.3f MHz  %u bytes",
             slot, (double)freq_hz / 1e6, (unsigned)ctx.written);
    return ESP_OK;
}

esp_err_t signal_replayer_play(uint8_t slot)
{
    if (slot >= REPLAYER_MAX_SIGNALS) return ESP_ERR_INVALID_ARG;
    if (!s_lib[slot].valid || !s_lib[slot].buf) {
        ESP_LOGW(TAG, "Slot %u is empty", slot);
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_dev) return ESP_ERR_INVALID_STATE;

    ESP_LOGI(TAG, "Replaying slot %u: %.3f MHz  %u bytes",
             slot, (double)s_lib[slot].freq_hz / 1e6, (unsigned)s_lib[slot].len);

    hackrf_set_freq(s_dev, s_lib[slot].freq_hz);
    hackrf_set_sample_rate(s_dev, s_lib[slot].sample_rate);
    hackrf_set_txvga_gain(s_dev, 20);
    hackrf_set_amp_enable(s_dev, true);

    return hackrf_transmit(s_dev, s_lib[slot].buf, s_lib[slot].len);
}

void signal_replayer_list(void)
{
    printf("Signal Replay Library (%d slots):\n", REPLAYER_MAX_SIGNALS);
    for (int i = 0; i < REPLAYER_MAX_SIGNALS; i++) {
        if (s_lib[i].valid) {
            printf("  [%d] %.3f MHz  %u bytes\n",
                   i, (double)s_lib[i].freq_hz / 1e6, (unsigned)s_lib[i].len);
        } else {
            printf("  [%d] (empty)\n", i);
        }
    }
}
