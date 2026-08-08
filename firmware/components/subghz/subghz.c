/**
 * @file subghz.c
 * @brief Sub-GHz transceiver implementation
 *
 * Leverages the HackRF USB driver to perform sub-GHz signal operations.
 * OOK encoding/decoding is performed in software on the ESP32 using the
 * captured I/Q streams.
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hackrf_usb.h"
#include "subghz.h"

static const char *TAG = "SUBGHZ";

/* ISM band scan list */
static const uint64_t SCAN_FREQS[] = {
    SUBGHZ_FREQ_300MHZ,
    SUBGHZ_FREQ_315MHZ,
    SUBGHZ_FREQ_433MHZ,
    SUBGHZ_FREQ_868MHZ,
    SUBGHZ_FREQ_915MHZ,
};
#define NUM_SCAN_FREQS (sizeof(SCAN_FREQS) / sizeof(SCAN_FREQS[0]))

/* ── Capture context ────────────────────────────────────────────────────── */
typedef struct {
    subghz_signal_t *sig;
    uint32_t         max_bytes;
    uint32_t         written;
    bool             done;
} capture_ctx_t;

/* ── RX callback for raw capture ─────────────────────────────────────────── */
static int capture_rx_cb(const int8_t *buf, uint32_t len, void *user_ctx)
{
    capture_ctx_t *ctx = (capture_ctx_t *)user_ctx;
    if (ctx->done) return 1;

    uint32_t space = ctx->max_bytes - ctx->written;
    uint32_t copy  = (len < space) ? len : space;
    memcpy(ctx->sig->raw_iq + ctx->written, buf, copy);
    ctx->written += copy;

    if (ctx->written >= ctx->max_bytes) {
        ctx->done = true;
        return 1;   /* stop streaming */
    }
    return 0;
}

/* ── OOK energy detector ─────────────────────────────────────────────────── */
static float iq_energy(const int8_t *buf, uint32_t len)
{
    float sum = 0.0f;
    for (uint32_t i = 0; i + 1 < len; i += 2) {
        float I = (float)buf[i];
        float Q = (float)buf[i + 1];
        sum += I * I + Q * Q;
    }
    return sum / (float)(len / 2);
}

/* ── Public API ─────────────────────────────────────────────────────────── */

esp_err_t subghz_init(void)
{
    /* Nothing to init beyond the HackRF (already done in main) */
    ESP_LOGI(TAG, "Sub-GHz module ready");
    return ESP_OK;
}

esp_err_t subghz_scan(subghz_signal_cb_t cb, void *user_ctx, uint32_t dwell_ms)
{
    if (!cb) return ESP_ERR_INVALID_ARG;

    hackrf_device_t *dev = hackrf_usb_get_device();
    if (!dev) return ESP_ERR_NOT_FOUND;

    /* Small capture buffer per frequency */
    uint32_t cap_bytes = (dwell_ms * 2000000ULL * 2) / 1000; /* 2 Msps × 2 bytes */
    cap_bytes = (cap_bytes < 4096) ? 4096 : cap_bytes;

    for (size_t fi = 0; fi < NUM_SCAN_FREQS; fi++) {
        uint64_t freq = SCAN_FREQS[fi];
        ESP_LOGI(TAG, "Scanning %.3f MHz ...", (double)freq / 1e6);

        hackrf_set_freq(dev, freq);
        hackrf_set_sample_rate(dev, HACKRF_SAMPLE_RATE_2M);
        hackrf_set_lna_gain(dev, 24);
        hackrf_set_vga_gain(dev, 20);

        subghz_signal_t sig = {
            .freq_hz    = freq,
            .modulation = SUBGHZ_MOD_RAW,
            .sample_rate = HACKRF_SAMPLE_RATE_2M,
        };
        sig.raw_iq = (uint8_t *)malloc(cap_bytes);
        if (!sig.raw_iq) continue;
        sig.raw_iq_len = cap_bytes;

        capture_ctx_t ctx = { .sig = &sig, .max_bytes = cap_bytes };
        hackrf_start_rx(dev, capture_rx_cb, &ctx);
        vTaskDelay(pdMS_TO_TICKS(dwell_ms + 50));
        hackrf_stop_rx(dev);

        sig.raw_iq_len = ctx.written;

        /* Simple energy threshold detection */
        float energy = iq_energy((const int8_t *)sig.raw_iq, ctx.written);
        ESP_LOGI(TAG, "  %.3f MHz energy=%.1f", (double)freq / 1e6, (double)energy);
        if (energy > 500.0f) {
            snprintf(sig.description, sizeof(sig.description),
                     "Signal at %.3f MHz (energy=%.0f)", (double)freq / 1e6, (double)energy);
            cb(&sig, user_ctx);
        }
        free(sig.raw_iq);
    }
    return ESP_OK;
}

esp_err_t subghz_receive(uint64_t freq_hz, subghz_modulation_t mod,
                           uint32_t timeout_ms, subghz_signal_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    hackrf_device_t *dev = hackrf_usb_get_device();
    if (!dev) return ESP_ERR_NOT_FOUND;

    uint32_t cap_bytes = (uint32_t)((uint64_t)timeout_ms * 2000000ULL * 2 / 1000);
    if (cap_bytes < 8192) cap_bytes = 8192;

    out->freq_hz    = freq_hz;
    out->modulation = mod;
    out->sample_rate = HACKRF_SAMPLE_RATE_2M;
    out->raw_iq     = (uint8_t *)malloc(cap_bytes);
    if (!out->raw_iq) return ESP_ERR_NO_MEM;
    out->raw_iq_len = cap_bytes;

    hackrf_set_freq(dev, freq_hz);
    hackrf_set_sample_rate(dev, HACKRF_SAMPLE_RATE_2M);
    hackrf_set_lna_gain(dev, 32);
    hackrf_set_vga_gain(dev, 24);

    capture_ctx_t ctx = { .sig = out, .max_bytes = cap_bytes };
    hackrf_start_rx(dev, capture_rx_cb, &ctx);
    vTaskDelay(pdMS_TO_TICKS(timeout_ms));
    hackrf_stop_rx(dev);

    out->raw_iq_len = ctx.written;

    float energy = iq_energy((const int8_t *)out->raw_iq, ctx.written);
    if (energy < 200.0f) {
        free(out->raw_iq);
        out->raw_iq = NULL;
        return ESP_ERR_TIMEOUT;
    }

    snprintf(out->description, sizeof(out->description),
             "Captured %.3f MHz mod=%d len=%u",
             (double)freq_hz / 1e6, (int)mod, (unsigned)ctx.written);
    ESP_LOGI(TAG, "%s", out->description);
    return ESP_OK;
}

esp_err_t subghz_replay(const subghz_signal_t *sig)
{
    if (!sig || !sig->raw_iq || sig->raw_iq_len == 0) return ESP_ERR_INVALID_ARG;
    hackrf_device_t *dev = hackrf_usb_get_device();
    if (!dev) return ESP_ERR_NOT_FOUND;

    ESP_LOGI(TAG, "Replaying signal at %.3f MHz (%u bytes)",
             (double)sig->freq_hz / 1e6, (unsigned)sig->raw_iq_len);

    hackrf_set_freq(dev, sig->freq_hz);
    hackrf_set_sample_rate(dev, sig->sample_rate);
    hackrf_set_txvga_gain(dev, 20);
    hackrf_set_amp_enable(dev, true);

    return hackrf_transmit(dev, (const int8_t *)sig->raw_iq, sig->raw_iq_len);
}

esp_err_t subghz_transmit_ook(uint64_t freq_hz,
                                const uint32_t *pulses_us, uint32_t num_pulses)
{
    if (!pulses_us || num_pulses == 0) return ESP_ERR_INVALID_ARG;
    hackrf_device_t *dev = hackrf_usb_get_device();
    if (!dev) return ESP_ERR_NOT_FOUND;

    /* Calculate total sample count at 2 Msps */
    uint32_t sps     = HACKRF_SAMPLE_RATE_2M;
    uint32_t total_samples = 0;
    for (uint32_t i = 0; i < num_pulses; i++) {
        total_samples += (uint32_t)((uint64_t)pulses_us[i] * sps / 1000000ULL);
    }

    int8_t *buf = (int8_t *)calloc(total_samples * 2, 1);
    if (!buf) return ESP_ERR_NO_MEM;

    uint32_t pos = 0;
    for (uint32_t i = 0; i < num_pulses; i++) {
        uint32_t samp = (uint32_t)((uint64_t)pulses_us[i] * sps / 1000000ULL);
        bool mark = (i % 2 == 0);   /* even = mark, odd = space */
        for (uint32_t s = 0; s < samp; s++) {
            buf[(pos + s) * 2]     = mark ? 127 : 0;   /* I */
            buf[(pos + s) * 2 + 1] = 0;                 /* Q */
        }
        pos += samp;
    }

    hackrf_set_freq(dev, freq_hz);
    hackrf_set_sample_rate(dev, sps);
    hackrf_set_txvga_gain(dev, 25);
    hackrf_set_amp_enable(dev, true);

    esp_err_t ret = hackrf_transmit(dev, buf, total_samples * 2);
    free(buf);
    return ret;
}

void subghz_signal_free(subghz_signal_t *sig)
{
    if (sig && sig->raw_iq) {
        free(sig->raw_iq);
        sig->raw_iq     = NULL;
        sig->raw_iq_len = 0;
    }
}
