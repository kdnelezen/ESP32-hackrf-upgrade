/**
 * @file spectrum_analyzer.c
 * @brief Spectrum analyser – magnitude-only FFT over HackRF I/Q data.
 *
 * Uses a simple Goertzel-based power estimation per bin to avoid
 * requiring a full FFT library, keeping the binary small.
 */

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hackrf_usb.h"
#include "spectrum_analyzer.h"

static const char *TAG = "SPECTRUM";

#define SPECTRUM_CAPTURE_MS  200
#define SPECTRUM_SCALE_COLS  40

static hackrf_device_t *s_dev = NULL;

/* ── Capture context ────────────────────────────────────────────────────── */
typedef struct {
    int8_t   *buf;
    uint32_t  max_bytes;
    uint32_t  written;
} spec_ctx_t;

static int spec_rx_cb(const int8_t *buf, uint32_t len, void *user_ctx)
{
    spec_ctx_t *ctx = (spec_ctx_t *)user_ctx;
    uint32_t space = ctx->max_bytes - ctx->written;
    uint32_t copy  = (len < space) ? len : space;
    memcpy(ctx->buf + ctx->written, buf, copy);
    ctx->written += copy;
    return (ctx->written >= ctx->max_bytes) ? 1 : 0;
}

/* ── Goertzel power at normalised frequency k/N ─────────────────────────── */
static float goertzel_power(const int8_t *iq, uint32_t n_samples, float norm_freq)
{
    float omega = 2.0f * (float)M_PI * norm_freq;
    float coeff = 2.0f * cosf(omega);
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f;
    for (uint32_t i = 0; i < n_samples; i++) {
        float sample = (float)iq[i * 2];   /* I channel only for speed */
        s0 = sample + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void spectrum_analyzer_init(hackrf_device_t *dev)
{
    s_dev = dev;
    ESP_LOGI(TAG, "Spectrum analyser initialised");
}

void spectrum_analyzer_run(uint64_t centre_hz, uint32_t bw_hz, uint32_t num_bins)
{
    if (!s_dev) { printf("Spectrum: no device\n"); return; }
    if (num_bins < 2 || num_bins > 256) num_bins = 64;

    uint32_t sps   = (bw_hz < HACKRF_SAMPLE_RATE_2M) ? HACKRF_SAMPLE_RATE_2M : bw_hz;
    uint32_t bytes = (uint32_t)((uint64_t)SPECTRUM_CAPTURE_MS * sps * 2 / 1000);
    if (bytes < 4096) bytes = 4096;

    int8_t *cap = (int8_t *)malloc(bytes);
    if (!cap) { printf("Spectrum: OOM\n"); return; }

    hackrf_set_freq(s_dev, centre_hz);
    hackrf_set_sample_rate(s_dev, sps);
    hackrf_set_lna_gain(s_dev, 24);
    hackrf_set_vga_gain(s_dev, 20);

    spec_ctx_t ctx = { .buf = cap, .max_bytes = bytes };
    hackrf_start_rx(s_dev, spec_rx_cb, &ctx);
    vTaskDelay(pdMS_TO_TICKS(SPECTRUM_CAPTURE_MS + 50));
    hackrf_stop_rx(s_dev);

    uint32_t n_samples = ctx.written / 2;

    /* Compute power for each bin */
    float powers[256] = {0};
    float max_power   = 1e-9f;
    for (uint32_t k = 0; k < num_bins; k++) {
        float nf   = (float)k / (float)num_bins - 0.5f;   /* [-0.5, +0.5) */
        powers[k]  = goertzel_power(cap, n_samples, nf + 0.5f);
        if (powers[k] > max_power) max_power = powers[k];
    }

    /* ASCII waterfall display */
    printf("\nSpectrum @ %.3f MHz  BW=%.2f MHz  %u bins\n",
           (double)centre_hz / 1e6, (double)bw_hz / 1e6, (unsigned)num_bins);
    printf("%-10s %s\n", "Freq(MHz)", "Power");

    double start_hz = (double)centre_hz - (double)bw_hz / 2.0;
    double step_hz  = (double)bw_hz / (double)num_bins;

    for (uint32_t k = 0; k < num_bins; k++) {
        double f = start_hz + k * step_hz;
        int bars = (int)(SPECTRUM_SCALE_COLS * powers[k] / max_power);
        printf("%10.3f |", f / 1e6);
        for (int b = 0; b < bars; b++) putchar('#');
        printf("\n");
    }
    putchar('\n');
    free(cap);
}
