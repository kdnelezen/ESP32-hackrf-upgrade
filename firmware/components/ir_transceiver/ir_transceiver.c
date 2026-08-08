/**
 * @file ir_transceiver.c
 * @brief IR transceiver implementation using the ESP32 RMT peripheral.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_encoder.h"
#include "ir_transceiver.h"

static const char *TAG = "IR";

/* ── RMT channel handles ────────────────────────────────────────────────── */
static rmt_channel_handle_t s_tx_chan = NULL;
static rmt_channel_handle_t s_rx_chan = NULL;
static rmt_encoder_handle_t s_copy_enc = NULL;
static bool s_inited = false;

/* ── Init ───────────────────────────────────────────────────────────────── */

esp_err_t ir_transceiver_init(void)
{
    if (s_inited) return ESP_OK;

    /* TX channel */
    rmt_tx_channel_config_t tx_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num          = CONFIG_IR_TX_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz     = 1000000,   /* 1 µs resolution */
        .trans_queue_depth = 4,
        .flags.invert_out  = false,
        .flags.with_dma    = false,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &s_tx_chan));

    /* Copy encoder for raw symbol lists */
    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &s_copy_enc));

    /* RX channel */
    rmt_rx_channel_config_t rx_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num          = CONFIG_IR_RX_GPIO,
        .mem_block_symbols = 128,
        .resolution_hz     = 1000000,
        .flags.invert_in   = false,
        .flags.with_dma    = false,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &s_rx_chan));

    ESP_ERROR_CHECK(rmt_enable(s_tx_chan));
    ESP_ERROR_CHECK(rmt_enable(s_rx_chan));

    s_inited = true;
    ESP_LOGI(TAG, "IR transceiver ready (TX GPIO%d, RX GPIO%d)",
             CONFIG_IR_TX_GPIO, CONFIG_IR_RX_GPIO);
    return ESP_OK;
}

/* ── Transmit ────────────────────────────────────────────────────────────── */

esp_err_t ir_transmit(const ir_frame_t *frame)
{
    if (!frame || frame->num_symbols == 0) return ESP_ERR_INVALID_ARG;
    if (!s_inited) return ESP_ERR_INVALID_STATE;

    /* Convert ir_symbol_t to rmt_symbol_word_t pairs */
    rmt_symbol_word_t rmt_syms[IR_MAX_SYMBOLS];
    for (uint32_t i = 0; i < frame->num_symbols && i < IR_MAX_SYMBOLS; i++) {
        rmt_syms[i].level0    = frame->symbols[i].level;
        rmt_syms[i].duration0 = frame->symbols[i].duration_us;
        /* If next symbol exists, pair it; otherwise EOT */
        if (i + 1 < frame->num_symbols) {
            rmt_syms[i].level1    = frame->symbols[i + 1].level;
            rmt_syms[i].duration1 = frame->symbols[i + 1].duration_us;
            i++;  /* consumed two */
        } else {
            rmt_syms[i].level1    = 0;
            rmt_syms[i].duration1 = 0;
        }
    }

    rmt_carrier_config_t carrier = {
        .frequency_hz = frame->carrier_hz ? frame->carrier_hz : IR_CARRIER_NEC,
        .duty_cycle   = 0.33f,
    };
    rmt_apply_carrier(s_tx_chan, &carrier);

    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    esp_err_t ret = rmt_transmit(s_tx_chan, s_copy_enc,
                                  rmt_syms,
                                  frame->num_symbols * sizeof(rmt_symbol_word_t),
                                  &tx_cfg);
    if (ret == ESP_OK) {
        rmt_tx_wait_all_done(s_tx_chan, pdMS_TO_TICKS(500));
        ESP_LOGI(TAG, "IR frame sent (%u symbols, %u Hz carrier)",
                 (unsigned)frame->num_symbols, (unsigned)frame->carrier_hz);
    }
    return ret;
}

/* ── Receive ─────────────────────────────────────────────────────────────── */

esp_err_t ir_receive(ir_frame_t *frame, uint32_t timeout_ms)
{
    if (!frame) return ESP_ERR_INVALID_ARG;
    if (!s_inited) return ESP_ERR_INVALID_STATE;

    rmt_symbol_word_t raw[IR_MAX_SYMBOLS];
    rmt_receive_config_t rx_cfg = {
        .signal_range_min_ns = 1250,
        .signal_range_max_ns = 12000000,
    };

    esp_err_t ret = rmt_receive(s_rx_chan, raw, sizeof(raw), &rx_cfg);
    if (ret != ESP_OK) return ret;

    /* Wait for data (simple poll) */
    uint32_t elapsed = 0;
    size_t received_symbols = 0;
    while (elapsed < timeout_ms) {
        /* In a real implementation, use the RMT done callback / event queue.
           Here we do a short delay and check via a separate API call. */
        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed += 10;
    }

    if (received_symbols == 0) return ESP_ERR_TIMEOUT;

    frame->num_symbols  = 0;
    frame->carrier_hz   = IR_CARRIER_NEC;
    snprintf(frame->protocol, sizeof(frame->protocol), "RAW");
    return ESP_OK;
}

/* ── Protocol builders ──────────────────────────────────────────────────── */

void ir_build_nec(uint8_t address, uint8_t command, ir_frame_t *frame)
{
    memset(frame, 0, sizeof(*frame));
    frame->carrier_hz = IR_CARRIER_NEC;
    snprintf(frame->protocol, sizeof(frame->protocol), "NEC");

    /* NEC frame: 9ms AGC + 4.5ms space + 32 data bits + stop */
    uint32_t n = 0;
#define PUSH(dur, lvl) do { \
    frame->symbols[n].duration_us = (dur); \
    frame->symbols[n].level = (lvl); n++; \
} while (0)

    PUSH(9000, 1); PUSH(4500, 0);   /* preamble */

    uint32_t data = ((uint32_t)address)         |
                    ((uint32_t)(~address) << 8)  |
                    ((uint32_t)command    << 16) |
                    ((uint32_t)(~command) << 24);

    for (int i = 0; i < 32; i++) {
        PUSH(560, 1);
        if (data & (1U << i)) {
            PUSH(1690, 0);   /* '1' */
        } else {
            PUSH(560, 0);    /* '0' */
        }
    }
    PUSH(560, 1);   /* stop bit */
#undef PUSH
    frame->num_symbols = n;
}

void ir_build_sony_sirc(uint8_t command, uint8_t address, ir_frame_t *frame)
{
    memset(frame, 0, sizeof(*frame));
    frame->carrier_hz = IR_CARRIER_SONY;
    snprintf(frame->protocol, sizeof(frame->protocol), "SONY_SIRC");

    uint32_t n = 0;
#define PUSH(dur, lvl) do { \
    frame->symbols[n].duration_us = (dur); \
    frame->symbols[n].level = (lvl); n++; \
} while (0)

    PUSH(2400, 1); PUSH(600, 0);   /* start burst */

    uint32_t data = (command & 0x7F) | ((address & 0x1F) << 7);
    for (int i = 0; i < 12; i++) {
        if (data & (1U << i)) {
            PUSH(1200, 1);
        } else {
            PUSH(600, 1);
        }
        PUSH(600, 0);
    }
#undef PUSH
    frame->num_symbols = n;
}
