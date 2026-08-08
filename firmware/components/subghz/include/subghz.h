/**
 * @file subghz.h
 * @brief Sub-GHz transceiver module
 *
 * Uses the HackRF as the RF front-end to implement Flipper Zero-style
 * sub-GHz operations:
 *   - Scan for active signals across common ISM bands
 *   - Receive and decode OOK / ASK / FSK modulated signals
 *   - Replay captured signals
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Common Sub-GHz frequency presets (Hz). */
#define SUBGHZ_FREQ_300MHZ   300000000ULL
#define SUBGHZ_FREQ_315MHZ   315000000ULL
#define SUBGHZ_FREQ_433MHZ   433920000ULL
#define SUBGHZ_FREQ_868MHZ   868350000ULL
#define SUBGHZ_FREQ_915MHZ   915000000ULL

/** Modulation types. */
typedef enum {
    SUBGHZ_MOD_OOK = 0,   /**< On-Off Keying          */
    SUBGHZ_MOD_ASK,        /**< Amplitude-Shift Keying */
    SUBGHZ_MOD_FSK,        /**< Frequency-Shift Keying */
    SUBGHZ_MOD_RAW,        /**< Raw I/Q capture        */
} subghz_modulation_t;

/** Captured signal record. */
typedef struct {
    uint64_t            freq_hz;
    subghz_modulation_t modulation;
    uint8_t            *raw_iq;        /**< Heap-allocated I/Q buffer */
    uint32_t            raw_iq_len;    /**< Byte length of raw_iq    */
    uint32_t            sample_rate;
    char                description[64];
} subghz_signal_t;

/**
 * @brief Callback invoked when a signal is detected during scanning.
 * @param sig  Decoded/raw signal (caller must NOT free).
 * @param ctx  User context pointer.
 */
typedef void (*subghz_signal_cb_t)(const subghz_signal_t *sig, void *ctx);

/**
 * @brief Initialise the sub-GHz subsystem.
 * @return ESP_OK on success.
 */
esp_err_t subghz_init(void);

/**
 * @brief Scan all preset ISM bands and report found signals.
 * @param cb         Callback for detected signals.
 * @param user_ctx   Caller context.
 * @param dwell_ms   Dwell time per frequency in milliseconds.
 * @return ESP_OK on success.
 */
esp_err_t subghz_scan(subghz_signal_cb_t cb, void *user_ctx, uint32_t dwell_ms);

/**
 * @brief Listen on a single frequency and capture a signal.
 * @param freq_hz    Centre frequency.
 * @param mod        Expected modulation.
 * @param timeout_ms Maximum capture duration.
 * @param out        Populated on success; caller must free out->raw_iq.
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if nothing detected.
 */
esp_err_t subghz_receive(uint64_t freq_hz, subghz_modulation_t mod,
                          uint32_t timeout_ms, subghz_signal_t *out);

/**
 * @brief Replay a previously captured signal.
 * @param sig  Signal to transmit.
 * @return ESP_OK on success.
 */
esp_err_t subghz_replay(const subghz_signal_t *sig);

/**
 * @brief Transmit a custom OOK pulse sequence.
 * @param freq_hz    Centre frequency.
 * @param pulses_us  Array of pulse widths in microseconds (alternating
 *                   mark/space, starting with mark).
 * @param num_pulses Number of elements in pulses_us.
 * @return ESP_OK on success.
 */
esp_err_t subghz_transmit_ook(uint64_t freq_hz,
                               const uint32_t *pulses_us, uint32_t num_pulses);

/**
 * @brief Free heap memory held by a subghz_signal_t.
 * @param sig Signal to free.
 */
void subghz_signal_free(subghz_signal_t *sig);

#ifdef __cplusplus
}
#endif
