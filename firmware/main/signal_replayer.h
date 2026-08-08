/**
 * @file signal_replayer.h
 * @brief Signal capture-and-replay engine
 */

#pragma once

#include <stdint.h>
#include "hackrf_usb.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of signals stored in the replay library. */
#define REPLAYER_MAX_SIGNALS 8

/**
 * @brief Initialise the signal replayer.
 * @param dev HackRF device handle.
 */
void signal_replayer_init(hackrf_device_t *dev);

/**
 * @brief Capture a signal and store it in the library.
 * @param freq_hz    Centre frequency.
 * @param duration_ms Capture duration in milliseconds.
 * @param slot        Storage slot [0, REPLAYER_MAX_SIGNALS).
 * @return ESP_OK on success.
 */
esp_err_t signal_replayer_capture(uint64_t freq_hz, uint32_t duration_ms, uint8_t slot);

/**
 * @brief Replay a stored signal.
 * @param slot Storage slot to replay.
 * @return ESP_OK on success.
 */
esp_err_t signal_replayer_play(uint8_t slot);

/**
 * @brief Print a summary of stored signals to stdout.
 */
void signal_replayer_list(void);

#ifdef __cplusplus
}
#endif
