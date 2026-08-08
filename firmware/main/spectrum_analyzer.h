/**
 * @file spectrum_analyzer.h
 * @brief Real-time spectrum analyser using HackRF I/Q data
 */

#pragma once

#include <stdint.h>
#include "hackrf_usb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the spectrum analyser with a HackRF device handle.
 * @param dev HackRF device handle.
 */
void spectrum_analyzer_init(hackrf_device_t *dev);

/**
 * @brief Capture and display a spectrum sweep.
 * @param centre_hz  Centre frequency in Hz.
 * @param bw_hz      Bandwidth in Hz.
 * @param num_bins   Number of frequency bins (resolution).
 */
void spectrum_analyzer_run(uint64_t centre_hz, uint32_t bw_hz, uint32_t num_bins);

#ifdef __cplusplus
}
#endif
