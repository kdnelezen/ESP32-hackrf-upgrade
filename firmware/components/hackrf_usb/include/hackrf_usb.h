/**
 * @file hackrf_usb.h
 * @brief HackRF One USB host driver for ESP32
 *
 * Provides initialisation, configuration, and raw I/Q streaming for the
 * HackRF One software-defined radio connected via USB OTG.
 *
 * USB VID/PID:  0x1D50 / 0x6089 (HackRF One)
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque handle returned after successful enumeration. */
typedef struct hackrf_device hackrf_device_t;

/** Sample rate constants (Hz). */
#define HACKRF_SAMPLE_RATE_2M    2000000U
#define HACKRF_SAMPLE_RATE_8M    8000000U
#define HACKRF_SAMPLE_RATE_10M  10000000U
#define HACKRF_SAMPLE_RATE_20M  20000000U

/** Frequency bounds (Hz). */
#define HACKRF_FREQ_MIN_HZ    1000000ULL      /**<  1 MHz */
#define HACKRF_FREQ_MAX_HZ 6000000000ULL      /**<  6 GHz */

/** Gain limits (dB). */
#define HACKRF_LNA_GAIN_MAX  40
#define HACKRF_VGA_GAIN_MAX  62
#define HACKRF_TXVGA_GAIN_MAX 47

/**
 * @brief Callback invoked with each received I/Q buffer.
 *
 * @param buf      Pointer to interleaved signed-8-bit I/Q samples.
 * @param len      Number of bytes (pairs of I/Q samples).
 * @param user_ctx Caller-supplied context pointer.
 * @return 0 to continue streaming, non-zero to stop.
 */
typedef int (*hackrf_rx_cb_t)(const int8_t *buf, uint32_t len, void *user_ctx);

/**
 * @brief Initialise the USB host stack and enumerate the HackRF.
 * @return ESP_OK on success.
 */
esp_err_t hackrf_usb_init(void);

/**
 * @brief Deinitialise and release the USB host stack.
 */
void hackrf_usb_deinit(void);

/**
 * @brief Retrieve the active device handle.
 * @return Pointer to the device, or NULL if not connected.
 */
hackrf_device_t *hackrf_usb_get_device(void);

/**
 * @brief Set the centre frequency.
 * @param dev  Device handle.
 * @param freq_hz Frequency in Hz [1 MHz – 6 GHz].
 * @return ESP_OK on success.
 */
esp_err_t hackrf_set_freq(hackrf_device_t *dev, uint64_t freq_hz);

/**
 * @brief Set the baseband sample rate.
 * @param dev         Device handle.
 * @param sample_rate Sample rate in samples/second.
 * @return ESP_OK on success.
 */
esp_err_t hackrf_set_sample_rate(hackrf_device_t *dev, uint32_t sample_rate);

/**
 * @brief Set receive LNA (IF) gain.
 * @param dev     Device handle.
 * @param gain_db Gain in dB (0–40, step 8).
 * @return ESP_OK on success.
 */
esp_err_t hackrf_set_lna_gain(hackrf_device_t *dev, uint8_t gain_db);

/**
 * @brief Set receive VGA (baseband) gain.
 * @param dev     Device handle.
 * @param gain_db Gain in dB (0–62, step 2).
 * @return ESP_OK on success.
 */
esp_err_t hackrf_set_vga_gain(hackrf_device_t *dev, uint8_t gain_db);

/**
 * @brief Set transmit VGA gain.
 * @param dev     Device handle.
 * @param gain_db Gain in dB (0–47, step 1).
 * @return ESP_OK on success.
 */
esp_err_t hackrf_set_txvga_gain(hackrf_device_t *dev, uint8_t gain_db);

/**
 * @brief Enable or disable the RF amplifier.
 * @param dev    Device handle.
 * @param enable true to enable.
 * @return ESP_OK on success.
 */
esp_err_t hackrf_set_amp_enable(hackrf_device_t *dev, bool enable);

/**
 * @brief Enable or disable antenna port power (bias-T).
 * @param dev    Device handle.
 * @param enable true to enable.
 * @return ESP_OK on success.
 */
esp_err_t hackrf_set_antenna_enable(hackrf_device_t *dev, bool enable);

/**
 * @brief Start receiving and call @p cb for each USB bulk transfer.
 * @param dev      Device handle.
 * @param cb       Callback to receive I/Q data.
 * @param user_ctx Context pointer passed to @p cb.
 * @return ESP_OK on success.
 */
esp_err_t hackrf_start_rx(hackrf_device_t *dev, hackrf_rx_cb_t cb, void *user_ctx);

/**
 * @brief Stop the active receive stream.
 * @param dev Device handle.
 * @return ESP_OK on success.
 */
esp_err_t hackrf_stop_rx(hackrf_device_t *dev);

/**
 * @brief Transmit a buffer of interleaved signed-8-bit I/Q samples.
 * @param dev  Device handle.
 * @param buf  Pointer to sample buffer.
 * @param len  Length of buffer in bytes.
 * @return ESP_OK on success.
 */
esp_err_t hackrf_transmit(hackrf_device_t *dev, const int8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif
