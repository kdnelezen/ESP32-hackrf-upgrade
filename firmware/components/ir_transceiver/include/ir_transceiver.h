/**
 * @file ir_transceiver.h
 * @brief Infrared transceiver – emit and capture IR signals
 *
 * Uses the ESP32 RMT peripheral for both TX and RX.
 * Default GPIO assignments can be overridden via sdkconfig.
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of RMT symbols in one IR frame. */
#define IR_MAX_SYMBOLS 256

/** Carrier frequencies for common IR protocols (Hz). */
#define IR_CARRIER_NEC      38000
#define IR_CARRIER_RC5      36000
#define IR_CARRIER_SONY     40000
#define IR_CARRIER_SAMSUNG  38000

/** GPIO defaults – override in sdkconfig. */
#ifndef CONFIG_IR_TX_GPIO
#define CONFIG_IR_TX_GPIO  18
#endif
#ifndef CONFIG_IR_RX_GPIO
#define CONFIG_IR_RX_GPIO  19
#endif

/** One RMT symbol: duration (µs) and level. */
typedef struct {
    uint16_t duration_us;
    uint8_t  level;
} ir_symbol_t;

/** A complete captured or prepared IR frame. */
typedef struct {
    ir_symbol_t symbols[IR_MAX_SYMBOLS];
    uint32_t    num_symbols;
    uint32_t    carrier_hz;
    char        protocol[32];
} ir_frame_t;

/**
 * @brief Initialise RMT TX and RX channels.
 * @return ESP_OK on success.
 */
esp_err_t ir_transceiver_init(void);

/**
 * @brief Transmit an IR frame.
 * @param frame Frame to send (carrier frequency and symbol list).
 * @return ESP_OK on success.
 */
esp_err_t ir_transmit(const ir_frame_t *frame);

/**
 * @brief Receive an IR frame (blocking, with timeout).
 * @param frame      Output frame; populated on success.
 * @param timeout_ms Receive timeout in milliseconds.
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if nothing received.
 */
esp_err_t ir_receive(ir_frame_t *frame, uint32_t timeout_ms);

/**
 * @brief Build a raw NEC command frame.
 * @param address  8-bit device address.
 * @param command  8-bit command.
 * @param frame    Output frame.
 */
void ir_build_nec(uint8_t address, uint8_t command, ir_frame_t *frame);

/**
 * @brief Build a raw Sony SIRC 12-bit frame.
 * @param command  7-bit command.
 * @param address  5-bit address.
 * @param frame    Output frame.
 */
void ir_build_sony_sirc(uint8_t command, uint8_t address, ir_frame_t *frame);

#ifdef __cplusplus
}
#endif
