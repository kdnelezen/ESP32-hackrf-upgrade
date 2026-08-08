/**
 * @file cli.h
 * @brief Interactive UART command-line interface
 *
 * Provides a shell-like interface over the UART console that exposes all
 * HackRF + ESP32 features in a Flipper Zero-inspired command structure.
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the CLI task (UART console with readline-style editing).
 *        Must be called after all subsystems are initialised.
 */
void cli_start(void);

#ifdef __cplusplus
}
#endif
