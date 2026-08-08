/**
 * @file cli.c
 * @brief UART CLI implementation using ESP-IDF console component.
 *
 * Commands:
 *   help
 *   freq <Hz>                  – set HackRF centre frequency
 *   samplerate <sps>           – set HackRF sample rate
 *   lna <dB>                   – set LNA gain
 *   vga <dB>                   – set VGA gain
 *   txvga <dB>                 – set TX VGA gain
 *   amp <0|1>                  – enable/disable RF amp
 *   subghz_scan [dwell_ms]     – scan ISM bands
 *   subghz_rx <freq> [timeout] – receive signal
 *   subghz_replay              – replay last captured signal
 *   ir_send <proto> <a1> <a2>  – send IR command (nec|sony)
 *   ble_scan [duration_ms]     – BLE advertisement scan
 *   wifi_scan                  – Wi-Fi AP scan
 *   spectrum [freq] [bw]       – show spectrum display
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "linenoise/linenoise.h"

#include "hackrf_usb.h"
#include "subghz.h"
#include "ir_transceiver.h"
#include "ble_wifi_scanner.h"
#include "spectrum_analyzer.h"
#include "signal_replayer.h"

static const char *TAG = "CLI";

/* ── Shared last-captured signal (for replay) ──────────────────────────── */
static subghz_signal_t s_last_signal;
static bool            s_has_signal = false;

/* ── Helper: parse uint64 from string ──────────────────────────────────── */
static uint64_t parse_u64(const char *s) { return (uint64_t)strtoull(s, NULL, 0); }

/* ────────────────────────────────────────────────────────────────────────── */
/*  Command: freq                                                             */
/* ────────────────────────────────────────────────────────────────────────── */
static struct { struct arg_str *hz; struct arg_end *end; } freq_args;

static int cmd_freq(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&freq_args);
    if (nerrors) { arg_print_errors(stderr, freq_args.end, argv[0]); return 1; }
    hackrf_device_t *dev = hackrf_usb_get_device();
    if (!dev) { printf("HackRF not connected\n"); return 1; }
    uint64_t hz = parse_u64(freq_args.hz->sval[0]);
    esp_err_t ret = hackrf_set_freq(dev, hz);
    printf("freq %s: %" PRIu64 " Hz\n", ret == ESP_OK ? "OK" : "FAIL", hz);
    return ret == ESP_OK ? 0 : 1;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Command: samplerate                                                       */
/* ────────────────────────────────────────────────────────────────────────── */
static struct { struct arg_int *sps; struct arg_end *end; } sr_args;

static int cmd_samplerate(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&sr_args);
    if (nerrors) { arg_print_errors(stderr, sr_args.end, argv[0]); return 1; }
    hackrf_device_t *dev = hackrf_usb_get_device();
    if (!dev) { printf("HackRF not connected\n"); return 1; }
    uint32_t sps = (uint32_t)sr_args.sps->ival[0];
    esp_err_t ret = hackrf_set_sample_rate(dev, sps);
    printf("sample rate %s: %u sps\n", ret == ESP_OK ? "OK" : "FAIL", (unsigned)sps);
    return ret == ESP_OK ? 0 : 1;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Commands: lna / vga / txvga (share argtable)                             */
/* ────────────────────────────────────────────────────────────────────────── */
static struct { struct arg_int *db; struct arg_end *end; } lna_args, vga_args, txvga_args;

static int cmd_lna(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&lna_args);
    if (nerrors) { arg_print_errors(stderr, lna_args.end, argv[0]); return 1; }
    hackrf_device_t *dev = hackrf_usb_get_device();
    if (!dev) { printf("HackRF not connected\n"); return 1; }
    return hackrf_set_lna_gain(dev, (uint8_t)lna_args.db->ival[0]) == ESP_OK ? 0 : 1;
}

static int cmd_vga(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&vga_args);
    if (nerrors) { arg_print_errors(stderr, vga_args.end, argv[0]); return 1; }
    hackrf_device_t *dev = hackrf_usb_get_device();
    if (!dev) { printf("HackRF not connected\n"); return 1; }
    return hackrf_set_vga_gain(dev, (uint8_t)vga_args.db->ival[0]) == ESP_OK ? 0 : 1;
}

static int cmd_txvga(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&txvga_args);
    if (nerrors) { arg_print_errors(stderr, txvga_args.end, argv[0]); return 1; }
    hackrf_device_t *dev = hackrf_usb_get_device();
    if (!dev) { printf("HackRF not connected\n"); return 1; }
    return hackrf_set_txvga_gain(dev, (uint8_t)txvga_args.db->ival[0]) == ESP_OK ? 0 : 1;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Command: amp                                                              */
/* ────────────────────────────────────────────────────────────────────────── */
static struct { struct arg_int *en; struct arg_end *end; } amp_args;

static int cmd_amp(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&amp_args);
    if (nerrors) { arg_print_errors(stderr, amp_args.end, argv[0]); return 1; }
    hackrf_device_t *dev = hackrf_usb_get_device();
    if (!dev) { printf("HackRF not connected\n"); return 1; }
    return hackrf_set_amp_enable(dev, amp_args.en->ival[0] != 0) == ESP_OK ? 0 : 1;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Sub-GHz commands                                                          */
/* ────────────────────────────────────────────────────────────────────────── */
static void subghz_scan_cb(const subghz_signal_t *sig, void *ctx)
{
    printf("[SubGHz] %s\n", sig->description);
}

static struct { struct arg_int *dwell; struct arg_end *end; } subghz_scan_args;

static int cmd_subghz_scan(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&subghz_scan_args);
    if (nerrors) { arg_print_errors(stderr, subghz_scan_args.end, argv[0]); return 1; }
    uint32_t dwell = subghz_scan_args.dwell->count > 0
                   ? (uint32_t)subghz_scan_args.dwell->ival[0]
                   : 500;
    printf("Scanning ISM bands (dwell %u ms) ...\n", (unsigned)dwell);
    esp_err_t ret = subghz_scan(subghz_scan_cb, NULL, dwell);
    printf("Scan %s\n", ret == ESP_OK ? "complete" : "failed");
    return ret == ESP_OK ? 0 : 1;
}

static struct {
    struct arg_str *freq;
    struct arg_int *timeout;
    struct arg_end *end;
} subghz_rx_args;

static int cmd_subghz_rx(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&subghz_rx_args);
    if (nerrors) { arg_print_errors(stderr, subghz_rx_args.end, argv[0]); return 1; }
    uint64_t freq    = parse_u64(subghz_rx_args.freq->sval[0]);
    uint32_t timeout = subghz_rx_args.timeout->count > 0
                     ? (uint32_t)subghz_rx_args.timeout->ival[0]
                     : 3000;

    if (s_has_signal) {
        subghz_signal_free(&s_last_signal);
        s_has_signal = false;
    }

    printf("Listening on %.3f MHz for %u ms ...\n", (double)freq / 1e6, (unsigned)timeout);
    esp_err_t ret = subghz_receive(freq, SUBGHZ_MOD_RAW, timeout, &s_last_signal);
    if (ret == ESP_OK) {
        s_has_signal = true;
        printf("Captured: %s\n", s_last_signal.description);
    } else {
        printf("No signal detected (timeout)\n");
    }
    return ret == ESP_OK ? 0 : 1;
}

static int cmd_subghz_replay(int argc, char **argv)
{
    if (!s_has_signal) {
        printf("No captured signal – use 'subghz_rx' first\n");
        return 1;
    }
    printf("Replaying: %s\n", s_last_signal.description);
    esp_err_t ret = subghz_replay(&s_last_signal);
    printf("Replay %s\n", ret == ESP_OK ? "complete" : "failed");
    return ret == ESP_OK ? 0 : 1;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  IR commands                                                               */
/* ────────────────────────────────────────────────────────────────────────── */
static struct {
    struct arg_str *proto;
    struct arg_int *arg1;
    struct arg_int *arg2;
    struct arg_end *end;
} ir_send_args;

static int cmd_ir_send(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&ir_send_args);
    if (nerrors) { arg_print_errors(stderr, ir_send_args.end, argv[0]); return 1; }

    ir_frame_t frame;
    const char *proto = ir_send_args.proto->sval[0];

    if (strcasecmp(proto, "nec") == 0) {
        uint8_t addr = (uint8_t)ir_send_args.arg1->ival[0];
        uint8_t cmd  = (uint8_t)ir_send_args.arg2->ival[0];
        ir_build_nec(addr, cmd, &frame);
        printf("Sending NEC addr=0x%02X cmd=0x%02X\n", addr, cmd);
    } else if (strcasecmp(proto, "sony") == 0) {
        uint8_t cmd  = (uint8_t)ir_send_args.arg1->ival[0];
        uint8_t addr = (uint8_t)ir_send_args.arg2->ival[0];
        ir_build_sony_sirc(cmd, addr, &frame);
        printf("Sending SONY SIRC cmd=0x%02X addr=0x%02X\n", cmd, addr);
    } else {
        printf("Unknown protocol: %s (use 'nec' or 'sony')\n", proto);
        return 1;
    }

    esp_err_t ret = ir_transmit(&frame);
    printf("IR send %s\n", ret == ESP_OK ? "OK" : "FAIL");
    return ret == ESP_OK ? 0 : 1;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  BLE scan command                                                          */
/* ────────────────────────────────────────────────────────────────────────── */
static struct { struct arg_int *duration; struct arg_end *end; } ble_args;

static void ble_result_cb(const ble_scan_result_t *res, void *ctx)
{
    char addr_str[18];
    snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             res->addr[0], res->addr[1], res->addr[2],
             res->addr[3], res->addr[4], res->addr[5]);
    printf("[BLE] %s  RSSI=%d  Name=%s\n", addr_str, res->rssi,
           res->name[0] ? res->name : "(unknown)");
}

static int cmd_ble_scan(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&ble_args);
    if (nerrors) { arg_print_errors(stderr, ble_args.end, argv[0]); return 1; }
    uint32_t dur = ble_args.duration->count > 0
                 ? (uint32_t)ble_args.duration->ival[0]
                 : 5000;
    printf("BLE scanning for %u ms ...\n", (unsigned)dur);
    esp_err_t ret = ble_scan_start(dur, ble_result_cb, NULL);
    if (ret == ESP_OK) vTaskDelay(pdMS_TO_TICKS(dur + 200));
    printf("BLE scan %s\n", ret == ESP_OK ? "complete" : "failed");
    return ret == ESP_OK ? 0 : 1;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Wi-Fi scan command                                                        */
/* ────────────────────────────────────────────────────────────────────────── */
static void wifi_result_cb(const wifi_scan_result_t *results,
                            uint16_t count, void *ctx)
{
    printf("Found %u access points:\n", (unsigned)count);
    for (uint16_t i = 0; i < count; i++) {
        printf("  [%2u] SSID=%-32s  CH=%2u  RSSI=%4d\n",
               i + 1, results[i].ssid, results[i].channel, results[i].rssi);
    }
}

static int cmd_wifi_scan(int argc, char **argv)
{
    printf("Wi-Fi scanning ...\n");
    esp_err_t ret = wifi_scan_all(wifi_result_cb, NULL);
    if (ret != ESP_OK) printf("Wi-Fi scan failed: %s\n", esp_err_to_name(ret));
    return ret == ESP_OK ? 0 : 1;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Spectrum command                                                           */
/* ────────────────────────────────────────────────────────────────────────── */
static struct {
    struct arg_str *freq;
    struct arg_int *bw;
    struct arg_end *end;
} spectrum_args;

static int cmd_spectrum(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&spectrum_args);
    if (nerrors) { arg_print_errors(stderr, spectrum_args.end, argv[0]); return 1; }
    uint64_t freq = spectrum_args.freq->count > 0
                  ? parse_u64(spectrum_args.freq->sval[0])
                  : SUBGHZ_FREQ_433MHZ;
    uint32_t bw   = spectrum_args.bw->count > 0
                  ? (uint32_t)spectrum_args.bw->ival[0]
                  : 2000000;
    printf("Spectrum @ %.3f MHz  BW=%u Hz\n", (double)freq / 1e6, (unsigned)bw);
    spectrum_analyzer_run(freq, bw, 64);
    return 0;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Registration                                                              */
/* ────────────────────────────────────────────────────────────────────────── */
static void register_commands(void)
{
    /* freq */
    freq_args.hz  = arg_strn(NULL, NULL, "<Hz>", 1, 1, "centre frequency in Hz");
    freq_args.end = arg_end(1);
    const esp_console_cmd_t freq_cmd = {
        .command = "freq", .help = "Set HackRF centre frequency (Hz)",
        .hint = "<Hz>", .func = cmd_freq, .argtable = &freq_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&freq_cmd));

    /* samplerate */
    sr_args.sps = arg_intn(NULL, NULL, "<sps>", 1, 1, "sample rate");
    sr_args.end = arg_end(1);
    const esp_console_cmd_t sr_cmd = {
        .command = "samplerate", .help = "Set HackRF sample rate (sps)",
        .hint = "<sps>", .func = cmd_samplerate, .argtable = &sr_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&sr_cmd));

    /* lna */
    lna_args.db  = arg_intn(NULL, NULL, "<dB>", 1, 1, "gain in dB (0-40)");
    lna_args.end = arg_end(1);
    const esp_console_cmd_t lna_cmd = {
        .command = "lna", .help = "Set LNA gain (0-40 dB)",
        .hint = "<dB>", .func = cmd_lna, .argtable = &lna_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&lna_cmd));

    /* vga */
    vga_args.db  = arg_intn(NULL, NULL, "<dB>", 1, 1, "gain in dB (0-62)");
    vga_args.end = arg_end(1);
    const esp_console_cmd_t vga_cmd = {
        .command = "vga", .help = "Set VGA gain (0-62 dB)",
        .hint = "<dB>", .func = cmd_vga, .argtable = &vga_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&vga_cmd));

    /* txvga */
    txvga_args.db  = arg_intn(NULL, NULL, "<dB>", 1, 1, "gain in dB (0-47)");
    txvga_args.end = arg_end(1);
    const esp_console_cmd_t txvga_cmd = {
        .command = "txvga", .help = "Set TX VGA gain (0-47 dB)",
        .hint = "<dB>", .func = cmd_txvga, .argtable = &txvga_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&txvga_cmd));

    /* amp */
    amp_args.en  = arg_intn(NULL, NULL, "<0|1>", 1, 1, "0=off 1=on");
    amp_args.end = arg_end(1);
    const esp_console_cmd_t amp_cmd = {
        .command = "amp", .help = "Enable/disable RF amp",
        .hint = "<0|1>", .func = cmd_amp, .argtable = &amp_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&amp_cmd));

    /* subghz scan */
    subghz_scan_args.dwell = arg_intn(NULL, NULL, "[dwell_ms]", 0, 1, "dwell per freq ms (default 500)");
    subghz_scan_args.end   = arg_end(1);
    const esp_console_cmd_t sg_scan_cmd = {
        .command = "subghz_scan", .help = "Scan ISM bands for sub-GHz signals",
        .hint = "[dwell_ms]", .func = cmd_subghz_scan, .argtable = &subghz_scan_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&sg_scan_cmd));

    /* subghz rx */
    subghz_rx_args.freq    = arg_strn(NULL, NULL, "<freq_hz>", 1, 1, "frequency in Hz");
    subghz_rx_args.timeout = arg_intn(NULL, NULL, "[timeout_ms]", 0, 1, "timeout ms (default 3000)");
    subghz_rx_args.end     = arg_end(2);
    const esp_console_cmd_t sg_rx_cmd = {
        .command = "subghz_rx", .help = "Capture sub-GHz signal",
        .hint = "<freq_hz> [timeout_ms]", .func = cmd_subghz_rx, .argtable = &subghz_rx_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&sg_rx_cmd));

    /* subghz replay */
    const esp_console_cmd_t sg_rp_cmd = {
        .command = "subghz_replay", .help = "Replay last captured sub-GHz signal",
        .hint = NULL, .func = cmd_subghz_replay, .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&sg_rp_cmd));

    /* ir send */
    ir_send_args.proto = arg_strn(NULL, NULL, "<proto>", 1, 1, "nec or sony");
    ir_send_args.arg1  = arg_intn(NULL, NULL, "<arg1>",  1, 1, "NEC: address; SONY: command");
    ir_send_args.arg2  = arg_intn(NULL, NULL, "<arg2>",  1, 1, "NEC: command; SONY: address");
    ir_send_args.end   = arg_end(3);
    const esp_console_cmd_t ir_cmd = {
        .command = "ir_send", .help = "Send IR command (nec|sony)",
        .hint = "<proto> <arg1> <arg2>", .func = cmd_ir_send, .argtable = &ir_send_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&ir_cmd));

    /* ble scan */
    ble_args.duration = arg_intn(NULL, NULL, "[ms]", 0, 1, "duration ms (default 5000)");
    ble_args.end      = arg_end(1);
    const esp_console_cmd_t ble_cmd = {
        .command = "ble_scan", .help = "BLE advertisement scan",
        .hint = "[duration_ms]", .func = cmd_ble_scan, .argtable = &ble_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&ble_cmd));

    /* wifi scan */
    const esp_console_cmd_t wifi_cmd = {
        .command = "wifi_scan", .help = "Scan for Wi-Fi access points",
        .hint = NULL, .func = cmd_wifi_scan, .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&wifi_cmd));

    /* spectrum */
    spectrum_args.freq = arg_strn(NULL, NULL, "[freq_hz]", 0, 1, "centre freq Hz (default 433920000)");
    spectrum_args.bw   = arg_intn(NULL, NULL, "[bw_hz]",   0, 1, "bandwidth Hz (default 2000000)");
    spectrum_args.end  = arg_end(2);
    const esp_console_cmd_t spec_cmd = {
        .command = "spectrum", .help = "Show spectrum analyser",
        .hint = "[freq_hz] [bw_hz]", .func = cmd_spectrum, .argtable = &spectrum_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&spec_cmd));
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  CLI start                                                                 */
/* ────────────────────────────────────────────────────────────────────────── */
void cli_start(void)
{
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt           = "hackrf> ";
    repl_cfg.max_cmdline_length = 256;

    esp_console_dev_uart_config_t uart_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    esp_console_repl_t *repl;
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_cfg, &repl_cfg, &repl));

    esp_console_register_help_command();
    register_commands();

    ESP_LOGI(TAG, "CLI ready – type 'help' for commands");
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
