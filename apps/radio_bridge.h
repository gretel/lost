// SPDX-License-Identifier: ISC
//
// Unified pure-C bridge API for SDR hardware access.
//
// Two implementations:
//   - uhd_radio_bridge.c   (direct UHD C API, preferred)
//   - soapy_radio_bridge.c (SoapySDR C API, legacy fallback)
//
// Only one is linked into each binary. Both implement the same symbols.
// NO C++ types cross this boundary — only C types and functions.
//
// The bridge is compiled as a static library with the toolchain's C
// compiler. On macOS with dual-libc++ (LLVM 19 + system), the X300 PCIe
// transport may crash during enumeration. Pass a device type filter
// (e.g. "type=b200") in device_args to avoid this.

#ifndef RADIO_BRIDGE_H
#define RADIO_BRIDGE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- Opaque handles ---

typedef struct radio_bridge radio_bridge_t;
typedef struct radio_bridge_multi radio_bridge_multi_t;

// --- Constants ---

#define RADIO_BRIDGE_RX  0
#define RADIO_BRIDGE_TX  1

#define RADIO_BRIDGE_ERR_TIMEOUT  (-1)
#define RADIO_BRIDGE_ERR_OVERFLOW (-4)
#define RADIO_BRIDGE_ERR_NULL     (-100)
#define RADIO_BRIDGE_ERR_DEVICE   (-101)

// --- Single-channel configuration ---

// All string fields accept NULL for auto/default behavior.
// device_args: backend-specific device selector string.
//   UHD:      "type=b200", "addr=192.168.10.2", or "" for auto-discover.
//   SoapySDR: "driver=uhd", "driver=rtlsdr", etc.
// clock_source: "external", "internal", "gpsdo", or NULL for device default.
// antenna: "RX2", "TX/RX", etc., or NULL for device default.
typedef struct {
    const char* device_args;
    const char* clock_source;
    double      sample_rate;
    double      center_freq;
    double      gain_db;
    double      bandwidth;       // 0 = auto
    size_t      channel;         // channel index (usually 0)
    const char* antenna;         // NULL for default
    int         direction;       // RADIO_BRIDGE_RX or RADIO_BRIDGE_TX
} radio_bridge_config_t;

// --- Dual-channel RX configuration ---

// Both channels share sample_rate and clock_source. Per-channel settings
// for center_freq, gain, bandwidth, and antenna.
typedef struct {
    const char* device_args;
    const char* clock_source;
    double      sample_rate;
    double      center_freq_0;
    double      gain_db_0;
    double      bandwidth_0;     // 0 = auto
    const char* antenna_0;       // NULL for default
    double      center_freq_1;
    double      gain_db_1;
    double      bandwidth_1;     // 0 = auto
    const char* antenna_1;       // NULL for default
} radio_bridge_multi_config_t;

// --- Functions ---

/// Probe for available devices. Returns 1 if found, 0 otherwise.
/// device_args: filter string, or NULL/"" for any device.
int radio_bridge_probe(const char* device_args);

/// Last error message (thread-local, valid until next bridge call).
const char* radio_bridge_last_error(void);

/// Create and configure a device for RX or TX.
/// Returns NULL on failure; check radio_bridge_last_error().
radio_bridge_t* radio_bridge_create(const radio_bridge_config_t* config);

/// Read IQ samples (RX). buf = interleaved float (I,Q,I,Q,...).
/// buf must hold at least num_samples * 2 * sizeof(float) bytes.
/// Returns sample count (>= 0) or negative error code.
int radio_bridge_read(radio_bridge_t* bridge, float* buf,
                      size_t num_samples, double timeout_sec);

/// Write IQ samples (TX). buf = interleaved float (I,Q,I,Q,...).
/// Returns sample count (>= 0) or negative error code.
/// Set end_burst=1 on the last write of a transmission.
int radio_bridge_write(radio_bridge_t* bridge, const float* buf,
                       size_t num_samples, double timeout_sec,
                       int end_burst);

/// Get actual configured values (for verification after create).
double radio_bridge_get_sample_rate(const radio_bridge_t* bridge);
double radio_bridge_get_center_freq(const radio_bridge_t* bridge);
double radio_bridge_get_gain(const radio_bridge_t* bridge);

/// Destroy bridge. Stops streams and releases resources.
/// If skip_free is nonzero, skip device deallocation to avoid teardown
/// crashes under dual-libc++ on macOS. Use at process exit with _exit().
void radio_bridge_destroy_ex(radio_bridge_t* bridge, int skip_free);

/// Convenience: full destroy (skip_free=0).
void radio_bridge_destroy(radio_bridge_t* bridge);

// --- Dual-channel RX ---

/// Create a dual-channel RX bridge (e.g. 2x2 MIMO on B210).
/// Returns NULL on failure; check radio_bridge_last_error().
radio_bridge_multi_t* radio_bridge_multi_create(
    const radio_bridge_multi_config_t* config);

/// Read IQ from both channels simultaneously.
/// buf0/buf1: per-channel interleaved float buffers.
/// Returns samples per channel (>= 0) or negative error code.
int radio_bridge_multi_read(radio_bridge_multi_t* bridge,
                            float* buf0, float* buf1,
                            size_t num_samples, double timeout_sec);

double radio_bridge_multi_get_sample_rate(const radio_bridge_multi_t* bridge);
double radio_bridge_multi_get_center_freq(const radio_bridge_multi_t* bridge,
                                          size_t channel);
double radio_bridge_multi_get_gain(const radio_bridge_multi_t* bridge,
                                   size_t channel);

void radio_bridge_multi_destroy_ex(radio_bridge_multi_t* bridge,
                                   int skip_free);

#ifdef __cplusplus
}
#endif

#endif  // RADIO_BRIDGE_H
