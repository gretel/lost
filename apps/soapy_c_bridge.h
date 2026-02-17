// SPDX-License-Identifier: ISC
//
// Pure-C bridge to SoapySDR, avoiding C++ ABI issues between LLVM 19 libc++
// and system libc++ (used by Homebrew SoapySDR/UHD).
//
// This header is included from both:
//   - soapy_c_bridge.c  (compiled with system compiler, links system libc++)
//   - SoapySource.hpp   (compiled with LLVM 19, links LLVM 19 libc++)
//   - SoapySink.hpp     (compiled with LLVM 19, links LLVM 19 libc++)
//
// NO C++ types cross this boundary. Only C types and functions.

#ifndef SOAPY_C_BRIDGE_H
#define SOAPY_C_BRIDGE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to the bridge state (SoapySDRDevice + stream(s))
typedef struct soapy_bridge soapy_bridge_t;

// Stream direction
#define SOAPY_BRIDGE_RX  0
#define SOAPY_BRIDGE_TX  1

// Configuration for device initialization
typedef struct {
    const char* driver;          // e.g. "uhd"
    const char* clock_source;    // e.g. "external" or NULL for default
    double      sample_rate;     // e.g. 250000.0
    double      center_freq;     // e.g. 869618000.0
    double      gain_db;         // e.g. 40.0
    double      bandwidth;       // 0 = auto
    size_t      channel;         // channel index (usually 0)
    const char* antenna;         // NULL for default
    // Additional UHD-specific parameters
    const char* device_args;     // e.g. "recv_frame_size=16360" or NULL
    int         direction;       // SOAPY_BRIDGE_RX or SOAPY_BRIDGE_TX
} soapy_bridge_config_t;

// Error codes returned by read/write (match SoapySDR error codes)
#define SOAPY_BRIDGE_OK          0
#define SOAPY_BRIDGE_ERR_TIMEOUT (-1)
#define SOAPY_BRIDGE_ERR_STREAM  (-2)
#define SOAPY_BRIDGE_ERR_CORRUPT (-3)
#define SOAPY_BRIDGE_ERR_OVERFLOW (-4)
#define SOAPY_BRIDGE_ERR_UNDERFLOW (-7)
#define SOAPY_BRIDGE_ERR_NULL    (-100)
#define SOAPY_BRIDGE_ERR_MAKE    (-101)

// Probe for available devices. Returns 1 if a device matching the driver
// is found, 0 otherwise. Useful for checking USB connection before create().
int soapy_bridge_probe(const char* driver);

// Create and configure a SoapySDR device for RX or TX.
// Set config->direction to SOAPY_BRIDGE_RX or SOAPY_BRIDGE_TX.
// Returns NULL on failure; check soapy_bridge_last_error().
soapy_bridge_t* soapy_bridge_create(const soapy_bridge_config_t* config);

// Get the last error message (thread-local, valid until next call).
const char* soapy_bridge_last_error(void);

// Read IQ samples (RX). buf = interleaved complex float (IQIQIQ...).
// buf must hold at least num_samples * 2 * sizeof(float) bytes.
// Returns number of samples read (>= 0) or negative error code.
// SOAPY_BRIDGE_ERR_TIMEOUT (-1) is returned on timeout (not an error).
// SOAPY_BRIDGE_ERR_OVERFLOW (-4) indicates dropped samples (logged).
int soapy_bridge_read(soapy_bridge_t* bridge, float* buf,
                      size_t num_samples, long timeout_us);

// Write IQ samples (TX). buf = interleaved complex float (IQIQIQ...).
// Returns number of samples written (>= 0) or negative error code.
// Set end_burst=1 on the last write of a transmission to signal end-of-burst.
int soapy_bridge_write(soapy_bridge_t* bridge, const float* buf,
                       size_t num_samples, long timeout_us,
                       int end_burst);

// Get actual configured values (for verification).
double soapy_bridge_get_sample_rate(const soapy_bridge_t* bridge);
double soapy_bridge_get_center_freq(const soapy_bridge_t* bridge);
double soapy_bridge_get_gain(const soapy_bridge_t* bridge);

// Deactivate stream and destroy device. Safe to call with NULL.
// If skip_unmake is nonzero, only deactivate/close the stream but do NOT
// call SoapySDRDevice_unmake(). Use this at process exit to avoid the
// known UHD segfault in device teardown on macOS.
void soapy_bridge_destroy_ex(soapy_bridge_t* bridge, int skip_unmake);

// Convenience: full destroy (deactivate + unmake + free).
void soapy_bridge_destroy(soapy_bridge_t* bridge);

#ifdef __cplusplus
}
#endif

#endif  // SOAPY_C_BRIDGE_H
