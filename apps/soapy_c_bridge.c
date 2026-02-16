// SPDX-License-Identifier: ISC
//
// Pure-C implementation of SoapySDR bridge.
// MUST be compiled with system compiler (Apple Clang) to match the ABI of
// Homebrew SoapySDR and UHD libraries. Do NOT compile with LLVM 19.

#include "soapy_c_bridge.h"

#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <SoapySDR/Types.h>
#include <SoapySDR/Version.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

_Static_assert(SOAPY_SDR_END_BURST == (1 << 1),
               "SOAPY_SDR_END_BURST value changed — update soapy_bridge_write()");

struct soapy_bridge {
    SoapySDRDevice* device;
    SoapySDRStream* stream;
    size_t          channel;
    int             direction;     // SOAPY_SDR_RX or SOAPY_SDR_TX
    double          sample_rate;
    double          center_freq;
    double          gain_db;
};

// Thread-local error message buffer
static _Thread_local char s_error_buf[512] = {0};

static void set_error(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(s_error_buf, sizeof(s_error_buf), fmt, args);
    va_end(args);
}

// Parse "key1=val1,key2=val2" into SoapySDRKwargs
static void parse_extra_args(SoapySDRKwargs* kwargs, const char* args) {
    if (!args || !args[0]) return;
    char* copy = strdup(args);
    if (!copy) return;  // OOM
    char* saveptr = NULL;
    char* token = strtok_r(copy, ",", &saveptr);
    while (token) {
        char* eq = strchr(token, '=');
        if (eq) {
            *eq = '\0';
            char* key = token;
            while (*key == ' ') key++;
            char* val = eq + 1;
            while (*val == ' ') val++;
            SoapySDRKwargs_set(kwargs, key, val);
        }
        token = strtok_r(NULL, ",", &saveptr);
    }
    free(copy);
}

int soapy_bridge_probe(const char* driver) {
    if (!driver) return 0;

    SoapySDRKwargs args;
    memset(&args, 0, sizeof(args));
    SoapySDRKwargs_set(&args, "driver", driver);
    SoapySDRKwargs_set(&args, "type", "b200");

    size_t numResults = 0;
    SoapySDRKwargs* results = SoapySDRDevice_enumerate(&args, &numResults);
    SoapySDRKwargs_clear(&args);

    int found = (numResults > 0) ? 1 : 0;
    if (results) SoapySDRKwargsList_clear(results, numResults);
    return found;
}

soapy_bridge_t* soapy_bridge_create(const soapy_bridge_config_t* config) {
    if (!config || !config->driver) {
        set_error("NULL config or driver");
        return NULL;
    }

    int soapy_dir = (config->direction == SOAPY_BRIDGE_TX)
                    ? SOAPY_SDR_TX : SOAPY_SDR_RX;
    const char* dir_name = (soapy_dir == SOAPY_SDR_TX) ? "TX" : "RX";

    // Build device args
    SoapySDRKwargs devArgs;
    memset(&devArgs, 0, sizeof(devArgs));
    SoapySDRKwargs_set(&devArgs, "driver", config->driver);
    // Restrict UHD device enumeration to B200 series to avoid X300 PCIe
    // enumeration which crashes due to dual-libc++ (boost::locale conflict).
    SoapySDRKwargs_set(&devArgs, "type", "b200");

    // Add extra device args
    parse_extra_args(&devArgs, config->device_args);

    fprintf(stderr, "[soapy_bridge] Making device (driver=%s, %s)...\n",
            config->driver, dir_name);
    SoapySDRDevice* dev = SoapySDRDevice_make(&devArgs);
    SoapySDRKwargs_clear(&devArgs);

    if (!dev) {
        set_error("SoapySDRDevice_make failed: %s", SoapySDRDevice_lastError());
        return NULL;
    }
    fprintf(stderr, "[soapy_bridge] Device created\n");

    // Set clock source
    if (config->clock_source && config->clock_source[0]) {
        fprintf(stderr, "[soapy_bridge] Setting clock_source=%s\n", config->clock_source);
        SoapySDRDevice_setClockSource(dev, config->clock_source);
    }

    // Set sample rate
    if (config->sample_rate > 0) {
        fprintf(stderr, "[soapy_bridge] Setting sample_rate=%.0f\n", config->sample_rate);
        if (SoapySDRDevice_setSampleRate(dev, soapy_dir, config->channel,
                                          config->sample_rate) != 0) {
            set_error("setSampleRate failed");
            SoapySDRDevice_unmake(dev);
            return NULL;
        }
    }

    // Set center frequency
    if (config->center_freq > 0) {
        SoapySDRKwargs tuneArgs;
        memset(&tuneArgs, 0, sizeof(tuneArgs));
        fprintf(stderr, "[soapy_bridge] Setting center_freq=%.0f\n", config->center_freq);
        if (SoapySDRDevice_setFrequency(dev, soapy_dir, config->channel,
                                          config->center_freq, &tuneArgs) != 0) {
            set_error("setFrequency failed");
            SoapySDRDevice_unmake(dev);
            return NULL;
        }
    }

    // Set gain
    if (config->gain_db != 0) {
        fprintf(stderr, "[soapy_bridge] Setting gain=%.1f dB\n", config->gain_db);
        if (SoapySDRDevice_setGain(dev, soapy_dir, config->channel,
                                    config->gain_db) != 0) {
            set_error("setGain failed");
            SoapySDRDevice_unmake(dev);
            return NULL;
        }
    }

    // Set bandwidth
    if (config->bandwidth > 0) {
        fprintf(stderr, "[soapy_bridge] Setting bandwidth=%.0f\n", config->bandwidth);
        if (SoapySDRDevice_setBandwidth(dev, soapy_dir, config->channel,
                                         config->bandwidth) != 0) {
            set_error("setBandwidth failed");
            SoapySDRDevice_unmake(dev);
            return NULL;
        }
    }

    // Set antenna
    if (config->antenna && config->antenna[0]) {
        fprintf(stderr, "[soapy_bridge] Setting antenna=%s\n", config->antenna);
        if (SoapySDRDevice_setAntenna(dev, soapy_dir, config->channel,
                                       config->antenna) != 0) {
            set_error("setAntenna failed");
            SoapySDRDevice_unmake(dev);
            return NULL;
        }
    }

    // Setup stream
    fprintf(stderr, "[soapy_bridge] Setting up %s stream (CF32, channel %zu)...\n",
            dir_name, config->channel);
    size_t channels[] = {config->channel};
    SoapySDRStream* stream = SoapySDRDevice_setupStream(
        dev, soapy_dir, SOAPY_SDR_CF32, channels, 1, NULL);
    if (!stream) {
        set_error("setupStream failed: %s", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(dev);
        return NULL;
    }

    // Activate stream
    if (SoapySDRDevice_activateStream(dev, stream, 0, 0, 0) != 0) {
        set_error("activateStream failed");
        SoapySDRDevice_closeStream(dev, stream);
        SoapySDRDevice_unmake(dev);
        return NULL;
    }
    fprintf(stderr, "[soapy_bridge] Stream activated\n");

    // Report actual values
    double actualRate = SoapySDRDevice_getSampleRate(dev, soapy_dir, config->channel);
    double actualFreq = SoapySDRDevice_getFrequency(dev, soapy_dir, config->channel);
    double actualGain = SoapySDRDevice_getGain(dev, soapy_dir, config->channel);
    fprintf(stderr, "[soapy_bridge] Actual: rate=%.0f freq=%.0f gain=%.1f\n",
            actualRate, actualFreq, actualGain);

    // Allocate and return bridge handle
    soapy_bridge_t* bridge = calloc(1, sizeof(soapy_bridge_t));
    if (!bridge) {
        set_error("malloc failed");
        SoapySDRDevice_deactivateStream(dev, stream, 0, 0);
        SoapySDRDevice_closeStream(dev, stream);
        SoapySDRDevice_unmake(dev);
        return NULL;
    }

    bridge->device      = dev;
    bridge->stream      = stream;
    bridge->channel     = config->channel;
    bridge->direction   = soapy_dir;
    bridge->sample_rate = actualRate;
    bridge->center_freq = actualFreq;
    bridge->gain_db     = actualGain;

    return bridge;
}

const char* soapy_bridge_last_error(void) {
    return s_error_buf;
}

int soapy_bridge_read(soapy_bridge_t* bridge, float* buf,
                      size_t num_samples, long timeout_us) {
    if (!bridge || !bridge->device || !bridge->stream || !buf) {
        return SOAPY_BRIDGE_ERR_NULL;
    }

    void* buffs[] = {buf};
    int flags = 0;
    long long time_ns = 0;

    return SoapySDRDevice_readStream(bridge->device, bridge->stream,
                                      buffs, num_samples,
                                      &flags, &time_ns, timeout_us);
}

int soapy_bridge_write(soapy_bridge_t* bridge, const float* buf,
                       size_t num_samples, long timeout_us,
                       int end_burst) {
    if (!bridge || !bridge->device || !bridge->stream || !buf) {
        return SOAPY_BRIDGE_ERR_NULL;
    }

    const void* buffs[] = {buf};
    int flags = end_burst ? SOAPY_SDR_END_BURST : 0;
    long long time_ns = 0;

    return SoapySDRDevice_writeStream(bridge->device, bridge->stream,
                                       buffs, num_samples,
                                       &flags, time_ns, timeout_us);
}

double soapy_bridge_get_sample_rate(const soapy_bridge_t* bridge) {
    return bridge ? bridge->sample_rate : 0.0;
}

double soapy_bridge_get_center_freq(const soapy_bridge_t* bridge) {
    return bridge ? bridge->center_freq : 0.0;
}

double soapy_bridge_get_gain(const soapy_bridge_t* bridge) {
    return bridge ? bridge->gain_db : 0.0;
}

void soapy_bridge_destroy(soapy_bridge_t* bridge) {
    if (!bridge) return;

    if (bridge->stream && bridge->device) {
        fprintf(stderr, "[soapy_bridge] Deactivating stream...\n");
        SoapySDRDevice_deactivateStream(bridge->device, bridge->stream, 0, 0);
        SoapySDRDevice_closeStream(bridge->device, bridge->stream);
    }
    if (bridge->device) {
        fprintf(stderr, "[soapy_bridge] Unmaking device...\n");
        SoapySDRDevice_unmake(bridge->device);
    }
    free(bridge);
    fprintf(stderr, "[soapy_bridge] Destroyed\n");
}
