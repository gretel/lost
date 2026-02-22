// SPDX-License-Identifier: ISC
//
// SoapySDR backend for the unified radio_bridge API.
// MUST be compiled with system compiler (Apple Clang) to match the ABI of
// Homebrew SoapySDR and UHD libraries. Do NOT compile with LLVM 19.

#include "radio_bridge.h"

#include <SoapySDR/Device.h>
#include <SoapySDR/Errors.h>
#include <SoapySDR/Formats.h>
#include <SoapySDR/Types.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

_Static_assert(SOAPY_SDR_END_BURST == (1 << 1),
               "SOAPY_SDR_END_BURST value changed — update radio_bridge_write()");

struct radio_bridge {
    SoapySDRDevice* device;
    SoapySDRStream* stream;
    size_t          channel;
    int             direction;     // SOAPY_SDR_RX or SOAPY_SDR_TX
    double          sample_rate;
    double          center_freq;
    double          gain_db;
    // Runtime stats
    uint64_t        overflow_count;
    uint64_t        underflow_count;
    uint64_t        read_count;
    uint64_t        write_count;
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

int radio_bridge_probe(const char* driver) {
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

radio_bridge_t* radio_bridge_create(const radio_bridge_config_t* config) {
    if (!config) {
        set_error("NULL config");
        return NULL;
    }

    int soapy_dir = (config->direction == RADIO_BRIDGE_TX)
                    ? SOAPY_SDR_TX : SOAPY_SDR_RX;
    const char* dir_name = (soapy_dir == SOAPY_SDR_TX) ? "TX" : "RX";

    // Build device args from the unified device_args string.
    // Caller passes e.g. "driver=uhd,type=b200" or just "driver=uhd".
    SoapySDRKwargs devArgs;
    memset(&devArgs, 0, sizeof(devArgs));
    // Restrict UHD device enumeration to B200 series to avoid X300 PCIe
    // enumeration which crashes due to dual-libc++ (boost::locale conflict).
    SoapySDRKwargs_set(&devArgs, "type", "b200");
    // Parse device_args — may contain driver=, type= (overrides default), etc.
    parse_extra_args(&devArgs, config->device_args);

    fprintf(stderr, "[soapy_bridge] Making device (%s)...\n", dir_name);
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
        if (SoapySDRDevice_setClockSource(dev, config->clock_source) != 0) {
            fprintf(stderr, "[soapy_bridge] WARNING: setClockSource failed (non-fatal)\n");
        }
    }

    // Set sample rate
    if (config->sample_rate > 0) {
        fprintf(stderr, "[soapy_bridge] Setting sample_rate=%.0f\n", config->sample_rate);
        if (SoapySDRDevice_setSampleRate(dev, soapy_dir, config->channel,
                                          config->sample_rate) != 0) {
            set_error("setSampleRate failed: %s", SoapySDRDevice_lastError());
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
            set_error("setFrequency failed: %s", SoapySDRDevice_lastError());
            SoapySDRDevice_unmake(dev);
            return NULL;
        }
    }

    // Set gain
    if (config->gain_db != 0) {
        fprintf(stderr, "[soapy_bridge] Setting gain=%.1f dB\n", config->gain_db);
        if (SoapySDRDevice_setGain(dev, soapy_dir, config->channel,
                                    config->gain_db) != 0) {
            set_error("setGain failed: %s", SoapySDRDevice_lastError());
            SoapySDRDevice_unmake(dev);
            return NULL;
        }
    }

    // Set bandwidth
    if (config->bandwidth > 0) {
        fprintf(stderr, "[soapy_bridge] Setting bandwidth=%.0f\n", config->bandwidth);
        if (SoapySDRDevice_setBandwidth(dev, soapy_dir, config->channel,
                                         config->bandwidth) != 0) {
            set_error("setBandwidth failed: %s", SoapySDRDevice_lastError());
            SoapySDRDevice_unmake(dev);
            return NULL;
        }
    }

    // Set antenna
    if (config->antenna && config->antenna[0]) {
        fprintf(stderr, "[soapy_bridge] Setting antenna=%s\n", config->antenna);
        if (SoapySDRDevice_setAntenna(dev, soapy_dir, config->channel,
                                       config->antenna) != 0) {
            set_error("setAntenna failed: %s", SoapySDRDevice_lastError());
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
        set_error("activateStream failed: %s", SoapySDRDevice_lastError());
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
    radio_bridge_t* bridge = calloc(1, sizeof(radio_bridge_t));
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

const char* radio_bridge_last_error(void) {
    return s_error_buf;
}

int radio_bridge_read(radio_bridge_t* bridge, float* buf,
                      size_t num_samples, double timeout_sec) {
    if (!bridge || !bridge->device || !bridge->stream || !buf) {
        return RADIO_BRIDGE_ERR_NULL;
    }

    long timeout_us = (long)(timeout_sec * 1e6);

    void* buffs[] = {buf};
    int flags = 0;
    long long time_ns = 0;

    int ret = SoapySDRDevice_readStream(bridge->device, bridge->stream,
                                        buffs, num_samples,
                                        &flags, &time_ns, timeout_us);

    if (ret == SOAPY_SDR_OVERFLOW) {
        bridge->overflow_count++;
        // Log first occurrence and then every 100th
        if (bridge->overflow_count == 1 ||
            bridge->overflow_count % 100 == 0) {
            fprintf(stderr, "[soapy_bridge] RX overflow #%llu (samples dropped)\n",
                    (unsigned long long)bridge->overflow_count);
        }
    } else if (ret > 0) {
        bridge->read_count += (uint64_t)ret;
    }

    return ret;
}

int radio_bridge_write(radio_bridge_t* bridge, const float* buf,
                       size_t num_samples, double timeout_sec,
                       int end_burst) {
    if (!bridge || !bridge->device || !bridge->stream || !buf) {
        return RADIO_BRIDGE_ERR_NULL;
    }

    long timeout_us = (long)(timeout_sec * 1e6);

    const void* buffs[] = {buf};
    int flags = end_burst ? SOAPY_SDR_END_BURST : 0;
    long long time_ns = 0;

    int ret = SoapySDRDevice_writeStream(bridge->device, bridge->stream,
                                         buffs, num_samples,
                                         &flags, time_ns, timeout_us);

    if (ret == SOAPY_SDR_UNDERFLOW) {
        bridge->underflow_count++;
        if (bridge->underflow_count == 1 ||
            bridge->underflow_count % 100 == 0) {
            fprintf(stderr, "[soapy_bridge] TX underflow #%llu\n",
                    (unsigned long long)bridge->underflow_count);
        }
    } else if (ret > 0) {
        bridge->write_count += (uint64_t)ret;
    }

    return ret;
}

int radio_bridge_wait_burst_ack(radio_bridge_t* bridge, double timeout_sec) {
    (void)bridge;
    (void)timeout_sec;
    // SoapySDR has no async message API. The write call is synchronous
    // enough that samples are flushed by the time writeStream returns.
    // Return success (1) to indicate "no wait needed".
    return 1;
}

double radio_bridge_get_sample_rate(const radio_bridge_t* bridge) {
    return bridge ? bridge->sample_rate : 0.0;
}

double radio_bridge_get_center_freq(const radio_bridge_t* bridge) {
    return bridge ? bridge->center_freq : 0.0;
}

double radio_bridge_get_gain(const radio_bridge_t* bridge) {
    return bridge ? bridge->gain_db : 0.0;
}

void radio_bridge_destroy_ex(radio_bridge_t* bridge, int skip_unmake) {
    if (!bridge) return;

    // Log stats
    if (bridge->overflow_count > 0) {
        fprintf(stderr, "[soapy_bridge] Total RX overflows: %llu\n",
                (unsigned long long)bridge->overflow_count);
    }
    if (bridge->underflow_count > 0) {
        fprintf(stderr, "[soapy_bridge] Total TX underflows: %llu\n",
                (unsigned long long)bridge->underflow_count);
    }

    if (bridge->stream && bridge->device) {
        fprintf(stderr, "[soapy_bridge] Deactivating stream...\n");
        SoapySDRDevice_deactivateStream(bridge->device, bridge->stream, 0, 0);
        SoapySDRDevice_closeStream(bridge->device, bridge->stream);
        bridge->stream = NULL;
    }
    if (!skip_unmake && bridge->device) {
        fprintf(stderr, "[soapy_bridge] Unmaking device...\n");
        SoapySDRDevice_unmake(bridge->device);
        bridge->device = NULL;
        fprintf(stderr, "[soapy_bridge] Destroyed\n");
    } else if (bridge->device) {
        fprintf(stderr, "[soapy_bridge] Skipping device unmake (process exit)\n");
        bridge->device = NULL;
    }
    free(bridge);
}

void radio_bridge_destroy(radio_bridge_t* bridge) {
    radio_bridge_destroy_ex(bridge, 0);
}

// --- Multi-channel RX ---

struct radio_bridge_multi {
    SoapySDRDevice* device;
    SoapySDRStream* stream;
    double          sample_rate;
    double          center_freq[2];
    double          gain_db[2];
    uint64_t        overflow_count;
    uint64_t        read_count;
};

radio_bridge_multi_t* radio_bridge_multi_create(
        const radio_bridge_multi_config_t* config) {
    if (!config) {
        set_error("NULL config");
        return NULL;
    }

    // Build device args from the unified device_args string.
    // Set master_clock_rate to prevent B200 auto-tick-rate from changing
    // the clock during setupStream, which resets DSP tuning (frequencies).
    // For dual-channel RX the minimum clock is 4 × sample_rate, but we
    // use a higher value for better timing resolution.
    SoapySDRKwargs devArgs;
    memset(&devArgs, 0, sizeof(devArgs));
    SoapySDRKwargs_set(&devArgs, "type", "b200");
    // Lock master clock to avoid reconfiguration during setupStream.
    // 4 × sample_rate is the minimum; using exactly that.
    char mcr_buf[32];
    snprintf(mcr_buf, sizeof(mcr_buf), "%.0f", config->sample_rate * 4.0);
    SoapySDRKwargs_set(&devArgs, "master_clock_rate", mcr_buf);
    // Parse device_args — may contain driver=, type= (overrides default), etc.
    parse_extra_args(&devArgs, config->device_args);

    fprintf(stderr, "[soapy_multi] Making device (dual RX, mcr=%s)...\n", mcr_buf);
    SoapySDRDevice* dev = SoapySDRDevice_make(&devArgs);
    SoapySDRKwargs_clear(&devArgs);

    if (!dev) {
        set_error("SoapySDRDevice_make failed: %s", SoapySDRDevice_lastError());
        return NULL;
    }

    // Clock source
    if (config->clock_source && config->clock_source[0]) {
        fprintf(stderr, "[soapy_multi] Setting clock_source=%s\n",
                config->clock_source);
        SoapySDRDevice_setClockSource(dev, config->clock_source);
    }

    // Sample rate (shared across channels on B210)
    if (config->sample_rate > 0) {
        fprintf(stderr, "[soapy_multi] Setting sample_rate=%.0f\n",
                config->sample_rate);
        if (SoapySDRDevice_setSampleRate(dev, SOAPY_SDR_RX, 0,
                                          config->sample_rate) != 0) {
            set_error("setSampleRate failed: %s", SoapySDRDevice_lastError());
            SoapySDRDevice_unmake(dev);
            return NULL;
        }
    }

    // Per-channel settings
    struct { double freq; double gain; double bw; const char* ant; } ch_cfg[2] = {
        {config->center_freq_0, config->gain_db_0, config->bandwidth_0, config->antenna_0},
        {config->center_freq_1, config->gain_db_1, config->bandwidth_1, config->antenna_1},
    };
    for (size_t ch = 0; ch < 2; ch++) {
        if (ch_cfg[ch].freq > 0) {
            SoapySDRKwargs tuneArgs;
            memset(&tuneArgs, 0, sizeof(tuneArgs));
            fprintf(stderr, "[soapy_multi] ch%zu: setting freq=%.0f\n", ch, ch_cfg[ch].freq);
            if (SoapySDRDevice_setFrequency(dev, SOAPY_SDR_RX, ch,
                                              ch_cfg[ch].freq, &tuneArgs) != 0) {
                set_error("ch%zu setFrequency failed: %s", ch,
                          SoapySDRDevice_lastError());
                SoapySDRDevice_unmake(dev);
                return NULL;
            }
            double actual = SoapySDRDevice_getFrequency(dev, SOAPY_SDR_RX, ch);
            fprintf(stderr, "[soapy_multi] ch%zu: actual freq=%.0f\n", ch, actual);
        }
        if (ch_cfg[ch].gain != 0) {
            fprintf(stderr, "[soapy_multi] ch%zu: gain=%.1f dB\n", ch, ch_cfg[ch].gain);
            SoapySDRDevice_setGain(dev, SOAPY_SDR_RX, ch, ch_cfg[ch].gain);
        }
        if (ch_cfg[ch].bw > 0) {
            fprintf(stderr, "[soapy_multi] ch%zu: bandwidth=%.0f\n", ch, ch_cfg[ch].bw);
            SoapySDRDevice_setBandwidth(dev, SOAPY_SDR_RX, ch, ch_cfg[ch].bw);
        }
        if (ch_cfg[ch].ant && ch_cfg[ch].ant[0]) {
            fprintf(stderr, "[soapy_multi] ch%zu: antenna=%s\n", ch, ch_cfg[ch].ant);
            SoapySDRDevice_setAntenna(dev, SOAPY_SDR_RX, ch, ch_cfg[ch].ant);
        }
    }

    // Setup multi-channel stream: channels=[0, 1]
    // Pass recv_frame_size via stream args for UHD transport tuning.
    SoapySDRKwargs streamArgs;
    memset(&streamArgs, 0, sizeof(streamArgs));
    parse_extra_args(&streamArgs, config->device_args);

    size_t channels[] = {0, 1};
    fprintf(stderr, "[soapy_multi] Setting up dual RX stream (CF32, ch 0+1)...\n");
    fflush(stderr);
    SoapySDRStream* stream = SoapySDRDevice_setupStream(
        dev, SOAPY_SDR_RX, SOAPY_SDR_CF32, channels, 2, &streamArgs);
    SoapySDRKwargs_clear(&streamArgs);
    if (!stream) {
        set_error("setupStream failed: %s", SoapySDRDevice_lastError());
        fprintf(stderr, "[soapy_multi] ERROR: %s\n", s_error_buf);
        SoapySDRDevice_unmake(dev);
        return NULL;
    }
    fprintf(stderr, "[soapy_multi] Stream setup OK\n");

    // Re-apply per-channel frequencies after setupStream.
    // B200's setupStream can reconfigure the DSP, resetting channel tuning.
    for (size_t ch = 0; ch < 2; ch++) {
        if (ch_cfg[ch].freq > 0) {
            SoapySDRKwargs retuneArgs;
            memset(&retuneArgs, 0, sizeof(retuneArgs));
            SoapySDRDevice_setFrequency(dev, SOAPY_SDR_RX, ch,
                                         ch_cfg[ch].freq, &retuneArgs);
            double actual = SoapySDRDevice_getFrequency(dev, SOAPY_SDR_RX, ch);
            fprintf(stderr, "[soapy_multi] ch%zu: re-tuned freq=%.0f (actual=%.0f)\n",
                    ch, ch_cfg[ch].freq, actual);
        }
    }

    fprintf(stderr, "[soapy_multi] Activating...\n");
    fflush(stderr);

    // Multi-channel streams on B200/B210 require a timed start command.
    // UHD rejects stream_now=true for multi-channel streamers because it
    // cannot guarantee time alignment between channels. We use
    // SOAPY_SDR_HAS_TIME with a time 100ms in the future to allow the
    // hardware to synchronize both channels.
    long long now_ns = SoapySDRDevice_getHardwareTime(dev, "");
    long long start_ns = now_ns + 100000000LL;  // +100ms
    fprintf(stderr, "[soapy_multi] Timed start: now=%lld start=%lld (+100ms)\n",
            now_ns, start_ns);

    if (SoapySDRDevice_activateStream(dev, stream,
            SOAPY_SDR_HAS_TIME, start_ns, 0) != 0) {
        set_error("activateStream failed: %s", SoapySDRDevice_lastError());
        fprintf(stderr, "[soapy_multi] ERROR: %s\n", s_error_buf);
        SoapySDRDevice_closeStream(dev, stream);
        SoapySDRDevice_unmake(dev);
        return NULL;
    }
    fprintf(stderr, "[soapy_multi] Dual RX stream activated\n");

    // Allocate bridge
    radio_bridge_multi_t* bridge = calloc(1, sizeof(radio_bridge_multi_t));
    if (!bridge) {
        set_error("malloc failed");
        SoapySDRDevice_deactivateStream(dev, stream, 0, 0);
        SoapySDRDevice_closeStream(dev, stream);
        SoapySDRDevice_unmake(dev);
        return NULL;
    }

    bridge->device      = dev;
    bridge->stream      = stream;
    bridge->sample_rate = SoapySDRDevice_getSampleRate(dev, SOAPY_SDR_RX, 0);
    for (size_t ch = 0; ch < 2; ch++) {
        bridge->center_freq[ch] = SoapySDRDevice_getFrequency(dev, SOAPY_SDR_RX, ch);
        bridge->gain_db[ch]     = SoapySDRDevice_getGain(dev, SOAPY_SDR_RX, ch);
    }

    fprintf(stderr, "[soapy_multi] Actual: rate=%.0f ch0_freq=%.0f ch1_freq=%.0f\n",
            bridge->sample_rate, bridge->center_freq[0], bridge->center_freq[1]);

    return bridge;
}

int radio_bridge_multi_read(radio_bridge_multi_t* bridge,
                            float* buf0, float* buf1,
                            size_t num_samples, double timeout_sec) {
    if (!bridge || !bridge->device || !bridge->stream || !buf0 || !buf1) {
        return RADIO_BRIDGE_ERR_NULL;
    }

    long timeout_us = (long)(timeout_sec * 1e6);

    void* buffs[] = {buf0, buf1};
    int flags = 0;
    long long time_ns = 0;

    int ret = SoapySDRDevice_readStream(bridge->device, bridge->stream,
                                        buffs, num_samples,
                                        &flags, &time_ns, timeout_us);

    if (ret == SOAPY_SDR_OVERFLOW) {
        bridge->overflow_count++;
        if (bridge->overflow_count == 1 ||
            bridge->overflow_count % 100 == 0) {
            fprintf(stderr, "[soapy_multi] RX overflow #%llu (samples dropped)\n",
                    (unsigned long long)bridge->overflow_count);
        }
    } else if (ret > 0) {
        bridge->read_count += (uint64_t)ret;
    }

    return ret;
}

double radio_bridge_multi_get_sample_rate(const radio_bridge_multi_t* bridge) {
    return bridge ? bridge->sample_rate : 0.0;
}

double radio_bridge_multi_get_center_freq(const radio_bridge_multi_t* bridge,
                                          size_t channel) {
    return (bridge && channel < 2) ? bridge->center_freq[channel] : 0.0;
}

double radio_bridge_multi_get_gain(const radio_bridge_multi_t* bridge,
                                   size_t channel) {
    return (bridge && channel < 2) ? bridge->gain_db[channel] : 0.0;
}

void radio_bridge_multi_destroy_ex(radio_bridge_multi_t* bridge,
                                   int skip_unmake) {
    if (!bridge) return;

    if (bridge->overflow_count > 0) {
        fprintf(stderr, "[soapy_multi] Total RX overflows: %llu\n",
                (unsigned long long)bridge->overflow_count);
    }

    if (bridge->stream && bridge->device) {
        fprintf(stderr, "[soapy_multi] Deactivating stream...\n");
        SoapySDRDevice_deactivateStream(bridge->device, bridge->stream, 0, 0);
        SoapySDRDevice_closeStream(bridge->device, bridge->stream);
        bridge->stream = NULL;
    }
    if (!skip_unmake && bridge->device) {
        fprintf(stderr, "[soapy_multi] Unmaking device...\n");
        SoapySDRDevice_unmake(bridge->device);
        bridge->device = NULL;
    } else if (bridge->device) {
        fprintf(stderr, "[soapy_multi] Skipping device unmake (process exit)\n");
        bridge->device = NULL;
    }
    free(bridge);
}
