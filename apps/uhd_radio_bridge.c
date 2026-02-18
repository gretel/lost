// SPDX-License-Identifier: ISC
//
// Pure-C implementation of radio_bridge API using the UHD C API.
// MUST be compiled with system compiler (Apple Clang) to match the ABI of
// Homebrew UHD library. Do NOT compile with LLVM 19.

#include "radio_bridge.h"

#include <uhd/usrp/usrp.h>
#include <uhd/types/metadata.h>
#include <uhd/types/sensors.h>
#include <uhd/version.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// --- Internal state ---

struct radio_bridge {
    uhd_usrp_handle      usrp;
    uhd_rx_streamer_handle rx_streamer;
    uhd_tx_streamer_handle tx_streamer;
    uhd_rx_metadata_handle rx_md;
    uhd_tx_metadata_handle tx_md;
    size_t               channel;
    int                  direction;   // RADIO_BRIDGE_RX or RADIO_BRIDGE_TX
    double               sample_rate;
    double               center_freq;
    double               gain_db;
    size_t               max_samps;   // max samples per recv/send call
    // Runtime stats
    uint64_t             overflow_count;
    uint64_t             underflow_count;
    uint64_t             sample_count;
};

struct radio_bridge_multi {
    uhd_usrp_handle      usrp;
    uhd_rx_streamer_handle rx_streamer;
    uhd_rx_metadata_handle rx_md;
    double               sample_rate;
    double               center_freq[2];
    double               gain_db[2];
    size_t               max_samps;
    uint64_t             overflow_count;
    uint64_t             sample_count;
};

// Thread-local error message buffer
static _Thread_local char s_error_buf[512] = {0};

static void set_error(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(s_error_buf, sizeof(s_error_buf), fmt, args);
    va_end(args);
}

// Helper: get UHD error string from handle
static void report_usrp_error(uhd_usrp_handle usrp, const char* context) {
    char err[512];
    uhd_usrp_last_error(usrp, err, sizeof(err));
    set_error("%s: %s", context, err);
    fprintf(stderr, "[uhd_bridge] ERROR: %s: %s\n", context, err);
}

// Helper: check ref_locked sensor and wait up to timeout_sec
static int wait_ref_locked(uhd_usrp_handle usrp, double timeout_sec) {
    uhd_sensor_value_handle sensor;
    if (uhd_sensor_value_make(&sensor) != UHD_ERROR_NONE) return 0;

    int locked = 0;
    int attempts = (int)(timeout_sec * 10);  // check every 100ms
    if (attempts < 1) attempts = 1;

    for (int i = 0; i < attempts; i++) {
        if (uhd_usrp_get_mboard_sensor(usrp, "ref_locked", 0, &sensor)
                != UHD_ERROR_NONE) {
            // Sensor not available — skip lock check
            locked = 1;
            break;
        }
        bool val = false;
        uhd_sensor_value_to_bool(sensor, &val);
        if (val) {
            locked = 1;
            break;
        }
        usleep(100000);  // 100ms
    }

    uhd_sensor_value_free(&sensor);
    return locked;
}

// Helper: check if device_args contains "type=" to restrict enumeration.
// Without it, UHD probes ALL transport types including X300 PCIe, whose
// boost::serialization code uses std::locale — which crashes when two
// copies of libc++ coexist (e.g. LLVM 19 + system). Bug in UHD <= 4.9.
static void warn_if_no_type_filter(const char* args) {
    if (!args || !args[0] || !strstr(args, "type=")) {
        fprintf(stderr,
            "[uhd_bridge] WARNING: no 'type=' in device args.\n"
            "  UHD will probe all transports including X300/PCIe.\n"
            "  On macOS with dual-libc++ this may crash. Use e.g.\n"
            "  --args type=b200 to restrict enumeration.\n");
    }
}

// --- Probe ---

int radio_bridge_probe(const char* device_args) {
    uhd_string_vector_handle results;
    if (uhd_string_vector_make(&results) != UHD_ERROR_NONE) return 0;

    const char* args = (device_args && device_args[0]) ? device_args : "";
    warn_if_no_type_filter(args);
    uhd_usrp_find(args, &results);

    size_t count = 0;
    uhd_string_vector_size(results, &count);
    uhd_string_vector_free(&results);

    return (count > 0) ? 1 : 0;
}

// --- Single-channel create ---

radio_bridge_t* radio_bridge_create(const radio_bridge_config_t* config) {
    if (!config) {
        set_error("NULL config");
        return NULL;
    }

    const char* dir_name = (config->direction == RADIO_BRIDGE_TX) ? "TX" : "RX";
    const char* args = (config->device_args && config->device_args[0])
                       ? config->device_args : "";

    // Print UHD version
    char ver[64] = {0};
    uhd_get_version_string(ver, sizeof(ver));
    fprintf(stderr, "[uhd_bridge] UHD version: %s\n", ver);

    // Create USRP
    fprintf(stderr, "[uhd_bridge] Making USRP (args=\"%s\", %s)...\n", args, dir_name);
    uhd_usrp_handle usrp = NULL;
    if (uhd_usrp_make(&usrp, args) != UHD_ERROR_NONE) {
        set_error("uhd_usrp_make failed");
        return NULL;
    }

    // Print device info
    char pp[1024];
    uhd_usrp_get_pp_string(usrp, pp, sizeof(pp));
    fprintf(stderr, "%s", pp);

    // Clock source
    if (config->clock_source && config->clock_source[0]) {
        fprintf(stderr, "[uhd_bridge] Setting clock_source=%s\n", config->clock_source);
        if (uhd_usrp_set_clock_source(usrp, config->clock_source, 0)
                != UHD_ERROR_NONE) {
            fprintf(stderr, "[uhd_bridge] WARNING: set_clock_source failed (non-fatal)\n");
        }
        // Wait for reference lock
        fprintf(stderr, "[uhd_bridge] Waiting for ref lock...\n");
        if (!wait_ref_locked(usrp, 3.0)) {
            fprintf(stderr, "[uhd_bridge] WARNING: ref not locked after 3s\n");
        } else {
            fprintf(stderr, "[uhd_bridge] Ref locked\n");
        }
    }

    size_t ch = config->channel;

    // Sample rate
    if (config->sample_rate > 0) {
        fprintf(stderr, "[uhd_bridge] Setting %s rate=%.0f ch=%zu\n",
                dir_name, config->sample_rate, ch);
        uhd_error err;
        if (config->direction == RADIO_BRIDGE_TX)
            err = uhd_usrp_set_tx_rate(usrp, config->sample_rate, ch);
        else
            err = uhd_usrp_set_rx_rate(usrp, config->sample_rate, ch);
        if (err != UHD_ERROR_NONE) {
            report_usrp_error(usrp, "set_rate");
            uhd_usrp_free(&usrp);
            return NULL;
        }
    }

    // Center frequency
    if (config->center_freq > 0) {
        uhd_tune_request_t tune_req = {
            .target_freq     = config->center_freq,
            .rf_freq_policy  = UHD_TUNE_REQUEST_POLICY_AUTO,
            .dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO,
        };
        uhd_tune_result_t tune_result;
        fprintf(stderr, "[uhd_bridge] Setting %s freq=%.0f ch=%zu\n",
                dir_name, config->center_freq, ch);
        uhd_error err;
        if (config->direction == RADIO_BRIDGE_TX)
            err = uhd_usrp_set_tx_freq(usrp, &tune_req, ch, &tune_result);
        else
            err = uhd_usrp_set_rx_freq(usrp, &tune_req, ch, &tune_result);
        if (err != UHD_ERROR_NONE) {
            report_usrp_error(usrp, "set_freq");
            uhd_usrp_free(&usrp);
            return NULL;
        }
    }

    // Gain
    if (config->gain_db != 0) {
        fprintf(stderr, "[uhd_bridge] Setting %s gain=%.1f dB ch=%zu\n",
                dir_name, config->gain_db, ch);
        uhd_error err;
        if (config->direction == RADIO_BRIDGE_TX)
            err = uhd_usrp_set_tx_gain(usrp, config->gain_db, ch, "");
        else
            err = uhd_usrp_set_rx_gain(usrp, config->gain_db, ch, "");
        if (err != UHD_ERROR_NONE) {
            report_usrp_error(usrp, "set_gain");
            uhd_usrp_free(&usrp);
            return NULL;
        }
    }

    // Bandwidth
    if (config->bandwidth > 0) {
        fprintf(stderr, "[uhd_bridge] Setting %s bandwidth=%.0f ch=%zu\n",
                dir_name, config->bandwidth, ch);
        if (config->direction == RADIO_BRIDGE_TX)
            uhd_usrp_set_tx_bandwidth(usrp, config->bandwidth, ch);
        else
            uhd_usrp_set_rx_bandwidth(usrp, config->bandwidth, ch);
    }

    // Antenna
    if (config->antenna && config->antenna[0]) {
        fprintf(stderr, "[uhd_bridge] Setting %s antenna=%s ch=%zu\n",
                dir_name, config->antenna, ch);
        if (config->direction == RADIO_BRIDGE_TX)
            uhd_usrp_set_tx_antenna(usrp, config->antenna, ch);
        else
            uhd_usrp_set_rx_antenna(usrp, config->antenna, ch);
    }

    // Create streamer
    uhd_stream_args_t stream_args = {
        .cpu_format   = "fc32",
        .otw_format   = "sc16",
        .args         = "",
        .channel_list = &ch,
        .n_channels   = 1,
    };

    radio_bridge_t* bridge = calloc(1, sizeof(radio_bridge_t));
    if (!bridge) {
        set_error("malloc failed");
        uhd_usrp_free(&usrp);
        return NULL;
    }
    bridge->usrp      = usrp;
    bridge->channel   = ch;
    bridge->direction = config->direction;

    if (config->direction == RADIO_BRIDGE_RX) {
        // RX streamer
        if (uhd_rx_streamer_make(&bridge->rx_streamer) != UHD_ERROR_NONE) {
            set_error("uhd_rx_streamer_make failed");
            uhd_usrp_free(&usrp);
            free(bridge);
            return NULL;
        }
        if (uhd_usrp_get_rx_stream(usrp, &stream_args, bridge->rx_streamer)
                != UHD_ERROR_NONE) {
            report_usrp_error(usrp, "get_rx_stream");
            uhd_rx_streamer_free(&bridge->rx_streamer);
            uhd_usrp_free(&usrp);
            free(bridge);
            return NULL;
        }
        uhd_rx_streamer_max_num_samps(bridge->rx_streamer, &bridge->max_samps);
        fprintf(stderr, "[uhd_bridge] RX max_samps_per_packet=%zu\n", bridge->max_samps);

        // RX metadata
        uhd_rx_metadata_make(&bridge->rx_md);

        // Issue stream command: start continuous
        uhd_stream_cmd_t stream_cmd = {
            .stream_mode = UHD_STREAM_MODE_START_CONTINUOUS,
            .num_samps   = 0,
            .stream_now  = true,
        };
        if (uhd_rx_streamer_issue_stream_cmd(bridge->rx_streamer, &stream_cmd)
                != UHD_ERROR_NONE) {
            set_error("issue_stream_cmd failed");
            uhd_rx_metadata_free(&bridge->rx_md);
            uhd_rx_streamer_free(&bridge->rx_streamer);
            uhd_usrp_free(&usrp);
            free(bridge);
            return NULL;
        }
        fprintf(stderr, "[uhd_bridge] RX stream started (continuous)\n");
    } else {
        // TX streamer
        if (uhd_tx_streamer_make(&bridge->tx_streamer) != UHD_ERROR_NONE) {
            set_error("uhd_tx_streamer_make failed");
            uhd_usrp_free(&usrp);
            free(bridge);
            return NULL;
        }
        if (uhd_usrp_get_tx_stream(usrp, &stream_args, bridge->tx_streamer)
                != UHD_ERROR_NONE) {
            report_usrp_error(usrp, "get_tx_stream");
            uhd_tx_streamer_free(&bridge->tx_streamer);
            uhd_usrp_free(&usrp);
            free(bridge);
            return NULL;
        }
        uhd_tx_streamer_max_num_samps(bridge->tx_streamer, &bridge->max_samps);
        fprintf(stderr, "[uhd_bridge] TX max_samps_per_packet=%zu\n", bridge->max_samps);

        // TX metadata: no time spec, start-of-burst, not end-of-burst
        uhd_tx_metadata_make(&bridge->tx_md, false, 0, 0.0, true, false);
    }

    // Read back actual values
    double actual_rate = 0, actual_freq = 0, actual_gain = 0;
    if (config->direction == RADIO_BRIDGE_RX) {
        uhd_usrp_get_rx_rate(usrp, ch, &actual_rate);
        uhd_usrp_get_rx_freq(usrp, ch, &actual_freq);
        uhd_usrp_get_rx_gain(usrp, ch, "", &actual_gain);
    } else {
        uhd_usrp_get_tx_rate(usrp, ch, &actual_rate);
        uhd_usrp_get_tx_freq(usrp, ch, &actual_freq);
        uhd_usrp_get_tx_gain(usrp, ch, "", &actual_gain);
    }
    bridge->sample_rate = actual_rate;
    bridge->center_freq = actual_freq;
    bridge->gain_db     = actual_gain;

    fprintf(stderr, "[uhd_bridge] Actual: rate=%.0f freq=%.0f gain=%.1f\n",
            actual_rate, actual_freq, actual_gain);

    return bridge;
}

const char* radio_bridge_last_error(void) {
    return s_error_buf;
}

// --- Single-channel RX ---

int radio_bridge_read(radio_bridge_t* bridge, float* buf,
                    size_t num_samples, double timeout_sec) {
    if (!bridge || !bridge->rx_streamer || !buf) {
        return RADIO_BRIDGE_ERR_NULL;
    }

    void* buffs[] = {buf};
    size_t items_recvd = 0;

    if (uhd_rx_streamer_recv(bridge->rx_streamer, buffs, num_samples,
                             &bridge->rx_md, timeout_sec, false, &items_recvd)
            != UHD_ERROR_NONE) {
        return RADIO_BRIDGE_ERR_DEVICE;
    }

    // Check metadata error code
    uhd_rx_metadata_error_code_t error_code;
    uhd_rx_metadata_error_code(bridge->rx_md, &error_code);

    if (error_code == UHD_RX_METADATA_ERROR_CODE_TIMEOUT) {
        return RADIO_BRIDGE_ERR_TIMEOUT;
    }
    if (error_code == UHD_RX_METADATA_ERROR_CODE_OVERFLOW) {
        bridge->overflow_count++;
        if (bridge->overflow_count == 1 ||
            bridge->overflow_count % 100 == 0) {
            fprintf(stderr, "[uhd_bridge] RX overflow #%llu (samples dropped)\n",
                    (unsigned long long)bridge->overflow_count);
        }
        return RADIO_BRIDGE_ERR_OVERFLOW;
    }
    if (error_code != UHD_RX_METADATA_ERROR_CODE_NONE) {
        char strerr[256];
        uhd_rx_metadata_strerror(bridge->rx_md, strerr, sizeof(strerr));
        fprintf(stderr, "[uhd_bridge] RX error 0x%x: %s\n",
                (unsigned)error_code, strerr);
        return RADIO_BRIDGE_ERR_DEVICE;
    }

    bridge->sample_count += items_recvd;
    return (int)items_recvd;
}

// --- Single-channel TX ---

int radio_bridge_write(radio_bridge_t* bridge, const float* buf,
                     size_t num_samples, double timeout_sec,
                     int end_burst) {
    if (!bridge || !bridge->tx_streamer || !buf) {
        return RADIO_BRIDGE_ERR_NULL;
    }

    // Update end-of-burst flag in metadata
    // We need to recreate the metadata for each call since the C API
    // doesn't expose a setter for end_of_burst.
    if (bridge->tx_md) {
        uhd_tx_metadata_free(&bridge->tx_md);
    }
    // After first write, no start-of-burst
    int sob = (bridge->sample_count == 0) ? 1 : 0;
    uhd_tx_metadata_make(&bridge->tx_md, false, 0, 0.0,
                         sob ? true : false,
                         end_burst ? true : false);

    const void* buffs[] = {buf};
    size_t items_sent = 0;

    if (uhd_tx_streamer_send(bridge->tx_streamer, buffs, num_samples,
                             &bridge->tx_md, timeout_sec, &items_sent)
            != UHD_ERROR_NONE) {
        return RADIO_BRIDGE_ERR_DEVICE;
    }

    bridge->sample_count += items_sent;
    return (int)items_sent;
}

// --- Getters ---

double radio_bridge_get_sample_rate(const radio_bridge_t* bridge) {
    return bridge ? bridge->sample_rate : 0.0;
}

double radio_bridge_get_center_freq(const radio_bridge_t* bridge) {
    return bridge ? bridge->center_freq : 0.0;
}

double radio_bridge_get_gain(const radio_bridge_t* bridge) {
    return bridge ? bridge->gain_db : 0.0;
}

// --- Destroy ---

void radio_bridge_destroy_ex(radio_bridge_t* bridge, int skip_free) {
    if (!bridge) return;

    // Log stats
    if (bridge->overflow_count > 0) {
        fprintf(stderr, "[uhd_bridge] Total RX overflows: %llu\n",
                (unsigned long long)bridge->overflow_count);
    }
    if (bridge->underflow_count > 0) {
        fprintf(stderr, "[uhd_bridge] Total TX underflows: %llu\n",
                (unsigned long long)bridge->underflow_count);
    }
    fprintf(stderr, "[uhd_bridge] Total samples: %llu\n",
            (unsigned long long)bridge->sample_count);

    // Stop RX stream
    if (bridge->rx_streamer) {
        uhd_stream_cmd_t stop_cmd = {
            .stream_mode = UHD_STREAM_MODE_STOP_CONTINUOUS,
            .num_samps   = 0,
            .stream_now  = true,
        };
        uhd_rx_streamer_issue_stream_cmd(bridge->rx_streamer, &stop_cmd);
        uhd_rx_streamer_free(&bridge->rx_streamer);
    }
    if (bridge->rx_md) {
        uhd_rx_metadata_free(&bridge->rx_md);
    }

    // Free TX resources
    if (bridge->tx_streamer) {
        uhd_tx_streamer_free(&bridge->tx_streamer);
    }
    if (bridge->tx_md) {
        uhd_tx_metadata_free(&bridge->tx_md);
    }

    // Free USRP
    if (!skip_free && bridge->usrp) {
        fprintf(stderr, "[uhd_bridge] Freeing USRP...\n");
        uhd_usrp_free(&bridge->usrp);
        fprintf(stderr, "[uhd_bridge] Destroyed\n");
    } else if (bridge->usrp) {
        fprintf(stderr, "[uhd_bridge] Skipping USRP free (process exit)\n");
    }
    free(bridge);
}

void radio_bridge_destroy(radio_bridge_t* bridge) {
    radio_bridge_destroy_ex(bridge, 0);
}

// --- Multi-channel RX ---

radio_bridge_multi_t* radio_bridge_multi_create(
        const radio_bridge_multi_config_t* config) {
    if (!config) {
        set_error("NULL config");
        return NULL;
    }

    const char* args = (config->device_args && config->device_args[0])
                       ? config->device_args : "";

    // Print UHD version
    char ver[64] = {0};
    uhd_get_version_string(ver, sizeof(ver));
    fprintf(stderr, "[uhd_multi] UHD version: %s\n", ver);

    // Create USRP
    fprintf(stderr, "[uhd_multi] Making USRP (args=\"%s\", dual RX)...\n", args);
    uhd_usrp_handle usrp = NULL;
    if (uhd_usrp_make(&usrp, args) != UHD_ERROR_NONE) {
        set_error("uhd_usrp_make failed");
        return NULL;
    }

    // Print device info
    char pp[1024];
    uhd_usrp_get_pp_string(usrp, pp, sizeof(pp));
    fprintf(stderr, "%s", pp);

    // Clock source
    if (config->clock_source && config->clock_source[0]) {
        fprintf(stderr, "[uhd_multi] Setting clock_source=%s\n",
                config->clock_source);
        uhd_usrp_set_clock_source(usrp, config->clock_source, 0);
        fprintf(stderr, "[uhd_multi] Waiting for ref lock...\n");
        if (!wait_ref_locked(usrp, 3.0)) {
            fprintf(stderr, "[uhd_multi] WARNING: ref not locked after 3s\n");
        } else {
            fprintf(stderr, "[uhd_multi] Ref locked\n");
        }
    }

    // Sample rate (shared across channels on B210)
    if (config->sample_rate > 0) {
        fprintf(stderr, "[uhd_multi] Setting RX rate=%.0f\n",
                config->sample_rate);
        // Set on both channels
        if (uhd_usrp_set_rx_rate(usrp, config->sample_rate, 0)
                != UHD_ERROR_NONE) {
            report_usrp_error(usrp, "set_rx_rate ch0");
            uhd_usrp_free(&usrp);
            return NULL;
        }
        uhd_usrp_set_rx_rate(usrp, config->sample_rate, 1);
    }

    // Per-channel settings
    struct { double freq; double gain; double bw; const char* ant; } ch_cfg[2] = {
        {config->center_freq_0, config->gain_db_0, config->bandwidth_0, config->antenna_0},
        {config->center_freq_1, config->gain_db_1, config->bandwidth_1, config->antenna_1},
    };

    for (size_t ch = 0; ch < 2; ch++) {
        if (ch_cfg[ch].freq > 0) {
            uhd_tune_request_t tune_req = {
                .target_freq     = ch_cfg[ch].freq,
                .rf_freq_policy  = UHD_TUNE_REQUEST_POLICY_AUTO,
                .dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO,
            };
            uhd_tune_result_t tune_result;
            fprintf(stderr, "[uhd_multi] ch%zu: setting freq=%.0f\n",
                    ch, ch_cfg[ch].freq);
            if (uhd_usrp_set_rx_freq(usrp, &tune_req, ch, &tune_result)
                    != UHD_ERROR_NONE) {
                report_usrp_error(usrp, "set_rx_freq");
                uhd_usrp_free(&usrp);
                return NULL;
            }
            double actual = 0;
            uhd_usrp_get_rx_freq(usrp, ch, &actual);
            fprintf(stderr, "[uhd_multi] ch%zu: actual freq=%.0f\n", ch, actual);
        }
        if (ch_cfg[ch].gain != 0) {
            fprintf(stderr, "[uhd_multi] ch%zu: gain=%.1f dB\n",
                    ch, ch_cfg[ch].gain);
            uhd_usrp_set_rx_gain(usrp, ch_cfg[ch].gain, ch, "");
        }
        if (ch_cfg[ch].bw > 0) {
            fprintf(stderr, "[uhd_multi] ch%zu: bandwidth=%.0f\n",
                    ch, ch_cfg[ch].bw);
            uhd_usrp_set_rx_bandwidth(usrp, ch_cfg[ch].bw, ch);
        }
        if (ch_cfg[ch].ant && ch_cfg[ch].ant[0]) {
            fprintf(stderr, "[uhd_multi] ch%zu: antenna=%s\n",
                    ch, ch_cfg[ch].ant);
            uhd_usrp_set_rx_antenna(usrp, ch_cfg[ch].ant, ch);
        }
    }

    // Create RX streamer with both channels
    size_t channels[] = {0, 1};
    uhd_stream_args_t stream_args = {
        .cpu_format   = "fc32",
        .otw_format   = "sc16",
        .args         = "",
        .channel_list = channels,
        .n_channels   = 2,
    };

    uhd_rx_streamer_handle rx_streamer = NULL;
    if (uhd_rx_streamer_make(&rx_streamer) != UHD_ERROR_NONE) {
        set_error("uhd_rx_streamer_make failed");
        uhd_usrp_free(&usrp);
        return NULL;
    }

    fprintf(stderr, "[uhd_multi] Creating dual-channel RX stream...\n");
    if (uhd_usrp_get_rx_stream(usrp, &stream_args, rx_streamer)
            != UHD_ERROR_NONE) {
        report_usrp_error(usrp, "get_rx_stream (dual)");
        uhd_rx_streamer_free(&rx_streamer);
        uhd_usrp_free(&usrp);
        return NULL;
    }

    size_t max_samps = 0;
    uhd_rx_streamer_max_num_samps(rx_streamer, &max_samps);
    fprintf(stderr, "[uhd_multi] RX max_samps_per_packet=%zu\n", max_samps);

    // Verify frequencies are still correct after get_rx_stream
    for (size_t ch = 0; ch < 2; ch++) {
        double actual = 0;
        uhd_usrp_get_rx_freq(usrp, ch, &actual);
        fprintf(stderr, "[uhd_multi] ch%zu: post-stream freq=%.0f\n", ch, actual);
    }

    // Create metadata
    uhd_rx_metadata_handle rx_md = NULL;
    uhd_rx_metadata_make(&rx_md);

    // Start streaming with timed command for multi-channel alignment.
    // For multi-channel, stream_now=false with a time spec is required
    // by UHD's super_recv_packet_handler for time alignment.
    int64_t now_full = 0;
    double now_frac = 0;
    uhd_usrp_get_time_now(usrp, 0, &now_full, &now_frac);

    uhd_stream_cmd_t stream_cmd = {
        .stream_mode         = UHD_STREAM_MODE_START_CONTINUOUS,
        .num_samps           = 0,
        .stream_now          = false,
        .time_spec_full_secs = now_full,
        .time_spec_frac_secs = now_frac + 0.1,  // +100ms
    };

    fprintf(stderr, "[uhd_multi] Starting stream at t=%lld + %.3f (+100ms)\n",
            (long long)now_full, now_frac + 0.1);

    if (uhd_rx_streamer_issue_stream_cmd(rx_streamer, &stream_cmd)
            != UHD_ERROR_NONE) {
        set_error("issue_stream_cmd failed");
        uhd_rx_metadata_free(&rx_md);
        uhd_rx_streamer_free(&rx_streamer);
        uhd_usrp_free(&usrp);
        return NULL;
    }
    fprintf(stderr, "[uhd_multi] Dual RX stream started\n");

    // Allocate bridge
    radio_bridge_multi_t* bridge = calloc(1, sizeof(radio_bridge_multi_t));
    if (!bridge) {
        set_error("malloc failed");
        uhd_stream_cmd_t stop = {.stream_mode = UHD_STREAM_MODE_STOP_CONTINUOUS,
                                 .stream_now = true};
        uhd_rx_streamer_issue_stream_cmd(rx_streamer, &stop);
        uhd_rx_metadata_free(&rx_md);
        uhd_rx_streamer_free(&rx_streamer);
        uhd_usrp_free(&usrp);
        return NULL;
    }

    bridge->usrp         = usrp;
    bridge->rx_streamer  = rx_streamer;
    bridge->rx_md        = rx_md;
    bridge->max_samps    = max_samps;

    // Read back actual values
    uhd_usrp_get_rx_rate(usrp, 0, &bridge->sample_rate);
    for (size_t ch = 0; ch < 2; ch++) {
        uhd_usrp_get_rx_freq(usrp, ch, &bridge->center_freq[ch]);
        uhd_usrp_get_rx_gain(usrp, ch, "", &bridge->gain_db[ch]);
    }

    fprintf(stderr, "[uhd_multi] Actual: rate=%.0f ch0_freq=%.0f ch1_freq=%.0f\n",
            bridge->sample_rate, bridge->center_freq[0], bridge->center_freq[1]);

    return bridge;
}

// --- Multi-channel RX read ---

int radio_bridge_multi_read(radio_bridge_multi_t* bridge,
                          float* buf0, float* buf1,
                          size_t num_samples, double timeout_sec) {
    if (!bridge || !bridge->rx_streamer || !buf0 || !buf1) {
        return RADIO_BRIDGE_ERR_NULL;
    }

    // UHD's C API recv takes void** — one buffer per channel.
    void* buffs[] = {buf0, buf1};
    size_t items_recvd = 0;

    if (uhd_rx_streamer_recv(bridge->rx_streamer, buffs, num_samples,
                             &bridge->rx_md, timeout_sec, false, &items_recvd)
            != UHD_ERROR_NONE) {
        return RADIO_BRIDGE_ERR_DEVICE;
    }

    uhd_rx_metadata_error_code_t error_code;
    uhd_rx_metadata_error_code(bridge->rx_md, &error_code);

    if (error_code == UHD_RX_METADATA_ERROR_CODE_TIMEOUT) {
        return RADIO_BRIDGE_ERR_TIMEOUT;
    }
    if (error_code == UHD_RX_METADATA_ERROR_CODE_OVERFLOW) {
        bridge->overflow_count++;
        if (bridge->overflow_count == 1 ||
            bridge->overflow_count % 100 == 0) {
            fprintf(stderr, "[uhd_multi] RX overflow #%llu (samples dropped)\n",
                    (unsigned long long)bridge->overflow_count);
        }
        return RADIO_BRIDGE_ERR_OVERFLOW;
    }
    if (error_code == UHD_RX_METADATA_ERROR_CODE_ALIGNMENT) {
        // Log but don't treat as fatal — data may still be usable
        fprintf(stderr, "[uhd_multi] RX alignment error (time-align failed)\n");
        // Return whatever we got, if anything
        if (items_recvd > 0) {
            bridge->sample_count += items_recvd;
            return (int)items_recvd;
        }
        return RADIO_BRIDGE_ERR_OVERFLOW;  // treat as overflow for retry
    }
    if (error_code != UHD_RX_METADATA_ERROR_CODE_NONE) {
        char strerr[256];
        uhd_rx_metadata_strerror(bridge->rx_md, strerr, sizeof(strerr));
        fprintf(stderr, "[uhd_multi] RX error 0x%x: %s\n",
                (unsigned)error_code, strerr);
        return RADIO_BRIDGE_ERR_DEVICE;
    }

    bridge->sample_count += items_recvd;
    return (int)items_recvd;
}

// --- Multi-channel getters ---

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

// --- Multi-channel destroy ---

void radio_bridge_multi_destroy_ex(radio_bridge_multi_t* bridge, int skip_free) {
    if (!bridge) return;

    if (bridge->overflow_count > 0) {
        fprintf(stderr, "[uhd_multi] Total RX overflows: %llu\n",
                (unsigned long long)bridge->overflow_count);
    }
    fprintf(stderr, "[uhd_multi] Total samples: %llu\n",
            (unsigned long long)bridge->sample_count);

    // Stop stream
    if (bridge->rx_streamer) {
        uhd_stream_cmd_t stop_cmd = {
            .stream_mode = UHD_STREAM_MODE_STOP_CONTINUOUS,
            .num_samps   = 0,
            .stream_now  = true,
        };
        uhd_rx_streamer_issue_stream_cmd(bridge->rx_streamer, &stop_cmd);
        uhd_rx_streamer_free(&bridge->rx_streamer);
    }
    if (bridge->rx_md) {
        uhd_rx_metadata_free(&bridge->rx_md);
    }

    // Free USRP
    if (!skip_free && bridge->usrp) {
        fprintf(stderr, "[uhd_multi] Freeing USRP...\n");
        uhd_usrp_free(&bridge->usrp);
        fprintf(stderr, "[uhd_multi] Destroyed\n");
    } else if (bridge->usrp) {
        fprintf(stderr, "[uhd_multi] Skipping USRP free (process exit)\n");
    }
    free(bridge);
}
