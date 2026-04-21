// SPDX-License-Identifier: ISC
//
// common.hpp — shared utilities for lora_trx and lora_scan.

#ifndef LORA_APPS_COMMON_HPP
#define LORA_APPS_COMMON_HPP

#include <atomic>
#include <chrono>
#include <complex>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include <gnuradio-4.0/lora/log.hpp>

// log_hardware_info() uses gr::blocks::sdr::soapy types — include
// SoapySource.hpp and/or SoapyRaiiWrapper.hpp before this header.

namespace lora_apps {

using cf32 = std::complex<float>;

inline std::atomic<bool> g_stop{false}; // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

inline void signal_handler(int /*sig*/) { g_stop.store(true, std::memory_order_relaxed); }

inline std::atomic<bool>& install_signal_handler() {
    std::ignore = std::signal(SIGINT, signal_handler);
    std::ignore = std::signal(SIGTERM, signal_handler);
    return g_stop;
}

// Homebrew UHD patches B2XX_FPGA_FILE_NAME to absolute Cellar paths that
// break after revision bumps — override with bare filenames.
// Only applies to B200-family devices (B200, B200mini, B210, B220).
inline void apply_fpga_workaround(const std::string& device, std::string& device_param) {
    if (device == "uhd" && device_param.find("type=b2") != std::string::npos && device_param.find("fpga=") == std::string::npos) {
        device_param += ",fpga=usrp_b210_fpga.bin";
    }
}

struct HardwareInfo {
    std::string serial;
    std::size_t tx_channels{0};
    std::size_t rx_channels{0};
};

/// Query hardware info via SoapySDR enumerate (lightweight USB/mDNS scan).
/// Does NOT open the device — the scheduler's reinitDevice() will be the
/// first and only device open, avoiding UHD/libusb re-enumeration races.
/// Channel counts are inferred from the driver name since enumerate doesn't
/// expose them; callers can refine after the device is actually open.
inline HardwareInfo log_hardware_info(const char* app_name, const std::string& device, const std::string& device_param) {
    auto devKwargs = gr::blocks::sdr::soapy::Kwargs{{"driver", device}};
    if (!device_param.empty()) {
        std::stringstream ss(device_param);
        std::string       keyVal;
        while (std::getline(ss, keyVal, ',')) {
            auto pos = keyVal.find('=');
            if (pos != std::string::npos) {
                devKwargs[keyVal.substr(0, pos)] = keyVal.substr(pos + 1);
            }
        }
    }

    // Enumerate to get serial + driver label without a full device open
    std::string serial, driver_label;
    {
        SoapySDRKwargs enumArgs{};
        for (auto& [k, v] : devKwargs) {
            SoapySDRKwargs_set(&enumArgs, k.c_str(), v.c_str());
        }
        std::size_t enumLen     = 0;
        auto*       enumResults = SoapySDRDevice_enumerate(&enumArgs, &enumLen);
        for (std::size_t i = 0; i < enumLen; ++i) {
            for (std::size_t j = 0; j < enumResults[i].size; ++j) {
                auto key = std::string_view(enumResults[i].keys[j]);
                if (key == "serial" && serial.empty()) {
                    serial = enumResults[i].vals[j];
                }
                if (key == "driver" && driver_label.empty()) {
                    driver_label = enumResults[i].vals[j];
                }
                if (key == "label" && driver_label.empty()) {
                    driver_label = enumResults[i].vals[j];
                }
            }
        }
        SoapySDRKwargsList_clear(enumResults, enumLen);
        SoapySDRKwargs_clear(&enumArgs);
    }

    // Infer channel counts from driver — avoids a full device open.
    // B210/B220 via UHD: 2 TX, 4 RX.  Single-channel devices: 1 each.
    std::size_t nTx = 1, nRx = 1;
    if (device == "uhd" && device_param.find("type=b2") != std::string::npos) {
        nTx = 2;
        nRx = 4;
    }

    // Plumbing details — demoted to debug.  The human-facing "device" line
    // is emitted by the caller (lora_trx/lora_scan) using the returned
    // HardwareInfo so it can include the clock source inline.
    gr::lora::log_ts("debug", app_name, "hw probe   driver=%s (%zuT%zuR inferred)", driver_label.empty() ? device.c_str() : driver_label.c_str(), nTx, nRx);
    if (!device_param.empty()) {
        gr::lora::log_ts("debug", app_name, "hw params  %s", device_param.c_str());
    }

    return {.serial = serial, .tx_channels = nTx, .rx_channels = nRx};
}

// --- SoapyBlock reliability defaults (shared by lora_trx and lora_scan) ---

constexpr uint32_t kSoapyChunkSize   = 512U << 4U; // 8192 samples per poll
constexpr uint32_t kSoapyMaxOverflow = 0U;         // disabled: upstream logs overflows without throttling
// readStream timeout: must exceed chunk airtime at the LOWEST sample rate we use,
// otherwise the IO thread busy-spins on SOAPY_SDR_TIMEOUT.  At 250 kS/s narrowband
// decode and 8192-sample chunks, one chunk = 32.768 ms.  100 ms gives 3x safety
// margin and is still <<100 ms at 16 MS/s scan rate (chunk = 0.512 ms there).
constexpr uint32_t kSoapyTimeoutUs = 100'000U;

inline gr::property_map soapy_reliability_defaults() {
    return {
        {"max_chunk_size", kSoapyChunkSize},
        {"max_overflow_count", gr::Size_t{kSoapyMaxOverflow}},
        {"max_time_out_us", kSoapyTimeoutUs},
        {"max_fragment_count", gr::Size_t{0U}}, // disabled
        {"verbose_overflow", false},
    };
}

inline void log_version_banner(const char* app_name, const char* git_rev) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdate-time"
    gr::lora::log_ts("info ", app_name, "starting %s %s", app_name, git_rev);
    gr::lora::log_ts("debug", app_name, "build      " __DATE__ " " __TIME__);
#pragma GCC diagnostic pop
}

} // namespace lora_apps

#endif // LORA_APPS_COMMON_HPP
