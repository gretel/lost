// SPDX-License-Identifier: ISC
//
// common.hpp — shared utilities for lora_trx and lora_scan.

#ifndef LORA_APPS_COMMON_HPP
#define LORA_APPS_COMMON_HPP

#include <atomic>
#include <complex>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <sstream>
#include <string>

#include <gnuradio-4.0/lora/log.hpp>

// log_hardware_info() uses gr::blocks::sdr::soapy types — include
// SoapySource.hpp and/or SoapyRaiiWrapper.hpp before this header.

namespace lora_apps {

using cf32 = std::complex<float>;

inline std::atomic<bool> g_stop{false};  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

inline void signal_handler(int /*sig*/) {
    g_stop.store(true, std::memory_order_relaxed);
}

inline std::atomic<bool>& install_signal_handler() {
    std::ignore = std::signal(SIGINT, signal_handler);
    std::ignore = std::signal(SIGTERM, signal_handler);
    return g_stop;
}

// Homebrew UHD patches B2XX_FPGA_FILE_NAME to absolute Cellar paths that
// break after revision bumps — override with bare filenames.
// Only applies to B200-family devices (B200, B200mini, B210, B220).
inline void apply_fpga_workaround(const std::string& device, std::string& device_param) {
    if (device == "uhd"
        && device_param.find("type=b2") != std::string::npos
        && device_param.find("fpga=") == std::string::npos) {
        device_param += ",fpga=usrp_b210_fpga.bin";
    }
}

/// Query hardware info and log it. Returns the device serial (empty if unavailable).
inline std::string log_hardware_info(const char* app_name,
                                     const std::string& device,
                                     const std::string& device_param) {
    auto devKwargs = gr::blocks::sdr::soapy::Kwargs{{"driver", device}};
    if (!device_param.empty()) {
        std::stringstream ss(device_param);
        std::string keyVal;
        while (std::getline(ss, keyVal, ',')) {
            auto pos = keyVal.find('=');
            if (pos != std::string::npos) {
                devKwargs[keyVal.substr(0, pos)] = keyVal.substr(pos + 1);
            }
        }
    }
    auto devResult = gr::blocks::sdr::soapy::Device::make(devKwargs);
    if (!devResult) {
        std::fprintf(stderr, "  Cannot open device: %s\n", devResult.error().message.c_str());
        return {};
    }
    auto& dev = *devResult;
    // Use C API for hardware info (not wrapped in the RAII Device class)
    SoapySDRKwargs hwInfo = SoapySDRDevice_getHardwareInfo(dev.get());
    gr::blocks::sdr::soapy::Kwargs info;
    for (std::size_t i = 0; i < hwInfo.size; ++i) {
        info[hwInfo.keys[i]] = hwInfo.vals[i];
    }
    SoapySDRKwargs_clear(&hwInfo);
    std::string fpga_ver, serial;
    if (auto it = info.find("fpga_version"); it != info.end()) {
        fpga_ver = it->second;
    }
    if (auto it = info.find("serial"); it != info.end()) {
        serial = it->second;
    }
    if (!fpga_ver.empty() || !serial.empty()) {
        gr::lora::log_ts("info ", app_name,
            "hardware: FPGA v%s  serial %s",
            fpga_ver.empty() ? "?" : fpga_ver.c_str(),
            serial.empty() ? "?" : serial.c_str());
    }
    return serial;
}

// --- SoapyBlock reliability defaults (shared by lora_trx and lora_scan) ---

constexpr uint32_t kSoapyChunkSize   = 512U << 4U;   // 8192 samples per poll
constexpr uint32_t kSoapyMaxOverflow = 0U;            // disabled — upstream logs but no longer throws
// readStream timeout: must exceed chunk airtime at the LOWEST sample rate we use,
// otherwise the IO thread busy-spins on SOAPY_SDR_TIMEOUT.  At 250 kS/s narrowband
// decode and 8192-sample chunks, one chunk = 32.768 ms.  100 ms gives 3x safety
// margin and is still <<100 ms at 16 MS/s scan rate (chunk = 0.512 ms there).
constexpr uint32_t kSoapyTimeoutUs   = 100'000U;

inline gr::property_map soapy_reliability_defaults() {
    return {
        {"max_chunk_size",     kSoapyChunkSize},
        {"max_overflow_count", gr::Size_t{kSoapyMaxOverflow}},
        {"max_time_out_us",    kSoapyTimeoutUs},
        {"max_fragment_count", gr::Size_t{0U}},        // disabled
        {"verbose_overflow",   false},
    };
}

inline void log_version_banner(const char* app_name, const char* git_rev) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdate-time"
    gr::lora::log_ts("info ", app_name,
        "starting %s %s (built " __DATE__ " " __TIME__ ")", app_name, git_rev);
#pragma GCC diagnostic pop
}

} // namespace lora_apps

#endif // LORA_APPS_COMMON_HPP
