// SPDX-License-Identifier: ISC
//
// common.hpp — shared utilities for lora_trx and lora_scan.

#ifndef LORA_APPS_COMMON_HPP
#define LORA_APPS_COMMON_HPP

#include <atomic>
#include <complex>
#include <csignal>
#include <cstdint>
#include <string>

#include <gnuradio-4.0/lora/log.hpp>

// log_hardware_info() and soapy_reliability_defaults() use gr::blocks::soapy
// types — include Soapy.hpp before this header.

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
    auto args = gr::blocks::soapy::detail::buildDeviceArgs(device, device_param);
    gr::blocks::soapy::Device dev(args);
    if (dev.get() == nullptr) {
        return {};
    }
    auto info = dev.getHardwareInfo();
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

constexpr uint32_t kSoapyChunkSize       = 512U << 4U;   // 8192 samples per poll
constexpr uint32_t kSoapyMaxOverflow     = 0U;            // disabled — throws gr::exception that kills scheduler
constexpr uint32_t kSoapyMaxConsecErrors = 0U;            // disabled — restart kills MIMO streams
constexpr uint32_t kSoapyTimeoutUs       = 10'000U;      // readStream timeout

inline gr::property_map soapy_reliability_defaults() {
    return {
        {"max_chunck_size",        kSoapyChunkSize},
        {"max_overflow_count",     gr::Size_t{kSoapyMaxOverflow}},
        {"max_consecutive_errors", gr::Size_t{kSoapyMaxConsecErrors}},
        {"max_time_out_us",        kSoapyTimeoutUs},
        {"max_fragment_count",     gr::Size_t{0U}},        // disabled — throws gr::exception that kills scheduler
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
