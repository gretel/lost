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

// log_hardware_info() uses gr::blocks::soapy types — include Soapy.hpp
// or SoapyRaiiWrapper.hpp before this header.

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
inline void apply_fpga_workaround(const std::string& device, std::string& device_param) {
    if (device == "uhd" && device_param.find("fpga=") == std::string::npos) {
        device_param += ",fpga=usrp_b210_fpga.bin";
    }
}

inline void log_hardware_info(const char* app_name,
                              const std::string& device,
                              const std::string& device_param) {
    auto args = gr::blocks::soapy::detail::buildDeviceArgs(device, device_param);
    gr::blocks::soapy::Device dev(args);
    if (dev.get() == nullptr) {
        return;
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
