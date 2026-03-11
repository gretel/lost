// SPDX-License-Identifier: ISC
//
// lora_scan — LoRa spectral scanner (M1: wideband FFT streaming).
//
// Streams continuously at a fixed L1 sample rate with a pinned FPGA master
// clock.  Computes wideband FFT spectrum in the main thread and optionally
// emits CBOR frames on stdout for the TUI spectrum analyzer.
//
// M2 will add decimation, CAD detection, and multi-mode scanning.

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/soapy/Soapy.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>

#include "common.hpp"

#include <gnuradio-4.0/lora/SpectrumTapBlock.hpp>
#include <gnuradio-4.0/lora/algorithm/SpectrumTap.hpp>
#include <gnuradio-4.0/lora/cbor.hpp>
#include <gnuradio-4.0/lora/log.hpp>

#ifndef GIT_REV
#define GIT_REV "dev"
#endif

using lora_apps::cf32;

namespace {

struct ScanConfig {
    std::string device       = "uhd";
    std::string device_param = "type=b200";
    double      freq_start   = 863.0e6;
    double      freq_stop    = 870.0e6;
    double      gain         = 40.0;
    double      l1_rate      = 16.0e6;
    double      master_clock = 32.0e6;
    bool        cbor_out     = false;
    uint32_t    fft_size     = 4096;

    [[nodiscard]] double center_freq() const {
        return (freq_start + freq_stop) / 2.0;
    }
};

void print_usage() {
    std::fprintf(stderr,
        "Usage: lora_scan [options]\n\n"
        "LoRa spectral scanner (M1: wideband FFT streaming).\n\n"
        "Options:\n"
        "  --device <driver>     SoapySDR driver (default: uhd)\n"
        "  --device-param <args> Device parameters (default: type=b200)\n"
        "  --freq-start <Hz>     Scan start frequency (default: 863e6)\n"
        "  --freq-stop <Hz>      Scan stop frequency (default: 870e6)\n"
        "  --gain <dB>           RX gain (default: 40)\n"
        "  --l1-rate <Hz>        Wideband sample rate (default: 16e6)\n"
        "  --master-clock <Hz>   FPGA master clock rate (default: 32e6)\n"
        "  --fft-size <N>        FFT size for spectrum (default: 4096)\n"
        "  --cbor                Emit CBOR spectrum on stdout\n"
        "  --log-level <level>   DEBUG, INFO, WARNING, ERROR\n"
        "  --version             Show version and exit\n"
        "  -h, --help            Show this help\n");
}

int parse_args(int argc, char** argv, ScanConfig& cfg) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (++i >= argc) {
                std::fprintf(stderr, "missing value for %s\n", arg.c_str());
                return {};
            }
            return argv[i];
        };

        if (arg == "-h" || arg == "--help")        { print_usage(); return 2; }
        if (arg == "--version")                     { std::fprintf(stderr, "lora_scan %s\n", GIT_REV); return 2; }
        if (arg == "--device")                      { cfg.device = next(); }
        else if (arg == "--device-param")           { cfg.device_param = next(); }
        else if (arg == "--freq-start")             { cfg.freq_start = std::stod(next()); }
        else if (arg == "--freq-stop")              { cfg.freq_stop = std::stod(next()); }
        else if (arg == "--gain")                   { cfg.gain = std::stod(next()); }
        else if (arg == "--l1-rate")                { cfg.l1_rate = std::stod(next()); }
        else if (arg == "--master-clock")           { cfg.master_clock = std::stod(next()); }
        else if (arg == "--fft-size")               { cfg.fft_size = static_cast<uint32_t>(std::stoul(next())); }
        else if (arg == "--cbor")                   { cfg.cbor_out = true; }
        else if (arg == "--log-level")              { gr::lora::set_log_level(next()); }
        else {
            std::fprintf(stderr, "unknown option: %s\n", arg.c_str());
            print_usage();
            return 1;
        }
    }
    return 0;
}

void emit_spectrum_cbor(gr::lora::SpectrumState& spec) {
    namespace cb = gr::lora::cbor;
    std::vector<uint8_t> buf;
    buf.reserve(spec.fft_size * sizeof(float) + 128);

    cb::encode_map_begin(buf, 6);
    cb::kv_text(buf, "type", "scan_spectrum");
    cb::kv_text(buf, "ts", gr::lora::ts_now());
    cb::kv_float64(buf, "center_freq", spec.center_freq);
    cb::kv_float64(buf, "sample_rate", static_cast<double>(spec.sample_rate));
    cb::kv_uint(buf, "fft_size", spec.fft_size);

    {
        std::lock_guard lock(spec.result_mutex);
        std::vector<uint8_t> float_buf(spec.fft_size * sizeof(float));
        std::memcpy(float_buf.data(), spec.magnitude_db.data(), float_buf.size());
        cb::kv_bytes(buf, "bins", float_buf.data(), float_buf.size());
    }

    std::fwrite(buf.data(), 1, buf.size(), stdout);
    std::fflush(stdout);
}

} // namespace

int main(int argc, char** argv) {
    ScanConfig cfg;
    if (int rc = parse_args(argc, argv, cfg); rc != 0) {
        return rc == 2 ? 0 : 1;
    }

    auto& stop = lora_apps::install_signal_handler();
    lora_apps::apply_fpga_workaround(cfg.device, cfg.device_param);
    lora_apps::log_version_banner("lora_scan", GIT_REV);
    lora_apps::log_hardware_info("lora_scan", cfg.device, cfg.device_param);

    gr::lora::log_ts("info ", "lora_scan",
        "L1 rate=%.1f MS/s  master_clock=%.0f MHz  gain=%.0f dB  fft=%u",
        cfg.l1_rate / 1.0e6, cfg.master_clock / 1.0e6, cfg.gain, cfg.fft_size);
    gr::lora::log_ts("info ", "lora_scan",
        "scan range %.3f-%.3f MHz  center %.3f MHz",
        cfg.freq_start / 1.0e6, cfg.freq_stop / 1.0e6, cfg.center_freq() / 1.0e6);

    // stdout protection for CBOR pipe: device init may print to stdout
    int saved_stdout = -1;
    if (cfg.cbor_out) {
        std::fflush(stdout);
        saved_stdout = dup(STDOUT_FILENO);
        dup2(STDERR_FILENO, STDOUT_FILENO);
    }

    // Spectrum state: set on the concrete block ref BEFORE exchange()
    auto spectrum = std::make_shared<gr::lora::SpectrumState>();
    spectrum->fft_size    = cfg.fft_size;
    spectrum->sample_rate = static_cast<float>(cfg.l1_rate);
    spectrum->center_freq = cfg.center_freq();
    spectrum->init();

    // Build GR4 graph: SoapySimpleSource → SpectrumTapBlock → NullSink
    gr::Graph graph;

    auto& source = graph.emplaceBlock<
        gr::blocks::soapy::SoapyBlock<cf32, 1>>({
        {"device",               cfg.device},
        {"device_parameter",     cfg.device_param},
        {"sample_rate",          static_cast<float>(cfg.l1_rate)},
        {"rx_center_frequency",  gr::Tensor<double>({cfg.center_freq()})},
        {"rx_gains",             gr::Tensor<double>({cfg.gain})},
        {"master_clock_rate",    cfg.master_clock},
        {"max_chunck_size",      uint32_t{512U << 4U}},
    });

    auto& tap  = graph.emplaceBlock<gr::lora::SpectrumTapBlock>();
    tap._spectrum = spectrum;

    auto& sink = graph.emplaceBlock<gr::testing::NullSink<cf32>>();

    auto ok = [](gr::ConnectionResult r) { return r == gr::ConnectionResult::SUCCESS; };
    if (!ok(graph.connect<"out">(source).to<"in">(tap))) {
        gr::lora::log_ts("error", "lora_scan", "failed to connect source -> tap");
        return 1;
    }
    if (!ok(graph.connect<"out">(tap).to<"in">(sink))) {
        gr::lora::log_ts("error", "lora_scan", "failed to connect tap -> sink");
        return 1;
    }

    if (saved_stdout >= 0) {
        dup2(saved_stdout, STDOUT_FILENO);
        close(saved_stdout);
    }

    // singleThreadedBlocking doesn't use the pool; shrink to avoid idle thread CPU waste
    gr::thread_pool::Manager::defaultCpuPool()->setThreadBounds(1, 1);

    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::singleThreadedBlocking> sched;
    sched.timeout_inactivity_count = 1U;
    sched.watchdog_min_stall_count = 10U;
    sched.watchdog_max_warnings    = 30U;

    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        gr::lora::log_ts("error", "lora_scan", "scheduler init failed");
        return 1;
    }

    // Run scheduler in background thread
    std::atomic<bool> sched_done{false};
    std::thread sched_thread([&sched, &sched_done]() {
        auto ret = sched.runAndWait();
        if (!ret.has_value()) {
            gr::lora::log_ts("warn ", "lora_scan",
                "scheduler stopped: %s",
                std::format("{}", ret.error()).c_str());
        }
        sched_done.store(true, std::memory_order_release);
    });

    gr::lora::log_ts("info ", "lora_scan", "streaming started");

    // Main loop: compute spectrum FFT and emit CBOR
    while (!stop.load(std::memory_order_relaxed)
           && !sched_done.load(std::memory_order_relaxed)) {

        if (spectrum->compute()) {
            if (cfg.cbor_out) {
                emit_spectrum_cbor(*spectrum);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Shutdown
    gr::lora::log_ts("info ", "lora_scan", "stopping");
    if (!sched_done.load(std::memory_order_relaxed)) {
        sched.requestStop();
    }
    sched_thread.join();
    gr::lora::log_ts("info ", "lora_scan", "stopped");

    // Skip static destructors — SoapySDR/GR4 hang during normal exit
    std::quick_exit(0);
}
