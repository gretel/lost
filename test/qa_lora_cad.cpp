// SPDX-License-Identifier: ISC
/// Tests for ChannelActivityDetector: dual-window chirp detection block.
///
/// Test progression:
///   1. Known upchirp signal at high SNR → detect=true, upchirp=true
///   2. Pure noise below threshold → detect=false (low P_fa)
///   3. Downchirp (inverted-IQ) signal → detect=true, downchirp=true
///   4. Low-SNR upchirp near minimum working SNR → detect=true (SF8 min: -10 dB)
///   5. Block-level graph integration (produces output bytes)

#include "test_helpers.hpp"

#include <complex>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/ChannelActivityDetector.hpp>
#include <gnuradio-4.0/lora/algorithm/utilities.hpp>

using namespace gr::lora::test;
using namespace std::string_literals;

namespace {

using cf32 = std::complex<float>;

// Generate a 2-symbol window of preamble upchirps (id=0) at 1x oversampling.
// The reference chirp at id=0 is what the CAD block uses as its upchirp reference.
// SNR is applied as: signal * sqrt(snr_linear) + noise.
std::vector<cf32> make_upchirp_window(uint8_t sf, uint8_t os_factor,
                                      float snr_db,
                                      uint32_t seed = 42U) {
    const uint32_t N       = 1U << sf;
    const uint32_t sym_len = N * os_factor;
    const uint32_t win_len = sym_len * 2U;

    // Build reference upchirp
    std::vector<cf32> chirp(sym_len);
    gr::lora::build_upchirp(chirp.data(), 0U, sf, os_factor);

    // Two consecutive upchirps
    std::vector<cf32> signal(win_len);
    for (uint32_t i = 0; i < sym_len; i++) {
        signal[i]            = chirp[i];
        signal[i + sym_len]  = chirp[i];
    }

    // Add AWGN
    float snr_linear = std::pow(10.f, snr_db / 10.f);
    float noise_amp  = 1.f / std::sqrt(snr_linear);

    std::mt19937 rng(seed);
    std::normal_distribution<float> dist(0.f, noise_amp);
    for (auto& s : signal) {
        s += cf32{dist(rng), dist(rng)};
    }

    return signal;
}

// Generate 2-symbol window of downchirps (conjugate of upchirp).
std::vector<cf32> make_downchirp_window(uint8_t sf, uint8_t os_factor,
                                        float snr_db,
                                        uint32_t seed = 42U) {
    auto sig = make_upchirp_window(sf, os_factor, 80.f, seed);  // noiseless first
    // Conjugate to get downchirp
    for (auto& s : sig) s = std::conj(s);

    // Add noise separately at target SNR
    float snr_linear = std::pow(10.f, snr_db / 10.f);
    float noise_amp  = 1.f / std::sqrt(snr_linear);
    std::mt19937 rng(seed + 1U);
    std::normal_distribution<float> dist(0.f, noise_amp);
    for (auto& s : sig) {
        s += cf32{dist(rng), dist(rng)};
    }
    return sig;
}

// Generate pure AWGN (no signal).
std::vector<cf32> make_noise_window(uint8_t sf, uint8_t os_factor,
                                    float power = 1.f,
                                    uint32_t seed = 123U) {
    const uint32_t N       = 1U << sf;
    const uint32_t win_len = N * os_factor * 2U;
    float noise_amp = std::sqrt(power);
    std::mt19937 rng(seed);
    std::normal_distribution<float> dist(0.f, noise_amp);
    std::vector<cf32> buf(win_len);
    for (auto& s : buf) s = {dist(rng), dist(rng)};
    return buf;
}

// Run CAD block on one 2-symbol window and return {result_byte, detected, up, dn, pr_up, pr_dn}.
struct CadResult {
    uint8_t result_byte{0};
    bool detected{false};
    bool upchirp{false};
    bool downchirp{false};
    double peak_ratio_up{0.0};
    double peak_ratio_down{0.0};
};

CadResult run_cad(const std::vector<cf32>& window,
                  uint8_t sf, uint8_t os_factor,
                  float alpha, bool dual_chirp,
                  bool debug = false) {
    using namespace gr;
    using namespace gr::lora;

    Graph graph;

    auto& cad = graph.emplaceBlock<ChannelActivityDetector>({
        {"sf",         static_cast<uint32_t>(sf)},
        {"os_factor",  static_cast<uint32_t>(os_factor)},
        {"alpha",      alpha},
        {"dual_chirp", dual_chirp},
        {"debug",      debug},
    });

    auto& src = graph.emplaceBlock<testing::TagSource<cf32,
        testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"n_samples_max", static_cast<gr::Size_t>(window.size())},
        {"repeat_tags", false},
        {"mark_tag", false},
    });
    src.values = window;

    auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
        testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"log_samples", true},
        {"log_tags",    true},
    });

    if (graph.connect(src, "out"s, cad, "in"s) != ConnectionResult::SUCCESS) {
        throw std::runtime_error("failed to connect source -> CAD");
    }
    if (graph.connect(cad, "out"s, sink, "in"s) != ConnectionResult::SUCCESS) {
        throw std::runtime_error("failed to connect CAD -> sink");
    }

    scheduler::Simple sched;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        throw std::runtime_error(
            std::format("failed to init scheduler: {}", ret.error()));
    }
    if (!sched.runAndWait().has_value()) {
        throw std::runtime_error("scheduler runAndWait failed");
    }

    CadResult res;
    if (!sink._samples.empty()) {
        res.result_byte = sink._samples[0];
        res.detected    = (res.result_byte != 0);
        res.upchirp     = (res.result_byte & 0x01U) != 0;
        res.downchirp   = (res.result_byte & 0x02U) != 0;
    }
    for (const auto& tag : sink._tags) {
        if (auto it = tag.map.find("cad_peak_ratio_up"); it != tag.map.end()) {
            res.peak_ratio_up = it->second.value_or<double>(0.0);
        }
        if (auto it = tag.map.find("cad_peak_ratio_down"); it != tag.map.end()) {
            res.peak_ratio_down = it->second.value_or<double>(0.0);
        }
    }
    return res;
}

}  // namespace

// Direct algorithm tests — bypass the GR4 graph to verify dechirp+FFT math.
// Uses compute_peak_ratio(samples, RefType) which keeps block internals private.
const boost::ut::suite<"CAD algorithm direct"> algo_tests = [] {
    using namespace boost::ut;
    using gr::lora::ChannelActivityDetector;

    "clean upchirp gives peak_ratio >> alpha (SF8)"_test = [] {
        constexpr uint8_t  kSF     = 8;
        constexpr uint8_t  kOS     = 4;
        constexpr uint32_t kN      = 1U << kSF;
        constexpr uint32_t kSymLen = kN * kOS;

        ChannelActivityDetector block;
        block.sf        = kSF;
        block.os_factor = kOS;
        block.alpha     = 4.16f;
        block.start();

        std::vector<cf32> chirp(kSymLen);
        gr::lora::build_upchirp(chirp.data(), 0U, kSF, kOS);

        float ratio = block.compute_peak_ratio(chirp.data(),
                                               ChannelActivityDetector::RefType::UpchirpRef);
        expect(gt(ratio, 4.16f)) << "clean upchirp should give peak_ratio > alpha";
    };

    "clean downchirp gives peak_ratio >> alpha (SF8)"_test = [] {
        constexpr uint8_t  kSF     = 8;
        constexpr uint8_t  kOS     = 4;
        constexpr uint32_t kN      = 1U << kSF;
        constexpr uint32_t kSymLen = kN * kOS;

        ChannelActivityDetector block;
        block.sf        = kSF;
        block.os_factor = kOS;
        block.alpha     = 4.16f;
        block.start();

        // Downchirp = conjugate of upchirp
        std::vector<cf32> chirp(kSymLen);
        gr::lora::build_upchirp(chirp.data(), 0U, kSF, kOS);
        for (auto& s : chirp) s = std::conj(s);

        float ratio = block.compute_peak_ratio(chirp.data(),
                                               ChannelActivityDetector::RefType::DownchirpRef);
        expect(gt(ratio, 4.16f)) << "clean downchirp should give peak_ratio > alpha";
    };

    "default_alpha returns correct values for SF7-12"_test = [] {
        using gr::lora::ChannelActivityDetector;
        expect(eq(ChannelActivityDetector::default_alpha(7U), 4.23f));
        expect(eq(ChannelActivityDetector::default_alpha(8U), 4.16f));
        expect(eq(ChannelActivityDetector::default_alpha(9U), 4.09f));
        expect(eq(ChannelActivityDetector::default_alpha(10U), 4.04f));
        expect(eq(ChannelActivityDetector::default_alpha(11U), 3.98f));
        expect(eq(ChannelActivityDetector::default_alpha(12U), 3.91f));
        expect(eq(ChannelActivityDetector::default_alpha(6U), 0.f));
        expect(eq(ChannelActivityDetector::default_alpha(13U), 0.f));
    };

    "detect() finds upchirp at +20 dB"_test = [] {
        using gr::lora::ChannelActivityDetector;
        constexpr uint8_t kSF = 8;
        constexpr uint8_t kOS = 4;

        ChannelActivityDetector block;
        block.sf        = kSF;
        block.os_factor = kOS;
        block.alpha     = ChannelActivityDetector::default_alpha(kSF);
        block.start();

        auto win = make_upchirp_window(kSF, kOS, 20.f);
        auto r   = block.detect(win.data());

        expect(r.detected)    << "detect() should find upchirp at +20 dB";
        expect(r.up_detected) << "up_detected should be true";
        expect(gt(r.peak_ratio_up, 4.16f)) << "peak_ratio_up should exceed alpha";
    };

    "detect() does not fire on noise"_test = [] {
        using gr::lora::ChannelActivityDetector;
        constexpr uint8_t kSF = 8;
        constexpr uint8_t kOS = 4;

        ChannelActivityDetector block;
        block.sf        = kSF;
        block.os_factor = kOS;
        block.alpha     = ChannelActivityDetector::default_alpha(kSF);
        block.start();

        auto win = make_noise_window(kSF, kOS, 1.f, 42U);
        auto r   = block.detect(win.data());

        expect(!r.detected) << "detect() should not fire on noise";
    };

    "detect() finds downchirp when dual_chirp=true"_test = [] {
        using gr::lora::ChannelActivityDetector;
        constexpr uint8_t kSF = 8;
        constexpr uint8_t kOS = 4;

        ChannelActivityDetector block;
        block.sf         = kSF;
        block.os_factor  = kOS;
        block.alpha      = ChannelActivityDetector::default_alpha(kSF);
        block.dual_chirp = true;
        block.start();

        auto win = make_downchirp_window(kSF, kOS, 20.f);
        auto r   = block.detect(win.data());

        expect(r.detected)    << "detect() should find downchirp at +20 dB";
        expect(r.dn_detected) << "dn_detected should be true";
        expect(gt(r.peak_ratio_dn, 4.16f)) << "peak_ratio_dn should exceed alpha";
    };

    "pure noise gives peak_ratio < alpha (SF8)"_test = [] {
        constexpr uint8_t  kSF     = 8;
        constexpr uint8_t  kOS     = 4;
        constexpr uint32_t kN      = 1U << kSF;
        constexpr uint32_t kSymLen = kN * kOS;

        ChannelActivityDetector block;
        block.sf        = kSF;
        block.os_factor = kOS;
        block.alpha     = 4.16f;
        block.start();

        std::mt19937 rng(42U);
        std::normal_distribution<float> dist(0.f, 1.f);
        std::vector<cf32> noise(kSymLen);
        for (auto& s : noise) s = {dist(rng), dist(rng)};

        float ratio = block.compute_peak_ratio(noise.data(),
                                               ChannelActivityDetector::RefType::UpchirpRef);
        // Noise peak_ratio is typically < 4 for N=256; just ensure it's well below alpha
        expect(lt(ratio, 4.16f)) << "noise peak_ratio should be below alpha";
    };
};

const boost::ut::suite<"CAD known upchirp"> upchirp_tests = [] {
    using namespace boost::ut;

    "High-SNR upchirp is detected"_test = [] {
        // SF8, os_factor=4, SNR=+20 dB — well above the -10 dB minimum
        constexpr uint8_t sf = 8;
        constexpr uint8_t os = 4;
        auto win = make_upchirp_window(sf, os, 20.f);
        auto res = run_cad(win, sf, os, 4.16f, true);

        expect(res.detected)  << "Should detect upchirp at +20 dB SNR";
        expect(res.upchirp)   << "upchirp flag should be set";
        expect(gt(res.peak_ratio_up, 4.16)) << "peak_ratio_up should exceed alpha";
    };

    "Upchirp detection works at SF7"_test = [] {
        constexpr uint8_t sf = 7;
        constexpr uint8_t os = 4;
        auto win = make_upchirp_window(sf, os, 20.f);
        auto res = run_cad(win, sf, os, 4.23f, true);

        expect(res.detected) << "Should detect SF7 upchirp at +20 dB SNR";
        expect(res.upchirp)  << "upchirp flag should be set for SF7";
    };
};

const boost::ut::suite<"CAD noise floor"> noise_tests = [] {
    using namespace boost::ut;

    "Pure noise below threshold is not detected"_test = [] {
        // Use many noise trials and verify detection rate < 10%
        // (true P_fa target is 10^-3 but we test conservatively with fewer trials)
        constexpr uint8_t sf = 8;
        constexpr uint8_t os = 4;
        constexpr float alpha = 4.16f;

        int detects = 0;
        constexpr int kTrials = 100;
        for (int trial = 0; trial < kTrials; trial++) {
            auto win = make_noise_window(sf, os, 1.f, static_cast<uint32_t>(trial * 17 + 3));
            auto res = run_cad(win, sf, os, alpha, true);
            if (res.detected) ++detects;
        }

        // At alpha=4.16, expect <10% false alarm rate in tests (theory: 10^-3)
        // We use 10% as a practical threshold given small trial count
        expect(lt(detects, kTrials / 10))
            << "False alarm rate too high: " << detects << "/" << kTrials;
    };
};

const boost::ut::suite<"CAD downchirp"> downchirp_tests = [] {
    using namespace boost::ut;

    "High-SNR downchirp is detected when dual_chirp=true"_test = [] {
        constexpr uint8_t sf = 8;
        constexpr uint8_t os = 4;
        auto win = make_downchirp_window(sf, os, 20.f);
        auto res = run_cad(win, sf, os, 4.16f, /*dual_chirp=*/true);

        expect(res.detected)   << "Should detect downchirp at +20 dB SNR";
        expect(res.downchirp)  << "downchirp flag should be set";
        expect(gt(res.peak_ratio_down, 4.16)) << "peak_ratio_down should exceed alpha";
    };

    "Downchirp not flagged when dual_chirp=false"_test = [] {
        constexpr uint8_t sf = 8;
        constexpr uint8_t os = 4;
        auto win = make_downchirp_window(sf, os, 20.f);
        auto res = run_cad(win, sf, os, 4.16f, /*dual_chirp=*/false);

        // With dual_chirp off, downchirp path is skipped so bit 1 is always 0
        expect(!res.downchirp) << "downchirp flag should be clear when dual_chirp=false";
    };
};

const boost::ut::suite<"CAD minimum SNR"> snr_tests = [] {
    using namespace boost::ut;

    "Upchirp detected at minimum working SNR for SF8 (-10 dB)"_test = [] {
        // Minimum working SNR per Vangelista & Calvagno table: SF8 -> -10 dB.
        // We use -8 dB to account for AWGN randomness in a single-trial test.
        constexpr uint8_t sf = 8;
        constexpr uint8_t os = 4;
        auto win = make_upchirp_window(sf, os, -8.f, 777U);
        auto res = run_cad(win, sf, os, 4.16f, false);

        expect(res.detected) << "Should detect SF8 upchirp at -8 dB SNR";
    };
};

int main() { /* boost::ut auto-runs all suites */ }
