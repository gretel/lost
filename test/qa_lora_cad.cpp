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
#include <gnuradio-4.0/lora/algorithm/PreambleId.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>
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

// Generate a 2-symbol window where only the FIRST symbol has signal and the
// second symbol is noise.  AND mode should NOT detect (one window below thresh),
// but OR mode should detect (one window above thresh).
std::vector<cf32> make_half_signal_window(uint8_t sf, uint8_t os_factor,
                                          float snr_db,
                                          uint32_t seed = 42U) {
    const uint32_t N       = 1U << sf;
    const uint32_t sym_len = N * os_factor;

    // First half: clean upchirp with noise
    std::vector<cf32> chirp(sym_len);
    gr::lora::build_upchirp(chirp.data(), 0U, sf, os_factor);

    float snr_linear = std::pow(10.f, snr_db / 10.f);
    float noise_amp  = 1.f / std::sqrt(snr_linear);
    std::mt19937 rng(seed);
    std::normal_distribution<float> dist(0.f, noise_amp);

    std::vector<cf32> buf(sym_len * 2U);
    for (uint32_t i = 0; i < sym_len; i++) {
        buf[i] = chirp[i] + cf32{dist(rng), dist(rng)};
    }
    // Second half: pure noise
    for (uint32_t i = sym_len; i < sym_len * 2U; i++) {
        buf[i] = {dist(rng), dist(rng)};
    }
    return buf;
}

// Generate N consecutive preamble upchirps (id=0) at given SNR.
std::vector<cf32> make_upchirp_burst(uint8_t sf, uint8_t os_factor,
                                     uint32_t nChirps, float snr_db,
                                     uint32_t seed = 42U) {
    const uint32_t N       = 1U << sf;
    const uint32_t sym_len = N * os_factor;
    const uint32_t total   = sym_len * nChirps;

    std::vector<cf32> chirp(sym_len);
    gr::lora::build_upchirp(chirp.data(), 0U, sf, os_factor);

    std::vector<cf32> signal(total);
    for (uint32_t c = 0; c < nChirps; c++) {
        std::copy_n(chirp.begin(), sym_len, signal.begin() + c * sym_len);
    }

    float snr_linear = std::pow(10.f, snr_db / 10.f);
    float noise_amp  = 1.f / std::sqrt(snr_linear);
    std::mt19937 rng(seed);
    std::normal_distribution<float> dist(0.f, noise_amp);
    for (auto& s : signal) {
        s += cf32{dist(rng), dist(rng)};
    }
    return signal;
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

    if (!graph.connect(src, "out"s, cad, "in"s)) {
        throw std::runtime_error("failed to connect source -> CAD");
    }
    if (!graph.connect(cad, "out"s, sink, "in"s)) {
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
        const float kAlpha = ChannelActivityDetector::default_alpha(kSF);

        ChannelActivityDetector block;
        block.sf        = kSF;
        block.os_factor = kOS;
        block.alpha     = kAlpha;
        block.start();

        std::vector<cf32> chirp(kSymLen);
        gr::lora::build_upchirp(chirp.data(), 0U, kSF, kOS);

        float ratio = block.compute_peak_ratio(chirp.data(),
                                               ChannelActivityDetector::RefType::UpchirpRef);
        expect(gt(ratio, kAlpha)) << "clean upchirp should give peak_ratio > alpha";
    };

    "clean downchirp gives peak_ratio >> alpha (SF8)"_test = [] {
        constexpr uint8_t  kSF     = 8;
        constexpr uint8_t  kOS     = 4;
        constexpr uint32_t kN      = 1U << kSF;
        constexpr uint32_t kSymLen = kN * kOS;
        const float kAlpha = ChannelActivityDetector::default_alpha(kSF);

        ChannelActivityDetector block;
        block.sf        = kSF;
        block.os_factor = kOS;
        block.alpha     = kAlpha;
        block.start();

        // Downchirp = conjugate of upchirp
        std::vector<cf32> chirp(kSymLen);
        gr::lora::build_upchirp(chirp.data(), 0U, kSF, kOS);
        for (auto& s : chirp) s = std::conj(s);

        float ratio = block.compute_peak_ratio(chirp.data(),
                                               ChannelActivityDetector::RefType::DownchirpRef);
        expect(gt(ratio, kAlpha)) << "clean downchirp should give peak_ratio > alpha";
    };

    "default_alpha increases with SF (Vangelista formula)"_test = [] {
        using gr::lora::ChannelActivityDetector;
        // Alpha must INCREASE with SF: more bins → higher threshold for same P_fa
        float prev = 0.f;
        for (uint32_t sfVal = 7; sfVal <= 12; sfVal++) {
            float a = ChannelActivityDetector::default_alpha(sfVal);
            expect(gt(a, prev)) << "alpha(" << sfVal << ")=" << a << " should increase";
            prev = a;
        }
        // Out-of-range returns 0
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
        expect(gt(r.peak_ratio_up, ChannelActivityDetector::default_alpha(kSF)))
            << "peak_ratio_up should exceed alpha";
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
        expect(gt(r.peak_ratio_dn, ChannelActivityDetector::default_alpha(kSF)))
            << "peak_ratio_dn should exceed alpha";
    };

    "pure noise gives peak_ratio < alpha (SF8)"_test = [] {
        constexpr uint8_t  kSF     = 8;
        constexpr uint8_t  kOS     = 4;
        constexpr uint32_t kN      = 1U << kSF;
        constexpr uint32_t kSymLen = kN * kOS;
        const float kAlpha = ChannelActivityDetector::default_alpha(kSF);

        ChannelActivityDetector block;
        block.sf        = kSF;
        block.os_factor = kOS;
        block.alpha     = kAlpha;
        block.start();

        std::mt19937 rng(42U);
        std::normal_distribution<float> dist(0.f, 1.f);
        std::vector<cf32> noise(kSymLen);
        for (auto& s : noise) s = {dist(rng), dist(rng)};

        float ratio = block.compute_peak_ratio(noise.data(),
                                               ChannelActivityDetector::RefType::UpchirpRef);
        // Noise peak_ratio is typically < 4 for N=256; just ensure it's well below alpha
        expect(lt(ratio, kAlpha)) << "noise peak_ratio should be below alpha";
    };
};

const boost::ut::suite<"CAD known upchirp"> upchirp_tests = [] {
    using namespace boost::ut;

    "High-SNR upchirp is detected"_test = [] {
        using gr::lora::ChannelActivityDetector;
        // SF8, os_factor=4, SNR=+20 dB — well above the -10 dB minimum
        constexpr uint8_t sf = 8;
        constexpr uint8_t os = 4;
        const float kAlpha = ChannelActivityDetector::default_alpha(sf);
        auto win = make_upchirp_window(sf, os, 20.f);
        auto res = run_cad(win, sf, os, kAlpha, true);

        expect(res.detected)  << "Should detect upchirp at +20 dB SNR";
        expect(res.upchirp)   << "upchirp flag should be set";
        expect(gt(res.peak_ratio_up, static_cast<double>(kAlpha))) << "peak_ratio_up should exceed alpha";
    };

    "Upchirp detection works at SF7"_test = [] {
        using gr::lora::ChannelActivityDetector;
        constexpr uint8_t sf = 7;
        constexpr uint8_t os = 4;
        const float kAlpha = ChannelActivityDetector::default_alpha(sf);
        auto win = make_upchirp_window(sf, os, 20.f);
        auto res = run_cad(win, sf, os, kAlpha, true);

        expect(res.detected) << "Should detect SF7 upchirp at +20 dB SNR";
        expect(res.upchirp)  << "upchirp flag should be set for SF7";
    };
};

const boost::ut::suite<"CAD noise floor"> noise_tests = [] {
    using namespace boost::ut;

    "Pure noise below threshold is not detected"_test = [] {
        using gr::lora::ChannelActivityDetector;
        // Use many noise trials and verify detection rate < 10%
        // (true P_fa target is 10^-3 but we test conservatively with fewer trials)
        constexpr uint8_t sf = 8;
        constexpr uint8_t os = 4;
        const float alpha = ChannelActivityDetector::default_alpha(sf);

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
        using gr::lora::ChannelActivityDetector;
        constexpr uint8_t sf = 8;
        constexpr uint8_t os = 4;
        const float kAlpha = ChannelActivityDetector::default_alpha(sf);
        auto win = make_downchirp_window(sf, os, 20.f);
        auto res = run_cad(win, sf, os, kAlpha, /*dual_chirp=*/true);

        expect(res.detected)   << "Should detect downchirp at +20 dB SNR";
        expect(res.downchirp)  << "downchirp flag should be set";
        expect(gt(res.peak_ratio_down, static_cast<double>(kAlpha))) << "peak_ratio_down should exceed alpha";
    };

    "Downchirp not flagged when dual_chirp=false"_test = [] {
        using gr::lora::ChannelActivityDetector;
        constexpr uint8_t sf = 8;
        constexpr uint8_t os = 4;
        const float kAlpha = ChannelActivityDetector::default_alpha(sf);
        auto win = make_downchirp_window(sf, os, 20.f);
        auto res = run_cad(win, sf, os, kAlpha, /*dual_chirp=*/false);

        // With dual_chirp off, downchirp path is skipped so bit 1 is always 0
        expect(!res.downchirp) << "downchirp flag should be clear when dual_chirp=false";
    };
};

const boost::ut::suite<"CAD minimum SNR"> snr_tests = [] {
    using namespace boost::ut;

    "Upchirp detected at minimum working SNR for SF8 (-10 dB)"_test = [] {
        using gr::lora::ChannelActivityDetector;
        // Minimum working SNR per Vangelista & Calvagno table: SF8 -> -10 dB.
        // We use -8 dB to account for AWGN randomness in a single-trial test.
        constexpr uint8_t sf = 8;
        constexpr uint8_t os = 4;
        const float kAlpha = ChannelActivityDetector::default_alpha(sf);
        auto win = make_upchirp_window(sf, os, -8.f, 777U);
        auto res = run_cad(win, sf, os, kAlpha, false);

        expect(res.detected) << "Should detect SF8 upchirp at -8 dB SNR";
    };
};

const boost::ut::suite<"CAD alpha formula"> alpha_tests = [] {
    using namespace boost::ut;
    using gr::lora::ChannelActivityDetector;

    "compute_alpha matches Vangelista formula"_test = [] {
        // α = sqrt(−(4/π)·ln(1−(1−p_fa)^{1/L})), L = os_factor * M
        // os_factor=1 (scan mode), P_fa=0.001
        auto alpha7  = ChannelActivityDetector::compute_alpha(7, 1, 0.001f);
        auto alpha12 = ChannelActivityDetector::compute_alpha(12, 1, 0.001f);

        // Reference values from Vangelista & Calvagno 2022
        expect(approx(alpha7, 3.87f, 0.05f))
            << "SF7 alpha should be ~3.87, got " << alpha7;
        expect(approx(alpha12, 4.40f, 0.05f))
            << "SF12 alpha should be ~4.40, got " << alpha12;

        // Verify ordering: alpha INCREASES with SF (more bins = higher threshold)
        float prev = 0.f;
        for (uint32_t sfVal = 7; sfVal <= 12; sfVal++) {
            float a = ChannelActivityDetector::compute_alpha(sfVal, 1, 0.001f);
            expect(gt(a, prev))
                << "alpha(" << sfVal << ")=" << a << " should be > alpha(" << (sfVal - 1) << ")=" << prev;
            prev = a;
        }

        // Verify os_factor influence: higher os = higher alpha
        auto alpha_os1 = ChannelActivityDetector::compute_alpha(8, 1, 0.001f);
        auto alpha_os4 = ChannelActivityDetector::compute_alpha(8, 4, 0.001f);
        expect(gt(alpha_os4, alpha_os1))
            << "alpha at os=4 (" << alpha_os4 << ") should be > alpha at os=1 (" << alpha_os1 << ")";
    };

    "default_alpha delegates to compute_alpha with os=1"_test = [] {
        // default_alpha(sf) should be equivalent to compute_alpha(sf, 1, 0.001f)
        for (uint32_t sfVal = 7; sfVal <= 12; sfVal++) {
            float from_default = ChannelActivityDetector::default_alpha(sfVal);
            float from_compute = ChannelActivityDetector::compute_alpha(sfVal, 1, 0.001f);
            expect(approx(from_default, from_compute, 0.001f))
                << "default_alpha(" << sfVal << ")=" << from_default
                << " should match compute_alpha(" << sfVal << ",1)=" << from_compute;
        }
        // Out-of-range: default_alpha returns 0
        expect(eq(ChannelActivityDetector::default_alpha(6U), 0.f));
        expect(eq(ChannelActivityDetector::default_alpha(13U), 0.f));
    };
};

const boost::ut::suite<"CAD multi-SF detection"> multi_sf_tests = [] {
    using namespace boost::ut;
    using gr::lora::ChannelActivityDetector;

    "detectMultiSf identifies correct SF from shared buffer"_test = [] {
        // Generate an SF8 upchirp in an SF12-sized buffer.
        // detectMultiSf should identify SF8 as the winning SF.
        constexpr uint8_t kTargetSf = 8;
        constexpr uint8_t kOS       = 4;
        constexpr uint32_t kSf12Win = (1U << 12U) * kOS * 2U;

        // Build SF12-sized buffer with SF8 upchirp in the prefix
        auto chirpWin = make_upchirp_window(kTargetSf, kOS, 20.f);
        std::vector<cf32> buf(kSf12Win, {0.f, 0.f});
        std::copy(chirpWin.begin(), chirpWin.end(), buf.begin());

        ChannelActivityDetector block;
        block.os_factor  = kOS;
        block.dual_chirp = true;
        block.sf         = kTargetSf;  // needed for start()
        block.start();
        block.initMultiSf();

        auto r = block.detectMultiSf(buf.data());
        expect(r.detected) << "detectMultiSf should detect the SF8 chirp";
        expect(eq(r.sf, 8U)) << "winning SF should be 8, got " << r.sf;
        expect(gt(r.best_ratio, ChannelActivityDetector::default_alpha(kTargetSf)))
            << "best_ratio should exceed SF8 alpha";
    };

    "detectMultiSf identifies SF12 chirp"_test = [] {
        constexpr uint8_t kTargetSf = 12;
        constexpr uint8_t kOS       = 4;

        auto chirpWin = make_upchirp_window(kTargetSf, kOS, 20.f);

        ChannelActivityDetector block;
        block.os_factor  = kOS;
        block.dual_chirp = true;
        block.sf         = kTargetSf;
        block.start();
        block.initMultiSf();

        auto r = block.detectMultiSf(chirpWin.data());
        expect(r.detected) << "detectMultiSf should detect SF12 chirp";
        expect(eq(r.sf, 12U)) << "winning SF should be 12, got " << r.sf;
    };

    "detectMultiSf identifies SF7 chirp"_test = [] {
        constexpr uint8_t kTargetSf = 7;
        constexpr uint8_t kOS       = 4;
        constexpr uint32_t kSf12Win = (1U << 12U) * kOS * 2U;

        auto chirpWin = make_upchirp_window(kTargetSf, kOS, 20.f);
        std::vector<cf32> buf(kSf12Win, {0.f, 0.f});
        std::copy(chirpWin.begin(), chirpWin.end(), buf.begin());

        ChannelActivityDetector block;
        block.os_factor  = kOS;
        block.dual_chirp = true;
        block.sf         = kTargetSf;
        block.start();
        block.initMultiSf();

        auto r = block.detectMultiSf(buf.data());
        expect(r.detected) << "detectMultiSf should detect SF7 chirp";
        expect(eq(r.sf, 7U)) << "winning SF should be 7, got " << r.sf;
    };

    "detectMultiSf returns sf=0 on noise"_test = [] {
        constexpr uint8_t kOS = 4;
        constexpr uint32_t kSf12Win = (1U << 12U) * kOS * 2U;

        // Pure noise
        std::mt19937 rng(99U);
        std::normal_distribution<float> dist(0.f, 1.f);
        std::vector<cf32> buf(kSf12Win);
        for (auto& s : buf) s = {dist(rng), dist(rng)};

        ChannelActivityDetector block;
        block.os_factor  = kOS;
        block.dual_chirp = true;
        block.sf         = 8;
        block.start();
        block.initMultiSf();

        auto r = block.detectMultiSf(buf.data());
        expect(!r.detected) << "detectMultiSf should not detect on noise";
        expect(eq(r.sf, 0U)) << "sf should be 0 when nothing detected";
    };

    "detectMultiSf detects downchirp"_test = [] {
        constexpr uint8_t kTargetSf = 10;
        constexpr uint8_t kOS       = 4;
        constexpr uint32_t kSf12Win = (1U << 12U) * kOS * 2U;

        auto chirpWin = make_downchirp_window(kTargetSf, kOS, 20.f);
        std::vector<cf32> buf(kSf12Win, {0.f, 0.f});
        std::copy(chirpWin.begin(), chirpWin.end(), buf.begin());

        ChannelActivityDetector block;
        block.os_factor  = kOS;
        block.dual_chirp = true;
        block.sf         = kTargetSf;
        block.start();
        block.initMultiSf();

        auto r = block.detectMultiSf(buf.data());
        expect(r.detected) << "detectMultiSf should detect SF10 downchirp";
        expect(eq(r.sf, 10U)) << "winning SF should be 10, got " << r.sf;
        expect(r.dn_detected) << "dn_detected should be true";
    };

    "detectMultiSf with dual_chirp=false detects upchirp only"_test = [] {
        constexpr uint8_t kTargetSf = 8;
        constexpr uint8_t kOS       = 4;
        constexpr uint32_t kSf12Win = (1U << 12U) * kOS * 2U;

        auto chirpWin = make_upchirp_window(kTargetSf, kOS, 20.f);
        std::vector<cf32> buf(kSf12Win, {0.f, 0.f});
        std::copy(chirpWin.begin(), chirpWin.end(), buf.begin());

        ChannelActivityDetector block;
        block.os_factor  = kOS;
        block.dual_chirp = false;
        block.sf         = kTargetSf;
        block.start();
        block.initMultiSf();

        auto r = block.detectMultiSf(buf.data());
        expect(r.detected)    << "should detect upchirp with dual_chirp=false";
        expect(eq(r.sf, 8U))  << "winning SF should be 8";
        expect(r.up_detected) << "up_detected should be true";
        expect(!r.dn_detected) << "dn_detected should be false with dual_chirp=false";
    };
};

const boost::ut::suite<"CAD AND/OR mode"> and_or_tests = [] {
    using namespace boost::ut;
    using gr::lora::ChannelActivityDetector;

    "AND mode rejects signal in only one window"_test = [] {
        constexpr uint8_t kSF = 8;
        constexpr uint8_t kOS = 4;
        const float kAlpha = ChannelActivityDetector::default_alpha(kSF);

        ChannelActivityDetector block;
        block.sf        = kSF;
        block.os_factor = kOS;
        block.alpha     = kAlpha;
        block.dual_chirp = false;
        block.start();

        auto win = make_half_signal_window(kSF, kOS, 20.f);
        auto r   = block.detect(win.data(), /*require_both=*/true);

        expect(!r.detected) << "AND mode should NOT detect when only one window has signal";
        expect(!r.up_detected) << "up_detected should be false in AND mode";
    };

    "OR mode detects signal in only one window"_test = [] {
        constexpr uint8_t kSF = 8;
        constexpr uint8_t kOS = 4;
        const float kAlpha = ChannelActivityDetector::default_alpha(kSF);

        ChannelActivityDetector block;
        block.sf        = kSF;
        block.os_factor = kOS;
        block.alpha     = kAlpha;
        block.dual_chirp = false;
        block.start();

        auto win = make_half_signal_window(kSF, kOS, 20.f);
        auto r   = block.detect(win.data(), /*require_both=*/false);

        expect(r.detected)    << "OR mode should detect when one window has signal";
        expect(r.up_detected) << "up_detected should be true in OR mode";
    };

    "AND mode still detects when both windows have signal"_test = [] {
        constexpr uint8_t kSF = 8;
        constexpr uint8_t kOS = 4;
        const float kAlpha = ChannelActivityDetector::default_alpha(kSF);

        ChannelActivityDetector block;
        block.sf        = kSF;
        block.os_factor = kOS;
        block.alpha     = kAlpha;
        block.dual_chirp = false;
        block.start();

        auto win = make_upchirp_window(kSF, kOS, 20.f);
        auto r   = block.detect(win.data(), /*require_both=*/true);

        expect(r.detected) << "AND mode should detect when both windows have signal";
    };

    "OR mode does not fire on noise"_test = [] {
        constexpr uint8_t kSF = 8;
        constexpr uint8_t kOS = 4;
        const float kAlpha = ChannelActivityDetector::default_alpha(kSF);

        ChannelActivityDetector block;
        block.sf        = kSF;
        block.os_factor = kOS;
        block.alpha     = kAlpha;
        block.dual_chirp = false;
        block.start();

        auto win = make_noise_window(kSF, kOS, 1.f, 77U);
        auto r   = block.detect(win.data(), /*require_both=*/false);

        expect(!r.detected) << "OR mode should not fire on noise";
    };

    "detectMultiSf OR mode detects half-signal"_test = [] {
        constexpr uint8_t kSF = 8;
        constexpr uint8_t kOS = 4;
        constexpr uint32_t kSf12Win = (1U << 12U) * kOS * 2U;

        ChannelActivityDetector block;
        block.os_factor  = kOS;
        block.dual_chirp = false;
        block.sf         = kSF;
        block.start();
        block.initMultiSf();

        auto halfWin = make_half_signal_window(kSF, kOS, 20.f);
        std::vector<cf32> buf(kSf12Win, {0.f, 0.f});
        std::copy(halfWin.begin(), halfWin.end(), buf.begin());

        auto andResult = block.detectMultiSf(buf.data(), /*require_both=*/true);
        auto orResult  = block.detectMultiSf(buf.data(), /*require_both=*/false);

        expect(!andResult.detected) << "detectMultiSf AND should reject half-signal";
        expect(orResult.detected)   << "detectMultiSf OR should detect half-signal";
        expect(eq(orResult.sf, 8U)) << "OR mode should identify SF8";
    };
};

const boost::ut::suite<"CAD chirp-combined"> combined_tests = [] {
    using namespace boost::ut;
    using gr::lora::ChannelActivityDetector;

    "combined N=8 improves ratio over standard N=2"_test = [] {
        constexpr uint8_t  kSF = 8;
        constexpr uint8_t  kOS = 4;
        constexpr uint32_t kN2 = 2;
        constexpr uint32_t kN8 = 8;
        constexpr float    kSnr = 0.f;  // moderate SNR where improvement is measurable

        ChannelActivityDetector block;
        block.sf        = kSF;
        block.os_factor = kOS;
        block.alpha     = ChannelActivityDetector::default_alpha(kSF);
        block.dual_chirp = false;
        block.start();

        // N=2 standard 2-window detection
        auto win2 = make_upchirp_burst(kSF, kOS, kN2, kSnr);
        auto r2   = block.detect(win2.data());

        // N=8 combined detection
        auto win8 = make_upchirp_burst(kSF, kOS, kN8, kSnr);
        auto r8   = block.detectCombined(win8.data(), kN8);

        expect(gt(r8.peak_ratio_up, r2.peak_ratio_up))
            << "N=8 combined ratio (" << r8.peak_ratio_up
            << ") should exceed N=2 standard ratio (" << r2.peak_ratio_up << ")";
    };

    "combined detects at SNR where standard fails"_test = [] {
        constexpr uint8_t  kSF  = 8;
        constexpr uint8_t  kOS  = 4;
        constexpr float    kSnr = -15.f;  // well below standard 2-window threshold
        constexpr uint32_t kN8  = 8;

        ChannelActivityDetector block;
        block.sf        = kSF;
        block.os_factor = kOS;
        block.alpha     = ChannelActivityDetector::default_alpha(kSF);
        block.dual_chirp = false;
        block.start();

        // Standard 2-window: should fail at -15 dB (SF8 min is -10 dB)
        auto win2 = make_upchirp_burst(kSF, kOS, 2, kSnr, 333U);
        auto r2   = block.detect(win2.data());

        // Combined N=8: coherent integration gives +6 dB gain
        auto win8 = make_upchirp_burst(kSF, kOS, kN8, kSnr, 333U);
        auto r8   = block.detectCombined(win8.data(), kN8);

        expect(!r2.detected)
            << "standard 2-window should NOT detect at -15 dB SNR (ratio=" << r2.peak_ratio_up << ")";
        expect(r8.detected)
            << "combined N=8 SHOULD detect at -15 dB SNR (ratio=" << r8.peak_ratio_up << ")";
    };

    "combined on noise does not fire"_test = [] {
        constexpr uint8_t  kSF  = 8;
        constexpr uint8_t  kOS  = 4;
        constexpr uint32_t kN   = 8;
        const uint32_t symLen = (1U << kSF) * kOS;
        const uint32_t total  = symLen * kN;

        ChannelActivityDetector block;
        block.sf        = kSF;
        block.os_factor = kOS;
        block.alpha     = ChannelActivityDetector::default_alpha(kSF);
        block.dual_chirp = false;
        block.start();

        std::mt19937 rng(555U);
        std::normal_distribution<float> dist(0.f, 1.f);
        std::vector<cf32> noise(total);
        for (auto& s : noise) s = {dist(rng), dist(rng)};

        auto r = block.detectCombined(noise.data(), kN);
        expect(!r.detected)
            << "combined should not fire on noise (ratio=" << r.peak_ratio_up << ")";
    };

    "combined ratio scales with N (coherent integration)"_test = [] {
        constexpr uint8_t kSF  = 8;
        constexpr uint8_t kOS  = 4;
        constexpr float   kSnr = 5.f;

        ChannelActivityDetector block;
        block.sf        = kSF;
        block.os_factor = kOS;
        block.alpha     = ChannelActivityDetector::default_alpha(kSF);
        block.dual_chirp = false;
        block.start();

        auto win2 = make_upchirp_burst(kSF, kOS, 2, kSnr, 100U);
        auto r2   = block.detectCombined(win2.data(), 2);

        auto win4 = make_upchirp_burst(kSF, kOS, 4, kSnr, 100U);
        auto r4   = block.detectCombined(win4.data(), 4);

        auto win8 = make_upchirp_burst(kSF, kOS, 8, kSnr, 100U);
        auto r8   = block.detectCombined(win8.data(), 8);

        // Ratio should strictly increase: N=2 < N=4 < N=8
        expect(gt(r4.peak_ratio_up, r2.peak_ratio_up))
            << "N=4 ratio (" << r4.peak_ratio_up << ") should exceed N=2 (" << r2.peak_ratio_up << ")";
        expect(gt(r8.peak_ratio_up, r4.peak_ratio_up))
            << "N=8 ratio (" << r8.peak_ratio_up << ") should exceed N=4 (" << r4.peak_ratio_up << ")";
    };
};

// === Preamble characterization tests =========================================

const boost::ut::suite<"Preamble identification"> preamble_id_tests = [] {
    using namespace boost::ut;

    "SF8 clean signal: correct SF, sync word 0x12, zero CFO"_test = [] {
        constexpr uint8_t  kSF   = 8;
        constexpr uint8_t  kCR   = 4;
        constexpr uint8_t  kOS   = 4;
        constexpr uint16_t kSync = 0x12;
        constexpr uint16_t kPre  = 8;
        constexpr uint32_t kBW   = 62500;

        const std::vector<uint8_t> payload = {0x12, 0x00, 0xBB, 0x59};
        auto iq = gr::lora::generate_frame_iq(payload, kSF, kCR, kOS, kSync, kPre,
                                               /*has_crc=*/true, /*zero_pad=*/0,
                                               /*ldro=*/2, kBW);

        auto info = gr::lora::characterize_preamble(
            iq.data(), iq.size(), kSF, kBW, kOS, 868e6, kPre);

        expect(info.valid) << "preamble should be characterized";
        expect(eq(info.sf, kSF)) << "SF";
        expect(eq(info.bw, kBW)) << "BW";
        expect(eq(info.sync_word, kSync)) << "sync_word";
        expect(std::abs(info.cfo_frac) < 0.5f) << "cfo_frac near zero: " << info.cfo_frac;
        expect(std::abs(info.cfo_int) <= 1) << "cfo_int near zero: " << info.cfo_int;
        expect(info.snr_db > 10.f) << "SNR should be high for clean signal: " << info.snr_db;
        expect(info.preamble_len >= 5U) << "preamble_len: " << info.preamble_len;
    };

    "SF12 clean signal: correct identification"_test = [] {
        constexpr uint8_t  kSF   = 12;
        constexpr uint8_t  kCR   = 4;
        constexpr uint8_t  kOS   = 4;
        constexpr uint16_t kSync = 0x12;
        constexpr uint16_t kPre  = 8;
        constexpr uint32_t kBW   = 62500;

        const std::vector<uint8_t> payload = {0xAA, 0xBB};
        auto iq = gr::lora::generate_frame_iq(payload, kSF, kCR, kOS, kSync, kPre,
                                               /*has_crc=*/true, /*zero_pad=*/0,
                                               /*ldro=*/2, kBW);

        auto info = gr::lora::characterize_preamble(
            iq.data(), iq.size(), kSF, kBW, kOS, 868e6, kPre);

        expect(info.valid) << "SF12 preamble should be characterized";
        expect(eq(info.sf, kSF)) << "SF";
        expect(eq(info.sync_word, kSync)) << "sync_word";
        expect(info.snr_db > 10.f) << "SNR: " << info.snr_db;
    };

    "SF7 clean signal: correct identification"_test = [] {
        constexpr uint8_t  kSF   = 7;
        constexpr uint8_t  kCR   = 4;
        constexpr uint8_t  kOS   = 4;
        constexpr uint16_t kSync = 0x34;  // LoRaWAN sync word
        constexpr uint16_t kPre  = 8;
        constexpr uint32_t kBW   = 125000;

        const std::vector<uint8_t> payload = {0x01, 0x02, 0x03};
        auto iq = gr::lora::generate_frame_iq(payload, kSF, kCR, kOS, kSync, kPre,
                                               /*has_crc=*/true, /*zero_pad=*/0,
                                               /*ldro=*/2, kBW);

        auto info = gr::lora::characterize_preamble(
            iq.data(), iq.size(), kSF, kBW, kOS, 868e6, kPre);

        expect(info.valid) << "SF7 preamble should be characterized";
        expect(eq(info.sf, kSF)) << "SF";
        expect(eq(info.sync_word, kSync)) << "sync_word should be 0x34";
    };

    "noise-only buffer: valid=false"_test = [] {
        constexpr uint8_t  kSF = 8;
        constexpr uint32_t kBW = 62500;
        constexpr uint8_t  kOS = 4;
        const uint32_t kN   = 1U << kSF;
        const uint32_t kSps = kN * kOS;

        // 15 symbols of pure noise
        std::mt19937 rng(99);
        std::normal_distribution<float> dist(0.f, 0.01f);
        std::vector<cf32> noise(15 * kSps);
        for (auto& s : noise) {
            s = cf32(dist(rng), dist(rng));
        }

        auto info = gr::lora::characterize_preamble(
            noise.data(), noise.size(), kSF, kBW, kOS, 868e6, 8);

        expect(!info.valid) << "noise should not produce valid preamble";
    };

    "wrong SF: valid=false"_test = [] {
        constexpr uint8_t  kOS   = 4;
        constexpr uint16_t kSync = 0x12;
        constexpr uint16_t kPre  = 8;
        constexpr uint32_t kBW   = 62500;

        // Generate SF7 signal
        const std::vector<uint8_t> payload = {0x01};
        auto iq = gr::lora::generate_frame_iq(payload, /*sf=*/7, /*cr=*/4, kOS, kSync, kPre,
                                               true, 0, 2, kBW);

        // Try to characterize as SF8 — should fail
        auto info = gr::lora::characterize_preamble(
            iq.data(), iq.size(), /*sf=*/8, kBW, kOS, 868e6, kPre);

        expect(!info.valid) << "wrong SF should not produce valid preamble";
    };

    "SF8 with CFO +200 Hz"_test = [] {
        constexpr uint8_t  kSF   = 8;
        constexpr uint8_t  kOS   = 4;
        constexpr uint16_t kSync = 0x12;
        constexpr uint16_t kPre  = 8;
        constexpr uint32_t kBW   = 62500;

        const std::vector<uint8_t> payload = {0x12, 0x00, 0xBB, 0x59};
        auto iq = gr::lora::generate_frame_iq(payload, kSF, 4, kOS, kSync, kPre,
                                               true, 0, 2, kBW);

        // Apply +200 Hz CFO
        const double cfo_hz = 200.0;
        const double sample_rate = static_cast<double>(kBW) * kOS;
        for (std::size_t n = 0; n < iq.size(); n++) {
            double phase = 2.0 * std::numbers::pi * cfo_hz
                         / sample_rate * static_cast<double>(n);
            iq[n] *= cf32(static_cast<float>(std::cos(phase)),
                          static_cast<float>(std::sin(phase)));
        }

        auto info = gr::lora::characterize_preamble(
            iq.data(), iq.size(), kSF, kBW, kOS, 868e6, kPre);

        expect(info.valid) << "should characterize with CFO";
        expect(eq(info.sync_word, kSync)) << "sync_word despite CFO";
        // CFO = 200 Hz at BW=62500, bin_width=62500/256=244 Hz → ~0.82 bins
        // Should appear as cfo_frac ≈ 0.82 or split between cfo_int and cfo_frac
        float total_cfo_bins = static_cast<float>(cfo_hz * (1U << kSF) / kBW);
        float measured_cfo = info.cfo_frac + static_cast<float>(info.cfo_int);
        expect(std::abs(measured_cfo - total_cfo_bins) < 1.0f)
            << "CFO estimate: expected ~" << total_cfo_bins << " bins, got " << measured_cfo;
    };

    "SF8 with AWGN +10 dB"_test = [] {
        constexpr uint8_t  kSF   = 8;
        constexpr uint8_t  kOS   = 4;
        constexpr uint16_t kSync = 0x12;
        constexpr uint16_t kPre  = 8;
        constexpr uint32_t kBW   = 62500;

        const std::vector<uint8_t> payload = {0x12, 0x00, 0xBB, 0x59};
        auto iq = gr::lora::generate_frame_iq(payload, kSF, 4, kOS, kSync, kPre,
                                               true, 0, 2, kBW);

        // Add AWGN at +10 dB SNR
        float signal_power = 0.f;
        for (const auto& s : iq) {
            signal_power += s.real() * s.real() + s.imag() * s.imag();
        }
        signal_power /= static_cast<float>(iq.size());
        float noise_power = signal_power * 0.1f;  // 10 dB below signal

        std::mt19937 rng(42);
        std::normal_distribution<float> dist(0.f, std::sqrt(noise_power / 2.f));
        for (auto& s : iq) {
            s += cf32(dist(rng), dist(rng));
        }

        auto info = gr::lora::characterize_preamble(
            iq.data(), iq.size(), kSF, kBW, kOS, 868e6, kPre);

        expect(info.valid) << "should characterize at +10 dB SNR";
        expect(eq(info.sync_word, kSync)) << "sync_word at +10 dB";
        expect(info.snr_db > 5.f) << "SNR estimate: " << info.snr_db;
    };

    "buffer too small: valid=false"_test = [] {
        // Only 3 symbols — way too short for preamble detection
        constexpr uint32_t kN = 256;
        constexpr uint32_t kSps = kN * 4;
        std::vector<cf32> short_buf(3 * kSps, cf32(0.f, 0.f));

        auto info = gr::lora::characterize_preamble(
            short_buf.data(), short_buf.size(), 8, 62500, 4, 868e6, 8);

        expect(!info.valid) << "too-short buffer should fail";
    };
};

int main() { /* boost::ut auto-runs all suites */ }
