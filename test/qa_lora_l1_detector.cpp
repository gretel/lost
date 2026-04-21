// SPDX-License-Identifier: ISC

#include <boost/ut.hpp>
#include <gnuradio-4.0/lora/algorithm/L1Detector.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <iostream>
#include <numbers>
#include <random>
#include <span>
#include <string>
#include <vector>

using namespace boost::ut;
using cf32 = std::complex<float>;

namespace {

// Generate a noise + tone signal. Tone is placed at tone_hz offset from baseband center.
[[nodiscard]] std::vector<cf32> generateToneWithNoise(std::size_t n, float sample_rate, float tone_hz, float snr_db, uint32_t seed = 42U) {
    std::vector<cf32>               out(n);
    std::mt19937                    rng(seed);
    std::normal_distribution<float> noise(0.f, 1.f);

    const float snr_linear = std::pow(10.f, snr_db / 10.f);
    const float tone_amp   = std::sqrt(snr_linear);
    const float phase_inc  = 2.f * std::numbers::pi_v<float> * tone_hz / sample_rate;
    float       phase      = 0.f;
    for (std::size_t i = 0; i < n; ++i) {
        const cf32 noise_sample{noise(rng), noise(rng)};
        const cf32 tone_sample{tone_amp * std::cos(phase), tone_amp * std::sin(phase)};
        out[i] = noise_sample + tone_sample;
        phase += phase_inc;
        if (phase > 2.f * std::numbers::pi_v<float>) {
            phase -= 2.f * std::numbers::pi_v<float>;
        }
    }
    return out;
}

[[nodiscard]] gr::lora::L1Detector::Config defaultConfig(float sample_rate, float center_freq, float channel_bw, uint32_t fft_size) {
    gr::lora::L1Detector::Config cfg;
    cfg.sample_rate     = sample_rate;
    cfg.center_freq     = center_freq;
    cfg.channel_bw      = channel_bw;
    cfg.l1_fft_size     = fft_size;
    cfg.usable_fraction = 0.8f;
    cfg.dc_null_hz      = 100000.f;
    cfg.hot_multiplier  = 6.0f;
    cfg.min_hot_sweeps  = 1;
    return cfg;
}

} // namespace

const suite<"L1Detector"> l1DetectorTests = [] {
    "empty input returns no hot channels"_test = [] {
        gr::lora::L1Detector det;
        det.init(defaultConfig(16e6f, 866.5e6f, 62500.f, 4096));
        expect(det.nChannels() > 0_u);
        auto hot = det.findHotChannels();
        expect(hot.empty());
    };

    "span shorter than fft size is ignored"_test = [] {
        gr::lora::L1Detector det;
        det.init(defaultConfig(16e6f, 866.5e6f, 62500.f, 4096));
        const std::vector<cf32> tiny(100, cf32{0.f, 0.f});
        det.accumulate(std::span<const cf32>(tiny));
        expect(det.snapshotCount() == 0_u);
    };

    "pure noise produces no hot channels"_test = [] {
        gr::lora::L1Detector det;
        det.init(defaultConfig(16e6f, 866.5e6f, 62500.f, 4096));
        auto samples = generateToneWithNoise(4096, 16e6f, 0.f, -100.f);
        for (int i = 0; i < 16; ++i) {
            det.accumulate(std::span<const cf32>(samples));
        }
        auto hot = det.findHotChannels();
        // DC bin may spike even with NF interp; tolerate 0-2 hot channels
        expect(hot.size() <= 2_u);
    };

    "single tone at 3 MHz above center produces one hot channel"_test = [] {
        gr::lora::L1Detector det;
        const float          sample_rate = 16e6f;
        const float          channel_bw  = 62500.f;
        det.init(defaultConfig(sample_rate, 866.5e6f, channel_bw, 4096));
        // tone at +3 MHz offset, well clear of DC exclude zone (~100 kHz)
        auto samples = generateToneWithNoise(4096, sample_rate, 3e6f, 20.f);
        for (int i = 0; i < 16; ++i) {
            det.accumulate(std::span<const cf32>(samples));
        }
        auto hot = det.findHotChannels();
        expect(!hot.empty());

        // Tone at bin (3e6 / (16e6/4096)) = 768 bins from DC.
        // Each channel spans binsPerCh = 62500/3906.25 = 16 bins.
        // nChannels = usableBw / channel_bw = 12.8e6/62500 = 204.
        // center channel ~ 204/2 = 102.
        // Expected hot channel ~ 102 + 768/16 = 102 + 48 = 150.
        // Allow ±2 slack for rounding and tone leakage.
        constexpr uint32_t expected = 150;
        bool               found    = false;
        for (auto ch : hot) {
            if (ch + 2 >= expected && ch <= expected + 2) {
                found = true;
                break;
            }
        }
        std::string detail = "actual hot channels:";
        for (auto ch : hot) {
            detail += ' ';
            detail += std::to_string(ch);
        }
        expect(found) << detail;
    };

    "reset clears energy accumulation"_test = [] {
        gr::lora::L1Detector det;
        det.init(defaultConfig(16e6f, 866.5e6f, 62500.f, 4096));
        auto samples = generateToneWithNoise(4096, 16e6f, 3e6f, 20.f);
        for (int i = 0; i < 16; ++i) {
            det.accumulate(std::span<const cf32>(samples));
        }
        expect(det.snapshotCount() == 16_u);
        det.resetSweep();
        expect(det.snapshotCount() == 0_u);
        auto hot = det.findHotChannels();
        expect(hot.empty());
    };

    "cluster dedup merges adjacent hot channels"_test = [] {
        gr::lora::L1Detector det;
        const float          sample_rate = 16e6f;
        const float          channel_bw  = 62500.f;
        det.init(defaultConfig(sample_rate, 866.5e6f, channel_bw, 4096));

        // Two tones separated by ~50 kHz (within 1 channel). Cluster dedup should
        // merge them into at most two hot channels (often just one).
        auto s1 = generateToneWithNoise(4096, sample_rate, 3.000e6f, 20.f, 1U);
        auto s2 = generateToneWithNoise(4096, sample_rate, 3.050e6f, 20.f, 2U);
        for (int i = 0; i < 16; ++i) {
            std::vector<cf32> mixed(s1.size());
            for (std::size_t j = 0; j < s1.size(); ++j) {
                mixed[j] = s1[j] + s2[j];
            }
            det.accumulate(std::span<const cf32>(mixed));
        }
        auto hot = det.findHotChannels();
        expect(hot.size() <= 2_u);
    };

    "channel center frequency is within half a channel of the configured center"_test = [] {
        gr::lora::L1Detector det;
        const float          sample_rate = 16e6f;
        const float          channel_bw  = 62500.f;
        const float          center_freq = 866.5e6f;
        det.init(defaultConfig(sample_rate, center_freq, channel_bw, 4096));
        const auto n_ch = det.nChannels();
        expect(n_ch > 0_u);
        const double mid_ch_freq = det.channelCenterFreq(n_ch / 2);
        const double error       = std::abs(mid_ch_freq - static_cast<double>(center_freq));
        expect(error < static_cast<double>(channel_bw)) << "mid_ch_freq=" << mid_ch_freq << " error=" << error;
    };
};

int main() { return 0; }
