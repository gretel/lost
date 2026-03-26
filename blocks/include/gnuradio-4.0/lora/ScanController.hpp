// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_SCAN_CONTROLLER_HPP
#define GNURADIO_LORA_SCAN_CONTROLLER_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <numeric>
#include <ranges>
#include <span>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/ChannelActivityDetector.hpp>
#include <gnuradio-4.0/lora/algorithm/Channelize.hpp>
#include <gnuradio-4.0/lora/algorithm/DCBlocker.hpp>
#include <gnuradio-4.0/lora/algorithm/RingBuffer.hpp>
#include <gnuradio-4.0/lora/log.hpp>

namespace gr::lora {

struct ScanController : gr::Block<ScanController, gr::NoDefaultTagForwarding> {
    using Description = Doc<"Wideband IQ sink with integrated L1 energy and L2 digital channelization.">;
    using cf32 = std::complex<float>;

    // --- ports ---
    gr::PortIn<cf32>                             in;
    gr::PortOut<gr::DataSet<float>, gr::Async>   detect_out;
    gr::PortOut<gr::DataSet<float>, gr::Async>   spectrum_out;

    // --- settings ---
    float       sample_rate{16000000.f};
    float       center_freq{866500000.f};
    float       min_ratio{8.0f};
    float       buffer_ms{512.f};
    float       channel_bw{62500.f};
    uint32_t    l1_interval{64};       // processBulk calls between L1 energy snapshots
    uint32_t    l1_snapshots{16};      // snapshots to accumulate before probing
    uint32_t    l1_fft_size{4096};
    std::string probe_bws{"62500"};    // comma-separated BWs to probe (Hz)
    float       dc_blocker_cutoff{2000.f};  // DSP DC blocker cutoff (Hz), 0 = disabled

    GR_MAKE_REFLECTABLE(ScanController, in, detect_out, spectrum_out,
                        sample_rate, center_freq, min_ratio, buffer_ms,
                        channel_bw, l1_interval, l1_snapshots, l1_fft_size,
                        probe_bws, dc_blocker_cutoff);

    // --- internal types ---

    enum class ScanState : uint8_t { Accumulate, Probe, Report };

    struct ProbeResult {
        double   freq{0.0};
        uint32_t sf{0};
        float    bw{0.f};
        float    ratio{0.f};
    };

    // --- internal state ---

    RingBuffer               _ring;
    ScanState                _state{ScanState::Accumulate};
    uint32_t                 _callCount{0};
    uint32_t                 _snapshotCount{0};
    std::vector<float>       _channelEnergy;
    std::vector<uint32_t>    _channelHotCount;   // consecutive sweeps channel was hot
    std::vector<uint32_t>    _hotChannels;
    std::size_t              _probeIndex{0};
    std::size_t              _probeBwIndex{0};
    uint32_t                 _sweepCount{0};
    uint32_t                 _nChannels{0};
    std::vector<ProbeResult> _probeResults;
    ProbeResult              _currentBest;
    std::vector<float>       _bws;
    std::vector<uint32_t>    _channelPeakBin;  // DC-centered FFT bin with max |Y|² per channel
    std::vector<float>       _channelPeakMag;  // max |Y|² per channel
    std::chrono::steady_clock::time_point _sweepStart{std::chrono::steady_clock::now()};
    uint32_t                 _probeWindowStart{0};  // rotating probe window position

    // L1 FFT state
    gr::algorithm::FFT<cf32> _fft;
    std::vector<float>       _window;

    std::vector<ChannelActivityDetector> _cadPerBw;

    // DC blocker (embedded, applied before ring buffer push + L1 FFT)
    DCBlocker           _dc;
    std::vector<cf32>   _dcBuf;

    // --- lifecycle ---

    void start() {
        const auto bufferSamples = static_cast<std::size_t>(buffer_ms * sample_rate / 1000.f);
        _ring.resize(bufferSamples);

        const float usableBw = sample_rate * 0.8f;
        _nChannels = static_cast<uint32_t>(usableBw / channel_bw);
        _channelEnergy.assign(_nChannels, 0.f);
        _channelHotCount.assign(_nChannels, 0);
        _channelPeakBin.assign(_nChannels, 0);
        _channelPeakMag.assign(_nChannels, 0.f);

        // Parse configurable BWs from comma-separated string
        _bws.clear();
        {
            std::string s = probe_bws;
            std::size_t pos = 0;
            while (pos < s.size()) {
                auto end = s.find(',', pos);
                if (end == std::string::npos) end = s.size();
                if (end > pos) {
                    _bws.push_back(std::stof(s.substr(pos, end - pos)));
                }
                pos = end + 1;
            }
        }
        if (_bws.empty()) _bws = {62500.f};
        _cadPerBw.clear();
        _cadPerBw.reserve(_bws.size());
        for (float bw : _bws) {
            ChannelActivityDetector cad;
            cad.os_factor = 1U;
            cad.bandwidth = static_cast<uint32_t>(bw);
            cad.initMultiSf();
            _cadPerBw.push_back(std::move(cad));
        }

        // L1 FFT window (Hann)
        _window.resize(l1_fft_size);
        for (uint32_t i = 0; i < l1_fft_size; ++i) {
            _window[i] = 0.5f * (1.f - std::cos(2.f * std::numbers::pi_v<float>
                                                  * static_cast<float>(i)
                                                  / static_cast<float>(l1_fft_size)));
        }
        _fft = gr::algorithm::FFT<cf32>{};

        _state         = ScanState::Accumulate;
        _callCount     = 0;
        _snapshotCount = 0;
        _sweepCount    = 0;

        // DC blocker: remove DC spur before L1 energy + ring buffer
        if (dc_blocker_cutoff > 0.f) {
            _dc.init(sample_rate, dc_blocker_cutoff);
        }

        gr::lora::log_ts("info ", "scanctrl",
            "started: %.1f MS/s, %.3f MHz center, %u ch (%.1f kHz), buffer %u ms, L1 every %u calls, dc_hp %.0f Hz",
            static_cast<double>(sample_rate) / 1e6,
            static_cast<double>(center_freq) / 1e6, _nChannels,
            static_cast<double>(channel_bw) / 1e3,
            static_cast<unsigned>(buffer_ms), l1_interval,
            static_cast<double>(dc_blocker_cutoff));
    }

    // --- processBulk ---

    gr::work::Status processBulk(
        gr::InputSpanLike auto&   inPort,
        gr::OutputSpanLike auto&  detectOutPort,
        gr::OutputSpanLike auto&  spectrumOutPort) noexcept
    {
        // 1. Check overflow tags
        if (this->inputTagsPresent()) {
            const auto& tag = this->mergedInputTag();
            if (tag.map.contains("overflow")) {
                _ring.reset();
                _state         = ScanState::Accumulate;
                _snapshotCount = 0;
                _channelEnergy.assign(_nChannels, 0.f);
            }
        }

        // 2. DC blocker + push into ring buffer
        auto inSpan = std::span(inPort);
        std::span<const cf32> cleanSpan = inSpan;
        if (_dc.initialised()) {
            _dcBuf.resize(inSpan.size());
            _dc.processBlock(inSpan, std::span<cf32>(_dcBuf));
            cleanSpan = std::span<const cf32>(_dcBuf);
        }
        _ring.push(cleanSpan);
        std::ignore = inPort.consume(inSpan.size());

        // 3. Periodic L1 energy snapshot (cheap: one FFT per interval)
        ++_callCount;
        if (_state == ScanState::Accumulate && _callCount >= l1_interval) {
            _callCount = 0;
            computeEnergySnapshot();
        }

        // 4. State machine
        std::size_t spectrumPublished = 0;

        switch (_state) {
        case ScanState::Accumulate:
            if (_snapshotCount >= l1_snapshots) {
                findHotChannels();  // still run for spectrum metadata

                // L1 FFT energy gating cannot detect BW62.5k chirp signals at
                // 16 MS/s (energy dilutes to ~1× median across FFT bins).
                // Use a rotating probe window: probe kProbeWindowSize channels
                // per sweep, cycling through the full band. L1 hot channels
                // are merged in so energy-detected signals still get priority.
                constexpr uint32_t kProbeWindowSize = 8;
                const uint32_t windowEnd = std::min(_probeWindowStart + kProbeWindowSize, _nChannels);

                // Build probe list: rotating window + any L1 hot channels
                std::vector<uint32_t> probeChannels;
                for (uint32_t ch = _probeWindowStart; ch < windowEnd; ++ch) {
                    probeChannels.push_back(ch);
                }
                // Merge L1 hot channels not already in the window
                for (uint32_t ch : _hotChannels) {
                    if (ch < _probeWindowStart || ch >= windowEnd) {
                        probeChannels.push_back(ch);
                    }
                }

                // Advance window for next sweep (wrap around)
                _probeWindowStart = windowEnd >= _nChannels ? 0 : windowEnd;

                _hotChannels = std::move(probeChannels);
                _probeIndex   = 0;
                _probeBwIndex = 0;
                _probeResults.clear();
                _state = ScanState::Probe;
            }
            break;

        case ScanState::Probe: {
            bool done = probeStep();
            if (done) {
                _state = ScanState::Report;
            }
            break;
        }

        case ScanState::Report:
            spectrumPublished = emitSpectrum(spectrumOutPort);
            if (spectrumPublished > 0) {
                _sweepCount++;
                resetAccumulation();
                _state = ScanState::Accumulate;
            }
            // else: output full, retry next processBulk call
            break;
        }

        spectrumOutPort.publish(spectrumPublished);
        return gr::work::Status::OK;
    }

    // --- L1 energy ---

    void computeEnergySnapshot() noexcept {
        // Need at least fft_size recent samples
        if (_ring.count < l1_fft_size) return;

        auto samples = _ring.recent(l1_fft_size);
        if (samples.empty()) return;

        // Apply window
        for (uint32_t i = 0; i < l1_fft_size; ++i) {
            samples[i] *= _window[i];
        }

        auto fftOut = _fft.compute(
            std::span<const cf32>(samples.data(), l1_fft_size));

        // Replace DC spur bins with noise-floor estimate.
        // The B210 AD9361 produces a broadband DC artifact at 0 Hz baseband
        // (~22 dB above NF at 16 MS/s) that no IIR HP filter can sufficiently
        // suppress. Replacing (not zeroing) avoids a visible notch in the
        // spectrum while removing the spur from energy detection.
        {
            // Scale null radius to ~±100 kHz regardless of FFT size / sample rate
            const float binHz = sample_rate / static_cast<float>(l1_fft_size);
            const auto kDcNullRadius = std::min(
                static_cast<uint32_t>(100000.f / binHz),
                l1_fft_size / 8);  // cap at 12.5% of FFT to preserve usable spectrum
            const auto kRefBins = std::max(kDcNullRadius, 8u);
            float refMagSq = 0.f;
            for (uint32_t b = kDcNullRadius + 1; b <= kDcNullRadius + kRefBins && b < l1_fft_size / 2; ++b) {
                const auto& s1 = fftOut[b];
                refMagSq += s1.real() * s1.real() + s1.imag() * s1.imag();
                const auto& s2 = fftOut[l1_fft_size - b];
                refMagSq += s2.real() * s2.real() + s2.imag() * s2.imag();
            }
            // Average magnitude for replacement (preserve phase=0, set magnitude to NF)
            float nfMag = std::sqrt(refMagSq / static_cast<float>(2 * kRefBins));
            for (uint32_t b = 0; b <= kDcNullRadius; ++b) {
                fftOut[b] = {nfMag, 0.f};
                if (b > 0) fftOut[l1_fft_size - b] = {nfMag, 0.f};
            }
        }

        // Accumulate |FFT|² into channel bins (DC-centered via fftshift)
        const auto half    = l1_fft_size / 2;
        const float binBw  = sample_rate / static_cast<float>(l1_fft_size);
        const auto binsPerCh = static_cast<uint32_t>(channel_bw / binBw);

        // Usable band starts at bin offset corresponding to -usableBw/2
        const float usableBw  = sample_rate * 0.8f;
        const auto  startBin  = static_cast<uint32_t>(
            (static_cast<float>(l1_fft_size) - usableBw / binBw) / 2.f);

        for (uint32_t ch = 0; ch < _nChannels && ch * binsPerCh + startBin + binsPerCh <= l1_fft_size; ++ch) {
            float energy = 0.f;
            for (uint32_t b = 0; b < binsPerCh; ++b) {
                const auto fftIdx = (startBin + ch * binsPerCh + b + half) % l1_fft_size;
                const auto& s = fftOut[fftIdx];
                const float mag = s.real() * s.real() + s.imag() * s.imag();
                energy += mag;
                // Track peak bin within this channel for frequency refinement
                if (mag > _channelPeakMag[ch]) {
                    _channelPeakMag[ch] = mag;
                    _channelPeakBin[ch] = startBin + ch * binsPerCh + b;
                }
            }
            _channelEnergy[ch] += energy;
        }

        ++_snapshotCount;
    }

    // --- hot channel detection ---

    void findHotChannels() noexcept {
        _hotChannels.clear();
        if (_nChannels == 0 || _snapshotCount == 0) return;

        // Threshold: channels with energy > 6× median
        std::vector<float> sorted(_channelEnergy.begin(),
                                  _channelEnergy.begin() + static_cast<std::ptrdiff_t>(_nChannels));
        std::ranges::sort(sorted);
        const float median = sorted[sorted.size() / 2];

        constexpr float    kHotMultiplier = 6.0f;
        constexpr uint32_t kMinHotSweeps  = 1;  // report on first sweep (transient signals span ~1 sweep)
        const float threshold = median * kHotMultiplier;

        // Update persistence counters
        for (uint32_t ch = 0; ch < _nChannels; ++ch) {
            if (_channelEnergy[ch] > threshold) {
                _channelHotCount[ch]++;
            } else {
                _channelHotCount[ch] = 0;
            }
        }

        // Only report channels that have been hot for kMinHotSweeps consecutive sweeps
        std::vector<uint32_t> raw;
        for (uint32_t ch = 0; ch < _nChannels; ++ch) {
            if (_channelHotCount[ch] >= kMinHotSweeps) {
                raw.push_back(ch);
            }
        }

        // Cluster dedup: merge adjacent hot channels, keep peak-energy channel per cluster
        std::size_t i = 0;
        while (i < raw.size()) {
            uint32_t bestCh = raw[i];
            float    bestE  = _channelEnergy[bestCh];
            std::size_t j = i + 1;
            while (j < raw.size() && raw[j] == raw[j - 1] + 1) {
                if (_channelEnergy[raw[j]] > bestE) {
                    bestCh = raw[j];
                    bestE  = _channelEnergy[raw[j]];
                }
                ++j;
            }
            _hotChannels.push_back(bestCh);
            i = j;
        }
    }

    // --- L2 probe (one BW per call) ---

    bool probeStep() noexcept {
        if (_probeIndex >= _hotChannels.size()) return true;

        const uint32_t ch = _hotChannels[_probeIndex];
        const double channelFreq = channelCenterFreq(ch);

        // Initialize best result for this channel on first BW
        if (_probeBwIndex == 0) {
            _currentBest = ProbeResult{};
            _currentBest.freq = channelFreq;
        }

        if (_probeBwIndex < _bws.size() && _currentBest.ratio < min_ratio) {
            // Probe narrowest BW first; stop as soon as one detects above threshold.
            // Narrowest BW gives correct SF identification (wider BWs detect
            // chirp-slope equivalents at different SFs due to k = BW²/2^SF).
            const float probeBw = _bws[_probeBwIndex];
            auto& cad = _cadPerBw[_probeBwIndex];

            constexpr uint32_t kMaxSf     = 12;
            const uint32_t     fftSize    = 1U << kMaxSf;
            const uint32_t     cadSamples = fftSize * 2;
            const auto         osFactor   = static_cast<uint32_t>(sample_rate / probeBw);
            const uint32_t     wbSamples  = cadSamples * osFactor;

            auto wbIq = _ring.recent(wbSamples);
            if (!wbIq.empty()) {
                auto nbIq = channelize(wbIq.data(), static_cast<uint32_t>(wbIq.size()),
                                       channelFreq, static_cast<double>(center_freq),
                                       static_cast<double>(sample_rate),
                                       static_cast<double>(probeBw));

                if (nbIq.size() >= cadSamples) {
                    auto det = cad.detectMultiSf(nbIq.data());
                    if (det.detected && det.best_ratio > _currentBest.ratio) {
                        _currentBest.sf    = det.sf;
                        _currentBest.bw    = probeBw;
                        _currentBest.ratio = det.best_ratio;
                    }
                }
            }

            ++_probeBwIndex;
        }

        // Done if: all BWs tried, or narrowest BW already detected above threshold
        if (_probeBwIndex >= _bws.size() || _currentBest.ratio >= min_ratio) {
            if (_currentBest.ratio >= min_ratio) {
                _probeResults.push_back(_currentBest);
            }
            ++_probeIndex;
            _probeBwIndex = 0;
        }

        return _probeIndex >= _hotChannels.size();
    }

    // --- output ---

    [[nodiscard]] std::size_t emitSpectrum(gr::OutputSpanLike auto& outPort) noexcept {
        auto outSpan = std::span(outPort);
        if (outSpan.empty() || _nChannels == 0) return 0;

        gr::DataSet<float> ds;
        ds.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        ds.signal_names.emplace_back("channel_energy");
        ds.signal_units.emplace_back("linear");
        ds.extents = {static_cast<int32_t>(_nChannels)};

        const float scale = (_snapshotCount > 0)
            ? 1.0f / static_cast<float>(_snapshotCount)
            : 1.0f;
        ds.signal_values.resize(_nChannels);
        for (uint32_t i = 0; i < _nChannels; ++i) {
            ds.signal_values[i] = _channelEnergy[i] * scale;
        }

        ds.axis_names.emplace_back("frequency");
        ds.axis_units.emplace_back("Hz");
        ds.axis_values.resize(1);
        ds.axis_values[0].resize(_nChannels);
        for (uint32_t i = 0; i < _nChannels; ++i) {
            ds.axis_values[0][i] = static_cast<float>(channelCenterFreq(i));
        }

        ds.meta_information.resize(1);
        const auto sweepDur = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - _sweepStart).count();

        ds.meta_information[0]["type"]        = std::string("scan_spectrum");
        ds.meta_information[0]["sweep"]       = static_cast<int64_t>(_sweepCount);
        ds.meta_information[0]["n_hot"]       = static_cast<int64_t>(_hotChannels.size());
        ds.meta_information[0]["n_snapshots"] = static_cast<int64_t>(_snapshotCount);
        ds.meta_information[0]["n_det"]       = static_cast<int64_t>(_probeResults.size());
        ds.meta_information[0]["duration_ms"] = static_cast<int64_t>(sweepDur);

        // Embed hot channel indices as comma-separated string
        // (pmt::Value doesn't support vector<int64_t>, so we serialize)
        {
            std::string hotStr;
            for (std::size_t i = 0; i < _hotChannels.size(); ++i) {
                if (i > 0) hotStr += ',';
                hotStr += std::to_string(_hotChannels[i]);
            }
            ds.meta_information[0]["hot_indices"] = std::move(hotStr);
        }

        // Embed detection results as indexed metadata fields
        for (std::size_t i = 0; i < _probeResults.size(); ++i) {
            const auto& r = _probeResults[i];
            const auto idx = std::to_string(i);
            ds.meta_information[0][std::pmr::string("det" + idx + "_freq")]  = r.freq;
            ds.meta_information[0][std::pmr::string("det" + idx + "_sf")]    = static_cast<int64_t>(r.sf);
            ds.meta_information[0][std::pmr::string("det" + idx + "_bw")]    = static_cast<float>(r.bw);
            ds.meta_information[0][std::pmr::string("det" + idx + "_ratio")] = static_cast<float>(r.ratio);
        }

        outSpan[0] = std::move(ds);
        return 1;
    }

    void resetAccumulation() noexcept {
        _channelEnergy.assign(_nChannels, 0.f);
        _channelPeakBin.assign(_nChannels, 0);
        _channelPeakMag.assign(_nChannels, 0.f);
        _snapshotCount = 0;
        _callCount     = 0;
        _hotChannels.clear();
        _sweepStart = std::chrono::steady_clock::now();
    }

    /// Refined frequency from L1 FFT peak bin (Fix B: ~3.9 kHz resolution vs 62.5 kHz channel center).
    [[nodiscard]] double refinedFreq(uint32_t ch) const noexcept {
        if (ch < _nChannels && _channelPeakMag[ch] > 0.f) {
            const double binBw = static_cast<double>(sample_rate) / static_cast<double>(l1_fft_size);
            const double usableBw = static_cast<double>(sample_rate) * 0.8;
            const double bandStart = static_cast<double>(center_freq) - usableBw / 2.0;
            return bandStart + static_cast<double>(_channelPeakBin[ch]) * binBw + binBw * 0.5;
        }
        return channelCenterFreq(ch);
    }

    [[nodiscard]] double channelCenterFreq(uint32_t ch) const noexcept {
        const double usableBw  = static_cast<double>(sample_rate) * 0.8;
        const double bandStart = static_cast<double>(center_freq) - usableBw / 2.0;
        return bandStart + (static_cast<double>(ch) + 0.5) * static_cast<double>(channel_bw);
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_SCAN_CONTROLLER_HPP
