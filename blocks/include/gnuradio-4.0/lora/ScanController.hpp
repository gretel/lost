// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_SCAN_CONTROLLER_HPP
#define GNURADIO_LORA_SCAN_CONTROLLER_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numeric>
#include <ranges>
#include <span>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/lora/ChannelActivityDetector.hpp>
#include <gnuradio-4.0/lora/algorithm/Channelize.hpp>
#include <gnuradio-4.0/lora/log.hpp>

namespace gr::lora {

/// Wideband IQ sink with L1 energy analysis and L2 digital channelization for LoRa channel scanning.
///
/// Receives wideband IQ on a sync input, maintains an IQ ring buffer, reads
/// spectrum energy reports from an async input (SpectrumTap), and runs a
/// two-phase scan state machine: L1 energy accumulation followed by L2 CAD
/// probing of hot channels via digital channelization.  Detection results
/// and averaged spectrum data are emitted on async output ports.
struct ScanController : gr::Block<ScanController, gr::NoDefaultTagForwarding> {
    using Description = Doc<"Wideband IQ sink with L1 energy analysis and L2 digital channelization for LoRa channel scanning.">;
    using cf32 = std::complex<float>;

    // --- ports ---
    gr::PortIn<cf32>                             in;           // sync: wideband IQ (sink — no sync output)
    gr::PortIn<gr::DataSet<float>, gr::Async>    energy_in;    // async: spectrum energy from SpectrumTap
    gr::PortOut<gr::DataSet<float>, gr::Async>   detect_out;   // async: detection results
    gr::PortOut<gr::DataSet<float>, gr::Async>   spectrum_out;  // async: spectrum for display

    // --- settings (snake_case, reflected) ---
    float    sample_rate{16000000.f};
    float    center_freq{866500000.f};
    float    min_ratio{8.0f};
    float    buffer_ms{512.f};
    float    channel_bw{62500.f};
    uint32_t l1_reports{64};

    GR_MAKE_REFLECTABLE(ScanController, in, energy_in, detect_out, spectrum_out,
                        sample_rate, center_freq, min_ratio, buffer_ms,
                        channel_bw, l1_reports);

    // --- internal types ---

    enum class ScanState : uint8_t { Idle, Accumulate, Probe, Report };

    struct RingBuffer {
        std::vector<cf32> data;
        std::size_t       writeIdx{0};
        std::size_t       count{0};

        void resize(std::size_t capacity) {
            data.assign(capacity, {0.f, 0.f});
            writeIdx = 0;
            count    = 0;
        }

        void push(std::span<const cf32> samples) noexcept {
            const auto cap = data.size();
            for (const auto& s : samples) {
                data[writeIdx] = s;
                writeIdx = (writeIdx + 1) % cap;
            }
            count += samples.size();
        }

        [[nodiscard]] std::vector<cf32> recent(std::size_t n) const {
            if (count < n || n > data.size()) return {};
            std::vector<cf32> out(n);
            const auto        cap   = data.size();
            const std::size_t start = (writeIdx + cap - n) % cap;
            for (std::size_t i = 0; i < n; ++i) {
                out[i] = data[(start + i) % cap];
            }
            return out;
        }

        void reset() noexcept {
            writeIdx = 0;
            count    = 0;
            std::ranges::fill(data, cf32{0.f, 0.f});
        }
    };

    struct ProbeResult {
        double   freq{0.0};
        uint32_t sf{0};
        float    bw{0.f};
        float    ratio{0.f};
    };

    // --- internal state (public, prefixed _) ---

    RingBuffer               _ring;
    ScanState                _state{ScanState::Idle};
    uint32_t                 _energyReportCount{0};
    std::vector<float>       _channelEnergy;
    std::vector<uint32_t>    _hotChannels;
    std::size_t              _probeIndex{0};
    uint32_t                 _sweepCount{0};
    uint32_t                 _nChannels{0};
    std::vector<ProbeResult> _probeResults;
    std::vector<float>       _bws;

    std::vector<ChannelActivityDetector> _cadPerBw;

    // --- lifecycle ---

    void start() {
        const auto bufferSamples = static_cast<std::size_t>(buffer_ms * sample_rate / 1000.f);
        _ring.resize(bufferSamples);

        const float usableBw = sample_rate * 0.8f;
        _nChannels = static_cast<uint32_t>(usableBw / channel_bw);
        _channelEnergy.assign(_nChannels, 0.f);

        _bws = {62500.f, 125000.f, 250000.f};
        _cadPerBw.clear();
        _cadPerBw.reserve(_bws.size());
        for (float bw : _bws) {
            ChannelActivityDetector cad;
            cad.os_factor = 1U;
            cad.bandwidth = static_cast<uint32_t>(bw);
            cad.initMultiSf();
            _cadPerBw.push_back(std::move(cad));
        }

        _state             = ScanState::Idle;
        _energyReportCount = 0;
        _sweepCount        = 0;

        gr::lora::log_ts("info ", "scanctrl",
            "started: %.1f MS/s, %.3f MHz center, %u channels (%.1f kHz), buffer %u ms",
            static_cast<double>(sample_rate) / 1e6,
            static_cast<double>(center_freq) / 1e6, _nChannels,
            static_cast<double>(channel_bw) / 1e3,
            static_cast<unsigned>(buffer_ms));
    }

    // --- processBulk ---

    gr::work::Status processBulk(
        gr::InputSpanLike auto&    inPort,
        gr::InputSpanLike auto&    energyInPort,
        gr::OutputSpanLike auto&  detectOutPort,
        gr::OutputSpanLike auto&  spectrumOutPort) noexcept
    {
        // 1. Check overflow tags — reset state on sample discontinuity
        if (this->inputTagsPresent()) {
            const auto& tag = this->mergedInputTag();
            if (tag.map.contains("overflow")) {
                gr::lora::log_ts("warn ", "scanctrl", "overflow detected — resetting state");
                _ring.reset();
                _state = ScanState::Idle;
                _energyReportCount = 0;
                _channelEnergy.assign(_nChannels, 0.f);
            }
        }

        // 2. Push all input IQ into ring buffer (always consume all — keeps scheduler fast)
        auto inSpan = std::span(inPort);
        _ring.push(inSpan);
        std::ignore = inPort.consume(inSpan.size());

        // 3. Read async energy reports (if available)
        auto energySpan = std::span(energyInPort);
        if (!energySpan.empty()) {
            for (std::size_t i = 0; i < energySpan.size(); ++i) {
                processEnergyReport(energySpan[i]);
            }
            std::ignore = energyInPort.consume(energySpan.size());
        }

        // 4. Run state machine
        std::size_t detectPublished   = 0;
        std::size_t spectrumPublished = 0;

        switch (_state) {
        case ScanState::Idle:
            _state = ScanState::Accumulate;
            resetAccumulation();
            break;

        case ScanState::Accumulate:
            if (_energyReportCount >= l1_reports) {
                findHotChannels();
                if (_hotChannels.empty()) {
                    spectrumPublished = emitSpectrum(spectrumOutPort);
                    _sweepCount++;
                    _state = ScanState::Idle;
                } else {
                    _probeIndex = 0;
                    _probeResults.clear();
                    _state = ScanState::Probe;
                }
            }
            break;

        case ScanState::Probe:
            probeNextChannel();
            if (_probeIndex >= _hotChannels.size()) {
                _state = ScanState::Report;
            }
            break;

        case ScanState::Report:
            detectPublished   = emitDetections(detectOutPort);
            spectrumPublished = emitSpectrum(spectrumOutPort);
            _sweepCount++;
            _state = ScanState::Idle;
            break;
        }

        detectOutPort.publish(detectPublished);
        spectrumOutPort.publish(spectrumPublished);
        return gr::work::Status::OK;
    }

    // --- helper methods ---

    void processEnergyReport(const gr::DataSet<float>& ds) noexcept {
        if (ds.signal_values.empty()) return;

        const auto nValues = std::min(ds.signal_values.size(), static_cast<std::size_t>(_nChannels));
        for (std::size_t i = 0; i < nValues; ++i) {
            _channelEnergy[i] += ds.signal_values[i];
        }
        ++_energyReportCount;
    }

    void findHotChannels() noexcept {
        _hotChannels.clear();
        if (_nChannels == 0 || _energyReportCount == 0) return;

        // Compute median energy for threshold
        std::vector<float> sorted(_channelEnergy.begin(),
                                  _channelEnergy.begin() + static_cast<std::ptrdiff_t>(_nChannels));
        std::ranges::sort(sorted);
        const float median = sorted[sorted.size() / 2];

        // Hot = energy > 6× median (well above noise floor)
        constexpr float kHotMultiplier = 6.0f;
        const float threshold = median * kHotMultiplier;

        for (uint32_t ch = 0; ch < _nChannels; ++ch) {
            if (_channelEnergy[ch] > threshold) {
                _hotChannels.push_back(ch);
            }
        }
    }

    void probeNextChannel() noexcept {
        if (_probeIndex >= _hotChannels.size()) return;

        const uint32_t ch = _hotChannels[_probeIndex];
        const double channelFreq = channelCenterFreq(ch);

        ProbeResult bestResult;
        bestResult.freq = channelFreq;

        for (std::size_t bwIdx = 0; bwIdx < _bws.size(); ++bwIdx) {
            const float probeBw = _bws[bwIdx];
            auto& cad = _cadPerBw[bwIdx];

            // Determine how many narrowband samples we need for 2-symbol CAD at SF12
            // SF12 at 1x: 2 * 4096 = 8192 samples per 2-symbol window
            constexpr uint32_t kMaxSf       = 12;
            const uint32_t     fftSize      = 1U << kMaxSf;
            const uint32_t     cadSamples   = fftSize * 2;
            const auto         osFactor     = static_cast<uint32_t>(sample_rate / probeBw);
            const uint32_t     wbSamples    = cadSamples * osFactor;

            auto wbIq = _ring.recent(wbSamples);
            if (wbIq.empty()) continue;

            // Channelize: freq-shift + decimate to narrowband
            auto nbIq = channelize(wbIq.data(), static_cast<uint32_t>(wbIq.size()),
                                   channelFreq, static_cast<double>(center_freq),
                                   static_cast<double>(sample_rate),
                                   static_cast<double>(probeBw));

            if (nbIq.size() < cadSamples) continue;

            // Run multi-SF CAD on narrowband IQ
            auto det = cad.detectMultiSf(nbIq.data());
            if (det.detected && det.best_ratio > bestResult.ratio) {
                bestResult.sf    = det.sf;
                bestResult.bw    = probeBw;
                bestResult.ratio = det.best_ratio;
            }
        }

        if (bestResult.ratio >= min_ratio) {
            _probeResults.push_back(bestResult);
        }

        ++_probeIndex;
    }

    [[nodiscard]] std::size_t emitDetections(gr::OutputSpanLike auto& outPort) noexcept {
        std::size_t published = 0;
        auto outSpan = std::span(outPort);

        for (const auto& result : _probeResults) {
            if (published >= outSpan.size()) break;

            gr::DataSet<float> ds;
            ds.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            ds.signal_names.emplace_back("detection");
            ds.signal_units.emplace_back("ratio");
            ds.signal_values = {result.ratio};
            ds.extents       = {1};

            ds.axis_names.emplace_back("frequency");
            ds.axis_units.emplace_back("Hz");
            ds.axis_values.push_back({static_cast<float>(result.freq)});

            ds.meta_information.resize(1);
            ds.meta_information[0]["type"]    = std::string("scan_detect");
            ds.meta_information[0]["sf"]      = static_cast<int64_t>(result.sf);
            ds.meta_information[0]["bw"]      = static_cast<float>(result.bw);
            ds.meta_information[0]["ratio"]   = static_cast<float>(result.ratio);
            ds.meta_information[0]["freq"]    = result.freq;
            ds.meta_information[0]["sweep"]   = static_cast<int64_t>(_sweepCount);

            outSpan[published] = std::move(ds);
            ++published;
        }
        return published;
    }

    [[nodiscard]] std::size_t emitSpectrum(gr::OutputSpanLike auto& outPort) noexcept {
        auto outSpan = std::span(outPort);
        if (outSpan.empty() || _nChannels == 0) return 0;

        gr::DataSet<float> ds;
        ds.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        ds.signal_names.emplace_back("channel_energy");
        ds.signal_units.emplace_back("linear");
        ds.extents = {static_cast<int32_t>(_nChannels)};

        // Average the accumulated energy over the report count
        const float scale = (_energyReportCount > 0)
            ? 1.0f / static_cast<float>(_energyReportCount)
            : 1.0f;
        ds.signal_values.resize(_nChannels);
        for (uint32_t i = 0; i < _nChannels; ++i) {
            ds.signal_values[i] = _channelEnergy[i] * scale;
        }

        // Build frequency axis
        ds.axis_names.emplace_back("frequency");
        ds.axis_units.emplace_back("Hz");
        ds.axis_values.resize(1);
        ds.axis_values[0].resize(_nChannels);
        for (uint32_t i = 0; i < _nChannels; ++i) {
            ds.axis_values[0][i] = static_cast<float>(channelCenterFreq(i));
        }

        ds.meta_information.resize(1);
        ds.meta_information[0]["type"]       = std::string("scan_spectrum");
        ds.meta_information[0]["sweep"]      = static_cast<int64_t>(_sweepCount);
        ds.meta_information[0]["n_hot"]      = static_cast<int64_t>(_hotChannels.size());
        ds.meta_information[0]["n_reports"]  = static_cast<int64_t>(_energyReportCount);

        outSpan[0] = std::move(ds);
        return 1;
    }

    void resetAccumulation() noexcept {
        _channelEnergy.assign(_nChannels, 0.f);
        _energyReportCount = 0;
        _hotChannels.clear();
    }

    [[nodiscard]] double channelCenterFreq(uint32_t ch) const noexcept {
        // Channels are spaced by channel_bw, centered around center_freq.
        // Channel 0 is at the lowest frequency of the usable band.
        const float usableBw    = sample_rate * 0.8f;
        const float bandStart   = center_freq - usableBw / 2.f;
        return static_cast<double>(bandStart + (static_cast<float>(ch) + 0.5f) * channel_bw);
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_SCAN_CONTROLLER_HPP
