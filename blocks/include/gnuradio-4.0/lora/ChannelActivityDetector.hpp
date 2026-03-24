// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_CHANNEL_ACTIVITY_DETECTOR_HPP
#define GNURADIO_LORA_CHANNEL_ACTIVITY_DETECTOR_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/algorithm/utilities.hpp>
#include <gnuradio-4.0/lora/log.hpp>

namespace gr::lora {

/// ChannelActivityDetector — detects LoRa chirp activity in a 2-symbol window.
///
/// Algorithm (Vangelista & Calvagno 2024, Algorithm 1/2):
///   1. Collect 2*N*os_factor IQ samples (2 symbol periods at the input rate).
///   2. For each symbol window (2 windows) and each chirp polarity:
///        a. Try os_factor sub-chip timing offsets to mitigate lack of symbol
///           synchronisation (analogous to the paper's NStep sliding window).
///        b. At each offset: decimate the oversampled input by os_factor (take
///           every os_factor-th sample starting at that offset), element-wise
///           multiply by the conjugate 1x-rate reference chirp, FFT over N bins.
///        c. Compute peak_ratio = max|Y| / mean|Y|.  Keep the maximum across offsets.
///   3. Upchirp detected if peak_ratio > alpha in BOTH symbol windows.
///      (Same for downchirp when dual_chirp = true.)
///
/// Decimation-then-1x-dechirp derivation (paper uses os=2):
///   paper: y[m] = d_{Tc/2}(k·Step + 2m + ℓ) · e^{-j2π m²/2M}
///   here:  y[m] = x[ℓ + m·os] · conj(upchirp_1x[m])    (ℓ = 0..os-1)
/// These are equivalent because decimating e^{j·phase(m·os)} by os gives the
/// same phase sequence as the 1x chirp evaluated at m.
///
/// Output: one uint8_t per detection window.  Bit 0 = upchirp detected,
///         bit 1 = downchirp detected.  A tag is published on detection with
///         keys: cad_detected, cad_upchirp, cad_downchirp,
///               cad_peak_ratio_up, cad_peak_ratio_down.
///
/// Use with Splitter to run in parallel with the main decode pipeline.
GR_REGISTER_BLOCK("gr::lora::ChannelActivityDetector", gr::lora::ChannelActivityDetector)
struct ChannelActivityDetector
    : gr::Block<ChannelActivityDetector, gr::NoDefaultTagForwarding> {
    using Description = Doc<"Detects LoRa chirp activity in a 2-symbol window">;
    using cf32 = std::complex<float>;

    /// Which chirp polarity to use as the dechirp reference.
    enum class RefType { UpchirpRef, DownchirpRef };

    struct PeakResult {
        float    ratio{0.F};
        uint32_t bin{0};
    };

    struct DetectResult {
        float    peak_ratio_up{0.F};
        float    peak_ratio_dn{0.F};
        uint32_t peak_bin_up{0};
        uint32_t peak_bin_dn{0};
        bool     up_detected{false};
        bool     dn_detected{false};
        bool     detected{false};
    };

    struct MultiSfResult {
        float    best_ratio{0.F};
        float    peak_ratio_up{0.F};
        float    peak_ratio_dn{0.F};
        uint32_t peak_bin_up{0};
        uint32_t peak_bin_dn{0};
        uint32_t sf{0};              ///< winning SF (0 = no detection)
        bool     up_detected{false};
        bool     dn_detected{false};
        bool     detected{false};
    };

    gr::PortIn<cf32>                in;
    gr::PortOut<uint8_t, gr::Async> out;

    Annotated<uint32_t, "sf",         gr::Visible, gr::Limits<6U, 12U>>      sf        = 8U;
    Annotated<uint32_t, "bandwidth",  gr::Visible, gr::Limits<1U, 2000000U>> bandwidth = 62500U;
    Annotated<uint32_t, "os_factor",  gr::Visible, gr::Limits<1U, 16U>>      os_factor = 4U;
    Annotated<float,    "alpha",      gr::Visible> alpha      = 4.16F;  ///< SF-dependent threshold
    Annotated<bool,     "dual_chirp", gr::Visible> dual_chirp = true;   ///< also detect inverted-IQ
    bool debug = false;

    GR_MAKE_REFLECTABLE(ChannelActivityDetector, in, out,
                        sf, bandwidth, os_factor, alpha, dual_chirp, debug);

private:
    uint32_t _symLen{0};   ///< N * os_factor: samples per oversampled symbol
    uint32_t _winLen{0};   ///< 2 * _symLen: samples in a full 2-symbol window
    uint32_t _fftSize{0};  ///< 2^sf: FFT size / samples per 1x symbol

    std::vector<cf32> _upRef{};     ///< upchirp 1x (ref for downchirp detection)
    std::vector<cf32> _downRef{};   ///< conj(upchirp) = downchirp (ref for upchirp detection)

    std::vector<cf32> _buf{};       ///< accumulation buffer (length _winLen)
    uint32_t          _bufFill{0};

    std::vector<cf32>                       _scratch{};  ///< dechirp workspace (length _fftSize)
    std::vector<cf32>                       _fftOut{};   ///< FFT output workspace (length _fftSize)
    gr::algorithm::FFT<cf32>                _fft{};

    std::atomic<bool>* _channelBusy{nullptr};  ///< external LBT flag (nullable)

    /// Pre-built reference chirps and workspace for one SF (used by detectMultiSf).
    struct SfRefs {
        uint32_t sf{0};
        uint32_t fftSize{0};    ///< 2^sf
        uint32_t symLen{0};     ///< fftSize * os_factor
        std::vector<cf32> upRef{};
        std::vector<cf32> downRef{};
        std::vector<cf32> scratch{};    ///< dechirp workspace (length fftSize)
        std::vector<cf32> fftOut{};     ///< FFT output workspace (length fftSize)
        gr::algorithm::FFT<cf32> fft{};
    };

    std::vector<SfRefs> _sfTable{};  ///< SF7-12 refs (populated by initMultiSf)

public:
    explicit ChannelActivityDetector(gr::property_map init = {})
        : gr::Block<ChannelActivityDetector, gr::NoDefaultTagForwarding>(std::move(init)) {}

    void start() { rebuildRefs(); }

    void settingsChanged(const gr::property_map& /*old*/, const gr::property_map& changed) {
        if (changed.contains("sf") || changed.contains("os_factor") || changed.contains("bandwidth")) {
            rebuildRefs();
        }
    }

    gr::work::Status processBulk(gr::InputSpanLike auto& input,
                                 gr::OutputSpanLike auto& output) noexcept {
        auto inSpan  = std::span(input);
        auto outSpan = std::span(output);

        const uint32_t available = static_cast<uint32_t>(inSpan.size());
        const uint32_t needed    = _winLen - _bufFill;
        const uint32_t consume   = std::min(available, needed);

        std::copy_n(inSpan.begin(), consume, _buf.begin() + static_cast<std::ptrdiff_t>(_bufFill));
        _bufFill += consume;

        if (_bufFill < _winLen) {
            std::ignore = input.consume(consume);
            output.publish(0UZ);
            return gr::work::Status::OK;
        }

        const auto det = detect(_buf.data());

        if (_channelBusy != nullptr) {
            _channelBusy->store(det.detected, std::memory_order_release);
        }

        uint8_t result = 0U;
        if (det.up_detected) { result |= 0x01U; }
        if (det.dn_detected) { result |= 0x02U; }

        if (!outSpan.empty()) {
            outSpan[0] = result;
            if (det.detected) {
                gr::property_map tagMap;
                tagMap["cad_detected"]        = det.detected;
                tagMap["cad_upchirp"]         = det.up_detected;
                tagMap["cad_downchirp"]       = det.dn_detected;
                tagMap["cad_peak_ratio_up"]   = static_cast<double>(det.peak_ratio_up);
                tagMap["cad_peak_ratio_down"] = static_cast<double>(det.peak_ratio_dn);
                this->publishTag(tagMap, 0UZ);
            }
            output.publish(1UZ);
        } else {
            output.publish(0UZ);
        }

        _bufFill = 0U;
        std::ignore = input.consume(consume);
        return gr::work::Status::OK;
    }

    /// Set an external channel-busy flag for listen-before-talk.
    void set_channel_busy_flag(std::atomic<bool>* flag) { _channelBusy = flag; }

    /// Compute CAD detection threshold per Vangelista & Calvagno (2022).
    /// α = sqrt(−(4/π)·ln(1−(1−p_fa)^{1/L})), where L = os_factor × M.
    /// Higher SF → more bins (M = 2^SF) → higher alpha for same P_fa.
    [[nodiscard]] static float compute_alpha(
            uint32_t sfVal, uint32_t osf, float p_fa = 0.001f) {
        const double M     = static_cast<double>(1U << sfVal);
        const double L     = static_cast<double>(osf) * M;
        const double inner = 1.0 - std::pow(1.0 - static_cast<double>(p_fa), 1.0 / L);
        return static_cast<float>(std::sqrt(-(4.0 / std::numbers::pi) * std::log(inner)));
    }

    /// Backward-compatible wrapper: compute_alpha with os_factor=1, P_fa=0.001.
    /// Returns 0 for SF outside [7, 12].
    [[nodiscard]] static float default_alpha(uint32_t sfVal) {
        if (sfVal < 7U || sfVal > 12U) return 0.F;
        return compute_alpha(sfVal, 1U, 0.001f);
    }

    /// Compute the best peak_ratio across os_factor sub-chip timing offsets.
    /// Testing entry-point; call start() before using directly.
    [[nodiscard]] float compute_peak_ratio(const cf32* samples, RefType refType) {
        const cf32* ref = (refType == RefType::UpchirpRef) ? _downRef.data() : _upRef.data();
        return peakRatio(samples, ref, _fftSize, static_cast<uint32_t>(os_factor),
                         _scratch, _fftOut, _fft).ratio;
    }

    /// Run full 2-window detection on a buffer of 2*_symLen oversampled samples.
    [[nodiscard]] DetectResult detect(const cf32* samples) {
        return detectTwoWindow(samples, _downRef.data(), _upRef.data(),
                               _symLen, _fftSize, static_cast<uint32_t>(os_factor),
                               static_cast<float>(alpha), static_cast<bool>(dual_chirp),
                               _scratch, _fftOut, _fft);
    }

    /// Detect chirp activity across SF7-12 on a shared SF12-sized buffer.
    /// Call initMultiSf() once before using this method.
    [[nodiscard]] MultiSfResult detectMultiSf(const cf32* samples) {
        assert(!_sfTable.empty() && "detectMultiSf called before initMultiSf()");
        MultiSfResult best;
        for (auto& sr : _sfTable) {
            const float thresh = compute_alpha(sr.sf, static_cast<uint32_t>(os_factor));
            const auto det = detectTwoWindow(
                samples, sr.downRef.data(), sr.upRef.data(),
                sr.symLen, sr.fftSize, static_cast<uint32_t>(os_factor),
                thresh, static_cast<bool>(dual_chirp),
                sr.scratch, sr.fftOut, sr.fft);
            if (!det.detected) { continue; }

            const float winRatio = std::max(det.peak_ratio_up, det.peak_ratio_dn);
            if (winRatio > best.best_ratio) {
                best.best_ratio    = winRatio;
                best.peak_ratio_up = det.peak_ratio_up;
                best.peak_ratio_dn = det.peak_ratio_dn;
                best.peak_bin_up   = det.peak_bin_up;
                best.peak_bin_dn   = det.peak_bin_dn;
                best.sf            = sr.sf;
                best.up_detected   = det.up_detected;
                best.dn_detected   = det.dn_detected;
                best.detected      = true;
            }
        }
        return best;
    }

    /// Pre-build reference chirps for SF7-12.  Must be called once before
    /// detectMultiSf().  Uses the block's current os_factor and dual_chirp.
    void initMultiSf() {
        _sfTable.clear();
        for (uint32_t trySf = 7; trySf <= 12; ++trySf) {
            _sfTable.push_back(buildSfRefs(trySf));
        }
    }

private:
    /// Core dechirp+FFT peak-ratio computation for one oversampled symbol window.
    /// Tries osf sub-chip timing offsets; returns the best ratio and its FFT bin.
    [[nodiscard]] static PeakResult peakRatio(
            const cf32* samples, const cf32* ref,
            uint32_t fftSz, uint32_t osf,
            std::vector<cf32>& scratch,
            std::vector<cf32>& fftOutBuf,
            gr::algorithm::FFT<cf32>& fft) {
        PeakResult best;
        for (uint32_t offset = 0; offset < osf; ++offset) {
            for (uint32_t idx = 0; idx < fftSz; ++idx) {
                scratch[idx] = samples[offset + idx * osf] * ref[idx];
            }
            fft.compute(std::span<const cf32>(scratch.data(), fftSz), fftOutBuf);

            float    peak = 0.F;
            float    total = 0.F;
            uint32_t peakIdx = 0;
            for (uint32_t i = 0; i < fftSz; ++i) {
                const float mag = std::abs(fftOutBuf[i]);
                total += mag;
                if (mag > peak) { peak = mag; peakIdx = i; }
            }
            const float mean  = total / static_cast<float>(fftSz);
            const float ratio = (mean > 0.F) ? (peak / mean) : 0.F;
            if (ratio > best.ratio) { best.ratio = ratio; best.bin = peakIdx; }
        }
        return best;
    }

    /// Run full 2-window detection on a buffer using provided workspace.
    [[nodiscard]] static DetectResult detectTwoWindow(
            const cf32* samples,
            const cf32* downRef, const cf32* upRef,
            uint32_t symLen, uint32_t fftSz, uint32_t osf,
            float thresh, bool dualChirp,
            std::vector<cf32>& scratch,
            std::vector<cf32>& fftOutBuf,
            gr::algorithm::FFT<cf32>& fft) {
        const cf32* w1 = samples;
        const cf32* w2 = samples + symLen;

        DetectResult result;
        const auto p1Up = peakRatio(w1, downRef, fftSz, osf, scratch, fftOutBuf, fft);
        const auto p2Up = peakRatio(w2, downRef, fftSz, osf, scratch, fftOutBuf, fft);
        result.peak_ratio_up = std::max(p1Up.ratio, p2Up.ratio);
        result.peak_bin_up   = (p1Up.ratio >= p2Up.ratio) ? p1Up.bin : p2Up.bin;
        result.up_detected   = (p1Up.ratio > thresh) && (p2Up.ratio > thresh);

        if (dualChirp) {
            const auto p1Dn = peakRatio(w1, upRef, fftSz, osf, scratch, fftOutBuf, fft);
            const auto p2Dn = peakRatio(w2, upRef, fftSz, osf, scratch, fftOutBuf, fft);
            result.peak_ratio_dn = std::max(p1Dn.ratio, p2Dn.ratio);
            result.peak_bin_dn   = (p1Dn.ratio >= p2Dn.ratio) ? p1Dn.bin : p2Dn.bin;
            result.dn_detected   = (p1Dn.ratio > thresh) && (p2Dn.ratio > thresh);
        }

        result.detected = result.up_detected || result.dn_detected;
        return result;
    }

    void rebuildRefs() {
        _fftSize = 1U << sf;
        _symLen  = _fftSize * os_factor;
        _winLen  = _symLen * 2U;

        _upRef.resize(_fftSize);
        _downRef.resize(_fftSize);
        build_ref_chirps(_upRef.data(), _downRef.data(),
                         static_cast<uint8_t>(sf), /*os_factor=*/1U);

        _scratch.resize(_fftSize);
        _fftOut.resize(_fftSize);
        _buf.assign(_winLen, {0.F, 0.F});
        _bufFill = 0U;
        _fft     = gr::algorithm::FFT<cf32>{};

        if (!_sfTable.empty()) { initMultiSf(); }
    }

    [[nodiscard]] SfRefs buildSfRefs(uint32_t trySf) const {
        SfRefs refs;
        refs.sf      = trySf;
        refs.fftSize = 1U << trySf;
        refs.symLen  = refs.fftSize * os_factor;
        refs.upRef.resize(refs.fftSize);
        refs.downRef.resize(refs.fftSize);
        build_ref_chirps(refs.upRef.data(), refs.downRef.data(),
                         static_cast<uint8_t>(trySf), /*os_factor=*/1U);
        refs.scratch.resize(refs.fftSize);
        refs.fftOut.resize(refs.fftSize);
        refs.fft = gr::algorithm::FFT<cf32>{};
        return refs;
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_CHANNEL_ACTIVITY_DETECTOR_HPP
