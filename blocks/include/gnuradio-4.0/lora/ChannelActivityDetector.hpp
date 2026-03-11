// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_CHANNEL_ACTIVITY_DETECTOR_HPP
#define GNURADIO_LORA_CHANNEL_ACTIVITY_DETECTOR_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
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

    gr::PortIn<std::complex<float>>  in;
    gr::PortOut<uint8_t, gr::Async>  out;

    Annotated<uint32_t, "sf",         gr::Visible, gr::Limits<6U, 12U>>      sf        = 8U;
    Annotated<uint32_t, "bandwidth",  gr::Visible, gr::Limits<1U, 2000000U>> bandwidth = 62500U;
    Annotated<uint32_t, "os_factor",  gr::Visible, gr::Limits<1U, 16U>>      os_factor = 4U;
    Annotated<float,    "alpha",      gr::Visible> alpha      = 4.16f;  ///< SF-dependent threshold
    Annotated<bool,     "dual_chirp", gr::Visible> dual_chirp = true;   ///< also detect inverted-IQ
    bool debug = false;

    GR_MAKE_REFLECTABLE(ChannelActivityDetector, in, out,
                        sf, bandwidth, os_factor, alpha, dual_chirp, debug);

    explicit ChannelActivityDetector(gr::property_map init = {})
        : gr::Block<ChannelActivityDetector, gr::NoDefaultTagForwarding>(std::move(init)) {}

    /// Set an external channel-busy flag for listen-before-talk.
    /// Updated every 2-symbol detection window.  Not a GR4 property
    /// (atomics aren't serializable); call after graph construction.
    void set_channel_busy_flag(std::atomic<bool>* flag) { _channel_busy = flag; }

    /// Which chirp polarity to use as the dechirp reference.
    /// UpchirpRef  : multiply by conj(upchirp) = downchirp  → detects upchirp signal
    /// DownchirpRef: multiply by conj(downchirp) = upchirp  → detects downchirp signal
    enum class RefType { UpchirpRef, DownchirpRef };

    /// SF-dependent detection threshold (Vangelista & Calvagno 2022, P_fa = 1e-3).
    /// Returns 0 for SF < 7.
    static constexpr float default_alpha(uint32_t sf_val) {
        constexpr std::array<float, 13> kTable = {
            0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
            4.23f, 4.16f, 4.09f, 4.04f, 3.98f, 3.91f,
        };
        return (sf_val < kTable.size()) ? kTable[sf_val] : 0.f;
    }

    struct DetectResult {
        float peak_ratio_up{0.f};
        float peak_ratio_dn{0.f};
        bool  up_detected{false};
        bool  dn_detected{false};
        bool  detected{false};
    };

    /// Compute the best peak_ratio = max|Y|/mean|Y| across os_factor sub-chip
    /// timing offsets for one oversampled symbol window.
    ///
    /// `samples` must contain at least sym_len = N*os_factor elements.
    /// `ref_type` selects which 1x reference chirp is used for dechirping:
    ///   RefType::UpchirpRef   → conj(upchirp), used to detect upchirp signals
    ///   RefType::DownchirpRef → conj(downchirp) = upchirp, used to detect downchirps
    ///
    /// This is the primary testing entry-point: it does not modify the block's
    /// internal sample buffer.  Call start() before using this method directly.
    [[nodiscard]] float compute_peak_ratio(const std::complex<float>* samples,
                                           RefType ref_type) {
        const std::complex<float>* ref =
            (ref_type == RefType::UpchirpRef) ? _down_ref.data() : _up_ref.data();
        return peak_ratio_impl(samples, ref);
    }

    /// Run full 2-window detection on a buffer of 2*sym_len oversampled samples.
    /// Standalone entry-point for non-graph callers (e.g., lora_scan).
    /// Call start() before first use.
    [[nodiscard]] DetectResult detect(const std::complex<float>* samples) {
        const std::complex<float>* w1 = samples;
        const std::complex<float>* w2 = samples + _sym_len;
        const float thresh = static_cast<float>(alpha);

        DetectResult r;
        const float r1_up = peak_ratio_impl(w1, _down_ref.data());
        const float r2_up = peak_ratio_impl(w2, _down_ref.data());
        r.peak_ratio_up = std::max(r1_up, r2_up);
        r.up_detected   = (r1_up > thresh) && (r2_up > thresh);

        if (dual_chirp) {
            const float r1_dn = peak_ratio_impl(w1, _up_ref.data());
            const float r2_dn = peak_ratio_impl(w2, _up_ref.data());
            r.peak_ratio_dn = std::max(r1_dn, r2_dn);
            r.dn_detected   = (r1_dn > thresh) && (r2_dn > thresh);
        }

        r.detected = r.up_detected || r.dn_detected;
        return r;
    }

private:
    uint32_t _sym_len{0};  ///< N * os_factor: samples per oversampled symbol
    uint32_t _win_len{0};  ///< 2 * sym_len: samples in a full 2-symbol window
    uint32_t _N{0};        ///< 2^sf: FFT size / samples per 1x symbol

    /// 1x-rate reference chirps (length _N each).
    /// _up_ref   = upchirp(id=0, os=1)        used as ref for downchirp detection
    /// _down_ref = conj(upchirp) = downchirp  used as ref for upchirp detection
    std::vector<std::complex<float>> _up_ref{};
    std::vector<std::complex<float>> _down_ref{};

    std::vector<std::complex<float>> _buf{};      ///< accumulation (length _win_len)
    uint32_t                         _buf_fill{0};

    std::vector<std::complex<float>>           _scratch{};  ///< dechirp workspace (length _N)
    gr::algorithm::FFT<std::complex<float>>    _fft{};

    std::atomic<bool>* _channel_busy{nullptr};  ///< external LBT flag (nullable)

    void rebuild_refs() {
        _N       = 1U << sf;
        _sym_len = _N * os_factor;
        _win_len = _sym_len * 2U;

        _up_ref.resize(_N);
        _down_ref.resize(_N);
        build_ref_chirps(_up_ref.data(), _down_ref.data(),
                         static_cast<uint8_t>(sf), /*os_factor=*/1U);

        _scratch.resize(_N);
        _buf.assign(_win_len, {0.f, 0.f});
        _buf_fill = 0U;
        _fft      = gr::algorithm::FFT<std::complex<float>>{};

    }

    /// Core dechirp+FFT peak-ratio computation, no buffer side-effects.
    /// Tries os_factor sub-chip timing offsets; returns the best ratio found.
    [[nodiscard]] float peak_ratio_impl(const std::complex<float>* samples,
                                        const std::complex<float>* ref) {
        float best = 0.f;
        for (uint32_t l = 0; l < static_cast<uint32_t>(os_factor); ++l) {
            for (uint32_t m = 0; m < _N; ++m) {
                _scratch[m] = samples[l + m * os_factor] * ref[m];
            }
            auto fft_out = _fft.compute(
                std::span<const std::complex<float>>(_scratch.data(), _N));

            float peak = 0.f, total = 0.f;
            for (uint32_t i = 0; i < _N; ++i) {
                float mag = std::abs(fft_out[i]);
                total += mag;
                if (mag > peak) peak = mag;
            }
            float mean  = total / static_cast<float>(_N);
            float ratio = (mean > 0.f) ? (peak / mean) : 0.f;
            if (ratio > best) best = ratio;
        }
        return best;
    }

public:
    void start() { rebuild_refs(); }

    void settingsChanged(const gr::property_map& /*old*/, const gr::property_map& changed) {
        if (changed.contains("sf") || changed.contains("os_factor") || changed.contains("bandwidth")) {
            rebuild_refs();
        }
    }

    gr::work::Status processBulk(gr::InputSpanLike auto& input,
                                 gr::OutputSpanLike auto& output) noexcept {
        auto in_span  = std::span(input);
        auto out_span = std::span(output);

        // Accumulate into internal buffer until we have a full 2-symbol window.
        const uint32_t available = static_cast<uint32_t>(in_span.size());
        const uint32_t needed    = _win_len - _buf_fill;
        const uint32_t consume   = std::min(available, needed);

        std::copy_n(in_span.begin(), consume, _buf.begin() + static_cast<std::ptrdiff_t>(_buf_fill));
        _buf_fill += consume;

        if (_buf_fill < _win_len) {
            std::ignore = input.consume(consume);
            output.publish(0UZ);
            return gr::work::Status::OK;
        }

        const auto det = detect(_buf.data());

        // Update external LBT flag (if wired)
        if (_channel_busy != nullptr) {
            _channel_busy->store(det.detected, std::memory_order_release);
        }

        uint8_t result = 0U;
        if (det.up_detected) result |= 0x01U;
        if (det.dn_detected) result |= 0x02U;

        // Write result to output port if a consumer is connected (out_span
        // is non-empty).  When the output is unconnected (LBT-only mode),
        // the detection result is still available via the _channel_busy atomic.
        if (!out_span.empty()) {
            out_span[0] = result;
            if (det.detected) {
                gr::property_map tag_map;
                tag_map["cad_detected"]        = det.detected;
                tag_map["cad_upchirp"]         = det.up_detected;
                tag_map["cad_downchirp"]       = det.dn_detected;
                tag_map["cad_peak_ratio_up"]   = static_cast<double>(det.peak_ratio_up);
                tag_map["cad_peak_ratio_down"] = static_cast<double>(det.peak_ratio_dn);
                this->publishTag(tag_map, 0UZ);
            }
            output.publish(1UZ);
        } else {
            output.publish(0UZ);
        }

        _buf_fill = 0U;
        std::ignore = input.consume(consume);
        return gr::work::Status::OK;
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_CHANNEL_ACTIVITY_DETECTOR_HPP
