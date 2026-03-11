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
        float    peak_ratio_up{0.f};
        float    peak_ratio_dn{0.f};
        uint32_t peak_bin_up{0};    ///< FFT bin of best upchirp peak (0..N-1)
        uint32_t peak_bin_dn{0};    ///< FFT bin of best downchirp peak (0..N-1)
        bool     up_detected{false};
        bool     dn_detected{false};
        bool     detected{false};
    };

    /// Result of multi-SF detection on a shared buffer.
    struct MultiSfResult {
        float    best_ratio{0.f};       ///< max(peak_ratio_up, peak_ratio_dn) of winning SF
        float    peak_ratio_up{0.f};
        float    peak_ratio_dn{0.f};
        uint32_t peak_bin_up{0};
        uint32_t peak_bin_dn{0};
        uint32_t sf{0};                 ///< winning SF (0 = no detection)
        bool     up_detected{false};
        bool     dn_detected{false};
        bool     detected{false};
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
        return peak_ratio_impl(samples, ref).ratio;
    }

    /// Run full 2-window detection on a buffer of 2*sym_len oversampled samples.
    /// Standalone entry-point for non-graph callers (e.g., lora_scan).
    /// Call start() before first use.
    [[nodiscard]] DetectResult detect(const std::complex<float>* samples) {
        const std::complex<float>* w1 = samples;
        const std::complex<float>* w2 = samples + _sym_len;
        const float thresh = static_cast<float>(alpha);

        DetectResult r;
        const auto p1_up = peak_ratio_impl(w1, _down_ref.data());
        const auto p2_up = peak_ratio_impl(w2, _down_ref.data());
        r.peak_ratio_up = std::max(p1_up.ratio, p2_up.ratio);
        r.peak_bin_up   = (p1_up.ratio >= p2_up.ratio) ? p1_up.bin : p2_up.bin;
        r.up_detected   = (p1_up.ratio > thresh) && (p2_up.ratio > thresh);

        if (dual_chirp) {
            const auto p1_dn = peak_ratio_impl(w1, _up_ref.data());
            const auto p2_dn = peak_ratio_impl(w2, _up_ref.data());
            r.peak_ratio_dn = std::max(p1_dn.ratio, p2_dn.ratio);
            r.peak_bin_dn   = (p1_dn.ratio >= p2_dn.ratio) ? p1_dn.bin : p2_dn.bin;
            r.dn_detected   = (p1_dn.ratio > thresh) && (p2_dn.ratio > thresh);
        }

        r.detected = r.up_detected || r.dn_detected;
        return r;
    }

    /// Detect chirp activity across SF7-12 on a shared SF12-sized buffer.
    ///
    /// The buffer must contain at least `2 * (1<<12) * os_factor` samples at
    /// the target BW's sample rate.  Each SF trial reads only its required
    /// prefix (`2 * 2^sf * os_factor` samples).  Returns the SF with the
    /// highest peak ratio among those that pass the α threshold.
    ///
    /// Call `initMultiSf()` once before using this method (builds reference
    /// chirps for all 6 SFs).
    [[nodiscard]] MultiSfResult detectMultiSf(const std::complex<float>* samples) {
        MultiSfResult best;
        for (auto& sr : _sf_table) {
            const float thresh = default_alpha(sr.sf);
            const auto r = detect_with_refs(samples, sr, thresh);
            if (!r.detected) continue;

            const float winRatio = std::max(r.peak_ratio_up, r.peak_ratio_dn);
            if (winRatio > best.best_ratio) {
                best.best_ratio    = winRatio;
                best.peak_ratio_up = r.peak_ratio_up;
                best.peak_ratio_dn = r.peak_ratio_dn;
                best.peak_bin_up   = r.peak_bin_up;
                best.peak_bin_dn   = r.peak_bin_dn;
                best.sf            = sr.sf;
                best.up_detected   = r.up_detected;
                best.dn_detected   = r.dn_detected;
                best.detected      = true;
            }
        }
        return best;
    }

    /// Pre-build reference chirps for SF7-12.  Must be called once before
    /// `detectMultiSf()`.  Uses the block's current `os_factor` and
    /// `dual_chirp` settings.
    void initMultiSf() {
        _sf_table.clear();
        for (uint32_t trySf = 7; trySf <= 12; ++trySf) {
            _sf_table.push_back(build_sf_refs(trySf));
        }
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

    struct PeakResult {
        float    ratio{0.f};
        uint32_t bin{0};
    };

    /// Pre-built reference chirps and workspace for one SF (used by detectMultiSf).
    struct SfRefs {
        uint32_t sf{0};
        uint32_t N{0};         ///< 2^sf
        uint32_t sym_len{0};   ///< N * os_factor
        std::vector<std::complex<float>> up_ref{};    ///< upchirp 1x
        std::vector<std::complex<float>> down_ref{};  ///< downchirp 1x
        std::vector<std::complex<float>> scratch{};    ///< FFT workspace (length N)
        gr::algorithm::FFT<std::complex<float>> fft{};
    };

    std::vector<SfRefs> _sf_table{};  ///< SF7-12 refs (populated by initMultiSf)

    [[nodiscard]] SfRefs build_sf_refs(uint32_t trySf) const {
        SfRefs sr;
        sr.sf      = trySf;
        sr.N       = 1U << trySf;
        sr.sym_len = sr.N * os_factor;
        sr.up_ref.resize(sr.N);
        sr.down_ref.resize(sr.N);
        build_ref_chirps(sr.up_ref.data(), sr.down_ref.data(),
                         static_cast<uint8_t>(trySf), /*os_factor=*/1U);
        sr.scratch.resize(sr.N);
        sr.fft = gr::algorithm::FFT<std::complex<float>>{};
        return sr;
    }

    /// Run peak_ratio computation using an SfRefs' workspace (not the block's _scratch/_fft).
    [[nodiscard]] PeakResult peak_ratio_with_refs(
            const std::complex<float>* samples,
            SfRefs& sr,
            const std::complex<float>* ref) {
        PeakResult best;
        const auto os = static_cast<uint32_t>(os_factor);
        for (uint32_t l = 0; l < os; ++l) {
            for (uint32_t m = 0; m < sr.N; ++m) {
                sr.scratch[m] = samples[l + m * os] * ref[m];
            }
            auto fft_out = sr.fft.compute(
                std::span<const std::complex<float>>(sr.scratch.data(), sr.N));

            float    peak = 0.f, total = 0.f;
            uint32_t peak_idx = 0;
            for (uint32_t i = 0; i < sr.N; ++i) {
                float mag = std::abs(fft_out[i]);
                total += mag;
                if (mag > peak) { peak = mag; peak_idx = i; }
            }
            float mean  = total / static_cast<float>(sr.N);
            float ratio = (mean > 0.f) ? (peak / mean) : 0.f;
            if (ratio > best.ratio) { best.ratio = ratio; best.bin = peak_idx; }
        }
        return best;
    }

    /// Run full 2-window detection using pre-built SfRefs (for detectMultiSf).
    [[nodiscard]] DetectResult detect_with_refs(
            const std::complex<float>* samples,
            SfRefs& sr,
            float thresh) {
        const std::complex<float>* w1 = samples;
        const std::complex<float>* w2 = samples + sr.sym_len;

        DetectResult r;
        const auto p1_up = peak_ratio_with_refs(w1, sr, sr.down_ref.data());
        const auto p2_up = peak_ratio_with_refs(w2, sr, sr.down_ref.data());
        r.peak_ratio_up = std::max(p1_up.ratio, p2_up.ratio);
        r.peak_bin_up   = (p1_up.ratio >= p2_up.ratio) ? p1_up.bin : p2_up.bin;
        r.up_detected   = (p1_up.ratio > thresh) && (p2_up.ratio > thresh);

        if (dual_chirp) {
            const auto p1_dn = peak_ratio_with_refs(w1, sr, sr.up_ref.data());
            const auto p2_dn = peak_ratio_with_refs(w2, sr, sr.up_ref.data());
            r.peak_ratio_dn = std::max(p1_dn.ratio, p2_dn.ratio);
            r.peak_bin_dn   = (p1_dn.ratio >= p2_dn.ratio) ? p1_dn.bin : p2_dn.bin;
            r.dn_detected   = (p1_dn.ratio > thresh) && (p2_dn.ratio > thresh);
        }

        r.detected = r.up_detected || r.dn_detected;
        return r;
    }

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
    /// Tries os_factor sub-chip timing offsets; returns the best ratio and its bin.
    [[nodiscard]] PeakResult peak_ratio_impl(const std::complex<float>* samples,
                                             const std::complex<float>* ref) {
        PeakResult best;
        for (uint32_t l = 0; l < static_cast<uint32_t>(os_factor); ++l) {
            for (uint32_t m = 0; m < _N; ++m) {
                _scratch[m] = samples[l + m * os_factor] * ref[m];
            }
            auto fft_out = _fft.compute(
                std::span<const std::complex<float>>(_scratch.data(), _N));

            float    peak = 0.f, total = 0.f;
            uint32_t peak_idx = 0;
            for (uint32_t i = 0; i < _N; ++i) {
                float mag = std::abs(fft_out[i]);
                total += mag;
                if (mag > peak) { peak = mag; peak_idx = i; }
            }
            float mean  = total / static_cast<float>(_N);
            float ratio = (mean > 0.f) ? (peak / mean) : 0.f;
            if (ratio > best.ratio) { best.ratio = ratio; best.bin = peak_idx; }
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
