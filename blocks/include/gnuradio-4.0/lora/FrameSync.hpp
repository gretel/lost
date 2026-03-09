// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_FRAME_SYNC_HPP
#define GNURADIO_LORA_FRAME_SYNC_HPP

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <complex>
#include <cstdint>
#include <memory>
#include <numbers>
#include <unordered_map>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/SpectrumTap.hpp>
#include <gnuradio-4.0/lora/algorithm/crc.hpp>
#include <gnuradio-4.0/lora/algorithm/hamming.hpp>
#include <gnuradio-4.0/lora/algorithm/interleaving.hpp>
#include <gnuradio-4.0/lora/algorithm/utilities.hpp>

namespace gr::lora {

/// Detection thresholds — fixed by LoRa PHY signal processing requirements.
/// Not runtime-configurable, but named for readability and tuning experiments.
static constexpr int kPreambleBinTolerance = 1;  ///< max FFT bin drift between consecutive upchirps
static constexpr int kSyncWordBinTolerance = 2;  ///< max bin error for sync word (net ID) verification
static constexpr uint8_t kMaxAdditionalUpchirps = 3;  ///< extra preamble symbols before rollover
static constexpr uint8_t kMaxPreambleRotations  = 4;  ///< max buffer rotations after extra upchirps

namespace detail {

/// Find the most frequent value in a vector.
[[nodiscard]] inline int most_frequent(const std::vector<int>& arr) {
    std::unordered_map<int, int> freq;
    for (int v : arr) {
        freq[v]++;
    }
    int max_count = 0, result = -1;
    for (auto& [val, cnt] : freq) {
        if (cnt > max_count) {
            max_count = cnt;
            result = val;
        }
    }
    return result;
}

/// Symmetric rounding (round half away from zero).
[[nodiscard]] inline int my_roundf(float x) noexcept {
    return x > 0 ? static_cast<int>(x + 0.5f)
                 : static_cast<int>(std::ceil(x - 0.5f));
}

/// Element-wise complex multiply: out[i] = a[i] * b[i].
inline void complex_multiply(std::complex<float>* out, const std::complex<float>* a,
                             const std::complex<float>* b, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) out[i] = a[i] * b[i];
}

}  // namespace detail

/// FrameSync: IQ samples -> aligned, downsampled, CFO/STO-corrected symbol blocks.
///
/// Scans for LoRa preamble (consecutive upchirps), estimates CFO (integer +
/// fractional), STO (fractional), and SFO, then outputs aligned N-sample
/// symbol blocks until burst energy drops below threshold.
///
/// Tags each burst start with `{sf, cfo_int, cfo_frac, burst_start, snr_db, rx_channel}`.
///
/// State machine:
///   DETECT -> look for preamble_len-3 consecutive upchirps
///   SYNC   -> estimate CFO/STO/SFO, verify sync word, align timing
///   OUTPUT -> emit aligned symbols until energy drops or max symbols reached
GR_REGISTER_BLOCK("gr::lora::FrameSync", gr::lora::FrameSync)
struct FrameSync : gr::Block<FrameSync, gr::NoDefaultTagForwarding> {
    gr::PortIn<std::complex<float>>  in;
    gr::PortOut<std::complex<float>> out;

    // Configuration parameters
    uint32_t center_freq  = 869618000;
    uint32_t bandwidth    = 62500;
    uint8_t  sf           = 8;
    uint16_t sync_word    = 0x12;
    uint8_t  os_factor    = 4;
    uint16_t preamble_len = 8;
    float    energy_thresh = 1e-6f;  ///< minimum symbol energy to continue output
    float    min_snr_db   = -10.0f;  ///< reject bursts below this SNR (dB)
    uint32_t max_symbols  = 600;     ///< safety limit (worst case: SF7/CR4/255B = 600 symbols)
    int32_t  rx_channel   = -1;      ///< RX channel index (-1 = not set)
    bool     debug        = false;

    GR_MAKE_REFLECTABLE(FrameSync, in, out,
                        center_freq, bandwidth, sf, sync_word, os_factor, preamble_len,
                        energy_thresh, min_snr_db, max_symbols, rx_channel, debug);

    // --- Internal state ---
    enum State : uint8_t { DETECT, SYNC, OUTPUT };
    enum SyncPhase : int32_t {
        NET_ID1 = 0, NET_ID2 = 1, DOWNCHIRP1 = 2, DOWNCHIRP2 = 3, QUARTER_DOWN = 4
    };

    State     _state = DETECT;

    uint32_t _N   = 0;   ///< 2^sf
    uint32_t _sps = 0;   ///< samples per symbol = N * os_factor

    // Sync word split into net ID symbol values
    uint16_t _sw0 = 0;
    uint16_t _sw1 = 0;

    // Reference chirps (at os_factor=1, length N)
    std::vector<std::complex<float>> _upchirp;
    std::vector<std::complex<float>> _downchirp;

    // Detect state
    uint32_t _n_up_req    = 0;     ///< preamble_len - 3
    uint32_t _up_symb_to_use = 0;  ///< n_up_req - 1
    int32_t  _symbol_cnt  = 1;
    int32_t  _bin_idx     = -1;
    int32_t  _bin_idx_new = -1;
    int      _k_hat       = 0;
    std::vector<int> _preamb_up_vals;

    // Preamble sample storage
    std::vector<std::complex<float>> _in_down;
    std::vector<std::complex<float>> _preamble_raw;
    std::vector<std::complex<float>> _preamble_raw_up;
    std::vector<std::complex<float>> _preamble_upchirps;
    std::vector<std::complex<float>> _net_id_samp;

    // Sync state
    int32_t  _sync_phase  = NET_ID1;
    bool     _cfo_frac_sto_frac_est = false;
    float    _cfo_frac  = 0.f;
    int      _cfo_int   = 0;
    float    _sto_frac  = 0.f;
    float    _sfo_hat   = 0.f;
    float    _sfo_cum   = 0.f;
    int      _down_val  = 0;
    uint8_t  _additional_upchirps = 0;
    uint8_t  _preamble_rotations  = 0;  ///< rotation count after max additional upchirps
    std::vector<std::complex<float>> _CFO_frac_correc;
    std::vector<std::complex<float>> _symb_corr;

    // Output state
    uint32_t _output_symb_cnt = 0;
    bool     _burst_tag_published = false;
    float    _snr_db    = 0.f;    ///< FFT peak-vs-rest SNR estimate (dB)
    float    _signal_db = -999.f; ///< mean signal power in dBFS, -999 = not yet estimated

    // Header decode in OUTPUT state (for exact symbol count)
    std::vector<std::complex<float>> _header_symbols;  ///< first 8 symbols (8*N samples)
    uint32_t _frame_symb_numb = 0;  ///< computed from header (0 = not yet known)
    bool     _header_decoded  = false;

    // Stale-burst watchdog: consecutive processBulk calls with insufficient input
    uint32_t _idle_call_count = 0;

    // Noise floor estimation (EMA of mean symbol power during DETECT idle)
    float    _noise_floor_db = -999.f;  ///< dBFS, -999 = not yet estimated
    float    _noise_ema      = 0.f;     ///< exponential moving average of mean |x|^2
    bool     _noise_ema_init = false;   ///< true after first update

    // Peak amplitude estimation (EMA of max |x|^2 per raw input span)
    float    _peak_db      = -999.f;    ///< dBFS, -999 = not yet estimated
    float    _peak_ema     = 0.f;       ///< exponential moving average of max |x|^2
    bool     _peak_ema_init = false;    ///< true after first update

    // Spectrum tap: shared state for waterfall display (optional, set by app)
    std::shared_ptr<SpectrumState> _spectrum_state;

    // Periodic noise floor logging (debug mode)
    uint64_t           _nf_sample_cnt   = 0;       ///< samples processed since last log
    uint64_t           _nf_log_interval = 0;       ///< samples per log interval (set in start)
    std::vector<float> _nf_history;                 ///< recent noise floor dB values for median (capped)
    static constexpr std::size_t kMaxNfHistory = 1024;  ///< max entries (~2.8 hours at 10s interval)

    // Pre-allocated scratch buffers (avoid per-call heap allocation)
    std::vector<std::complex<float>> _scratch_N;      ///< N-element scratch for dechirp
    std::vector<std::complex<float>> _scratch_2N;     ///< 2N-element scratch
    std::vector<float>               _scratch_2N_f;   ///< 2N-element float scratch
    std::vector<std::complex<float>> _corr_vec;       ///< preamble-length correction vector
    std::vector<std::complex<float>> _fft_val_buf;    ///< CFO estimation FFT values

    // FFT engine
    gr::algorithm::FFT<std::complex<float>> _fft;

    // Items to consume tracking
    int _items_to_consume = 0;

    void start() {
        recalculate();
        _nf_log_interval = static_cast<uint64_t>(bandwidth) * os_factor * 10;
        _nf_sample_cnt = 0;
        _nf_history.clear();
        _nf_history.reserve(1024);
    }

    void settingsChanged(const gr::property_map& /*oldSettings*/,
                         const gr::property_map& newSettings) {
        // PHY parameters require full recalculation (rebuilds chirp tables,
        // resizes buffers, resets state machine to DETECT).
        static constexpr std::array kPhyKeys = {
            "sf", "bandwidth", "os_factor", "preamble_len", "sync_word"
        };
        bool needRecalc = std::ranges::any_of(kPhyKeys, [&](const char* k) {
            return newSettings.contains(k);
        });
        if (needRecalc && _N > 0) {
            recalculate();
            _nf_log_interval = static_cast<uint64_t>(bandwidth) * os_factor * 10;
            if (debug) {
                std::fprintf(stderr, "[FrameSync] settingsChanged: recalculated"
                    " (sf=%u, bw=%u, os=%u, preamble=%u, sync=0x%02X)\n",
                    sf, bandwidth, os_factor, preamble_len, sync_word);
            }
        }
        // Hot-reconfig parameters (center_freq, energy_thresh, min_snr_db,
        // max_symbols, debug, rx_channel) are already updated by the
        // framework before this callback — no action needed.
    }

    void recalculate() {
        assert(sf >= 7 && sf <= 12 && "SF must be in [7, 12]");
        assert(bandwidth > 0 && "bandwidth must be > 0");
        assert(os_factor >= 1 && "os_factor must be >= 1");
        assert(preamble_len >= 5 && "preamble_len must be >= 5");

        _N   = 1u << sf;
        _sps = _N * os_factor;

        // min_samples for EOS detection: scheduler terminates when remaining < min_samples
        in.min_samples = _sps + 2u * os_factor;

        _sw0 = static_cast<uint16_t>(((sync_word & 0xF0) >> 4) << 3);
        _sw1 = static_cast<uint16_t>((sync_word & 0x0F) << 3);

        _upchirp.resize(_N);
        _downchirp.resize(_N);
        build_ref_chirps(_upchirp.data(), _downchirp.data(), sf);

        _n_up_req       = preamble_len >= 3 ? static_cast<uint32_t>(preamble_len - 3) : 0u;
        _up_symb_to_use = _n_up_req >= 1 ? _n_up_req - 1 : 0u;
        _preamb_up_vals.resize(_n_up_req, 0);

        _in_down.resize(_N);
        _preamble_raw.resize(static_cast<std::size_t>(preamble_len) * _N);
        _preamble_raw_up.resize(static_cast<std::size_t>(preamble_len + 3) * _sps);
        _preamble_upchirps.resize(static_cast<std::size_t>(preamble_len) * _N);
        _net_id_samp.resize(static_cast<std::size_t>(_sps * 2 + _sps / 2));

        _CFO_frac_correc.resize(_N);
        _symb_corr.resize(_N);
        _scratch_N.resize(_N);
        _scratch_2N.resize(2 * _N);
        _scratch_2N_f.resize(2 * _N);
        auto max_corr = static_cast<std::size_t>(
            _n_up_req + kMaxAdditionalUpchirps) * _N;
        _corr_vec.resize(max_corr);
        _fft_val_buf.resize(static_cast<std::size_t>(_up_symb_to_use) * _N);
        _header_symbols.resize(8UZ * _N);

        resetToDetect();

        _fft = gr::algorithm::FFT<std::complex<float>>{};
    }

    void resetToDetect() {
        _state       = DETECT;
        _symbol_cnt  = 1;
        _bin_idx     = -1;
        _bin_idx_new = -1;
        _k_hat       = 0;
        _sto_frac    = 0.f;
        _sfo_cum     = 0.f;
        _output_symb_cnt = 0;
        _burst_tag_published = false;
        _signal_db = -999.f;
        _frame_symb_numb = 0;
        _header_decoded  = false;
        _cfo_frac_sto_frac_est = false;
        _additional_upchirps = 0;
        _preamble_rotations  = 0;
        _sync_phase  = NET_ID1;
        _idle_call_count = 0;
    }

    /// Dechirp + FFT -> argmax (uses pre-allocated _scratch_N).
    [[nodiscard]] uint32_t get_symbol_val(const std::complex<float>* samples,
                                          const std::complex<float>* ref_chirp) {
        return dechirp_argmax(samples, ref_chirp, _scratch_N.data(), _N, _fft);
    }

    /// CFO fractional estimation using Bernier's algorithm.
    /// Uses pre-allocated _scratch_N, _fft_val_buf, _corr_vec.
    float estimate_CFO_frac_Bernier(const std::complex<float>* samples) {
        const auto n_syms = _up_symb_to_use;

        // Per-symbol peak index and magnitude (small, stack-friendly)
        float k0_best_mag = 0.f;
        uint32_t idx_max = 0;

        for (uint32_t i = 0; i < n_syms; i++) {
            detail::complex_multiply(_scratch_N.data(), &samples[_N * i], _downchirp.data(), _N);
            auto fft_out = _fft.compute(
                std::span<const std::complex<float>>(_scratch_N.data(), _N));

            float best = 0.f;
            int best_idx = 0;
            for (uint32_t j = 0; j < _N; j++) {
                float mag_sq = fft_out[j].real() * fft_out[j].real()
                             + fft_out[j].imag() * fft_out[j].imag();
                _fft_val_buf[j + i * _N] = fft_out[j];
                if (mag_sq > best) {
                    best = mag_sq;
                    best_idx = static_cast<int>(j);
                }
            }
            if (best > k0_best_mag) {
                k0_best_mag = best;
                idx_max = static_cast<uint32_t>(best_idx);
            }
        }

        std::complex<float> four_cum(0.f, 0.f);
        for (uint32_t i = 0; i + 1 < n_syms; i++) {
            four_cum += _fft_val_buf[idx_max + _N * i]
                      * std::conj(_fft_val_buf[idx_max + _N * (i + 1)]);
        }
        float cfo_frac = -std::arg(four_cum) / (2.f * static_cast<float>(std::numbers::pi));

        // Correct CFO in preamble using _corr_vec
        for (uint32_t n = 0; n < n_syms * _N; n++) {
            float phase = -2.f * static_cast<float>(std::numbers::pi) * cfo_frac
                        / static_cast<float>(_N) * static_cast<float>(n);
            _corr_vec[n] = std::complex<float>(std::cos(phase), std::sin(phase));
        }
        detail::complex_multiply(_preamble_upchirps.data(), samples, _corr_vec.data(),
                                 n_syms * _N);

        return cfo_frac;
    }

    /// STO fractional estimation using zero-padded FFT with RCTSL interpolation.
    /// Uses pre-allocated _scratch_N (dechirp), _scratch_2N (FFT input), _scratch_2N_f (mag accum).
    float estimate_STO_frac() {
        const auto n_syms = _up_symb_to_use;

        // Zero the accumulator
        std::fill(_scratch_2N_f.begin(), _scratch_2N_f.end(), 0.f);

        for (uint32_t i = 0; i < n_syms; i++) {
            detail::complex_multiply(_scratch_N.data(), &_preamble_upchirps[_N * i],
                                     _downchirp.data(), _N);

            std::copy_n(_scratch_N.begin(), _N, _scratch_2N.begin());
            std::fill(_scratch_2N.begin() + _N, _scratch_2N.end(),
                      std::complex<float>(0.f, 0.f));

            auto fft_out = _fft.compute(
                std::span<const std::complex<float>>(_scratch_2N.data(), 2 * _N));

            for (uint32_t j = 0; j < 2 * _N; j++) {
                _scratch_2N_f[j] += fft_out[j].real() * fft_out[j].real()
                                  + fft_out[j].imag() * fft_out[j].imag();
            }
        }

        auto max_it = std::max_element(_scratch_2N_f.begin(), _scratch_2N_f.end());
        auto k0 = static_cast<uint32_t>(std::distance(_scratch_2N_f.begin(), max_it));

        // RCTSL interpolation
        auto wrap = [N2 = 2LL * static_cast<int64_t>(_N)](int64_t i) -> std::size_t {
            return static_cast<std::size_t>(mod(i, N2));
        };
        double Y_1 = static_cast<double>(_scratch_2N_f[wrap(static_cast<int64_t>(k0) - 1)]);
        double Y0  = static_cast<double>(_scratch_2N_f[k0]);
        double Y1  = static_cast<double>(_scratch_2N_f[wrap(static_cast<int64_t>(k0) + 1)]);

        double u = 64.0 * _N / 406.5506497;
        double v = u * 2.4674;
        double wa = (Y1 - Y_1) / (u * (Y1 + Y_1) + v * Y0);
        double ka = wa * _N / std::numbers::pi;
        double k_residual = std::fmod((k0 + ka) / 2.0, 1.0);
        float sto_frac = static_cast<float>(k_residual - (k_residual > 0.5 ? 1.0 : 0.0));

        return sto_frac;
    }

    /// Preamble-based SNR estimation (GR3 determine_snr algorithm).
    /// Dechirps each preamble symbol, takes FFT, computes peak-vs-rest ratio.
    /// Must be called after CFO/SFO correction (_preamble_upchirps is corrected).
    struct SnrResult {
        float snr_db    = 0.f;    ///< FFT peak-vs-rest (in-band SNR)
        float signal_db = -999.f; ///< signal power in dBFS (FFT peak normalised by N²)
    };

    [[nodiscard]] SnrResult determine_snr() {
        const auto n_syms = _up_symb_to_use;
        if (n_syms == 0) return {};

        float snr_sum    = 0.f;
        float signal_sum = 0.f;
        uint32_t signal_cnt = 0;
        const float n_sq = static_cast<float>(_N) * static_cast<float>(_N);

        for (uint32_t i = 0; i < n_syms; i++) {
            detail::complex_multiply(_scratch_N.data(), &_preamble_upchirps[_N * i],
                                     _downchirp.data(), _N);
            auto fft_out = _fft.compute(
                std::span<const std::complex<float>>(_scratch_N.data(), _N));

            float total_energy = 0.f;
            float peak_energy  = 0.f;
            for (uint32_t j = 0; j < _N; j++) {
                float mag_sq = fft_out[j].real() * fft_out[j].real()
                             + fft_out[j].imag() * fft_out[j].imag();
                total_energy += mag_sq;
                if (mag_sq > peak_energy) peak_energy = mag_sq;
            }
            float noise_energy = total_energy - peak_energy;
            if (noise_energy > 0.f) {
                snr_sum += 10.f * std::log10(peak_energy / noise_energy);
            }
            if (peak_energy > 0.f) {
                signal_sum += 10.f * std::log10(peak_energy / n_sq);
                signal_cnt++;
            }
        }
        const float inv_n = 1.f / static_cast<float>(n_syms);
        float sig_db = (signal_cnt > 0)
            ? signal_sum / static_cast<float>(signal_cnt)
            : -999.f;
        return { snr_sum * inv_n, sig_db };
    }

    /// Check if downsampled symbol has sufficient energy.
    [[nodiscard]] bool has_energy(const std::complex<float>* samples, uint32_t len) const {
        float energy = 0.f;
        for (uint32_t i = 0; i < len; i++) {
            energy += samples[i].real() * samples[i].real()
                    + samples[i].imag() * samples[i].imag();
        }
        return energy >= energy_thresh;
    }

    /// Decode the LoRa explicit header from the first 8 buffered output symbols.
    /// Sets _frame_symb_numb to the exact number of symbols for the frame.
    /// Mirrors DemodDecoder's header decode: dechirp→FFT→gray→deinterleave→hamming→parse.
    void decodeHeader() {
        _header_decoded = true;

        // Build CFO-corrected downchirp (same as DemodDecoder::buildDownchirpWithCFO)
        std::vector<std::complex<float>> dc(_N);
        {
            std::vector<std::complex<float>> uc(_N);
            build_upchirp(uc.data(),
                          static_cast<uint32_t>(mod(_cfo_int, static_cast<int64_t>(_N))),
                          sf, 1);
            for (uint32_t n = 0; n < _N; n++) {
                dc[n] = std::conj(uc[n]);
            }
            float frac_scale = -2.f * static_cast<float>(std::numbers::pi)
                              * _cfo_frac / static_cast<float>(_N);
            for (uint32_t n = 0; n < _N; n++) {
                float phase = frac_scale * static_cast<float>(n);
                dc[n] *= std::complex<float>(std::cos(phase), std::sin(phase));
            }
        }

        // Dechirp + FFT + argmax for each of the 8 header symbols
        std::vector<uint16_t> symbols;
        symbols.reserve(8);
        for (uint32_t i = 0; i < 8; i++) {
            auto idx = dechirp_argmax(&_header_symbols[static_cast<std::size_t>(i) * _N],
                                      dc.data(), _scratch_N.data(), _N, _fft);
            auto sym = static_cast<uint16_t>(
                mod(static_cast<int64_t>(idx) - 1, static_cast<int64_t>(_N)));
            symbols.push_back(sym);
        }

        // Gray map + reduce to sf_app = sf - 2 bits (header rate)
        const uint8_t sf_app = static_cast<uint8_t>(sf - 2);
        std::vector<uint16_t> gray_syms;
        gray_syms.reserve(8);
        for (auto s : symbols) {
            gray_syms.push_back(static_cast<uint16_t>((s ^ (s >> 1)) >> 2));
        }

        // Deinterleave + Hamming decode → nibbles
        auto codewords = deinterleave_block(gray_syms, sf, 8, sf_app);
        if (codewords.size() < 5) return;  // shouldn't happen

        std::vector<uint8_t> nibbles;
        nibbles.reserve(codewords.size());
        for (auto cw : codewords) {
            nibbles.push_back(hamming_decode_hard(cw, 4));
        }

        auto info = parse_explicit_header(nibbles[0], nibbles[1], nibbles[2],
                                           nibbles[3], nibbles[4]);
        if (!info.checksum_valid || info.payload_len == 0) {
            if (debug) {
                std::fprintf(stderr, "[FrameSync] header decode failed: checksum=%s pay_len=%u",
                             info.checksum_valid ? "OK" : "FAIL", info.payload_len);
                if (rx_channel >= 0) std::fprintf(stderr, "  ch=%d", rx_channel);
                std::fprintf(stderr, "\n");
            }
            return;  // fall back to max_symbols safety cap
        }

        // Compute LDRO (same as DemodDecoder)
        bool ldro = (static_cast<float>(1u << sf) * 1e3f
                    / static_cast<float>(bandwidth)) > LDRO_MAX_DURATION_MS;

        _frame_symb_numb = 8 + static_cast<uint32_t>(
            std::ceil(static_cast<double>(
                2 * info.payload_len - sf + 2 + 5 + (info.has_crc ? 4 : 0))
                / (sf - 2 * static_cast<int>(ldro))))
            * (4 + info.cr);

        if (debug) {
            std::fprintf(stderr, "[FrameSync] header: pay_len=%u cr=%u has_crc=%d -> %u symbols",
                         info.payload_len, info.cr, info.has_crc, _frame_symb_numb);
            if (rx_channel >= 0) std::fprintf(stderr, "  ch=%d", rx_channel);
            std::fprintf(stderr, "\n");
        }
    }

    [[nodiscard]] gr::work::Status processBulk(
            gr::InputSpanLike auto& input,
            gr::OutputSpanLike auto& output) {
        auto in_span  = std::span(input);
        auto out_span = std::span(output);

        if (_N == 0) recalculate();

        if (out_span.size() < _N) {
            std::ignore = input.consume(0UZ);
            output.publish(0UZ);
            return gr::work::Status::OK;
        }

        if (in_span.size() < _sps) {
            _idle_call_count++;
            if (_idle_call_count > 100 && _state != DETECT) {
                if (debug) {
                    std::fprintf(stderr, "[FrameSync] stale burst reset after %u idle calls (state=%d)\n",
                                 _idle_call_count, static_cast<int>(_state));
                }
                resetToDetect();
            }
            std::ignore = input.consume(0UZ);
            output.publish(0UZ);
            return gr::work::Status::OK;
        }
        _idle_call_count = 0;

        int items_to_output = 0;
        _items_to_consume = static_cast<int>(_sps);

        // Downsample: pick every os_factor-th sample with STO correction
        const int os = static_cast<int>(os_factor);
        for (uint32_t ii = 0; ii < _N; ii++) {
            int idx = os / 2 + static_cast<int>(os_factor * ii)
                      - detail::my_roundf(_sto_frac * static_cast<float>(os_factor));
            if (idx < 0) idx += static_cast<int>(_sps);
            if (idx >= static_cast<int>(in_span.size())) idx = static_cast<int>(in_span.size()) - 1;
            _in_down[ii] = in_span[static_cast<std::size_t>(idx)];
        }

        // Push raw IQ to spectrum tap (if connected) — runs in ALL states
        // so the waterfall keeps scrolling during frame decode
        if (_spectrum_state) {
            const auto raw_len = std::min(in_span.size(),
                                          static_cast<std::size_t>(_sps));
            _spectrum_state->push(in_span.data(), raw_len);
        }

        switch (_state) {
        case DETECT: {
            _bin_idx_new = static_cast<int32_t>(get_symbol_val(_in_down.data(), _downchirp.data()));

            // Update noise floor EMA from idle symbols (no preamble building)
            if (_symbol_cnt <= 1) {
                float mean_power = 0.f;
                for (uint32_t i = 0; i < _N; i++) {
                    mean_power += _in_down[i].real() * _in_down[i].real()
                                + _in_down[i].imag() * _in_down[i].imag();
                }
                mean_power /= static_cast<float>(_N);
                constexpr float kNoiseEmaAlpha = 0.05f;  // ~20-symbol time constant
                if (!_noise_ema_init) {
                    _noise_ema = mean_power;
                    _noise_ema_init = true;
                } else {
                    _noise_ema += kNoiseEmaAlpha * (mean_power - _noise_ema);
                }
                if (_noise_ema > 0.f) {
                    _noise_floor_db = 10.f * std::log10(_noise_ema);
                }

                // Peak amplitude EMA: scan raw input for max |x|^2
                {
                    float max_sq = 0.f;
                    const auto raw_len = std::min(in_span.size(), static_cast<std::size_t>(_sps));
                    for (std::size_t i = 0; i < raw_len; i++) {
                        float sq = in_span[i].real() * in_span[i].real()
                                 + in_span[i].imag() * in_span[i].imag();
                        if (sq > max_sq) max_sq = sq;
                    }
                    if (!_peak_ema_init) {
                        _peak_ema = max_sq;
                        _peak_ema_init = true;
                    } else {
                        _peak_ema += kNoiseEmaAlpha * (max_sq - _peak_ema);
                    }
                    if (_peak_ema > 0.f) {
                        _peak_db = 10.f * std::log10(_peak_ema);
                    }
                }

                // Periodic noise floor log (debug mode, every ~10s)
                _nf_sample_cnt += _sps;
                if (debug && _nf_log_interval > 0 && _noise_floor_db > -999.f) {
                    if (_nf_sample_cnt >= _nf_log_interval) {
                        _nf_sample_cnt = 0;
                        if (_nf_history.size() >= kMaxNfHistory) {
                            _nf_history.erase(_nf_history.begin());
                        }
                        _nf_history.push_back(_noise_floor_db);
                        // Compute median via nth_element (O(n), no full sort)
                        auto tmp = _nf_history;
                        auto mid = tmp.begin() + static_cast<std::ptrdiff_t>(tmp.size() / 2);
                        std::nth_element(tmp.begin(), mid, tmp.end());
                        float median = *mid;
                        std::fprintf(stderr,
                            "[FrameSync] noise floor: %.1f dBFS  peak: %.1f dBFS  median: %.1f dBFS  (n=%zu)",
                            static_cast<double>(_noise_floor_db),
                            static_cast<double>(_peak_db),
                            static_cast<double>(median),
                            _nf_history.size());
                        if (rx_channel >= 0) std::fprintf(stderr, "  ch=%d", rx_channel);
                        std::fprintf(stderr, "\n");
                    }
                }
            }

            if (!has_energy(_in_down.data(), _N)) {
                _symbol_cnt = 1;
                _bin_idx = -1;
                break;
            }

            if (debug && _symbol_cnt > 2) {
                std::fprintf(stderr, "[FrameSync] DETECT: cnt=%d bin=%d new=%d",
                             _symbol_cnt, _bin_idx, _bin_idx_new);
                if (rx_channel >= 0) std::fprintf(stderr, "  ch=%d", rx_channel);
                std::fprintf(stderr, "\n");
            }

            // Look for consecutive reference upchirps (within +/-kPreambleBinTolerance)
            if (std::abs(mod(std::abs(_bin_idx_new - _bin_idx) + 1, static_cast<int64_t>(_N)) - 1)
                    <= kPreambleBinTolerance && _bin_idx_new != -1) {
                if (_symbol_cnt == 1 && _bin_idx != -1) {
                    _preamb_up_vals[0] = _bin_idx;
                }

                auto sc = static_cast<uint32_t>(_symbol_cnt);
                if (sc < _preamb_up_vals.size()) {
                    _preamb_up_vals[sc] = _bin_idx_new;
                }

                // Store raw preamble samples
                if (static_cast<std::size_t>(sc) * _N < _preamble_raw.size()) {
                    std::copy_n(_in_down.data(), _N, &_preamble_raw[static_cast<std::size_t>(sc) * _N]);
                }
                if (static_cast<std::size_t>(sc) * _sps + _sps <= _preamble_raw_up.size()) {
                    std::copy_n(&in_span[os_factor / 2], _sps,
                                &_preamble_raw_up[static_cast<std::size_t>(sc) * _sps]);
                }

                _symbol_cnt++;
            } else {
                // Reset -- keep this symbol as start of potential new preamble
                std::copy_n(_in_down.data(), _N, &_preamble_raw[0]);
                if (_sps + os_factor / 2 <= in_span.size()) {
                    std::copy_n(&in_span[os_factor / 2], _sps, &_preamble_raw_up[0]);
                }
                _symbol_cnt = 1;
            }

            _bin_idx = _bin_idx_new;

            if (static_cast<uint32_t>(_symbol_cnt) == _n_up_req) {
                if (debug) {
                    std::fprintf(stderr, "[FrameSync] DETECT->SYNC: k_hat=%d", _k_hat);
                    if (rx_channel >= 0) std::fprintf(stderr, "  ch=%d", rx_channel);
                    std::fprintf(stderr, "\n");
                }
                _additional_upchirps = 0;
                _state = SYNC;
                _sync_phase = NET_ID1;
                _symbol_cnt = 0;
                _cfo_frac_sto_frac_est = false;
                _k_hat = detail::most_frequent(_preamb_up_vals);

                // Copy net_id samples
                int src_start = static_cast<int>(_sps * 3 / 4) - _k_hat * os;
                for (uint32_t i = 0; i < _sps / 4; i++) {
                    int src_idx = src_start + static_cast<int>(i);
                    src_idx = static_cast<int>(mod(static_cast<int64_t>(src_idx),
                                                   static_cast<int64_t>(in_span.size())));
                    _net_id_samp[i] = in_span[static_cast<std::size_t>(src_idx)];
                }

                _items_to_consume = os * static_cast<int>(_N - static_cast<uint32_t>(_k_hat));
            } else {
                _items_to_consume = static_cast<int>(_sps);
            }
            items_to_output = 0;
            break;
        }

        case SYNC: {
            items_to_output = 0;

            if (!_cfo_frac_sto_frac_est) {
                // Skip slot 0 in _preamble_raw: it contains a non-preamble
                // sample from the DETECT reset path (line 541).  Start from
                // slot 1 (_N) which is the first correctly-stored upchirp.
                auto cfo_offset = static_cast<std::size_t>(_N)
                    + static_cast<std::size_t>(std::max(static_cast<int>(_N) - _k_hat, 0));
                if (cfo_offset + _up_symb_to_use * _N > _preamble_raw.size()) {
                    cfo_offset = static_cast<std::size_t>(_N);
                }
                _cfo_frac = estimate_CFO_frac_Bernier(&_preamble_raw[cfo_offset]);
                _sto_frac = estimate_STO_frac();

                for (uint32_t n = 0; n < _N; n++) {
                    float phase = -2.f * static_cast<float>(std::numbers::pi) * _cfo_frac
                                / static_cast<float>(_N) * static_cast<float>(n);
                    _CFO_frac_correc[n] = std::complex<float>(std::cos(phase), std::sin(phase));
                }
                _cfo_frac_sto_frac_est = true;
            }

            _items_to_consume = static_cast<int>(_sps);

            // Apply CFO frac correction to downsampled input
            detail::complex_multiply(_symb_corr.data(), _in_down.data(),
                                     _CFO_frac_correc.data(), _N);
            int32_t bin_idx = static_cast<int32_t>(
                get_symbol_val(_symb_corr.data(), _downchirp.data()));

            switch (_sync_phase) {
            case NET_ID1: {
                if (bin_idx == 0 || bin_idx == 1
                    || static_cast<uint32_t>(bin_idx) == _N - 1) {
                    // Additional upchirp
                    auto ns_start = static_cast<std::size_t>(_sps * 3 / 4);
                    uint32_t copy_len = std::min(_sps / 4,
                                                 static_cast<uint32_t>(_net_id_samp.size()));
                    for (uint32_t i = 0; i < copy_len; i++) {
                        auto idx = ns_start + i;
                        if (idx < in_span.size())
                            _net_id_samp[i] = in_span[idx];
                    }

                    if (_additional_upchirps >= kMaxAdditionalUpchirps) {
                        if (_preamble_rotations >= kMaxPreambleRotations) {
                            // Expected %u+%u upchirp-like symbols but never saw
                            // the sync word. Common cause: preamble detected but
                            // SNR too low to resolve the sync word offset (single
                            // symbol, no coherent integration).
                            if (debug) {
                                std::fprintf(stderr,
                                    "[FrameSync] NET_ID1: no sync word after %u+%u extra upchirp-like symbols "
                                    "(bin=%d), SNR too low? resetting",
                                    static_cast<unsigned>(kMaxAdditionalUpchirps),
                                    static_cast<unsigned>(_preamble_rotations),
                                    bin_idx);
                                if (rx_channel >= 0) std::fprintf(stderr, "  ch=%d", rx_channel);
                                std::fprintf(stderr, "\n");
                            }
                            resetToDetect();
                            _items_to_consume = 0;
                            items_to_output = 0;
                            break;
                        }
                        std::rotate(_preamble_raw_up.begin(),
                                    _preamble_raw_up.begin() + static_cast<std::ptrdiff_t>(_sps),
                                    _preamble_raw_up.end());
                        std::size_t dst = static_cast<std::size_t>(_n_up_req + 3) * _sps;
                        if (dst + _sps <= _preamble_raw_up.size()) {
                            auto src_off = static_cast<std::size_t>(
                                os_factor / 2 + static_cast<uint32_t>(_k_hat) * os_factor);
                            std::copy_n(&in_span[src_off], _sps,
                                        &_preamble_raw_up[dst]);
                        }
                        _preamble_rotations++;
                    } else {
                        std::size_t dst = static_cast<std::size_t>(
                            _n_up_req + _additional_upchirps) * _sps;
                        if (dst + _sps <= _preamble_raw_up.size()) {
                            auto src_off = static_cast<std::size_t>(
                                os_factor / 2 + static_cast<uint32_t>(_k_hat) * os_factor);
                            std::copy_n(&in_span[src_off], _sps,
                                        &_preamble_raw_up[dst]);
                        }
                        _additional_upchirps++;
                    }
                } else {
                    _sync_phase = NET_ID2;
                    std::size_t dst_off = _sps / 4;
                    std::size_t copy_len = std::min(static_cast<std::size_t>(_sps),
                                                     _net_id_samp.size() - dst_off);
                    copy_len = std::min(copy_len, in_span.size());
                    std::copy_n(in_span.data(), copy_len, &_net_id_samp[dst_off]);
                }
                break;
            }
            case NET_ID2: {
                _sync_phase = DOWNCHIRP1;
                std::size_t dst_off = _sps + _sps / 4;
                std::size_t avail = _net_id_samp.size() > dst_off
                                  ? _net_id_samp.size() - dst_off : 0;
                std::size_t copy_len = std::min(
                    static_cast<std::size_t>((_N + 1) * os_factor), avail);
                copy_len = std::min(copy_len, in_span.size());
                std::copy_n(in_span.data(), copy_len, &_net_id_samp[dst_off]);
                break;
            }
            case DOWNCHIRP1: {
                std::size_t dst_off = _sps * 2 + _sps / 4;
                std::size_t avail = _net_id_samp.size() > dst_off
                                  ? _net_id_samp.size() - dst_off : 0;
                std::size_t copy_len = std::min(
                    static_cast<std::size_t>(_sps / 4), avail);
                copy_len = std::min(copy_len, in_span.size());
                if (copy_len > 0)
                    std::copy_n(in_span.data(), copy_len, &_net_id_samp[dst_off]);
                _sync_phase = DOWNCHIRP2;
                break;
            }
            case DOWNCHIRP2: {
                _down_val = static_cast<int>(
                    get_symbol_val(_symb_corr.data(), _upchirp.data()));
                _sync_phase = QUARTER_DOWN;
                break;
            }
            case QUARTER_DOWN: {
                // Integer CFO from downchirp
                if (static_cast<uint32_t>(_down_val) < _N / 2) {
                    _cfo_int = _down_val / 2;
                } else {
                    _cfo_int = (_down_val - static_cast<int>(_N)) / 2;
                }

                // Correct STOint and CFOint in preamble upchirps
                auto rot_off = static_cast<std::size_t>(
                    mod(static_cast<int64_t>(_cfo_int), static_cast<int64_t>(_N)));
                std::rotate(_preamble_upchirps.begin(),
                            _preamble_upchirps.begin() + static_cast<std::ptrdiff_t>(rot_off),
                            _preamble_upchirps.end());

                uint32_t n_corr_syms = _n_up_req + _additional_upchirps;
                std::size_t corr_len = static_cast<std::size_t>(n_corr_syms) * _N;
                for (std::size_t n = 0; n < corr_len; n++) {
                    float phase = -2.f * static_cast<float>(std::numbers::pi)
                                * static_cast<float>(_cfo_int)
                                / static_cast<float>(_N) * static_cast<float>(n);
                    _corr_vec[n] = std::complex<float>(std::cos(phase), std::sin(phase));
                }
                detail::complex_multiply(_preamble_upchirps.data(), _preamble_upchirps.data(),
                                         _corr_vec.data(), _up_symb_to_use * _N);

                // SFO estimation
                _sfo_hat = (static_cast<float>(_cfo_int) + _cfo_frac)
                           * static_cast<float>(bandwidth)
                           / static_cast<float>(center_freq);

                // SFO correction in preamble
                double clk_off = static_cast<double>(_sfo_hat) / _N;
                double fs   = static_cast<double>(bandwidth);
                double fs_p = static_cast<double>(bandwidth) * (1.0 - clk_off);
                int N_int = static_cast<int>(_N);

                for (std::size_t n = 0; n < corr_len; n++) {
                    double n_mod = static_cast<double>(
                        mod(static_cast<int64_t>(n), static_cast<int64_t>(N_int)));
                    double floor_n_N = std::floor(static_cast<double>(n) / N_int);
                    double phase = -2.0 * std::numbers::pi * (
                        n_mod * n_mod / (2.0 * N_int)
                        * (bandwidth / fs_p * bandwidth / fs_p
                           - bandwidth / fs * bandwidth / fs)
                        + (floor_n_N * (bandwidth / fs_p * bandwidth / fs_p
                                       - bandwidth / fs_p)
                           + bandwidth / 2.0 * (1.0 / fs - 1.0 / fs_p)) * n_mod);
                    _corr_vec[n] = std::complex<float>(
                        static_cast<float>(std::cos(phase)),
                        static_cast<float>(std::sin(phase)));
                }
                detail::complex_multiply(_preamble_upchirps.data(), _preamble_upchirps.data(),
                                         _corr_vec.data(), _up_symb_to_use * _N);

                // Re-estimate STO frac after SFO correction
                float tmp_sto_frac = estimate_STO_frac();
                float diff_sto_frac = _sto_frac - tmp_sto_frac;
                float os_thresh = static_cast<float>(os_factor - 1u)
                                / static_cast<float>(os_factor);
                if (std::abs(diff_sto_frac) <= os_thresh) {
                    _sto_frac = tmp_sto_frac;
                }

                // Update sto_frac to its value at the net_id
                _sto_frac += _sfo_hat * static_cast<float>(preamble_len);
                if (std::abs(_sto_frac) > 0.5f) {
                    _sto_frac += (_sto_frac > 0 ? -1.f : 1.f);
                }

                // Decimate and correct net_id samples for sync word verification
                // Reuse _scratch_2N for decimated net ID samples
                std::fill(_scratch_2N.begin(), _scratch_2N.end(),
                          std::complex<float>(0.f, 0.f));
                int start_off = os / 2
                              - detail::my_roundf(_sto_frac * static_cast<float>(os_factor))
                              + os * (static_cast<int>(_N / 4) + _cfo_int);
                for (uint32_t i = 0; i < _N * 2; i++) {
                    int idx = start_off + static_cast<int>(i) * os;
                    if (idx >= 0 && static_cast<std::size_t>(idx) < _net_id_samp.size()) {
                        _scratch_2N[i] = _net_id_samp[static_cast<std::size_t>(idx)];
                    }
                }

                // Apply CFO_int correction (reuse _corr_vec for phasor)
                for (uint32_t n = 0; n < 2 * _N; n++) {
                    float phase = -2.f * static_cast<float>(std::numbers::pi)
                                * static_cast<float>(_cfo_int)
                                / static_cast<float>(_N) * static_cast<float>(n);
                    _corr_vec[n] = std::complex<float>(std::cos(phase), std::sin(phase));
                }
                detail::complex_multiply(_scratch_2N.data(), _scratch_2N.data(),
                                         _corr_vec.data(), 2 * _N);

                // Apply CFO_frac correction
                detail::complex_multiply(_scratch_2N.data(), _scratch_2N.data(),
                                         _CFO_frac_correc.data(), _N);
                detail::complex_multiply(&_scratch_2N[_N], &_scratch_2N[_N],
                                         _CFO_frac_correc.data(), _N);

                int netid1 = static_cast<int>(
                    get_symbol_val(_scratch_2N.data(), _downchirp.data()));
                int netid2 = static_cast<int>(
                    get_symbol_val(&_scratch_2N[_N], _downchirp.data()));

                // Sync word verification
                bool sync_ok = false;
                int net_id_off = 0;
                if (_sw0 == 0) {
                    sync_ok = true;
                    _items_to_consume = 0;
                } else if (std::abs(netid1 - static_cast<int>(_sw0)) > kSyncWordBinTolerance) {
                    // Wrong net ID 1
                } else {
                    net_id_off = netid1 - static_cast<int>(_sw0);
                    if (_sw1 != 0 && mod(netid2 - net_id_off, static_cast<int64_t>(_N))
                        != static_cast<int64_t>(_sw1)) {
                        // Wrong net ID 2
                    } else {
                        sync_ok = true;
                        _items_to_consume = -os * net_id_off;
                    }
                }

                if (!sync_ok) {
                    resetToDetect();
                    _items_to_consume = 0;
                    items_to_output = 0;
                    break;
                }

                auto [snr, sig] = determine_snr();
                _snr_db    = snr;
                _signal_db = sig;
                if (_snr_db < min_snr_db) {
                    if (debug) {
                        std::fprintf(stderr, "[FrameSync] SYNC rejected: snr=%.1f dB < min %.1f dB",
                                     static_cast<double>(_snr_db), static_cast<double>(min_snr_db));
                        if (rx_channel >= 0) std::fprintf(stderr, "  ch=%d", rx_channel);
                        std::fprintf(stderr, "\n");
                    }
                    resetToDetect();
                    _items_to_consume = 0;
                    items_to_output = 0;
                    break;
                }
                _state = OUTPUT;
                if (debug) {
                    std::fprintf(stderr, "[FrameSync] SYNC->OUTPUT: cfo_int=%d cfo_frac=%.3f sto_frac=%.3f snr=%.1f dB noise=%.1f dBFS",
                                 _cfo_int, static_cast<double>(_cfo_frac),
                                 static_cast<double>(_sto_frac), static_cast<double>(_snr_db),
                                 static_cast<double>(_noise_floor_db));
                    if (rx_channel >= 0) std::fprintf(stderr, "  ch=%d", rx_channel);
                    std::fprintf(stderr, "\n");
                }
                _output_symb_cnt = 0;
                _burst_tag_published = false;

                // Update sto_frac to payload beginning
                _sto_frac += _sfo_hat * 4.25f;
                _sfo_cum = ((_sto_frac * static_cast<float>(os_factor))
                           - static_cast<float>(detail::my_roundf(
                               _sto_frac * static_cast<float>(os_factor))))
                          / static_cast<float>(os_factor);

                _items_to_consume += static_cast<int>(_sps / 4)
                                   + os * _cfo_int;
                break;
            }
            default:
                break;
            }
            break;
        }

        case OUTPUT: {
            // Termination: header-derived exact count, or max_symbols safety cap
            uint32_t symb_limit = (_frame_symb_numb > 0) ? _frame_symb_numb : max_symbols;
            if (!has_energy(_in_down.data(), _N)
                || _output_symb_cnt >= symb_limit) {
                if (debug) {
                    std::fprintf(stderr, "[FrameSync] OUTPUT->DETECT: %u/%u symbols emitted",
                                 _output_symb_cnt, symb_limit);
                    if (rx_channel >= 0) std::fprintf(stderr, "  ch=%d", rx_channel);
                    std::fprintf(stderr, "\n");
                }
                resetToDetect();
                _items_to_consume = static_cast<int>(_sps);
                items_to_output = 0;
                break;
            }

            if (!_burst_tag_published) {
                gr::property_map tag;
                tag["burst_start"]  = gr::pmt::Value(true);
                tag["sf"]           = gr::pmt::Value(static_cast<int64_t>(sf));
                tag["cfo_int"]      = gr::pmt::Value(static_cast<int64_t>(_cfo_int));
                tag["cfo_frac"]     = gr::pmt::Value(static_cast<double>(_cfo_frac));
                tag["is_downchirp"] = gr::pmt::Value(false);
                tag["snr_db"]       = gr::pmt::Value(static_cast<double>(_snr_db));
                if (_noise_floor_db > -999.f) {
                    tag["noise_floor_db"] = gr::pmt::Value(static_cast<double>(_noise_floor_db));
                }
                if (_peak_db > -999.f) {
                    tag["peak_db"] = gr::pmt::Value(static_cast<double>(_peak_db));
                }
                if (_signal_db > -999.f && _noise_floor_db > -999.f) {
                    tag["snr_db_td"] = gr::pmt::Value(
                        static_cast<double>(_signal_db - _noise_floor_db));
                }
                if (rx_channel >= 0) {
                    tag["rx_channel"] = gr::pmt::Value(static_cast<int64_t>(rx_channel));
                }
                this->publishTag(tag, 0UZ);
                _burst_tag_published = true;
            }

            // Buffer first 8 symbols for header decode
            if (_output_symb_cnt < 8 && !_header_decoded) {
                std::copy_n(_in_down.data(), _N,
                            &_header_symbols[static_cast<std::size_t>(_output_symb_cnt) * _N]);
            }

            // Output downsampled signal
            std::size_t copy_len = std::min(static_cast<std::size_t>(_N), out_span.size());
            std::copy_n(_in_down.data(), copy_len, out_span.data());
            _items_to_consume = static_cast<int>(_sps);

            // SFO tracking
            if (std::abs(_sfo_cum) > 1.0f / (2.0f * static_cast<float>(os_factor))) {
                int sign_val = (_sfo_cum > 0) ? 1 : -1;
                _items_to_consume -= sign_val;
                _sfo_cum -= static_cast<float>(sign_val) / static_cast<float>(os_factor);
            }
            _sfo_cum += _sfo_hat;

            items_to_output = static_cast<int>(_N);
            _output_symb_cnt++;

            // After 8th symbol: decode header to learn exact frame length
            if (_output_symb_cnt == 8 && !_header_decoded) {
                decodeHeader();
            }
            break;
        }
        }  // switch

        // Clamp consume to available
        std::size_t consume_n = static_cast<std::size_t>(std::max(_items_to_consume, 0));
        consume_n = std::min(consume_n, in_span.size());
        std::ignore = input.consume(consume_n);
        output.publish(static_cast<std::size_t>(std::max(items_to_output, 0)));

        return gr::work::Status::OK;
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_FRAME_SYNC_HPP
