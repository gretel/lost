// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_BURST_DETECTOR_HPP
#define GNURADIO_LORA_BURST_DETECTOR_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <unordered_map>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/algorithm/utilities.hpp>

namespace gr::lora {

/// Detection thresholds — fixed by LoRa PHY signal processing requirements.
/// Not runtime-configurable, but named for readability and tuning experiments.
static constexpr int kPreambleBinTolerance = 1;  ///< max FFT bin drift between consecutive upchirps
static constexpr int kSyncWordBinTolerance = 2;  ///< max bin error for sync word (net ID) verification
static constexpr uint8_t kMaxAdditionalUpchirps = 3;  ///< extra preamble symbols before rollover

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

/// BurstDetector: IQ samples -> aligned, downsampled, CFO/STO-corrected symbol blocks.
///
/// Scans for LoRa preamble (consecutive upchirps), estimates CFO (integer +
/// fractional), STO (fractional), and SFO, then outputs aligned N-sample
/// symbol blocks until burst energy drops below threshold.
///
/// Tags each burst start with `{sf, cfo_int, cfo_frac, burst_start}`.
///
/// State machine:
///   DETECT -> look for preamble_len-3 consecutive upchirps
///   SYNC   -> estimate CFO/STO/SFO, verify sync word, align timing
///   OUTPUT -> emit aligned symbols until energy drops or max symbols reached
struct BurstDetector : gr::Block<BurstDetector, gr::NoDefaultTagForwarding> {
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
    uint32_t max_symbols  = 256;     ///< safety limit on symbols per burst

    GR_MAKE_REFLECTABLE(BurstDetector, in, out,
                        center_freq, bandwidth, sf, sync_word, os_factor, preamble_len,
                        energy_thresh, max_symbols);

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
    std::vector<std::complex<float>> _CFO_frac_correc;
    std::vector<std::complex<float>> _symb_corr;

    // Output state
    uint32_t _output_symb_cnt = 0;
    bool     _burst_tag_published = false;

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
    }

    void recalculate() {
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
        _cfo_frac_sto_frac_est = false;
        _additional_upchirps = 0;
        _sync_phase  = NET_ID1;
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

    /// Check if downsampled symbol has sufficient energy.
    [[nodiscard]] bool has_energy(const std::complex<float>* samples, uint32_t len) const {
        float energy = 0.f;
        for (uint32_t i = 0; i < len; i++) {
            energy += samples[i].real() * samples[i].real()
                    + samples[i].imag() * samples[i].imag();
        }
        return energy >= energy_thresh;
    }

    [[nodiscard]] gr::work::Status processBulk(
            gr::InputSpanLike auto& input,
            gr::OutputSpanLike auto& output) noexcept {
        auto in_span  = std::span(input);
        auto out_span = std::span(output);

        if (_N == 0) recalculate();

        if (out_span.size() < _N) {
            std::ignore = input.consume(0UZ);
            output.publish(0UZ);
            return gr::work::Status::OK;
        }

        if (in_span.size() < _sps) {
            std::ignore = input.consume(0UZ);
            output.publish(0UZ);
            return gr::work::Status::OK;
        }

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

        switch (_state) {
        case DETECT: {
            _bin_idx_new = static_cast<int32_t>(get_symbol_val(_in_down.data(), _downchirp.data()));

            if (!has_energy(_in_down.data(), _N)) {
                _symbol_cnt = 1;
                _bin_idx = -1;
                break;
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
                auto cfo_offset = static_cast<std::size_t>(
                    std::max(static_cast<int>(_N) - _k_hat, 0));
                if (cfo_offset + _up_symb_to_use * _N > _preamble_raw.size()) {
                    cfo_offset = 0;
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

                // Sync succeeded -- transition to OUTPUT
                _state = OUTPUT;
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
            if (!has_energy(_in_down.data(), _N)
                || _output_symb_cnt >= max_symbols) {
                resetToDetect();
                _items_to_consume = static_cast<int>(_sps);
                items_to_output = 0;
                break;
            }

            // Publish burst_start tag on first symbol
            if (!_burst_tag_published) {
                gr::property_map tag;
                tag["burst_start"]  = pmtv::pmt(true);
                tag["sf"]           = pmtv::pmt(static_cast<int64_t>(sf));
                tag["cfo_int"]      = pmtv::pmt(static_cast<int64_t>(_cfo_int));
                tag["cfo_frac"]     = pmtv::pmt(static_cast<double>(_cfo_frac));
                tag["is_downchirp"] = pmtv::pmt(false);
                this->publishTag(tag, 0UZ);
                _burst_tag_published = true;
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

#endif  // GNURADIO_LORA_BURST_DETECTOR_HPP
