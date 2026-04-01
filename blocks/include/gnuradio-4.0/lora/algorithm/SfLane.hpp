// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_ALGORITHM_SFLANE_HPP
#define GNURADIO_LORA_ALGORITHM_SFLANE_HPP

// Per-SF processing lane: fused detect/sync/decode state machine.
// Extracted from MultiSfDecoder.hpp to enable reuse by WidebandDecoder
// without GR_REGISTER_BLOCK include conflicts.
//
// Dependencies: algorithm headers only (no Block.hpp, no BlockRegistry.hpp).

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/algorithm/GrayPartition.hpp>
#include <gnuradio-4.0/lora/algorithm/SfLaneDetail.hpp>
#include <gnuradio-4.0/lora/algorithm/crc.hpp>
#include <gnuradio-4.0/lora/algorithm/hamming.hpp>
#include <gnuradio-4.0/lora/algorithm/interleaving.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>
#include <gnuradio-4.0/lora/algorithm/utilities.hpp>

namespace gr::lora {

// Alias for backward compatibility with call sites using multisf_detail::
namespace multisf_detail = sflane_detail;

/// Per-SF processing lane — holds the complete fused detect/sync/decode
/// state machine for one spreading factor.
struct SfLane {
    // === Configuration (set once in init) ===
    uint8_t  sf{};
    uint32_t N{};           // 2^sf
    uint32_t sps{};         // N * os_factor
    uint32_t os_factor{};
    uint32_t bandwidth{};
    uint16_t preamble_len{};

    // Sync word split into net ID symbol values
    uint16_t sw0{};
    uint16_t sw1{};

    // Preamble detection counts
    uint32_t n_up_req{};       // preamble_len - 3
    uint32_t up_symb_to_use{}; // n_up_req - 1

    // === Chirp tables ===
    std::vector<std::complex<float>> upchirp;     // length N
    std::vector<std::complex<float>> downchirp;   // length N
    std::vector<std::complex<float>> cfo_downchirp; // CFO-corrected, rebuilt per frame

    // === FFT engine + scratch ===
    gr::algorithm::FFT<std::complex<float>> fft;
    std::vector<std::complex<float>> scratch_N;    // length N
    std::vector<std::complex<float>> scratch_2N;   // length 2*N (used for net_id decimation)
    std::vector<std::complex<float>> corr_vec;     // preamble correction vector
    std::vector<std::complex<float>> fft_val_buf;  // CFO estimation FFT values

    // === detect/sync state ===
    enum State : uint8_t { DETECT, SYNC, OUTPUT };
    enum SyncPhase : int32_t {
        NET_ID1 = 0, NET_ID2 = 1, DOWNCHIRP1 = 2,
        DOWNCHIRP2 = 3, QUARTER_DOWN = 4
    };

    State      state{DETECT};
    SyncPhase  sync_phase{NET_ID1};
    int32_t    symbol_cnt{1};
    int32_t    bin_idx{-1};
    int32_t    bin_idx_new{-1};
    int        k_hat{0};
    float      cfo_frac{0.f};
    int        cfo_int{0};
    float      sto_frac{0.f};
    float      sfo_hat{0.f};
    float      sfo_cum{0.f};
    float      snr_db{0.f};
    float      signal_db{-999.f};
    uint32_t   output_symb_cnt{0};
    uint32_t   frame_symb_numb{0};
    bool       header_decoded{false};
    uint8_t    additional_upchirps{0};
    uint8_t    preamble_rotations{0};
    bool       cfo_frac_sto_frac_est{false};
    int        items_to_consume{0};
    uint32_t   stall_count{0};   // symbols since entering SYNC/OUTPUT without frame completion
    static constexpr uint32_t kMaxStallSymbols = 1000;  // safety timeout (must exceed max frame symbols)

    // Accumulation buffer: stores samples from the ring buffer that this
    // lane hasn't processed yet. The ring buffer cursor always advances at
    // _min_sps rate; slow lanes (high SF) accumulate here until they have
    // enough for one symbol period.
    std::vector<std::complex<float>> accum;

    // === Soft decode ===
    bool use_soft_decode{false};  // runtime toggle (config.toml: soft_decode = true/false)
    static constexpr float kPmrConfidenceThreshold = 4.0f;  // below this, scale LLR by 0.5

    GrayPartition gray_partition;
    // Pre-allocated soft decode buffers (avoid per-symbol heap allocation)
    std::vector<float>  _soft_mag_sq;       // size N, reused per symbol
    std::vector<float>  _soft_sym_mag_sq;   // size N, reused per symbol (remapped)
    std::vector<double> _soft_llr_buf;      // size sf, reused per symbol
    // Flat LLR storage: _soft_llr_flat[symbol_idx * sf + bit_idx]
    std::vector<double> _soft_llr_flat;
    size_t              _soft_llr_count{0}; // number of symbols stored
    // Per-symbol PMR values (for confidence scaling)
    std::vector<float> soft_symbol_pmrs;

    // === Preamble buffers ===
    std::vector<int>  preamb_up_vals;
    std::vector<std::complex<float>> in_down;          // length N (downsampled input)
    std::vector<std::complex<float>> preamble_raw;     // length preamble_len * N
    std::vector<std::complex<float>> preamble_raw_up;  // length (preamble_len+3) * sps
    std::vector<std::complex<float>> preamble_upchirps; // length preamble_len * N
    std::vector<std::complex<float>> net_id_samp;      // length 2.5 * sps
    std::vector<std::complex<float>> CFO_frac_correc;  // length N
    std::vector<std::complex<float>> symb_corr;        // length N

    // === Header decode (in OUTPUT state) ===
    std::vector<std::complex<float>> header_symbols; // first 8 symbols, 8*N samples
    int        down_val{0};

    // === decode state ===
    std::vector<std::complex<float>> dechirped;     // length N
    std::vector<uint16_t> symbol_buffer;
    uint32_t total_symbols_rx{0};
    uint8_t  cr{4};
    uint32_t pay_len{0};
    bool     has_crc{true};
    bool     ldro{false};
    uint32_t symb_numb{0};
    std::vector<uint8_t> nibbles;

    void init(uint8_t sf_, uint32_t bandwidth_, uint8_t os_factor_,
              uint16_t preamble_len_, uint16_t sync_word, bool soft_decode = false) {
        sf = sf_;
        N  = 1u << sf;
        os_factor = os_factor_;
        sps = N * os_factor;
        bandwidth = bandwidth_;
        preamble_len = preamble_len_;
        use_soft_decode = soft_decode;

        sw0 = static_cast<uint16_t>(((sync_word & 0xF0) >> 4) << 3);
        sw1 = static_cast<uint16_t>((sync_word & 0x0F) << 3);

        n_up_req       = preamble_len >= 3 ? static_cast<uint32_t>(preamble_len - 3) : 0u;
        up_symb_to_use = n_up_req >= 1 ? n_up_req - 1 : 0u;

        upchirp.resize(N);
        downchirp.resize(N);
        build_ref_chirps(upchirp.data(), downchirp.data(), sf);

        preamb_up_vals.resize(n_up_req, 0);
        in_down.resize(N);
        preamble_raw.resize(static_cast<std::size_t>(preamble_len) * N);
        preamble_raw_up.resize(static_cast<std::size_t>(preamble_len + 3) * sps);
        preamble_upchirps.resize(static_cast<std::size_t>(preamble_len) * N);
        net_id_samp.resize(static_cast<std::size_t>(sps * 2 + sps / 2));
        CFO_frac_correc.resize(N);
        symb_corr.resize(N);
        scratch_N.resize(N);
        scratch_2N.resize(2 * N);
        auto max_corr = static_cast<std::size_t>(n_up_req + multisf_detail::kMaxAdditionalUpchirps) * N;
        corr_vec.resize(max_corr);
        fft_val_buf.resize(static_cast<std::size_t>(up_symb_to_use) * N);
        header_symbols.resize(8UZ * N);
        cfo_downchirp.resize(N);
        dechirped.resize(N);

        accum.clear();
        if (use_soft_decode) {
            gray_partition.init(sf);
            _soft_mag_sq.resize(N);
            _soft_sym_mag_sq.resize(N);
            _soft_llr_buf.resize(sf);
            _soft_llr_flat.reserve(256 * sf);  // pre-reserve for typical frame
            soft_symbol_pmrs.reserve(256);
        }
        fft = gr::algorithm::FFT<std::complex<float>>{};
        resetToDetect();
    }

    void resetToDetect() {
        state       = DETECT;
        symbol_cnt  = 1;
        bin_idx     = -1;
        bin_idx_new = -1;
        k_hat       = 0;
        sto_frac    = 0.f;
        sfo_cum     = 0.f;
        output_symb_cnt = 0;
        signal_db = -999.f;
        frame_symb_numb = 0;
        header_decoded  = false;
        cfo_frac_sto_frac_est = false;
        additional_upchirps = 0;
        preamble_rotations  = 0;
        sync_phase  = NET_ID1;
        items_to_consume = 0;

        // Reset decode state
        symbol_buffer.clear();
        nibbles.clear();
        total_symbols_rx = 0;
        symb_numb = 0;
        cr = 4;
        pay_len = 0;
        stall_count = 0;
        _soft_llr_flat.clear();
        _soft_llr_count = 0;
        soft_symbol_pmrs.clear();
    }

    // === DSP helper methods ===

    [[nodiscard]] uint32_t get_symbol_val(const std::complex<float>* samples,
                                           const std::complex<float>* ref_chirp) {
        return dechirp_argmax(samples, ref_chirp, scratch_N.data(), N, fft);
    }

    [[nodiscard]] bool has_energy(const std::complex<float>* samples, uint32_t len,
                                   float energy_thresh) const {
        float energy = 0.f;
        for (uint32_t i = 0; i < len; i++) {
            energy += samples[i].real() * samples[i].real()
                    + samples[i].imag() * samples[i].imag();
        }
        return energy >= energy_thresh;
    }

    /// CFO fractional estimation using peak-bin cross-symbol correlation.
    /// Stores per-symbol complex FFT values in fft_val_buf for later use.
    float estimate_CFO_frac(const std::complex<float>* samples) {
        const auto n_syms = up_symb_to_use;
        float k0_best_mag = 0.f;
        uint32_t idx_max = 0;

        for (uint32_t i = 0; i < n_syms; i++) {
            multisf_detail::complex_multiply(scratch_N.data(), &samples[N * i], downchirp.data(), N);
            auto fft_out = fft.compute(
                std::span<const std::complex<float>>(scratch_N.data(), N));

            float best = 0.f;
            int best_idx = 0;
            for (uint32_t j = 0; j < N; j++) {
                float mag_sq = fft_out[j].real() * fft_out[j].real()
                             + fft_out[j].imag() * fft_out[j].imag();
                fft_val_buf[j + i * N] = fft_out[j];
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

        // Peak-bin cross-symbol correlation for CFO estimation.
        // Multi-bin (Xhonneux Eq. 14) introduces systematic phase bias due to
        // STO-dependent phase structure across chirp dechirp bins — reverted to
        // single peak bin which is unbiased for any STO.
        std::complex<float> four_cum(0.f, 0.f);
        for (uint32_t i = 0; i + 1 < n_syms; i++) {
            four_cum += fft_val_buf[idx_max + N * i]
                      * std::conj(fft_val_buf[idx_max + N * (i + 1)]);
        }
        float cfo_f = -std::arg(four_cum) / (2.f * static_cast<float>(std::numbers::pi));

        // Correct CFO in preamble
        for (uint32_t n = 0; n < n_syms * N; n++) {
            float phase = -2.f * static_cast<float>(std::numbers::pi) * cfo_f
                        / static_cast<float>(N) * static_cast<float>(n);
            corr_vec[n] = std::complex<float>(std::cos(phase), std::sin(phase));
        }
        multisf_detail::complex_multiply(preamble_upchirps.data(), samples, corr_vec.data(),
                                  n_syms * N);

        return cfo_f;
    }

    /// Backward-compatible alias.
    float estimate_CFO_frac_Bernier(const std::complex<float>* samples) {
        return estimate_CFO_frac(samples);
    }

    /// Fractional STO estimation using Xhonneux Eq. 20 (phase-corrected
    /// 3-bin interpolation on complex DFT values).
    ///
    /// Uses `k_hat` (integer STO from DETECT) to set M̂ = N − k_hat.
    /// Averages complex bins {i-1, i, i+1} over `up_symb_to_use` preamble
    /// chirps for noise reduction, then applies the closed-form estimator.
    float estimate_STO_frac() {
        const auto n_syms = up_symb_to_use;
        if (n_syms == 0) return 0.f;

        // Peak bin from preamble detection (integer STO estimate)
        uint32_t pk = static_cast<uint32_t>(std::max(0, k_hat)) % N;

        // Accumulate complex FFT bins {pk-1, pk, pk+1} across preamble symbols
        std::complex<double> Y_m1(0.0, 0.0), Y_0(0.0, 0.0), Y_p1(0.0, 0.0);

        for (uint32_t i = 0; i < n_syms; i++) {
            multisf_detail::complex_multiply(scratch_N.data(), &preamble_upchirps[N * i],
                                      downchirp.data(), N);
            auto fft_out = fft.compute(
                std::span<const std::complex<float>>(scratch_N.data(), N));

            auto to_d = [](std::complex<float> c) {
                return std::complex<double>(static_cast<double>(c.real()),
                                            static_cast<double>(c.imag()));
            };
            Y_m1 += to_d(fft_out[(pk + N - 1) % N]);
            Y_0  += to_d(fft_out[pk]);
            Y_p1 += to_d(fft_out[(pk + 1) % N]);
        }

        // Xhonneux Eq. 20: λ̂_STO = −Re[ (e^{−j2πM̂/N}·Y_{i+1} − e^{j2πM̂/N}·Y_{i−1})
        //                              / (2·Y_i − e^{−j2πM̂/N}·Y_{i+1} − e^{j2πM̂/N}·Y_{i−1}) ]
        // where M̂ = N − L̂_STO, and L̂_STO = k_hat (integer STO from preamble peak)
        uint32_t M_hat = (N - pk) % N;
        double phase = 2.0 * std::numbers::pi * static_cast<double>(M_hat)
                     / static_cast<double>(N);
        auto e_pos = std::complex<double>(std::cos(phase),  std::sin(phase));  // exp(+j2πM̂/N)
        auto e_neg = std::complex<double>(std::cos(phase), -std::sin(phase));  // exp(-j2πM̂/N)

        auto num = e_neg * Y_p1 - e_pos * Y_m1;
        auto den = 2.0 * Y_0 - e_neg * Y_p1 - e_pos * Y_m1;

        if (std::abs(den) < 1e-15) return 0.f;

        float lambda_STO = static_cast<float>(-std::real(num / den));
        return std::clamp(lambda_STO, -0.5f, 0.5f);
    }

    /// Overload accepting explicit integer STO (for re-estimation after CFO_int correction).
    float estimate_STO_frac(uint32_t L_STO_int) {
        int saved = k_hat;
        k_hat = static_cast<int>(L_STO_int);
        float result = estimate_STO_frac();
        k_hat = saved;
        return result;
    }

    /// Mod-8 sync-word-independent STO refinement.
    /// Sync word symbols are encoded as `nibble << 3`, so valid values are
    /// multiples of 8. Any residual mod-8 in the demodulated sync word bins
    /// reveals STO that wasn't captured by the integer estimate.
    [[nodiscard]] int mod8_sto_correction(int netid1, int netid2) const {
        uint32_t r1 = static_cast<uint32_t>(mod(static_cast<int64_t>(netid1), 8LL));
        uint32_t r2 = static_cast<uint32_t>(mod(static_cast<int64_t>(netid2), 8LL));
        if (r1 != r2 || r1 == 0) return 0;
        int delta = static_cast<int>(r1);
        if (delta > 3) delta -= 8;  // wrap: 4→-4, 5→-3, 6→-2, 7→-1
        return delta;
    }

    struct SnrResult { float snr_db{0.f}; float signal_db{-999.f}; };

    [[nodiscard]] SnrResult determine_snr() {
        const auto n_syms = up_symb_to_use;
        if (n_syms == 0) return {};
        float snr_sum = 0.f, signal_sum = 0.f;
        uint32_t signal_cnt = 0;
        const float n_sq = static_cast<float>(N) * static_cast<float>(N);

        for (uint32_t i = 0; i < n_syms; i++) {
            multisf_detail::complex_multiply(scratch_N.data(), &preamble_upchirps[N * i],
                                      downchirp.data(), N);
            auto fft_out = fft.compute(
                std::span<const std::complex<float>>(scratch_N.data(), N));
            float total_energy = 0.f, peak_energy = 0.f;
            for (uint32_t j = 0; j < N; j++) {
                float mag_sq = fft_out[j].real() * fft_out[j].real()
                             + fft_out[j].imag() * fft_out[j].imag();
                total_energy += mag_sq;
                if (mag_sq > peak_energy) peak_energy = mag_sq;
            }
            float noise_energy = total_energy - peak_energy;
            if (noise_energy > 0.f) snr_sum += 10.f * std::log10(peak_energy / noise_energy);
            if (peak_energy > 0.f) { signal_sum += 10.f * std::log10(peak_energy / n_sq); signal_cnt++; }
        }
        const float inv_n = 1.f / static_cast<float>(n_syms);
        float sig_db = (signal_cnt > 0) ? signal_sum / static_cast<float>(signal_cnt) : -999.f;
        return { snr_sum * inv_n, sig_db };
    }

    // === Decode helpers ===

    void buildDownchirpWithCFO() {
        std::vector<std::complex<float>> uc(N);
        build_upchirp(uc.data(),
                      static_cast<uint32_t>(mod(cfo_int, static_cast<int64_t>(N))),
                      sf, 1);
        for (uint32_t n = 0; n < N; n++) {
            cfo_downchirp[n] = std::conj(uc[n]);
        }
        float frac_scale = -2.f * static_cast<float>(std::numbers::pi)
                          * cfo_frac / static_cast<float>(N);
        for (uint32_t n = 0; n < N; n++) {
            float phase = frac_scale * static_cast<float>(n);
            cfo_downchirp[n] *= std::complex<float>(std::cos(phase), std::sin(phase));
        }
    }

    [[nodiscard]] uint16_t demodSymbol(const std::complex<float>* samples) {
        auto max_idx = dechirp_argmax(samples, cfo_downchirp.data(),
                                       dechirped.data(), N, fft);
        return static_cast<uint16_t>(
            mod(static_cast<int64_t>(max_idx) - 1, static_cast<int64_t>(N)));
    }

    /// Soft demodulate: returns hard symbol AND stores per-bit LLR + PMR.
    /// Zero heap allocations — uses pre-allocated buffers and flat LLR storage.
    [[nodiscard]] uint16_t demodSymbolSoft(const std::complex<float>* samples) {
        auto result = dechirp_soft(samples, cfo_downchirp.data(),
                                   dechirped.data(), N, fft,
                                   _soft_mag_sq.data());  // pre-allocated
        auto hard_sym = static_cast<uint16_t>(
            mod(static_cast<int64_t>(result.bin) - 1, static_cast<int64_t>(N)));

        soft_symbol_pmrs.push_back(result.pmr);  // pre-reserved, no realloc

        // Remap mag_sq from FFT bin order to symbol order (subtract 1 mod N)
        // since symbol s maps to FFT bin (s+1) mod N.
        for (uint32_t s = 0; s < N; s++) {
            _soft_sym_mag_sq[s] = _soft_mag_sq[(s + 1) % N];
        }

        // Compute per-bit LLR for sf bits into pre-allocated buffer
        compute_symbol_llr(_soft_sym_mag_sq.data(), N, sf, gray_partition,
                           _soft_llr_buf.data());

        // Apply PMR-based confidence scaling: low PMR → unreliable symbol
        if (result.pmr < kPmrConfidenceThreshold) {
            for (uint8_t k = 0; k < sf; k++) _soft_llr_buf[k] *= 0.5;
        }

        // Append to flat storage (no inner vector allocation)
        _soft_llr_flat.insert(_soft_llr_flat.end(),
                              _soft_llr_buf.begin(),
                              _soft_llr_buf.begin() + sf);
        _soft_llr_count++;

        return hard_sym;
    }

    [[nodiscard]] static uint16_t grayMap(uint16_t symbol) {
        return static_cast<uint16_t>(symbol ^ (symbol >> 1));
    }

    /// Compute per-Gray-bit LLR from FFT magnitude-squared vector (max-log approx).
    /// llr_out must have sf_bits elements. llr_out[k] > 0 means Gray bit k is likely 1.
    /// k=0 is the LSB of the Gray code.
    ///
    /// The max-log approximation computes max(mag_sq over partition where bit=1)
    /// minus max(mag_sq over partition where bit=0). The peak bin always dominates
    /// its partition, so the result is equivalent to peak-anchored max-log: the LLR
    /// magnitude for each bit is the difference between the signal peak and the
    /// strongest noise bin in the opposite partition.
    static void compute_symbol_llr(
            const float* mag_sq, uint32_t M, uint8_t sf_bits,
            const GrayPartition& gp,
            double* llr_out) {
        for (uint8_t k = 0; k < sf_bits; k++) {
            float max_one = -std::numeric_limits<float>::infinity();
            for (uint32_t s : gp.ones[k])
                if (s < M) max_one = std::max(max_one, mag_sq[s]);
            float max_zero = -std::numeric_limits<float>::infinity();
            for (uint32_t s : gp.zeros[k])
                if (s < M) max_zero = std::max(max_zero, mag_sq[s]);
            llr_out[k] = static_cast<double>(max_one - max_zero);
        }
    }

    /// Soft deinterleaver: permutes per-bit LLRs using the same diagonal
    /// pattern as the hard deinterleaver but operates on LLR values.
    ///
    /// Input:  cw_len symbols, each with sf_app LLR values
    /// Output: sf_app codewords, each with cw_len LLR values
    [[nodiscard]] static std::vector<std::vector<double>> deinterleave_block_soft(
            const std::vector<std::vector<double>>& symbol_llrs,
            uint8_t sf_app, uint8_t cw_len) {
        // symbol_llrs[i] has sf_app LLRs (one per bit position, k=0 is LSB)
        // The interleaver maps: inter[i][j] = cw[mod(i-j-1, sf_app)][i]
        // So the deinterleaver reverses: cw[mod(i-j-1, sf_app)][i] = inter[i][j]
        //
        // In our LLR representation, symbol_llrs[i][k] is the LLR for bit k
        // (Gray bit position, k=0=LSB) of symbol i.
        //
        // The hard deinterleaver treats the sf_app-bit symbol as MSB-first
        // via int2bool(symbol, sf_app), then does:
        //   deinter[mod(i-j-1, sf_app)][i] = inter_bin[i][j]
        //
        // For the soft version, the int2bool expansion maps bit j (in the
        // sf_app-wide MSB-first vector) to Gray bit position (sf_app-1-j)
        // (since int2bool puts MSB at index 0).
        //
        // So we permute: cw_llr[mod(i-j-1, sf_app)][i] = symbol_llrs[i][sf_app-1-j]

        std::vector<std::vector<double>> cw_llrs(sf_app, std::vector<double>(cw_len, 0.0));

        for (std::size_t i = 0; i < cw_len; i++) {
            for (std::size_t j = 0; j < sf_app; j++) {
                auto row = static_cast<std::size_t>(
                    mod(static_cast<int>(i) - static_cast<int>(j) - 1,
                        static_cast<int64_t>(sf_app)));
                // Map from Gray bit position to MSB-first bit position
                std::size_t gray_bit_k = sf_app - 1 - j;
                if (i < symbol_llrs.size() && gray_bit_k < symbol_llrs[i].size()) {
                    cw_llrs[row][i] = symbol_llrs[i][gray_bit_k];
                }
            }
        }
        return cw_llrs;
    }

    /// Soft decode path: takes per-symbol LLR vectors, deinterleaves in LLR domain,
    /// then calls hamming_decode_soft() per codeword.
    [[nodiscard]] std::vector<uint8_t> processBlockSoft(
            const std::vector<std::vector<double>>& sym_llrs,
            uint8_t sf_app, uint8_t cw_len, uint8_t cr_app) {
        // For header/LDRO (sf_app < sf): the hard path does gray >>= (sf - sf_app),
        // dropping the lowest (sf - sf_app) bits.  In the LLR domain (k=0=LSB),
        // we keep the UPPER sf_app bits: llrs[sf-sf_app .. sf-1], renumbered
        // to 0..sf_app-1 so the soft deinterleaver sees the same bit positions
        // as the hard deinterleaver sees after the shift.
        const uint8_t drop = sf - sf_app;  // 0 for payload, 2 for header/LDRO
        std::vector<std::vector<double>> trimmed;
        trimmed.reserve(sym_llrs.size());
        for (const auto& llrs : sym_llrs) {
            auto begin = llrs.begin() + static_cast<std::ptrdiff_t>(
                std::min(static_cast<std::size_t>(drop), llrs.size()));
            auto end = llrs.begin() + static_cast<std::ptrdiff_t>(
                std::min(static_cast<std::size_t>(sf), llrs.size()));
            trimmed.emplace_back(begin, end);
        }

        auto cw_llrs = deinterleave_block_soft(trimmed, sf_app, cw_len);

        std::vector<uint8_t> nibs;
        nibs.reserve(cw_llrs.size());
        for (const auto& cw : cw_llrs) {
            nibs.push_back(hamming_decode_soft(cw.data(), cr_app));
        }
        return nibs;
    }

    /// Decode one interleaver block. When soft LLRs are available
    /// (kUseSoftDecode && flat LLR storage not empty), uses the soft path;
    /// otherwise falls back to hard decode.
    ///
    /// Soft LLRs are consumed from the front of _soft_llr_flat in lock-step
    /// with symbol_buffer — callers must ensure they are accumulated together
    /// via demodSymbolSoft().
    [[nodiscard]] std::vector<uint8_t> processBlock(
            const std::vector<uint16_t>& symbols,
            uint8_t sf_app, uint8_t cw_len, uint8_t cr_app,
            bool /*is_header_block*/) {
        // Soft path: use accumulated LLRs from flat storage
        if (use_soft_decode) {
            if (_soft_llr_count >= symbols.size()) {
                // Extract block's LLRs from flat storage into per-symbol vectors
                // (one alloc per interleaver block of ~8 symbols, not per symbol)
                std::vector<std::vector<double>> block_llrs;
                block_llrs.reserve(symbols.size());
                for (size_t i = 0; i < symbols.size() && i < _soft_llr_count; i++) {
                    const double* base = &_soft_llr_flat[i * sf];
                    block_llrs.emplace_back(base, base + sf);
                }
                // Consume the LLRs and PMRs we just used
                size_t consumed = std::min(symbols.size(), _soft_llr_count);
                _soft_llr_flat.erase(_soft_llr_flat.begin(),
                    _soft_llr_flat.begin() + static_cast<std::ptrdiff_t>(consumed * sf));
                _soft_llr_count -= consumed;
                soft_symbol_pmrs.erase(soft_symbol_pmrs.begin(),
                    soft_symbol_pmrs.begin() + static_cast<std::ptrdiff_t>(
                        std::min(symbols.size(), soft_symbol_pmrs.size())));
                return processBlockSoft(block_llrs, sf_app, cw_len, cr_app);
            }
        }

        // Hard decode fallback
        std::vector<uint16_t> gray_syms;
        gray_syms.reserve(symbols.size());
        for (auto s : symbols) gray_syms.push_back(grayMap(s));
        if (sf_app != sf) {
            for (auto& s : gray_syms) s >>= (sf - sf_app);
        }
        auto codewords = deinterleave_block(gray_syms, sf, cw_len, sf_app);
        std::vector<uint8_t> nibs;
        nibs.reserve(codewords.size());
        for (auto cw : codewords) nibs.push_back(hamming_decode_hard(cw, cr_app));
        return nibs;
    }

    /// Decode header from first 8 buffered output symbols — sets frame_symb_numb.
    void decodeHeader() {
        header_decoded = true;
        buildDownchirpWithCFO();

        std::vector<uint16_t> symbols;
        symbols.reserve(8);
        for (uint32_t i = 0; i < 8; i++) {
            auto idx = dechirp_argmax(&header_symbols[static_cast<std::size_t>(i) * N],
                                       cfo_downchirp.data(), scratch_N.data(), N, fft);
            auto sym = static_cast<uint16_t>(
                mod(static_cast<int64_t>(idx) - 1, static_cast<int64_t>(N)));
            symbols.push_back(sym);
        }

        const uint8_t sf_app = static_cast<uint8_t>(sf - 2);
        std::vector<uint16_t> gray_syms;
        gray_syms.reserve(8);
        for (auto s : symbols) gray_syms.push_back(static_cast<uint16_t>((s ^ (s >> 1)) >> 2));
        auto codewords = deinterleave_block(gray_syms, sf, 8, sf_app);
        if (codewords.size() < 5) return;

        std::vector<uint8_t> hdr_nibbles;
        hdr_nibbles.reserve(codewords.size());
        for (auto cw : codewords) hdr_nibbles.push_back(hamming_decode_hard(cw, 4));

        auto info = parse_explicit_header(hdr_nibbles[0], hdr_nibbles[1], hdr_nibbles[2],
                                            hdr_nibbles[3], hdr_nibbles[4]);
        if (!info.checksum_valid || info.payload_len == 0) return;

        bool ldro_flag = (static_cast<float>(1u << sf) * 1e3f
                         / static_cast<float>(bandwidth)) > LDRO_MAX_DURATION_MS;
        frame_symb_numb = 8 + static_cast<uint32_t>(
            std::ceil(static_cast<double>(
                2 * info.payload_len - sf + 2 + 5 + (info.has_crc ? 4 : 0))
                / (sf - 2 * static_cast<int>(ldro_flag))))
            * (4 + info.cr);
    }

    /// Full frame decode — returns decoded payload bytes + frame metadata.
    struct FrameResult {
        std::vector<uint8_t> payload;
        uint32_t pay_len{0};
        uint8_t  cr{4};
        bool     has_crc{true};
        bool     crc_valid{true};
    };

    [[nodiscard]] FrameResult decodeFrame() {
        FrameResult result;
        if (nibbles.size() < 5) return result;

        auto info = parse_explicit_header(nibbles[0], nibbles[1], nibbles[2],
                                            nibbles[3], nibbles[4]);
        if (!info.checksum_valid || info.payload_len == 0) return result;

        result.pay_len = info.payload_len;
        result.cr      = info.cr;
        result.has_crc = info.has_crc;

        uint32_t total_data_nibs = info.payload_len * 2 + (info.has_crc ? 4 : 0);
        std::size_t data_start = 5;
        std::size_t avail_nibs = (nibbles.size() > data_start) ? nibbles.size() - data_start : 0;
        std::size_t use_nibs = std::min(static_cast<std::size_t>(total_data_nibs), avail_nibs);
        use_nibs &= ~std::size_t{1};

        auto decoded_bytes = dewhiten(
            std::span<const uint8_t>(&nibbles[data_start], use_nibs), info.payload_len);

        result.crc_valid = true;
        if (info.has_crc && info.payload_len >= 2
            && decoded_bytes.size() >= info.payload_len + 2u) {
            result.crc_valid = lora_verify_crc(
                std::span<const uint8_t>(decoded_bytes.data(), info.payload_len),
                decoded_bytes[info.payload_len],
                decoded_bytes[info.payload_len + 1]);
        }

        result.payload.assign(decoded_bytes.begin(),
                               decoded_bytes.begin()
                               + static_cast<std::ptrdiff_t>(
                                   std::min(static_cast<std::size_t>(info.payload_len),
                                            decoded_bytes.size())));
        return result;
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_ALGORITHM_SFLANE_HPP
