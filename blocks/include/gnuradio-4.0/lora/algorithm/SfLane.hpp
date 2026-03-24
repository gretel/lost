// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_ALGORITHM_SFLANE_HPP
#define GNURADIO_LORA_ALGORITHM_SFLANE_HPP

// Per-SF processing lane: fused FrameSync + DemodDecoder state machine.
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
#include <gnuradio-4.0/lora/algorithm/SfLaneDetail.hpp>
#include <gnuradio-4.0/lora/algorithm/crc.hpp>
#include <gnuradio-4.0/lora/algorithm/hamming.hpp>
#include <gnuradio-4.0/lora/algorithm/interleaving.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>
#include <gnuradio-4.0/lora/algorithm/utilities.hpp>

namespace gr::lora {

// Alias for backward compatibility with call sites using multisf_detail::
namespace multisf_detail = sflane_detail;

/// Per-SF processing lane — holds the complete fused FrameSync + DemodDecoder
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
    std::vector<std::complex<float>> scratch_2N;   // length 2*N
    std::vector<float>               scratch_2N_f; // length 2*N
    std::vector<std::complex<float>> corr_vec;     // preamble correction vector
    std::vector<std::complex<float>> fft_val_buf;  // CFO estimation FFT values

    // === FrameSync state ===
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

    // === DemodDecoder state ===
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
              uint16_t preamble_len_, uint16_t sync_word) {
        sf = sf_;
        N  = 1u << sf;
        os_factor = os_factor_;
        sps = N * os_factor;
        bandwidth = bandwidth_;
        preamble_len = preamble_len_;

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
        scratch_2N_f.resize(2 * N);
        auto max_corr = static_cast<std::size_t>(n_up_req + multisf_detail::kMaxAdditionalUpchirps) * N;
        corr_vec.resize(max_corr);
        fft_val_buf.resize(static_cast<std::size_t>(up_symb_to_use) * N);
        header_symbols.resize(8UZ * N);
        cfo_downchirp.resize(N);
        dechirped.resize(N);

        accum.clear();
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
    }

    // === DSP helper methods (ported from FrameSync) ===

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

    float estimate_CFO_frac_Bernier(const std::complex<float>* samples) {
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

        std::complex<float> four_cum(0.f, 0.f);
        for (uint32_t i = 0; i + 1 < n_syms; i++) {
            for (int p = -2; p <= 2; p++) {
                uint32_t bin = (idx_max + static_cast<uint32_t>(p + static_cast<int>(N))) % N;
                four_cum += fft_val_buf[bin + N * i]
                          * std::conj(fft_val_buf[bin + N * (i + 1)]);
            }
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

    float estimate_STO_frac() {
        const auto n_syms = up_symb_to_use;
        std::fill(scratch_2N_f.begin(), scratch_2N_f.end(), 0.f);

        for (uint32_t i = 0; i < n_syms; i++) {
            multisf_detail::complex_multiply(scratch_N.data(), &preamble_upchirps[N * i],
                                      downchirp.data(), N);
            std::copy_n(scratch_N.begin(), N, scratch_2N.begin());
            std::fill(scratch_2N.begin() + N, scratch_2N.end(),
                      std::complex<float>(0.f, 0.f));
            auto fft_out = fft.compute(
                std::span<const std::complex<float>>(scratch_2N.data(), 2 * N));
            for (uint32_t j = 0; j < 2 * N; j++) {
                scratch_2N_f[j] += fft_out[j].real() * fft_out[j].real()
                                  + fft_out[j].imag() * fft_out[j].imag();
            }
        }

        auto max_it = std::max_element(scratch_2N_f.begin(), scratch_2N_f.end());
        auto k0 = static_cast<uint32_t>(std::distance(scratch_2N_f.begin(), max_it));

        auto wrap = [N2 = 2LL * static_cast<int64_t>(N)](int64_t i) -> std::size_t {
            return static_cast<std::size_t>(mod(i, N2));
        };
        double Y_1 = static_cast<double>(scratch_2N_f[wrap(static_cast<int64_t>(k0) - 1)]);
        double Y0  = static_cast<double>(scratch_2N_f[k0]);
        double Y1  = static_cast<double>(scratch_2N_f[wrap(static_cast<int64_t>(k0) + 1)]);

        double u = 64.0 * N / 406.5506497;
        double v = u * 2.4674;
        double wa = (Y1 - Y_1) / (u * (Y1 + Y_1) + v * Y0);
        double ka = wa * N / std::numbers::pi;
        double k_residual = std::fmod((k0 + ka) / 2.0, 1.0);
        float sto_f = static_cast<float>(k_residual - (k_residual > 0.5 ? 1.0 : 0.0));
        return sto_f;
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

    // === Decode helpers (ported from DemodDecoder) ===

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

    [[nodiscard]] static uint16_t grayMap(uint16_t symbol) {
        return static_cast<uint16_t>(symbol ^ (symbol >> 1));
    }

    [[nodiscard]] std::vector<uint8_t> processBlock(
            const std::vector<uint16_t>& symbols,
            uint8_t sf_app, uint8_t cw_len, uint8_t cr_app,
            bool /*is_header_block*/) {
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
