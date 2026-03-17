// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_MULTI_SF_DECODER_HPP
#define GNURADIO_LORA_MULTI_SF_DECODER_HPP

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <string>
#include <unordered_map>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/algorithm/crc.hpp>
#include <gnuradio-4.0/lora/algorithm/hamming.hpp>
#include <gnuradio-4.0/lora/algorithm/interleaving.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>
#include <gnuradio-4.0/lora/algorithm/utilities.hpp>
#include <gnuradio-4.0/lora/log.hpp>

namespace gr::lora {

// Detection thresholds shared with FrameSync (duplicated to avoid include dependency
// on FrameSync.hpp which carries its own GR_REGISTER_BLOCK).
namespace multisf_detail {
static constexpr int kPreambleBinTolerance = 1;
static constexpr int kSyncWordBinTolerance = 2;
static constexpr uint8_t kMaxAdditionalUpchirps = 3;
static constexpr uint8_t kMaxPreambleRotations  = 4;
static constexpr float   kMinPreamblePMR        = 4.0f;

[[nodiscard]] inline int most_frequent(const std::vector<int>& arr) {
    std::unordered_map<int, int> freq;
    for (int v : arr) freq[v]++;
    int max_count = 0, result = -1;
    for (auto& [val, cnt] : freq) {
        if (cnt > max_count) { max_count = cnt; result = val; }
    }
    return result;
}

[[nodiscard]] inline int my_roundf(float x) noexcept {
    return x > 0 ? static_cast<int>(x + 0.5f)
                 : static_cast<int>(std::ceil(x - 0.5f));
}

inline void complex_multiply(std::complex<float>* out, const std::complex<float>* a,
                             const std::complex<float>* b, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) out[i] = a[i] * b[i];
}
}  // namespace multisf_detail

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
            bool is_header_block) {
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

// =============================================================================
// MultiSfDecoder: single GR4 block that simultaneously detects + decodes
// all configured SFs on a single narrowband channel.
// =============================================================================

GR_REGISTER_BLOCK("gr::lora::MultiSfDecoder", gr::lora::MultiSfDecoder)
struct MultiSfDecoder
    : gr::Block<MultiSfDecoder, gr::NoDefaultTagForwarding> {

    using cf32 = std::complex<float>;

    gr::PortIn<cf32>     in;
    gr::PortOut<uint8_t> out;
    gr::MsgPortOut       msg_out;

    // --- Settings ---
    uint32_t bandwidth    = 125000;
    uint16_t sync_word    = 0x12;
    uint8_t  os_factor    = 4;
    uint16_t preamble_len = 8;
    float    energy_thresh = 1e-6f;
    float    min_snr_db   = -10.0f;
    uint32_t max_symbols  = 600;
    uint32_t center_freq  = 869618000;
    int32_t  rx_channel   = -1;
    uint8_t  sf_min       = 7;
    uint8_t  sf_max       = 12;
    bool     debug        = false;

    GR_MAKE_REFLECTABLE(MultiSfDecoder, in, out, msg_out,
        bandwidth, sync_word, os_factor, preamble_len,
        energy_thresh, min_snr_db, max_symbols, center_freq,
        rx_channel, sf_min, sf_max, debug);

    std::vector<SfLane> _lanes;
    uint32_t _min_sps{0};
    float    _noise_floor_db{-999.f};
    float    _peak_db{-999.f};
    float    _noise_ema{0.f};
    bool     _noise_ema_init{false};
    std::atomic<bool>* _channel_busy{nullptr};  // LBT: set by any lane in SYNC/OUTPUT

    /// Set external channel-busy flag for listen-before-talk.
    void set_channel_busy_flag(std::atomic<bool>* flag) { _channel_busy = flag; }

    void start() { reconfigure(); }

    void settingsChanged(const gr::property_map& /*oldSettings*/,
                         const gr::property_map& newSettings) {
        static constexpr std::array kPhyKeys = {
            "sf_min", "sf_max", "bandwidth", "os_factor",
            "preamble_len", "sync_word"
        };
        bool needRecalc = std::ranges::any_of(kPhyKeys, [&](const char* k) {
            return newSettings.contains(k);
        });
        if (needRecalc && !_lanes.empty()) {
            reconfigure();
        }
    }

    void reconfigure() {
        assert(sf_min >= 7 && sf_max <= 12 && sf_min <= sf_max);
        assert(bandwidth > 0 && os_factor >= 1 && preamble_len >= 5);

        _lanes.clear();
        _lanes.reserve(sf_max - sf_min + 1);

        for (uint8_t s = sf_min; s <= sf_max; s++) {
            _lanes.emplace_back();
            _lanes.back().init(s, bandwidth, os_factor, preamble_len, sync_word);
        }

        _min_sps = _lanes.front().sps;  // sf_min has smallest sps
        // Set min_samples to smallest lane's sps (minimum useful input)
        in.min_samples = _min_sps + 2u * os_factor;
    }

    [[nodiscard]] gr::work::Status processBulk(
            gr::InputSpanLike auto& input,
            gr::OutputSpanLike auto& output) noexcept {
        auto in_span  = std::span(input);
        auto out_span = std::span(output);

        if (_lanes.empty()) reconfigure();

        if (in_span.size() < _min_sps) {
            std::ignore = input.consume(0UZ);
            output.publish(0UZ);
            return gr::work::Status::OK;
        }

        // Always consume the full input. Each lane appends to its own
        // accumulation buffer and processes symbols from there. This keeps
        // the ring buffer cursor advancing at full rate so the scheduler
        // stays in its fast path (no stall/sleep on consume(0)).
        const std::size_t n_in = in_span.size();
        std::size_t out_idx = 0;

        for (auto& lane : _lanes) {
            // Append new input to this lane's accumulation buffer
            lane.accum.insert(lane.accum.end(), in_span.begin(), in_span.end());

            // Process as many symbols as fit in the accumulated data
            auto accum_span = std::span<const cf32>(lane.accum);
            std::size_t pos = 0;
            while (pos + lane.sps <= accum_span.size()) {
                auto consumed = processLaneSymbol(
                    lane, accum_span, pos, out_span, out_idx);
                if (consumed == 0) { consumed = lane.sps; }
                pos += consumed;
            }

            // Erase consumed samples, keep the remainder for next call
            if (pos > 0) {
                lane.accum.erase(lane.accum.begin(),
                                  lane.accum.begin()
                                  + static_cast<std::ptrdiff_t>(pos));
            }
        }

        // Update LBT channel-busy flag
        if (_channel_busy != nullptr) {
            bool busy = false;
            for (const auto& lane : _lanes) {
                if (lane.state != SfLane::DETECT) { busy = true; break; }
            }
            _channel_busy->store(busy, std::memory_order_relaxed);
        }

        std::ignore = input.consume(n_in);
        output.publish(out_idx);
        return gr::work::Status::OK;
    }

    /// Process one symbol period for a single SF lane.
    /// Returns the number of input samples consumed.
    std::size_t processLaneSymbol(SfLane& lane,
                           std::span<const cf32> in_span, std::size_t in_offset,
                           std::span<uint8_t> out_span, std::size_t& out_idx) {
        // Downsample: pick every os_factor-th sample with STO correction
        const int os = static_cast<int>(os_factor);
        for (uint32_t ii = 0; ii < lane.N; ii++) {
            int idx = static_cast<int>(in_offset) + os / 2
                    + static_cast<int>(os_factor * ii)
                    - multisf_detail::my_roundf(lane.sto_frac * static_cast<float>(os_factor));
            if (idx < 0) idx += static_cast<int>(lane.sps);
            idx = std::clamp(idx, 0, static_cast<int>(in_span.size()) - 1);
            lane.in_down[ii] = in_span[static_cast<std::size_t>(idx)];
        }

        // Default: consume one full symbol period
        lane.items_to_consume = static_cast<int>(lane.sps);

        switch (lane.state) {
        case SfLane::DETECT:
            processDetect(lane, in_span, in_offset);
            break;
        case SfLane::SYNC:
            lane.stall_count++;
            if (lane.stall_count > SfLane::kMaxStallSymbols) {
                lane.resetToDetect();
                break;
            }
            processSync(lane, in_span, in_offset);
            break;
        case SfLane::OUTPUT:
            lane.stall_count++;
            if (lane.stall_count > SfLane::kMaxStallSymbols) {
                lane.resetToDetect();
                break;
            }
            processOutput(lane, in_span, in_offset, out_span, out_idx);
            break;
        }

        // Clamp consumed to valid range
        auto consumed = static_cast<std::size_t>(std::max(lane.items_to_consume, 1));
        return consumed;
    }

    // =========================================================================
    // DETECT state: look for consecutive upchirps matching the preamble
    // =========================================================================
    void processDetect(SfLane& lane, std::span<const cf32> in_span,
                       std::size_t in_offset) {
        auto [dechirp_bin, dechirp_pmr] = dechirp_and_quality(
            lane.in_down.data(), lane.downchirp.data(),
            lane.scratch_N.data(), lane.N, lane.fft, /*remove_dc=*/true);
        lane.bin_idx_new = static_cast<int32_t>(dechirp_bin);



        // Update noise floor EMA from idle symbols
        if (lane.symbol_cnt <= 1 && lane.sf == sf_min) {
            float mean_power = 0.f;
            for (uint32_t i = 0; i < lane.N; i++) {
                mean_power += lane.in_down[i].real() * lane.in_down[i].real()
                            + lane.in_down[i].imag() * lane.in_down[i].imag();
            }
            mean_power /= static_cast<float>(lane.N);
            constexpr float kAlpha = 0.05f;
            if (!_noise_ema_init) {
                _noise_ema = mean_power;
                _noise_ema_init = true;
            } else {
                _noise_ema += kAlpha * (mean_power - _noise_ema);
            }
            if (_noise_ema > 0.f) {
                _noise_floor_db = 10.f * std::log10(_noise_ema);
            }
        }

        if (!lane.has_energy(lane.in_down.data(), lane.N, energy_thresh)) {
            lane.symbol_cnt = 1;
            lane.bin_idx = -1;
            return;
        }

        if (dechirp_pmr < multisf_detail::kMinPreamblePMR) {
            lane.symbol_cnt = 1;
            lane.bin_idx = -1;
            return;
        }

        // Consecutive upchirp check
        if (std::abs(mod(std::abs(lane.bin_idx_new - lane.bin_idx) + 1,
                         static_cast<int64_t>(lane.N)) - 1)
                <= multisf_detail::kPreambleBinTolerance && lane.bin_idx_new != -1) {
            if (lane.symbol_cnt == 1 && lane.bin_idx != -1) {
                lane.preamb_up_vals[0] = lane.bin_idx;
            }
            auto sc = static_cast<uint32_t>(lane.symbol_cnt);
            if (sc < lane.preamb_up_vals.size()) {
                lane.preamb_up_vals[sc] = lane.bin_idx_new;
            }
            if (static_cast<std::size_t>(sc) * lane.N < lane.preamble_raw.size()) {
                std::copy_n(lane.in_down.data(), lane.N,
                            &lane.preamble_raw[static_cast<std::size_t>(sc) * lane.N]);
            }
            if (static_cast<std::size_t>(sc) * lane.sps + lane.sps <= lane.preamble_raw_up.size()) {
                std::size_t src_off = in_offset + os_factor / 2;
                if (src_off + lane.sps <= in_span.size()) {
                    std::copy_n(&in_span[src_off], lane.sps,
                                &lane.preamble_raw_up[static_cast<std::size_t>(sc) * lane.sps]);
                }
            }
            lane.symbol_cnt++;
        } else {
            std::copy_n(lane.in_down.data(), lane.N, &lane.preamble_raw[0]);
            std::size_t src_off = in_offset + os_factor / 2;
            if (src_off + lane.sps <= in_span.size()) {
                std::copy_n(&in_span[src_off], lane.sps, &lane.preamble_raw_up[0]);
            }
            lane.symbol_cnt = 1;
        }
        lane.bin_idx = lane.bin_idx_new;

        if (static_cast<uint32_t>(lane.symbol_cnt) == lane.n_up_req) {
            lane.additional_upchirps = 0;
            lane.state = SfLane::SYNC;
            lane.sync_phase = SfLane::NET_ID1;
            lane.symbol_cnt = 0;
            lane.cfo_frac_sto_frac_est = false;
            lane.k_hat = multisf_detail::most_frequent(lane.preamb_up_vals);

            if (debug) {
                log_ts("debug", "MultiSf", "SF%u DETECT->SYNC: k_hat=%d",
                       lane.sf, lane.k_hat);
            }

            // Copy net_id samples
            const int os_i = static_cast<int>(os_factor);
            int src_start = static_cast<int>(in_offset)
                          + static_cast<int>(lane.sps * 3 / 4)
                          - lane.k_hat * os_i;
            for (uint32_t i = 0; i < lane.sps / 4; i++) {
                int src_idx = src_start + static_cast<int>(i);
                src_idx = std::clamp(src_idx, 0, static_cast<int>(in_span.size()) - 1);
                lane.net_id_samp[i] = in_span[static_cast<std::size_t>(src_idx)];
            }

            // Align input to symbol boundary (match FrameSync line 694)
            lane.items_to_consume = os_i * static_cast<int>(lane.N - static_cast<uint32_t>(lane.k_hat));
        }
    }

    // =========================================================================
    // SYNC state: CFO/STO estimation, sync word verification
    // =========================================================================
    void processSync(SfLane& lane, std::span<const cf32> in_span,
                     std::size_t in_offset) {
        const int os = static_cast<int>(os_factor);

        if (!lane.cfo_frac_sto_frac_est) {
            auto cfo_offset = static_cast<std::size_t>(lane.N)
                + static_cast<std::size_t>(std::max(static_cast<int>(lane.N) - lane.k_hat, 0));
            if (cfo_offset + lane.up_symb_to_use * lane.N > lane.preamble_raw.size()) {
                cfo_offset = static_cast<std::size_t>(lane.N);
            }
            lane.cfo_frac = lane.estimate_CFO_frac_Bernier(&lane.preamble_raw[cfo_offset]);
            lane.sto_frac = lane.estimate_STO_frac();

            for (uint32_t n = 0; n < lane.N; n++) {
                float phase = -2.f * static_cast<float>(std::numbers::pi) * lane.cfo_frac
                            / static_cast<float>(lane.N) * static_cast<float>(n);
                lane.CFO_frac_correc[n] = cf32(std::cos(phase), std::sin(phase));
            }
            lane.cfo_frac_sto_frac_est = true;
        }

        // Apply CFO frac correction
        multisf_detail::complex_multiply(lane.symb_corr.data(), lane.in_down.data(),
                                  lane.CFO_frac_correc.data(), lane.N);
        int32_t bin_idx_sync = static_cast<int32_t>(
            lane.get_symbol_val(lane.symb_corr.data(), lane.downchirp.data()));

        switch (lane.sync_phase) {
        case SfLane::NET_ID1: {
            if (bin_idx_sync == 0 || bin_idx_sync == 1
                || static_cast<uint32_t>(bin_idx_sync) == lane.N - 1) {
                // Additional upchirp
                auto ns_start = in_offset + static_cast<std::size_t>(lane.sps * 3 / 4);
                uint32_t copy_len = std::min(lane.sps / 4,
                                              static_cast<uint32_t>(lane.net_id_samp.size()));
                for (uint32_t i = 0; i < copy_len; i++) {
                    auto idx = ns_start + i;
                    if (idx < in_span.size()) lane.net_id_samp[i] = in_span[idx];
                }

                if (lane.additional_upchirps >= multisf_detail::kMaxAdditionalUpchirps) {
                    if (lane.preamble_rotations >= multisf_detail::kMaxPreambleRotations) {
                        lane.resetToDetect();
                        lane.items_to_consume = 0;
                        return;
                    }
                    std::rotate(lane.preamble_raw_up.begin(),
                                lane.preamble_raw_up.begin() + static_cast<std::ptrdiff_t>(lane.sps),
                                lane.preamble_raw_up.end());
                    std::size_t dst = static_cast<std::size_t>(lane.n_up_req + 3) * lane.sps;
                    if (dst + lane.sps <= lane.preamble_raw_up.size()) {
                        auto src_off = in_offset
                            + static_cast<std::size_t>(os_factor / 2 + static_cast<uint32_t>(lane.k_hat) * os_factor);
                        if (src_off + lane.sps <= in_span.size()) {
                            std::copy_n(&in_span[src_off], lane.sps, &lane.preamble_raw_up[dst]);
                        }
                    }
                    lane.preamble_rotations++;
                } else {
                    std::size_t dst = static_cast<std::size_t>(
                        lane.n_up_req + lane.additional_upchirps) * lane.sps;
                    if (dst + lane.sps <= lane.preamble_raw_up.size()) {
                        auto src_off = in_offset
                            + static_cast<std::size_t>(os_factor / 2 + static_cast<uint32_t>(lane.k_hat) * os_factor);
                        if (src_off + lane.sps <= in_span.size()) {
                            std::copy_n(&in_span[src_off], lane.sps, &lane.preamble_raw_up[dst]);
                        }
                    }
                    lane.additional_upchirps++;
                }
            } else {
                lane.sync_phase = SfLane::NET_ID2;
                std::size_t dst_off = lane.sps / 4;
                std::size_t copy_len = std::min(static_cast<std::size_t>(lane.sps),
                                                  lane.net_id_samp.size() - dst_off);
                copy_len = std::min(copy_len, in_span.size() - in_offset);
                std::copy_n(&in_span[in_offset], copy_len, &lane.net_id_samp[dst_off]);
            }
            break;
        }
        case SfLane::NET_ID2: {
            lane.sync_phase = SfLane::DOWNCHIRP1;
            std::size_t dst_off = lane.sps + lane.sps / 4;
            std::size_t avail = lane.net_id_samp.size() > dst_off
                              ? lane.net_id_samp.size() - dst_off : 0;
            std::size_t copy_len = std::min(
                static_cast<std::size_t>((lane.N + 1) * os_factor), avail);
            copy_len = std::min(copy_len, in_span.size() - in_offset);
            std::copy_n(&in_span[in_offset], copy_len, &lane.net_id_samp[dst_off]);
            break;
        }
        case SfLane::DOWNCHIRP1: {
            std::size_t dst_off = lane.sps * 2 + lane.sps / 4;
            std::size_t avail = lane.net_id_samp.size() > dst_off
                              ? lane.net_id_samp.size() - dst_off : 0;
            std::size_t copy_len = std::min(static_cast<std::size_t>(lane.sps / 4), avail);
            copy_len = std::min(copy_len, in_span.size() - in_offset);
            if (copy_len > 0)
                std::copy_n(&in_span[in_offset], copy_len, &lane.net_id_samp[dst_off]);
            lane.sync_phase = SfLane::DOWNCHIRP2;
            break;
        }
        case SfLane::DOWNCHIRP2: {
            lane.down_val = static_cast<int>(
                lane.get_symbol_val(lane.symb_corr.data(), lane.upchirp.data()));
            lane.sync_phase = SfLane::QUARTER_DOWN;
            break;
        }
        case SfLane::QUARTER_DOWN: {
            // Integer CFO from downchirp
            if (static_cast<uint32_t>(lane.down_val) < lane.N / 2) {
                lane.cfo_int = lane.down_val / 2;
            } else {
                lane.cfo_int = (lane.down_val - static_cast<int>(lane.N)) / 2;
            }

            // Correct STOint and CFOint in preamble upchirps
            auto rot_off = static_cast<std::size_t>(
                mod(static_cast<int64_t>(lane.cfo_int), static_cast<int64_t>(lane.N)));
            std::rotate(lane.preamble_upchirps.begin(),
                        lane.preamble_upchirps.begin() + static_cast<std::ptrdiff_t>(rot_off),
                        lane.preamble_upchirps.end());

            uint32_t n_corr_syms = lane.n_up_req + lane.additional_upchirps;
            std::size_t corr_len = static_cast<std::size_t>(n_corr_syms) * lane.N;
            for (std::size_t n = 0; n < corr_len; n++) {
                float phase = -2.f * static_cast<float>(std::numbers::pi)
                            * static_cast<float>(lane.cfo_int)
                            / static_cast<float>(lane.N) * static_cast<float>(n);
                lane.corr_vec[n] = cf32(std::cos(phase), std::sin(phase));
            }
            multisf_detail::complex_multiply(lane.preamble_upchirps.data(), lane.preamble_upchirps.data(),
                                      lane.corr_vec.data(), lane.up_symb_to_use * lane.N);

            // SFO estimation
            lane.sfo_hat = (static_cast<float>(lane.cfo_int) + lane.cfo_frac)
                          * static_cast<float>(bandwidth)
                          / static_cast<float>(center_freq);

            // SFO correction in preamble
            double clk_off = static_cast<double>(lane.sfo_hat) / lane.N;
            double fs   = static_cast<double>(bandwidth);
            double fs_p = static_cast<double>(bandwidth) * (1.0 - clk_off);
            int N_int = static_cast<int>(lane.N);
            for (std::size_t n = 0; n < corr_len; n++) {
                double n_mod = static_cast<double>(
                    mod(static_cast<int64_t>(n), static_cast<int64_t>(N_int)));
                double floor_n_N = std::floor(static_cast<double>(n) / N_int);
                double phase = -2.0 * std::numbers::pi * (
                    n_mod * n_mod / (2.0 * N_int)
                    * (bandwidth / fs_p * bandwidth / fs_p - bandwidth / fs * bandwidth / fs)
                    + (floor_n_N * (bandwidth / fs_p * bandwidth / fs_p - bandwidth / fs_p)
                       + bandwidth / 2.0 * (1.0 / fs - 1.0 / fs_p)) * n_mod);
                lane.corr_vec[n] = cf32(static_cast<float>(std::cos(phase)),
                                         static_cast<float>(std::sin(phase)));
            }
            multisf_detail::complex_multiply(lane.preamble_upchirps.data(), lane.preamble_upchirps.data(),
                                      lane.corr_vec.data(), lane.up_symb_to_use * lane.N);

            // Re-estimate STO frac after SFO correction
            float tmp_sto_frac = lane.estimate_STO_frac();
            float diff = lane.sto_frac - tmp_sto_frac;
            float os_thresh = static_cast<float>(os_factor - 1u) / static_cast<float>(os_factor);
            if (std::abs(diff) <= os_thresh) lane.sto_frac = tmp_sto_frac;

            // Update sto_frac to net_id position
            lane.sto_frac += lane.sfo_hat * static_cast<float>(preamble_len);
            if (std::abs(lane.sto_frac) > 0.5f) {
                lane.sto_frac += (lane.sto_frac > 0 ? -1.f : 1.f);
            }

            // Decimate net_id for sync word verification
            std::fill(lane.scratch_2N.begin(), lane.scratch_2N.end(), cf32(0.f, 0.f));
            int start_off = os / 2
                          - multisf_detail::my_roundf(lane.sto_frac * static_cast<float>(os_factor))
                          + os * (static_cast<int>(lane.N / 4) + lane.cfo_int);
            for (uint32_t i = 0; i < lane.N * 2; i++) {
                int idx = start_off + static_cast<int>(i) * os;
                if (idx >= 0 && static_cast<std::size_t>(idx) < lane.net_id_samp.size()) {
                    lane.scratch_2N[i] = lane.net_id_samp[static_cast<std::size_t>(idx)];
                }
            }

            // Apply CFO corrections to net ID
            for (uint32_t n = 0; n < 2 * lane.N; n++) {
                float phase = -2.f * static_cast<float>(std::numbers::pi)
                            * static_cast<float>(lane.cfo_int)
                            / static_cast<float>(lane.N) * static_cast<float>(n);
                lane.corr_vec[n] = cf32(std::cos(phase), std::sin(phase));
            }
            multisf_detail::complex_multiply(lane.scratch_2N.data(), lane.scratch_2N.data(),
                                      lane.corr_vec.data(), 2 * lane.N);
            multisf_detail::complex_multiply(lane.scratch_2N.data(), lane.scratch_2N.data(),
                                      lane.CFO_frac_correc.data(), lane.N);
            multisf_detail::complex_multiply(&lane.scratch_2N[lane.N], &lane.scratch_2N[lane.N],
                                      lane.CFO_frac_correc.data(), lane.N);

            int netid1 = static_cast<int>(
                lane.get_symbol_val(lane.scratch_2N.data(), lane.downchirp.data()));
            int netid2 = static_cast<int>(
                lane.get_symbol_val(&lane.scratch_2N[lane.N], lane.downchirp.data()));

            // Sync word verification
            bool sync_ok = false;
            int net_id_off = 0;
            if (lane.sw0 == 0) {
                sync_ok = true;
            } else if (std::abs(netid1 - static_cast<int>(lane.sw0)) > multisf_detail::kSyncWordBinTolerance) {
                // wrong
            } else {
                net_id_off = netid1 - static_cast<int>(lane.sw0);
                if (lane.sw1 != 0 && mod(netid2 - net_id_off, static_cast<int64_t>(lane.N))
                    != static_cast<int64_t>(lane.sw1)) {
                    // wrong
                } else {
                    sync_ok = true;
                }
            }

            if (!sync_ok) {
                lane.resetToDetect();
                lane.items_to_consume = 0;
                return;
            }

            // Apply sync word timing offset (FrameSync line 953)
            lane.items_to_consume = -static_cast<int>(os_factor) * net_id_off;

            auto [snr, sig] = lane.determine_snr();
            lane.snr_db    = snr;
            lane.signal_db = sig;
            if (lane.snr_db < min_snr_db) {
                lane.resetToDetect();
                lane.items_to_consume = 0;
                return;
            }

            // Transition to OUTPUT
            lane.state = SfLane::OUTPUT;
            lane.output_symb_cnt = 0;
            lane.symbol_buffer.clear();
            lane.nibbles.clear();
            lane.total_symbols_rx = 0;
            lane.symb_numb = 0;
            lane.cr = 4;

            // Update sto_frac to payload beginning
            lane.sto_frac += lane.sfo_hat * 4.25f;
            lane.sfo_cum = ((lane.sto_frac * static_cast<float>(os_factor))
                           - static_cast<float>(multisf_detail::my_roundf(
                               lane.sto_frac * static_cast<float>(os_factor))))
                          / static_cast<float>(os_factor);

            // Adjust items_to_consume for quarter-down skip + CFO alignment
            // (FrameSync line 1006-1007)
            lane.items_to_consume += static_cast<int>(lane.sps / 4)
                                   + static_cast<int>(os_factor) * lane.cfo_int;

            // Build CFO-corrected downchirp for demodulation
            lane.buildDownchirpWithCFO();

            // Compute LDRO
            lane.ldro = (static_cast<float>(1u << lane.sf) * 1e3f
                        / static_cast<float>(bandwidth)) > LDRO_MAX_DURATION_MS;
            break;
        }
        }  // switch sync_phase
    }

    // =========================================================================
    // OUTPUT state: demodulate symbols + decode (fused FrameSync OUTPUT + DemodDecoder)
    // =========================================================================
    void processOutput(SfLane& lane, std::span<const cf32> /*in_span*/,
                       std::size_t /*in_offset*/,
                       std::span<uint8_t> out_span, std::size_t& out_idx) {
        // Termination
        uint32_t symb_limit = (lane.frame_symb_numb > 0) ? lane.frame_symb_numb : max_symbols;
        if (!lane.has_energy(lane.in_down.data(), lane.N, energy_thresh)
            || lane.output_symb_cnt >= symb_limit) {
            // If we have accumulated enough symbols, try to decode before resetting
            if (lane.symb_numb > 0 && lane.total_symbols_rx >= lane.symb_numb) {
                finishFrame(lane, out_span, out_idx);
            }
            lane.resetToDetect();
            return;
        }

        // Buffer first 8 symbols for early header decode (FrameSync-style)
        if (lane.output_symb_cnt < 8 && !lane.header_decoded) {
            std::copy_n(lane.in_down.data(), lane.N,
                        &lane.header_symbols[static_cast<std::size_t>(lane.output_symb_cnt) * lane.N]);
        }

        // Demodulate this symbol (fused: skip intermediate cf32 buffer)
        uint16_t symbol = lane.demodSymbol(lane.in_down.data());
        lane.symbol_buffer.push_back(symbol);
        lane.total_symbols_rx++;
        lane.output_symb_cnt++;

        // SFO tracking (same as FrameSync lines 1064-1069)
        if (std::abs(lane.sfo_cum) > 1.0f / (2.0f * static_cast<float>(os_factor))) {
            int sign_val = (lane.sfo_cum > 0) ? 1 : -1;
            lane.items_to_consume -= sign_val;
            lane.sfo_cum -= static_cast<float>(sign_val) / static_cast<float>(os_factor);
        }
        lane.sfo_cum += lane.sfo_hat;

        // After 8th symbol: decode header (FrameSync-style early termination)
        if (lane.output_symb_cnt == 8 && !lane.header_decoded) {
            lane.decodeHeader();
        }

        // Decode pipeline (DemodDecoder logic)
        if (lane.total_symbols_rx <= 8) {
            // HEADER block: accumulate 8 symbols, then decode header
            if (lane.symbol_buffer.size() == 8) {
                auto header_nibbles = lane.processBlock(
                    lane.symbol_buffer, static_cast<uint8_t>(lane.sf - 2), 8, 4, true);
                lane.nibbles.insert(lane.nibbles.end(),
                                     header_nibbles.begin(), header_nibbles.end());
                lane.symbol_buffer.clear();

                if (lane.nibbles.size() >= 5) {
                    auto info = parse_explicit_header(
                        lane.nibbles[0], lane.nibbles[1], lane.nibbles[2],
                        lane.nibbles[3], lane.nibbles[4]);

                    if (!info.checksum_valid || info.payload_len == 0) {
                        lane.resetToDetect();
                        return;
                    }

                    lane.pay_len = info.payload_len;
                    lane.cr      = info.cr;
                    lane.has_crc = info.has_crc;
                    lane.symb_numb = 8 + static_cast<uint32_t>(
                        std::ceil(static_cast<double>(
                            2 * lane.pay_len - lane.sf + 2 + 5 + (lane.has_crc ? 4 : 0))
                            / (lane.sf - 2 * static_cast<int>(lane.ldro))))
                        * (4 + lane.cr);
                }
            }
        } else {
            // PAYLOAD blocks
            uint8_t cw_len = 4 + lane.cr;
            if (lane.symbol_buffer.size() == cw_len) {
                uint8_t sf_app = lane.ldro ? static_cast<uint8_t>(lane.sf - 2) : lane.sf;
                auto payload_nibbles = lane.processBlock(
                    lane.symbol_buffer, sf_app, cw_len, lane.cr, false);
                lane.nibbles.insert(lane.nibbles.end(),
                                     payload_nibbles.begin(), payload_nibbles.end());
                lane.symbol_buffer.clear();
            }

            // Frame complete
            if (lane.symb_numb > 0 && lane.total_symbols_rx >= lane.symb_numb) {
                finishFrame(lane, out_span, out_idx);
                lane.resetToDetect();
            }
        }
    }

    /// Flush remaining symbols, decode frame, emit output bytes + tags + message.
    void finishFrame(SfLane& lane, std::span<uint8_t> out_span, std::size_t& out_idx) {
        // Flush remaining symbol buffer
        if (!lane.symbol_buffer.empty()) {
            uint8_t cw_len = 4 + lane.cr;
            uint8_t sf_app = lane.ldro ? static_cast<uint8_t>(lane.sf - 2) : lane.sf;
            while (lane.symbol_buffer.size() < cw_len) {
                lane.symbol_buffer.push_back(0);
            }
            auto last_nibbles = lane.processBlock(
                lane.symbol_buffer, sf_app, cw_len, lane.cr, false);
            lane.nibbles.insert(lane.nibbles.end(),
                                 last_nibbles.begin(), last_nibbles.end());
            lane.symbol_buffer.clear();
        }

        auto frame = lane.decodeFrame();
        if (frame.payload.empty()) return;

        // Copy to output
        for (uint32_t i = 0; i < frame.pay_len && out_idx < out_span.size(); i++) {
            if (i < frame.payload.size()) {
                out_span[out_idx++] = frame.payload[i];
            }
        }

        // Publish output tag
        gr::property_map out_tag;
        out_tag["sf"]            = gr::pmt::Value(static_cast<int64_t>(lane.sf));
        out_tag["pay_len"]       = gr::pmt::Value(static_cast<int64_t>(frame.pay_len));
        out_tag["cr"]            = gr::pmt::Value(static_cast<int64_t>(frame.cr));
        out_tag["crc_valid"]     = gr::pmt::Value(frame.crc_valid);
        out_tag["is_downchirp"]  = gr::pmt::Value(false);
        out_tag["snr_db"]        = gr::pmt::Value(static_cast<double>(lane.snr_db));
        if (_noise_floor_db > -999.f) {
            out_tag["noise_floor_db"] = gr::pmt::Value(static_cast<double>(_noise_floor_db));
        }
        if (_peak_db > -999.f) {
            out_tag["peak_db"] = gr::pmt::Value(static_cast<double>(_peak_db));
        }
        if (rx_channel >= 0) {
            out_tag["rx_channel"] = gr::pmt::Value(static_cast<int64_t>(rx_channel));
        }
        this->publishTag(out_tag, 0UZ);

        // Publish message
        std::string payload_str;
        for (uint32_t i = 0; i < frame.pay_len && i < frame.payload.size(); i++) {
            payload_str.push_back(static_cast<char>(frame.payload[i]));
        }
        gr::property_map msg_data;
        msg_data["payload"]       = gr::pmt::Value(payload_str);
        msg_data["crc_valid"]     = gr::pmt::Value(frame.crc_valid);
        msg_data["is_downchirp"]  = gr::pmt::Value(false);
        gr::sendMessage<gr::message::Command::Notify>(msg_out, "", "payload", msg_data);

        if (debug) {
            log_ts("info ", "MultiSf",
                   "SF%u frame: pay_len=%u cr=%u crc=%s snr=%.1f dB",
                   lane.sf, frame.pay_len, frame.cr,
                   frame.crc_valid ? "OK" : "FAIL",
                   static_cast<double>(lane.snr_db));
        }
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_MULTI_SF_DECODER_HPP
