// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_ALGORITHM_PREAMBLE_ID_HPP
#define GNURADIO_LORA_ALGORITHM_PREAMBLE_ID_HPP

// Standalone preamble characterization: runs a single SfLane DETECT→SYNC
// state machine on a captured IQ buffer (at os_factor rate) and extracts
// SF, sync word, CFO, STO, SFO, preamble length, and SNR.
//
// No GR4 block dependency — uses algorithm headers only.

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/algorithm/SfLaneDetail.hpp>
#include <gnuradio-4.0/lora/algorithm/utilities.hpp>

namespace gr::lora {

/// Result of preamble characterization.
struct PreambleInfo {
    uint8_t  sf{0};           // detected SF (7-12)
    uint32_t bw{0};           // bandwidth (Hz)
    uint16_t sync_word{0};    // reconstructed sync word (e.g. 0x12, 0x34)
    float    cfo_frac{0.f};   // fractional CFO (bins)
    int      cfo_int{0};      // integer CFO (bins)
    float    sto_frac{0.f};   // fractional STO (samples)
    float    sfo_hat{0.f};    // sampling frequency offset
    uint32_t preamble_len{0}; // detected number of preamble upchirps
    float    snr_db{-999.f};  // estimated SNR (dB)
    float    pmr{0.f};        // peak-to-mean ratio at detection
    bool     valid{false};    // true if SYNC completed (sync word verified)
};

namespace preamble_detail {

using cf32 = std::complex<float>;

/// Dechirp one symbol at 1× rate and return (peak_bin, peak_to_mean_ratio).
inline std::pair<uint32_t, float> dechirp_quality(const cf32* samples, const cf32* ref_chirp, cf32* scratch, uint32_t N, gr::algorithm::FFT<cf32>& fft) {
    sflane_detail::complex_multiply(scratch, samples, ref_chirp, N);
    auto     fft_out = fft.compute(std::span<const cf32>(scratch, N));
    float    total = 0.f, peak = 0.f;
    uint32_t peak_bin = 0;
    for (uint32_t j = 0; j < N; j++) {
        float mag_sq = fft_out[j].real() * fft_out[j].real() + fft_out[j].imag() * fft_out[j].imag();
        total += mag_sq;
        if (mag_sq > peak) {
            peak     = mag_sq;
            peak_bin = j;
        }
    }
    float mean = total / static_cast<float>(N);
    float pmr  = (mean > 0.f) ? peak / mean : 0.f;
    return {peak_bin, pmr};
}

/// Reconstruct sync word from two demodulated net_id symbol values.
/// Sync words are encoded as: sw0 = (high_nibble << 3), sw1 = (low_nibble << 3).
inline uint16_t reconstruct_sync_word(int netid1, int netid2, uint32_t N) {
    // Round to nearest multiple of 8
    int     sw0 = static_cast<int>(mod(static_cast<int64_t>(std::lround(netid1 / 8.0) * 8), static_cast<int64_t>(N)));
    int     sw1 = static_cast<int>(mod(static_cast<int64_t>(std::lround(netid2 / 8.0) * 8), static_cast<int64_t>(N)));
    uint8_t hi  = static_cast<uint8_t>((sw0 >> 3) & 0x0F);
    uint8_t lo  = static_cast<uint8_t>((sw1 >> 3) & 0x0F);
    return static_cast<uint16_t>((hi << 4) | lo);
}

} // namespace preamble_detail

/// Run preamble DETECT + SYNC on a captured narrowband IQ buffer and extract
/// full preamble characterization.
///
/// @param samples    Narrowband IQ at os_factor × BW rate
/// @param nSamples   Number of samples in buffer
/// @param sf         Candidate spreading factor (from prior CAD detection)
/// @param bandwidth  Channel bandwidth (Hz)
/// @param os_factor  Oversampling factor (typically 4)
/// @param center_freq Center frequency for SFO estimation (Hz)
/// @param preamble_len Expected preamble length (default 8 upchirps)
///
/// The function does NOT require a known sync word — it discovers it.
inline PreambleInfo characterize_preamble(const std::complex<float>* samples, std::size_t nSamples, uint8_t sf, uint32_t bandwidth, uint8_t os_factor, double center_freq = 868e6, uint16_t preamble_len = 8) {

    using cf32 = std::complex<float>;
    PreambleInfo info;
    info.sf = sf;
    info.bw = bandwidth;

    const uint32_t N   = 1U << sf;
    const uint32_t sps = N * os_factor;

    // We need at least (preamble_len + 5) symbols to run DETECT + SYNC
    // (preamble + 2 sync + 2 downchirp + quarter)
    const std::size_t minSymbols = static_cast<std::size_t>(preamble_len) + 5U;
    if (nSamples < minSymbols * sps) {
        return info; // buffer too small
    }

    // Build reference chirps at 1× rate (for dechirp after decimation)
    std::vector<cf32> upchirp(N), downchirp(N);
    build_ref_chirps(upchirp.data(), downchirp.data(), sf);

    gr::algorithm::FFT<cf32> fft;
    std::vector<cf32>        scratch(N);

    // n_up_req: preamble symbols needed before entering SYNC
    const uint32_t n_up_req       = (preamble_len >= 3) ? preamble_len - 3 : 0U;
    const uint32_t up_symb_to_use = (n_up_req >= 1) ? n_up_req - 1 : 0U;

    // ==== DETECT phase: find consecutive upchirps ====
    // Downsample each symbol period and dechirp to find preamble peak bin

    std::vector<cf32> in_down(N);
    std::vector<int>  preamb_vals;
    preamb_vals.reserve(preamble_len + 4);

    // Storage for preamble raw samples at 1× rate (for CFO/STO estimation)
    std::vector<cf32> preamble_raw(static_cast<std::size_t>(preamble_len + 4) * N);

    int32_t     prev_bin     = -1;
    uint32_t    consec_count = 0;
    std::size_t detect_pos   = 0; // sample position of detection start

    const std::size_t maxSymbols = nSamples / sps;

    for (std::size_t sym = 0; sym < maxSymbols; ++sym) {
        // Downsample: take every os_factor-th sample
        const cf32* sym_start = samples + sym * sps;
        for (uint32_t i = 0; i < N; ++i) {
            in_down[i] = sym_start[i * os_factor];
        }

        auto [bin, pmr] = preamble_detail::dechirp_quality(in_down.data(), downchirp.data(), scratch.data(), N, fft);

        if (pmr < sflane_detail::min_preamble_pmr(sf)) {
            consec_count = 0;
            prev_bin     = -1;
            preamb_vals.clear();
            continue;
        }

        auto new_bin = static_cast<int32_t>(bin);

        // Check if this bin is consistent with previous (±1 tolerance)
        if (prev_bin >= 0 && std::abs(mod(std::abs(new_bin - prev_bin) + 1, static_cast<int64_t>(N)) - 1) <= sflane_detail::kPreambleBinTolerance) {
            consec_count++;
            preamb_vals.push_back(new_bin);
            // Store 1× rate samples
            if (consec_count * N < preamble_raw.size()) {
                std::copy_n(in_down.data(), N, &preamble_raw[consec_count * N]);
            }
        } else {
            // Reset
            consec_count = 1;
            preamb_vals.clear();
            preamb_vals.push_back(new_bin);
            detect_pos = sym * sps;
            // Store first symbol
            std::copy_n(in_down.data(), N, &preamble_raw[0]);
        }
        prev_bin = new_bin;
        info.pmr = pmr;

        // Enough consecutive upchirps detected?
        if (consec_count >= n_up_req) {
            info.preamble_len = consec_count;
            break;
        }
    }

    if (consec_count < n_up_req) {
        return info; // preamble not found
    }

    // k_hat: most frequent preamble bin (integer STO estimate)
    int k_hat = sflane_detail::most_frequent(preamb_vals);

    // ==== CFO fractional estimation ====
    // Use preamble symbols starting after the first one, skip the STO-offset symbol.
    // Clamp to the actually-filled portion of preamble_raw (consec_count symbols).
    const std::size_t filled     = static_cast<std::size_t>(consec_count) * N;
    std::size_t       cfo_offset = N + static_cast<std::size_t>(std::max(static_cast<int>(N) - k_hat, 0));
    if (cfo_offset + up_symb_to_use * N > filled) {
        cfo_offset = (filled > up_symb_to_use * N) ? filled - up_symb_to_use * N : 0;
    }

    // Estimate CFO_frac from cross-symbol correlation
    std::vector<cf32> fft_val_buf(up_symb_to_use * N);
    float             k0_best_mag = 0.f;
    uint32_t          idx_max     = 0;
    for (uint32_t i = 0; i < up_symb_to_use; i++) {
        sflane_detail::complex_multiply(scratch.data(), &preamble_raw[cfo_offset + i * N], downchirp.data(), N);
        auto  fft_out  = fft.compute(std::span<const cf32>(scratch.data(), N));
        float best     = 0.f;
        int   best_idx = 0;
        for (uint32_t j = 0; j < N; j++) {
            float mag_sq           = fft_out[j].real() * fft_out[j].real() + fft_out[j].imag() * fft_out[j].imag();
            fft_val_buf[j + i * N] = fft_out[j];
            if (mag_sq > best) {
                best     = mag_sq;
                best_idx = static_cast<int>(j);
            }
        }
        if (best > k0_best_mag) {
            k0_best_mag = best;
            idx_max     = static_cast<uint32_t>(best_idx);
        }
    }

    cf32 four_cum(0.f, 0.f);
    for (uint32_t i = 0; i + 1 < up_symb_to_use; i++) {
        four_cum += fft_val_buf[idx_max + N * i] * std::conj(fft_val_buf[idx_max + N * (i + 1)]);
    }
    float cfo_frac = -std::arg(four_cum) / (2.f * static_cast<float>(std::numbers::pi));
    info.cfo_frac  = cfo_frac;

    // CFO-correct the preamble upchirps
    std::vector<cf32> preamble_upchirps(up_symb_to_use * N);
    std::vector<cf32> corr_vec(std::max<std::size_t>(static_cast<std::size_t>(n_up_req + 4) * N, 2 * N));
    for (uint32_t n = 0; n < up_symb_to_use * N; n++) {
        float phase = -2.f * static_cast<float>(std::numbers::pi) * cfo_frac / static_cast<float>(N) * static_cast<float>(n);
        corr_vec[n] = cf32(std::cos(phase), std::sin(phase));
    }
    sflane_detail::complex_multiply(preamble_upchirps.data(), &preamble_raw[cfo_offset], corr_vec.data(), up_symb_to_use * N);

    // ==== STO fractional estimation (Xhonneux Eq. 20) ====
    uint32_t             pk = static_cast<uint32_t>(std::max(0, k_hat)) % N;
    std::complex<double> Y_m1(0, 0), Y_0(0, 0), Y_p1(0, 0);
    for (uint32_t i = 0; i < up_symb_to_use; i++) {
        sflane_detail::complex_multiply(scratch.data(), &preamble_upchirps[N * i], downchirp.data(), N);
        auto fft_out = fft.compute(std::span<const cf32>(scratch.data(), N));
        auto to_d    = [](cf32 c) { return std::complex<double>(static_cast<double>(c.real()), static_cast<double>(c.imag())); };
        Y_m1 += to_d(fft_out[(pk + N - 1) % N]);
        Y_0 += to_d(fft_out[pk]);
        Y_p1 += to_d(fft_out[(pk + 1) % N]);
    }

    uint32_t M_hat    = (N - pk) % N;
    double   phase_d  = 2.0 * std::numbers::pi * static_cast<double>(M_hat) / static_cast<double>(N);
    auto     e_pos    = std::complex<double>(std::cos(phase_d), std::sin(phase_d));
    auto     e_neg    = std::complex<double>(std::cos(phase_d), -std::sin(phase_d));
    auto     num      = e_neg * Y_p1 - e_pos * Y_m1;
    auto     den      = 2.0 * Y_0 - e_neg * Y_p1 - e_pos * Y_m1;
    float    sto_frac = (std::abs(den) > 1e-15) ? std::clamp(static_cast<float>(-std::real(num / den)), -0.5f, 0.5f) : 0.f;
    info.sto_frac     = sto_frac;

    // ==== SYNC phase: navigate past preamble to net_id / downchirp symbols ====
    // Mirrors MultiSfDecoder::processDetect SYNC transition + processSync.
    //
    // After n_up_req consecutive upchirps, the detector transitions to SYNC.
    // The last detected upchirp ends at sample position:
    //   last_up_end = detect_pos + n_up_req * sps
    // But the preamble peak bin (k_hat) shifts the boundary:
    //   items_to_consume = os_factor * (N - k_hat)
    // So the "aligned" position after detection is:
    //   aligned_pos = last_up_end - k_hat * os_factor
    //
    // From there, the pipeline advances one sps per SYNC phase step.
    // NET_ID1 starts with ¼ symbol collected from the DETECT→SYNC transition.
    info.preamble_len = n_up_req;

    // Position of the last preamble upchirp symbol (oversampled buffer)
    std::size_t last_up_pos = detect_pos + static_cast<std::size_t>(n_up_req - 1) * sps;

    // Scan for additional upchirps (beyond n_up_req) — same as NET_ID1 phase
    // in processSync: bin near 0 means another upchirp.
    uint32_t    additional_ups = 0;
    std::size_t cursor         = last_up_pos + sps; // first symbol after n_up_req
    for (uint32_t extra = 0; extra < 4; ++extra) {
        if (cursor + sps > nSamples) {
            break;
        }
        const cf32* sym_start = samples + cursor;
        for (uint32_t i = 0; i < N; ++i) {
            in_down[i] = sym_start[i * os_factor];
        }
        // CFO frac correction
        for (uint32_t n = 0; n < N; n++) {
            float ph   = -2.f * static_cast<float>(std::numbers::pi) * cfo_frac / static_cast<float>(N) * static_cast<float>(n);
            scratch[n] = in_down[n] * cf32(std::cos(ph), std::sin(ph));
        }
        auto bin  = dechirp_argmax(scratch.data(), downchirp.data(), scratch.data(), N, fft);
        auto ibin = static_cast<int32_t>(bin);
        if (ibin == 0 || ibin == 1 || static_cast<uint32_t>(ibin) == N - 1) {
            additional_ups++;
            cursor += sps;
        } else {
            break;
        }
    }
    info.preamble_len += additional_ups;

    // `cursor` now points to NET_ID1 symbol start in the oversampled buffer.
    // Collect 2.5 symbols of oversampled data for net_id demod.
    const std::size_t net_id_len = sps * 2 + sps / 2;
    if (cursor + net_id_len + 2 * sps > nSamples) {
        return info; // not enough data for full sync
    }
    std::vector<cf32> net_id_samp(samples + cursor, samples + cursor + net_id_len);

    // Downchirp symbol: 2 symbols after net_id1 start
    std::size_t down_sym_pos = cursor + 2 * sps;
    for (uint32_t i = 0; i < N; ++i) {
        in_down[i] = samples[down_sym_pos + i * os_factor];
    }
    for (uint32_t n = 0; n < N; n++) {
        float ph   = -2.f * static_cast<float>(std::numbers::pi) * cfo_frac / static_cast<float>(N) * static_cast<float>(n);
        scratch[n] = in_down[n] * cf32(std::cos(ph), std::sin(ph));
    }
    int down_val = static_cast<int>(dechirp_argmax(scratch.data(), upchirp.data(), scratch.data(), N, fft));

    // ==== Integer CFO/STO separation (Xhonneux §5.2) ====
    auto sum_cfo = static_cast<int>(mod(static_cast<int64_t>(k_hat) + static_cast<int64_t>(down_val), static_cast<int64_t>(N)));
    if (static_cast<uint32_t>(sum_cfo) >= N / 2) {
        sum_cfo -= static_cast<int>(N);
    }
    int cfo_int  = sum_cfo / 2;
    info.cfo_int = cfo_int;

    // SFO estimation
    float sfo    = (static_cast<float>(cfo_int) + cfo_frac) * static_cast<float>(bandwidth) / static_cast<float>(center_freq);
    info.sfo_hat = sfo;

    // ==== Demodulate sync word symbols ====
    // Decimate net_id samples with timing correction (matches MultiSfDecoder QUARTER_DOWN).
    // The net_id_samp buffer starts at the net_id1 symbol boundary.
    // Decimation offset accounts for: os/2 centering, STO correction, N/4 quarter-symbol
    // offset, and CFO integer offset.
    std::vector<cf32> net_id_dec(2 * N, cf32(0.f, 0.f));
    int               os_i      = static_cast<int>(os_factor);
    int               start_off = os_i / 2 - sflane_detail::my_roundf(sto_frac * static_cast<float>(os_factor));
    for (uint32_t i = 0; i < 2 * N; i++) {
        int idx = start_off + static_cast<int>(i) * os_i;
        if (idx >= 0 && static_cast<std::size_t>(idx) < net_id_samp.size()) {
            net_id_dec[i] = net_id_samp[static_cast<std::size_t>(idx)];
        }
    }

    // Apply CFO correction (integer + fractional)
    for (uint32_t n = 0; n < 2 * N; n++) {
        float ph = -2.f * static_cast<float>(std::numbers::pi) * (static_cast<float>(cfo_int) + cfo_frac) / static_cast<float>(N) * static_cast<float>(n);
        net_id_dec[n] *= cf32(std::cos(ph), std::sin(ph));
    }

    int netid1 = static_cast<int>(dechirp_argmax(net_id_dec.data(), downchirp.data(), scratch.data(), N, fft));
    int netid2 = static_cast<int>(dechirp_argmax(&net_id_dec[N], downchirp.data(), scratch.data(), N, fft));

    // Mod-8 STO correction (sync-word-independent, gateway_arch §3)
    {
        uint32_t r1 = static_cast<uint32_t>(mod(static_cast<int64_t>(netid1), 8LL));
        uint32_t r2 = static_cast<uint32_t>(mod(static_cast<int64_t>(netid2), 8LL));
        if (r1 == r2 && r1 != 0) {
            int delta = static_cast<int>(r1);
            if (delta > 3) {
                delta -= 8;
            }
            netid1 = static_cast<int>(mod(static_cast<int64_t>(netid1) - delta, static_cast<int64_t>(N)));
            netid2 = static_cast<int>(mod(static_cast<int64_t>(netid2) - delta, static_cast<int64_t>(N)));
        }
    }

    info.sync_word = preamble_detail::reconstruct_sync_word(netid1, netid2, N);

    // ==== SNR estimation ====
    // For clean signals, all energy concentrates in one FFT bin → noise ≈ 0.
    // Cap at +60 dB (practical upper bound for clean synthetic signals).
    constexpr float kMaxSnrDb = 60.f;
    float           snr_sum   = 0.f;
    uint32_t        snr_count = 0;
    for (uint32_t i = 0; i < up_symb_to_use; i++) {
        sflane_detail::complex_multiply(scratch.data(), &preamble_upchirps[N * i], downchirp.data(), N);
        auto  fft_out      = fft.compute(std::span<const cf32>(scratch.data(), N));
        float total_energy = 0.f, peak_energy = 0.f;
        for (uint32_t j = 0; j < N; j++) {
            float mag_sq = fft_out[j].real() * fft_out[j].real() + fft_out[j].imag() * fft_out[j].imag();
            total_energy += mag_sq;
            if (mag_sq > peak_energy) {
                peak_energy = mag_sq;
            }
        }
        if (peak_energy <= 0.f) {
            continue; // skip zero-energy symbols
        }
        float noise = total_energy - peak_energy;
        if (noise > 0.f) {
            snr_sum += std::min(10.f * std::log10(peak_energy / noise), kMaxSnrDb);
        } else {
            snr_sum += kMaxSnrDb; // clean signal: all energy in one bin
        }
        snr_count++;
    }
    info.snr_db = (snr_count > 0) ? snr_sum / static_cast<float>(snr_count) : -999.f;

    info.valid = true;
    return info;
}

} // namespace gr::lora

#endif // GNURADIO_LORA_ALGORITHM_PREAMBLE_ID_HPP
