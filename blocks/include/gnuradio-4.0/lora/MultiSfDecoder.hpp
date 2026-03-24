// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_MULTI_SF_DECODER_HPP
#define GNURADIO_LORA_MULTI_SF_DECODER_HPP

#include <array>
#include <cassert>
#include <string>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/lora/algorithm/DCBlocker.hpp>
#include <gnuradio-4.0/lora/algorithm/SfLane.hpp>
#include <gnuradio-4.0/lora/algorithm/SpectrumTap.hpp>
#include <gnuradio-4.0/lora/algorithm/Telemetry.hpp>
#include <gnuradio-4.0/lora/log.hpp>

namespace gr::lora {

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
    float    dc_blocker_cutoff = 0.f;  ///< DC blocker cutoff Hz (0 = disabled)
    bool     debug        = false;

    GR_MAKE_REFLECTABLE(MultiSfDecoder, in, out, msg_out,
        bandwidth, sync_word, os_factor, preamble_len,
        energy_thresh, min_snr_db, max_symbols, center_freq,
        rx_channel, sf_min, sf_max, dc_blocker_cutoff, debug);

    std::vector<SfLane> _lanes;
    DCBlocker _dc;
    std::vector<cf32> _dcBuf;
    uint32_t _min_sps{0};
    float    _noise_floor_db{-999.f};
    float    _peak_db{-999.f};
    float    _noise_ema{0.f};
    bool     _noise_ema_init{false};
    std::atomic<bool>* _channel_busy{nullptr};  // LBT: set by any lane in SYNC/OUTPUT
    std::shared_ptr<SpectrumState> _spectrum_state;  // spectrum tap for waterfall display
    std::function<void(const gr::property_map&)> _telemetry;  // telemetry callback

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

        // DC blocker (embedded, applied before spectrum + decode)
        if (dc_blocker_cutoff > 0.f && bandwidth > 0) {
            float sr = static_cast<float>(bandwidth) * static_cast<float>(os_factor);
            _dc.init(sr, dc_blocker_cutoff);
            _dcBuf.reserve(8192);
        }
    }

    [[nodiscard]] gr::work::Status processBulk(
            gr::InputSpanLike auto& input,
            gr::OutputSpanLike auto& output) noexcept {
        auto in_span  = std::span(input);
        auto out_span = std::span(output);

        if (_lanes.empty()) reconfigure();

        // Handle overflow tags: push zeros to preserve symbol boundary alignment.
        // At 250 kS/s overflows are rare, but when they occur the missing samples
        // shift all subsequent symbol boundaries, corrupting dechirp+FFT output.
        if (this->inputTagsPresent()) {
            const auto& tag = this->mergedInputTag();
            if (tag.map.contains("overflow") && in_span.empty() && _min_sps > 0) {
                for (auto& lane : _lanes) {
                    lane.accum.resize(lane.accum.size() + _min_sps, cf32{0.f, 0.f});
                }
                std::ignore = input.consume(0UZ);
                output.publish(0UZ);
                return gr::work::Status::OK;
            }
        }

        if (in_span.size() < _min_sps) {
            std::ignore = input.consume(0UZ);
            output.publish(0UZ);
            return gr::work::Status::OK;
        }

        // DC blocker: filter in-place before spectrum + decode
        std::span<const cf32> clean_span = in_span;
        if (_dc.initialised()) {
            _dcBuf.resize(in_span.size());
            _dc.processBlock(in_span, std::span<cf32>(_dcBuf));
            clean_span = std::span<const cf32>(_dcBuf);
        }

        // Push DC-cleaned IQ to spectrum tap (waterfall display)
        if (_spectrum_state) {
            _spectrum_state->push(clean_span.data(),
                std::min(clean_span.size(), static_cast<std::size_t>(_min_sps)));
        }

        // Always consume the full input. Each lane appends to its own
        // accumulation buffer and processes symbols from there. This keeps
        // the ring buffer cursor advancing at full rate so the scheduler
        // stays in its fast path (no stall/sleep on consume(0)).
        const std::size_t n_in = in_span.size();
        std::size_t out_idx = 0;

        for (auto& lane : _lanes) {
            // Append DC-cleaned input to this lane's accumulation buffer
            lane.accum.insert(lane.accum.end(), clean_span.begin(), clean_span.end());

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
            lane.scratch_N.data(), lane.N, lane.fft, /*remove_dc=*/!_dc.initialised());
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

            if (_telemetry) {
                try {
                    gr::property_map evt;
                    evt["type"] = std::pmr::string("multisf_detect");
                    evt["sf"]   = static_cast<uint32_t>(lane.sf);
                    evt["bin"]  = static_cast<uint32_t>(lane.bin_idx_new);
                    _telemetry(evt);
                } catch (...) {}
            }

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
            lane.cfo_frac = lane.estimate_CFO_frac(&lane.preamble_raw[cfo_offset]);
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

            // Re-estimate STO frac after SFO correction.
            // After CFO_int rotation + SFO correction, peak is near bin 0.
            float tmp_sto_frac = lane.estimate_STO_frac(0);
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

            // Mod-8 sync-word-independent STO correction (gateway_arch §3).
            // Sync word values are nibble<<3, so valid demod values are multiples of 8.
            // Non-zero mod-8 residual reveals residual STO not captured by integer estimate.
            {
                uint32_t r1 = static_cast<uint32_t>(mod(static_cast<int64_t>(netid1), 8LL));
                uint32_t r2 = static_cast<uint32_t>(mod(static_cast<int64_t>(netid2), 8LL));
                if (r1 == r2 && r1 != 0) {
                    int delta = static_cast<int>(r1);
                    if (delta > 3) delta -= 8;  // wrap: 4→-4, 5→-3, 6→-2, 7→-1
                    netid1 = static_cast<int>(mod(
                        static_cast<int64_t>(netid1) - delta, static_cast<int64_t>(lane.N)));
                    netid2 = static_cast<int>(mod(
                        static_cast<int64_t>(netid2) - delta, static_cast<int64_t>(lane.N)));
                }
            }

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

            if (_telemetry) {
                try {
                    gr::property_map evt;
                    evt["type"]     = std::pmr::string("multisf_sync");
                    evt["sf"]       = static_cast<uint32_t>(lane.sf);
                    evt["cfo_int"]  = static_cast<int32_t>(lane.cfo_int);
                    evt["cfo_frac"] = lane.cfo_frac;
                    evt["snr_db"]   = static_cast<double>(lane.snr_db);
                    _telemetry(evt);
                } catch (...) {}
            }

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
        uint16_t symbol;
        if constexpr (SfLane::kUseSoftDecode) {
            symbol = lane.demodSymbolSoft(lane.in_down.data());
        } else {
            symbol = lane.demodSymbol(lane.in_down.data());
        }
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

        if (_telemetry) {
            try {
                gr::property_map evt;
                evt["type"]   = std::pmr::string("multisf_frame");
                evt["sf"]     = static_cast<uint32_t>(lane.sf);
                evt["crc_ok"] = frame.crc_valid;
                evt["len"]    = static_cast<uint32_t>(frame.pay_len);
                evt["cr"]     = static_cast<uint32_t>(frame.cr);
                evt["snr_db"] = static_cast<double>(lane.snr_db);
                _telemetry(evt);
            } catch (...) {}
        }

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
