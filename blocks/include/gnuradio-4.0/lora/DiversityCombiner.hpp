// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_DIVERSITY_COMBINER_HPP
#define GNURADIO_LORA_DIVERSITY_COMBINER_HPP

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

namespace gr::lora {

// N-input diversity combiner for LoRa RX decode chains.
//
// Each input receives decoded payload bytes (uint8_t) from an independent
// DemodDecoder, with per-frame tags (pay_len, crc_valid, snr_db, rx_channel).
// The combiner accumulates complete frames per channel, matches them by
// payload hash within a timing window, selects the best decode (CRC_OK
// preferred, then highest SNR), and emits the winner on a single output.
//
// For N=1, the block acts as a simple passthrough (no combining overhead).
GR_REGISTER_BLOCK("gr::lora::DiversityCombiner", gr::lora::DiversityCombiner)
struct DiversityCombiner : gr::Block<DiversityCombiner, gr::NoDefaultTagForwarding> {
    using Description = Doc<"N-input diversity combiner — selects best LoRa RX decode by CRC/SNR">;
    using clock = std::chrono::steady_clock;

    struct ChannelState {
        std::vector<uint8_t> payload;
        gr::property_map     tag;
        uint32_t             pay_len{0};
        bool                 collecting{false};
        bool                 crc_valid{false};
        double               snr_db{0.0};
        int64_t              rx_channel{-1};
    };

    struct Candidate {
        std::vector<uint8_t> payload;
        gr::property_map     tag;
        bool                 crc_valid{false};
        double               snr_db{0.0};
        int64_t              rx_channel{-1};
    };

    struct PendingGroup {
        uint64_t                     payload_hash{0};
        clock::time_point            created_at;
        clock::time_point            deadline;
        std::vector<Candidate>       candidates;
        std::size_t                  best_idx{0};
        uint32_t                     age{0};    // scheduler cycles since creation
    };

    std::vector<gr::PortIn<uint8_t, gr::Async>> in{};
    gr::PortOut<uint8_t, gr::Async>              out;

    Annotated<gr::Size_t, "n_inputs", gr::Visible, gr::Limits<1U, 8U>> n_inputs = 2U;
    uint32_t timeout_symbols = 30;
    uint32_t bandwidth       = 62500;
    uint8_t  sf              = 8;
    bool     debug           = false;

    GR_MAKE_REFLECTABLE(DiversityCombiner, in, out,
                        n_inputs, timeout_symbols, bandwidth, sf, debug);

    std::vector<ChannelState> _channels;
    std::vector<PendingGroup> _pending;

    void start() {
        _channels.resize(n_inputs);
        _pending.clear();
        _pending.reserve(4);
    }

    template<gr::InputSpanLike TInSpan, gr::OutputSpanLike TOutSpan>
    gr::work::Status processBulk(std::span<TInSpan>& ins, TOutSpan& outSpan) {
        auto now = clock::now();
        std::size_t nPublished = 0;

        // Age all pending groups at the start of each scheduler cycle.
        // Groups created this cycle start at age 0 (set in submitFrame).
        // A group at age >= 4 has survived multiple full cycles without
        // completing — a safety fallback for cases where the real-time
        // deadline hasn't expired. The primary drain mechanism is the
        // timeout_symbols deadline (condition b).
        for (auto& group : _pending) {
            group.age++;
        }

        // Track which channels had activity (data or tags) this cycle.
        // A channel with no activity is still processing upstream — its
        // decoder hasn't produced output yet. A channel WITH activity that
        // doesn't submit a matching candidate has made its decision (decoded
        // something else, or nothing).
        std::vector<bool> channelActive(ins.size(), false);

        // 1. Process each input channel: read tags, accumulate bytes
        for (std::size_t i = 0; i < ins.size() && i < _channels.size(); i++) {
            auto& inSpan = ins[i];
            auto& ch = _channels[i];

            if (!inSpan.rawTags.empty() || inSpan.size() > 0) {
                channelActive[i] = true;
            }

            // Read tags from this async input
            for (const auto& rawTag : inSpan.rawTags) {
                auto relIdx = static_cast<std::ptrdiff_t>(rawTag.index)
                            - static_cast<std::ptrdiff_t>(inSpan.streamIndex);
                if (relIdx < 0 || relIdx >= static_cast<std::ptrdiff_t>(inSpan.size()))
                    continue;

                if (auto it = rawTag.map.find("pay_len"); it != rawTag.map.end()) {
                    if (ch.collecting && !ch.payload.empty()) {
                        ch.pay_len = static_cast<uint32_t>(ch.payload.size());
                        submitFrame(i);
                    }
                    ch.pay_len = static_cast<uint32_t>(
                        it->second.template value_or<int64_t>(0));
                    ch.tag = rawTag.map;
                    applyTagFields(rawTag.map, ch);
                    ch.payload.clear();
                    ch.payload.reserve(ch.pay_len);
                    ch.collecting = true;
                }
            }

            // Accumulate bytes
            if (ch.collecting) {
                std::size_t need = ch.pay_len > ch.payload.size()
                                 ? ch.pay_len - ch.payload.size() : 0;
                std::size_t take = std::min(need, inSpan.size());
                ch.payload.insert(ch.payload.end(),
                                  inSpan.begin(),
                                  inSpan.begin() + static_cast<std::ptrdiff_t>(take));

                if (ch.payload.size() >= ch.pay_len) {
                    submitFrame(i);
                    ch.collecting = false;
                }
            }

            std::ignore = inSpan.consume(inSpan.size());
            inSpan.consumeTags(inSpan.size());
        }

        // 2. Emit pending groups that are ready.
        // A group is emitted when any of these conditions is true:
        //   (a) All N candidates have been received (complete)
        //   (b) The real-time deadline has expired (timeout)
        //   (c) N=1 (single-input passthrough, no combining needed)
        //   (d) The group's age >= 4 (safety fallback — survived multiple
        //       cycles; the real-time deadline is the primary drain)
        //   (e) All input channels were active this cycle but fewer than N
        //       candidates were collected (every channel had its chance this
        //       cycle — channels that were active but didn't contribute a
        //       matching candidate decoded something else or nothing)
        bool allChannelsActive = std::ranges::all_of(channelActive,
            [](bool active) { return active; });

        auto outIt = outSpan.begin() + static_cast<std::ptrdiff_t>(nPublished);
        std::size_t outAvail = outSpan.size() - nPublished;

        for (auto it = _pending.begin(); it != _pending.end(); ) {
            bool allReported = it->candidates.size()
                               >= static_cast<std::size_t>(n_inputs);
            bool expired = now >= it->deadline;
            bool singleInput = (n_inputs == 1U);
            bool aged = (it->age >= 4);
            bool resolvedByActivity = allChannelsActive && !allReported;

            if (allReported || expired || singleInput || aged || resolvedByActivity) {
                auto& winner = it->candidates[it->best_idx];
                std::size_t payLen = winner.payload.size();

                if (payLen <= outAvail) {
                    auto outTag = buildOutputTag(*it);
                    outSpan.publishTag(outTag, nPublished);

                    std::copy_n(winner.payload.begin(), payLen, outIt);
                    outIt += static_cast<std::ptrdiff_t>(payLen);
                    nPublished += payLen;
                    outAvail -= payLen;

                    if (debug && !allReported) {
                        const char* reason = aged ? "aged" :
                            resolvedByActivity ? "all-active" :
                            expired ? "timeout" : "drain";
                        std::fprintf(stderr, "[DiversityCombiner] %s: "
                            "emitting with %zu/%u candidates (age=%u)\n",
                            reason,
                            it->candidates.size(),
                            static_cast<unsigned>(n_inputs),
                            it->age);
                    }

                    it = _pending.erase(it);
                } else {
                    ++it;
                }
            } else {
                ++it;
            }
        }

        outSpan.publish(nPublished);
        return gr::work::Status::OK;
    }

    void stop() {
        // Flush any remaining pending groups on shutdown so frames decoded
        // by at least one channel are not silently lost.
        for (auto& group : _pending) {
            auto& winner = group.candidates[group.best_idx];
            if (debug) {
                std::fprintf(stderr, "[DiversityCombiner] stop: flushing "
                    "group hash=0x%016llx with %zu/%u candidates\n",
                    static_cast<unsigned long long>(group.payload_hash),
                    group.candidates.size(),
                    static_cast<unsigned>(n_inputs));
            }
            // Cannot publish to output port during stop() — the graph is
            // already torn down. Log the loss for diagnostics.
            std::ignore = winner;
        }
        _pending.clear();
    }

    void settingsChanged(const gr::property_map& /*oldSettings*/,
                         const gr::property_map& newSettings) {
        if (newSettings.contains("n_inputs")) {
            in.resize(n_inputs);
            _channels.resize(n_inputs);
        }
    }

    // FNV-1a 64-bit hash -- fast, good distribution, no dependencies.
    static uint64_t hashPayload(const std::vector<uint8_t>& data) {
        uint64_t hash = 14695981039346656037ULL;
        for (auto byte : data) {
            hash ^= byte;
            hash *= 1099511628211ULL;
        }
        return hash;
    }

    static bool isBetterCandidate(const Candidate& a, const Candidate& b) {
        if (a.crc_valid != b.crc_valid) return a.crc_valid;
        return a.snr_db > b.snr_db;
    }

    double timeoutMs() const {
        double symbol_ms = static_cast<double>(1U << sf) * 1000.0 / bandwidth;
        return timeout_symbols * symbol_ms;
    }

    static void applyTagFields(const gr::property_map& tagMap, ChannelState& ch) {
        if (auto it = tagMap.find("crc_valid"); it != tagMap.end())
            ch.crc_valid = it->second.template value_or<bool>(false);
        if (auto it = tagMap.find("snr_db"); it != tagMap.end())
            ch.snr_db = it->second.template value_or<double>(0.0);
        if (auto it = tagMap.find("rx_channel"); it != tagMap.end())
            ch.rx_channel = it->second.template value_or<int64_t>(-1);
    }

    void submitFrame(std::size_t ch_idx) {
        auto& ch = _channels[ch_idx];
        uint64_t hash = hashPayload(ch.payload);

        Candidate cand{
            .payload     = ch.payload,
            .tag         = ch.tag,
            .crc_valid   = ch.crc_valid,
            .snr_db      = ch.snr_db,
            .rx_channel  = ch.rx_channel,
        };

        // find existing pending group by hash
        auto it = std::ranges::find_if(_pending, [hash](const PendingGroup& g) {
            return g.payload_hash == hash;
        });

        if (it != _pending.end()) {
            it->candidates.push_back(std::move(cand));
            auto& best = it->candidates[it->best_idx];
            auto& newest = it->candidates.back();
            if (isBetterCandidate(newest, best)) {
                it->best_idx = it->candidates.size() - 1;
            }
        } else {
            auto now = clock::now();
            auto deadline_ms = std::chrono::duration<double, std::milli>(timeoutMs());
            _pending.push_back(PendingGroup{
                .payload_hash = hash,
                .created_at   = now,
                .deadline     = now + std::chrono::duration_cast<clock::duration>(deadline_ms),
                .candidates   = {std::move(cand)},
                .best_idx     = 0,
            });
        }

        if (debug) {
            std::fprintf(stderr, "[DiversityCombiner] frame from ch=%lld, "
                "crc=%s, snr=%.1f, hash=0x%016llx, pending=%zu\n",
                static_cast<long long>(ch.rx_channel),
                ch.crc_valid ? "OK" : "FAIL",
                ch.snr_db,
                static_cast<unsigned long long>(hash),
                _pending.size());
        }
    }

    gr::property_map buildOutputTag(const PendingGroup& group) {
        auto& winner = group.candidates[group.best_idx];
        gr::property_map tag = winner.tag;

        // diversity metadata: parallel vectors indexed by candidate order
        std::vector<int64_t> rx_channels;
        std::vector<double>  snr_values;
        std::vector<bool>    crc_values;
        rx_channels.reserve(group.candidates.size());
        snr_values.reserve(group.candidates.size());
        crc_values.reserve(group.candidates.size());

        for (const auto& c : group.candidates) {
            rx_channels.push_back(c.rx_channel);
            snr_values.push_back(c.snr_db);
            crc_values.push_back(c.crc_valid);
        }

        tag["diversity_rx_channels"]    = gr::pmt::Value(std::move(rx_channels));
        tag["diversity_decoded_channel"] = gr::pmt::Value(winner.rx_channel);
        tag["diversity_snr_db"]         = gr::pmt::Value(std::move(snr_values));
        tag["diversity_n_candidates"]   = gr::pmt::Value(static_cast<int64_t>(group.candidates.size()));
        // pmt::Value does not support vector<bool> (bitset); encode as
        // int64 bitmask instead (bit 0 = candidate 0, etc.).
        int64_t crc_mask = 0;
        for (std::size_t i = 0; i < crc_values.size() && i < 64; i++) {
            if (crc_values[i]) crc_mask |= (int64_t{1} << static_cast<int>(i));
        }
        tag["diversity_crc_mask"] = gr::pmt::Value(crc_mask);

        // Inter-chain timing: microseconds from group creation to emission.
        // Measures how long the combiner waited for all candidates.
        auto gap_us = std::chrono::duration_cast<std::chrono::microseconds>(
            clock::now() - group.created_at).count();
        tag["diversity_gap_us"] = gr::pmt::Value(static_cast<int64_t>(gap_us));

        if (debug) {
            std::fprintf(stderr, "[DiversityCombiner] emit: decoded_ch=%lld, "
                "candidates=%zu, crc_mask=0x%llx, gap=%lld us\n",
                static_cast<long long>(winner.rx_channel),
                group.candidates.size(),
                static_cast<unsigned long long>(crc_mask),
                static_cast<long long>(gap_us));
        }

        return tag;
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_DIVERSITY_COMBINER_HPP
