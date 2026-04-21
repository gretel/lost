// SPDX-License-Identifier: ISC
//
// Phase 8 debug harness: drive the PHY library directly on a TX-chain-
// generated IQ stream, bypassing the GR4 block wrapper. If this passes,
// the PHY library is correct and any bugs are in the block wrapper. If it
// fails, the library has a bug that the unit tests didn't catch.

#include <boost/ut.hpp>

#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>
#include <gnuradio-4.0/lora/phy/CssDemod.hpp>
#include <gnuradio-4.0/lora/phy/DecodeChain.hpp>
#include <gnuradio-4.0/lora/phy/PreambleSync.hpp>

using namespace boost::ut;
using cf32 = std::complex<float>;

namespace {

/// Drive the PHY library on a full TX-chain IQ stream. Does the same work
/// MultiSfDecoder does but without GR4 ports — pure in-memory loop. Mirrors
/// the block's state machine: sync → skip SFD tail (D2 + quarter) → demod.
struct LibPipelineResult {
    std::vector<uint8_t>             decoded;
    bool                             header_valid     = false;
    bool                             crc_valid        = false;
    uint32_t                         n_sync_ticks     = 0;
    uint32_t                         n_out_ticks      = 0;
    gr::lora::phy::SyncResult::State final_sync_state = gr::lora::phy::SyncResult::State::Detecting;
    gr::lora::phy::SyncResult        final_sync_result{};
    std::vector<uint16_t>            raw_bins; // first N payload bins for debug
};

[[nodiscard]] LibPipelineResult runLibPipeline(std::span<const cf32> iq, uint8_t sf, uint32_t bandwidth, uint8_t os_factor, uint16_t preamble_len, uint16_t sync_word) {
    using namespace gr::lora::phy;

    const uint32_t N   = uint32_t{1} << sf;
    const uint32_t sps = N * os_factor;

    PreambleSync sync;
    {
        PreambleSync::Config cfg;
        cfg.sf               = sf;
        cfg.bandwidth_hz     = bandwidth;
        cfg.preamble_len     = preamble_len;
        cfg.sync_word        = sync_word;
        cfg.verify_threshold = false;
        sync.init(cfg);
    }

    CssDemod demod;
    {
        CssDemod::Config cfg;
        cfg.sf           = sf;
        cfg.bandwidth_hz = bandwidth;
        cfg.soft_decode  = false;
        demod.init(cfg);
    }

    DecodeChain decode;
    {
        DecodeChain::Config cfg;
        cfg.sf           = sf;
        cfg.bandwidth_hz = bandwidth;
        cfg.soft_decode  = false;
        decode.init(cfg);
    }

    std::vector<cf32> nyquist(N);
    auto              decimate = [&](std::size_t in_off) {
        // Decimate with offset 0 (not os_factor/2) — an os/2 offset
        // introduces a half-bin shift when dechirped against a Nyquist-rate
        // reference, because the oversampled build_upchirp starts its
        // chirp at n=0 not n=os/2.
        for (uint32_t i = 0; i < N; ++i) {
            const std::size_t idx = in_off + i * os_factor;
            nyquist[i]            = (idx < iq.size()) ? iq[idx] : cf32{0.f, 0.f};
        }
    };

    LibPipelineResult result;

    enum class State { Detect, SkipSfdTail, Output, Done };
    State    state              = State::Detect;
    uint32_t sfd_tail_remaining = 0;

    std::size_t pos = 0;
    while (pos + sps <= iq.size() && state != State::Done) {
        if (state == State::SkipSfdTail) {
            const std::size_t skip = std::min<std::size_t>(sfd_tail_remaining, sps);
            sfd_tail_remaining -= static_cast<uint32_t>(skip);
            if (sfd_tail_remaining == 0) {
                state = State::Output;
            }
            pos += (skip == 0) ? sps : skip;
            continue;
        }

        decimate(pos);

        if (state == State::Detect) {
            ++result.n_sync_ticks;
            auto r                  = sync.tick(std::span<const cf32>(nyquist));
            result.final_sync_state = r.state;
            if (r.state == SyncResult::State::Locked) {
                result.final_sync_result = r;
                demod.set_cfo_correction(r.cfo_frac, r.cfo_int);
                sfd_tail_remaining = sps + sps / 4u;
                state              = State::SkipSfdTail;
            } else if (r.state == SyncResult::State::Failed) {
                break;
            }
            pos += sps;
            continue;
        }

        if (state == State::Output) {
            ++result.n_out_ticks;
            auto d = demod.demodHard(std::span<const cf32>(nyquist));
            if (result.raw_bins.size() < 24) {
                result.raw_bins.push_back(d.bin);
            }
            if (auto frame = decode.push_symbol(d.bin); frame.has_value()) {
                result.header_valid = frame->header_valid;
                result.crc_valid    = frame->crc_valid;
                result.decoded      = frame->payload;
                state               = State::Done;
            }
            pos += sps;
            continue;
        }
    }
    return result;
}

[[nodiscard]] bool payloadMatches(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

suite<"lib pipeline"> _pipeline = [] {
    "SF7 CR1 HELLO"_test = [] {
        const std::vector<uint8_t> payload{'H', 'E', 'L', 'L', 'O'};
        const uint8_t              sf = 7;
        const uint32_t             bw = 125000;
        const uint8_t              os = 4;

        auto iq = gr::lora::generate_frame_iq(payload, sf, /*cr=*/1, os, /*sync_word=*/0x12,
            /*preamble_len=*/8, /*has_crc=*/true,
            /*zero_pad=*/(1u << sf) * os * 5,
            /*ldro_mode=*/2, bw, /*inverted_iq=*/false);

        auto r = runLibPipeline(std::span<const cf32>(iq), sf, bw, os, 8, 0x12);
        std::fprintf(stderr,
            "[lib pipeline SF7] sync_ticks=%u out_ticks=%u sync_state=%d "
            "header=%d crc=%d dec_size=%zu\n",
            r.n_sync_ticks, r.n_out_ticks, static_cast<int>(r.final_sync_state), r.header_valid, r.crc_valid, r.decoded.size());
        std::fprintf(stderr,
            "[lib pipeline SF7] sync: cfo_int=%d cfo_frac=%.4f "
            "sto_int=%d sto_frac=%.4f bin_hat=%u\n",
            r.final_sync_result.cfo_int, static_cast<double>(r.final_sync_result.cfo_frac), r.final_sync_result.sto_int, static_cast<double>(r.final_sync_result.sto_frac), r.final_sync_result.bin_hat);
        std::fprintf(stderr, "[lib pipeline SF7] raw_bins[%zu]:", r.raw_bins.size());
        for (auto b : r.raw_bins) {
            std::fprintf(stderr, " %u", b);
        }
        std::fprintf(stderr, "\n");
        if (!r.decoded.empty()) {
            std::fprintf(stderr, "[lib pipeline SF7] decoded bytes:");
            for (auto b : r.decoded) {
                std::fprintf(stderr, " %02x", b);
            }
            std::fprintf(stderr, "\n");
            std::fprintf(stderr, "[lib pipeline SF7] expected bytes:");
            for (auto b : payload) {
                std::fprintf(stderr, " %02x", b);
            }
            std::fprintf(stderr, "\n");
        }

        // Compute the ground-truth bin sequence by running the TX chain
        // to symbol level (without modulation), so we can compare.
        {
            const bool ldro        = gr::lora::needs_ldro(sf, bw);
            auto       whitened    = gr::lora::whiten(payload);
            auto       with_header = gr::lora::insert_header(whitened, static_cast<uint8_t>(payload.size()), 1, true);
            auto       with_crc    = gr::lora::add_crc(with_header, payload, true);
            auto       encoded     = gr::lora::hamming_encode_frame(with_crc, sf, 1);
            auto       interleaved = gr::lora::interleave_frame(encoded, sf, 1, ldro);
            auto       gray_mapped = gr::lora::gray_demap(interleaved, sf);
            std::fprintf(stderr, "[lib pipeline SF7] expected bins[%zu]:", gray_mapped.size());
            for (std::size_t i = 0; i < std::min<std::size_t>(gray_mapped.size(), 24); ++i) {
                std::fprintf(stderr, " %u", gray_mapped[i]);
            }
            std::fprintf(stderr, "\n");
        }
        expect(r.final_sync_state == gr::lora::phy::SyncResult::State::Locked);
        expect(r.header_valid);
        expect(r.crc_valid);
        expect(payloadMatches(r.decoded, payload));
    };
};

} // namespace

int main() { return 0; }
