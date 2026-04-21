// SPDX-License-Identifier: ISC

// Phase 3 unit tests: DecodeChain.
//
// Strategy: drive the TX chain stages (whiten -> insert_header -> add_crc
// -> hamming_encode_frame -> interleave_frame -> gray_demap) to produce a
// vector of bin indices for a known payload, then push them one by one
// into phy::DecodeChain and assert the recovered FrameResult matches.

#include <boost/ut.hpp>

#include <gnuradio-4.0/lora/algorithm/hamming.hpp>
#include <gnuradio-4.0/lora/algorithm/interleaving.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>
#include <gnuradio-4.0/lora/phy/DecodeChain.hpp>

#include <cstdint>
#include <span>
#include <string>
#include <vector>

using namespace boost::ut;
using gr::lora::phy::DecodeChain;
using gr::lora::phy::FrameResult;

namespace {

/// Generate the bin sequence the modulator would emit for a given payload,
/// reproducing tx_chain.hpp generate_frame_iq() up to (but excluding)
/// modulate_frame(). Returns the gray_demap output, which is exactly what
/// the receiver would see at the FFT argmax (in the noiseless ideal case).
[[nodiscard]] std::vector<uint16_t> generateBins(std::span<const uint8_t> payload, uint8_t sf, uint8_t cr, bool has_crc, uint32_t bandwidth) {
    const bool ldro = gr::lora::needs_ldro(sf, bandwidth);

    auto whitened    = gr::lora::whiten(payload);
    auto with_header = gr::lora::insert_header(whitened, static_cast<uint8_t>(payload.size()), cr, has_crc);
    auto with_crc    = gr::lora::add_crc(with_header, payload, has_crc);
    auto encoded     = gr::lora::hamming_encode_frame(with_crc, sf, cr);
    auto interleaved = gr::lora::interleave_frame(encoded, sf, cr, ldro);
    auto gray_mapped = gr::lora::gray_demap(interleaved, sf);

    std::vector<uint16_t> bins;
    bins.reserve(gray_mapped.size());
    for (auto sym : gray_mapped) {
        bins.push_back(static_cast<uint16_t>(sym));
    }
    return bins;
}

/// Drive a payload through DecodeChain and return the resulting frame.
[[nodiscard]] FrameResult roundtrip(std::span<const uint8_t> payload, uint8_t sf, uint8_t cr, bool has_crc, uint32_t bandwidth) {
    auto bins = generateBins(payload, sf, cr, has_crc, bandwidth);

    DecodeChain dc;
    dc.init({sf, bandwidth, false});

    FrameResult last{};
    for (auto b : bins) {
        if (auto r = dc.push_symbol(b); r.has_value()) {
            last = std::move(*r);
            break;
        }
    }
    return last;
}

suite<"DecodeChain roundtrip"> _roundtrip = [] {
    "SF7 CR1 CRC HELLO"_test = [] {
        const std::vector<uint8_t> payload{'H', 'E', 'L', 'L', 'O'};
        auto                       r = roundtrip(payload, /*sf=*/7, /*cr=*/1, /*has_crc=*/true,
            /*bandwidth=*/125000);
        expect(r.header_valid);
        expect(r.crc_valid);
        expect(r.cr == 1_u);
        expect(r.payload_len == 5_u);
        expect(r.has_crc);
        expect(r.payload.size() == 5u);
        expect(r.payload == payload);
    };

    "SF8 CR2 CRC HELLO"_test = [] {
        const std::vector<uint8_t> payload{'H', 'E', 'L', 'L', 'O'};
        auto                       r = roundtrip(payload, 8, 2, true, 125000);
        expect(r.header_valid);
        expect(r.crc_valid);
        expect(r.cr == 2_u);
        expect(r.payload_len == 5_u);
        expect(r.payload == payload);
    };

    "SF9 CR3 CRC HELLO"_test = [] {
        const std::vector<uint8_t> payload{'H', 'E', 'L', 'L', 'O'};
        auto                       r = roundtrip(payload, 9, 3, true, 125000);
        expect(r.header_valid);
        expect(r.crc_valid);
        expect(r.cr == 3_u);
        expect(r.payload_len == 5_u);
        expect(r.payload == payload);
    };

    "SF10 CR4 CRC HELLO"_test = [] {
        const std::vector<uint8_t> payload{'H', 'E', 'L', 'L', 'O'};
        auto                       r = roundtrip(payload, 10, 4, true, 125000);
        expect(r.header_valid);
        expect(r.crc_valid);
        expect(r.cr == 4_u);
        expect(r.payload_len == 5_u);
        expect(r.payload == payload);
    };

    "SF11 CR1 CRC HELLO"_test = [] {
        const std::vector<uint8_t> payload{'H', 'E', 'L', 'L', 'O'};
        auto                       r = roundtrip(payload, 11, 1, true, 125000);
        expect(r.header_valid);
        expect(r.crc_valid);
        expect(r.payload == payload);
    };

    "SF12 CR1 CRC HELLO"_test = [] {
        const std::vector<uint8_t> payload{'H', 'E', 'L', 'L', 'O'};
        // SF12 @ 125k → LDRO active.
        auto r = roundtrip(payload, 12, 1, true, 125000);
        expect(r.header_valid);
        expect(r.crc_valid);
        expect(r.payload == payload);
    };

    "SF8 BW62500 LDRO?"_test = [] {
        // SF8 @ 62.5k → 256/62.5e3 = 4.1 ms < 16 ms → LDRO off.
        const std::vector<uint8_t> payload{'L', 'O', 'R', 'A'};
        auto                       r = roundtrip(payload, 8, 1, true, 62500);
        expect(r.header_valid);
        expect(r.crc_valid);
        expect(r.payload == payload);
    };

    "SF7 no CRC short payload"_test = [] {
        const std::vector<uint8_t> payload{0x42, 0x13};
        auto                       r = roundtrip(payload, 7, 1, false, 125000);
        expect(r.header_valid);
        expect(not r.has_crc);
        expect(r.crc_valid); // no CRC requested → vacuously valid
        expect(r.payload_len == 2_u);
        expect(r.payload == payload);
    };

    "SF8 long payload 64 bytes"_test = [] {
        std::vector<uint8_t> payload(64);
        for (std::size_t i = 0; i < payload.size(); ++i) {
            payload[i] = static_cast<uint8_t>(i * 7 + 13);
        }
        auto r = roundtrip(payload, 8, 4, true, 125000);
        expect(r.header_valid);
        expect(r.crc_valid);
        expect(r.payload_len == 64_u);
        expect(r.payload == payload);
    };

    "SF10 binary content"_test = [] {
        std::vector<uint8_t> payload{0x00, 0xFF, 0x01, 0xFE, 0x55, 0xAA};
        auto                 r = roundtrip(payload, 10, 2, true, 125000);
        expect(r.header_valid);
        expect(r.crc_valid);
        expect(r.payload == payload);
    };
};

suite<"DecodeChain CRC error detection"> _crc_err = [] {
    "heavily damaged payload bin breaks CRC"_test = [] {
        // Use SF8 CR4 (Hamming 4/8 — corrects single bit errors but fails on
        // multi-bit). Damage several payload bins severely so the residual
        // bit errors cluster in one Hamming codeword and overwhelm it.
        const std::vector<uint8_t> payload{'X', 'Y', 'Z', 'W', 'V', 'U', 'T', 'S'};
        auto                       bins = generateBins(payload, 8, 4, true, 125000);

        // Corrupt several payload symbols (skip the 8 header symbols).
        for (std::size_t i = 9; i < std::min<std::size_t>(bins.size(), 16); ++i) {
            bins[i] = static_cast<uint16_t>((bins[i] + 73) % 256);
        }

        DecodeChain dc;
        dc.init({8, 125000, false});
        FrameResult last{};
        for (auto b : bins) {
            if (auto r = dc.push_symbol(b); r.has_value()) {
                last = std::move(*r);
                break;
            }
        }
        expect(last.header_valid); // header is intact (untouched bins 0..7)
        // Payload should be corrupted OR CRC should fail. With 7 heavily-
        // damaged payload bins it's effectively impossible for both to hold.
        const bool damaged = (last.payload != payload) || not last.crc_valid;
        expect(damaged);
    };
};

suite<"DecodeChain header parse failure"> _hdr_fail = [] {
    "garbage symbols → Failed"_test = [] {
        // Push 8 zero bins; the header parse should reject this since the
        // payload_len will be 0 (or the checksum will fail).
        DecodeChain dc;
        dc.init({8, 125000, false});
        FrameResult last{};
        bool        got_result = false;
        for (int i = 0; i < 8; ++i) {
            if (auto r = dc.push_symbol(0); r.has_value()) {
                last       = std::move(*r);
                got_result = true;
                break;
            }
        }
        expect(got_result);
        expect(not last.header_valid);
        expect(dc.done());
        // Subsequent pushes return nullopt.
        expect(not dc.push_symbol(42).has_value());
    };
};

suite<"DecodeChain reset"> _reset = [] {
    "reset allows second decode"_test = [] {
        const std::vector<uint8_t> payload{'A', 'B', 'C'};
        auto                       bins = generateBins(payload, 8, 1, true, 125000);

        DecodeChain dc;
        dc.init({8, 125000, false});

        for (auto b : bins) {
            (void)dc.push_symbol(b);
        }
        expect(dc.done());

        dc.reset();
        expect(not dc.done());

        FrameResult last{};
        for (auto b : bins) {
            if (auto r = dc.push_symbol(b); r.has_value()) {
                last = std::move(*r);
                break;
            }
        }
        expect(last.header_valid);
        expect(last.crc_valid);
        expect(last.payload == payload);
    };
};

suite<"DecodeChain LDRO detection stats"> _ldro = [] {
    // LDRO signature (also documented in DecodeChain::push_symbol):
    //   Under LDRO, interleave_block packs the output symbol MSB-first as
    //   [data x sf_app | parity | zero]. After bool2int (MSB = vec[0])
    //   this puts a constant zero at bit 0 of every payload symbol.
    //   adjustBin undoes Gray + rotation at RX, so `adjustBin(bin) & 1`
    //   is ALWAYS 0 under LDRO (ideal) and ~50% under no-LDRO (data-only
    //   symbols post-whitening/Hamming/Gray).
    // Threshold 0.75 (MultiSfDecoder): ideal 1.0 vs uniform 0.5, keeps
    // false-positive rate < 1% at N >= 16 under Bin(n, 0.5).

    "SF8 BW62.5k → LDRO off, cfg matches"_test = [] {
        // 256 / 62500 = 4.1 ms < 16 ms → rule-based LDRO off.
        std::vector<uint8_t> payload(32);
        for (std::size_t i = 0; i < payload.size(); ++i) {
            payload[i] = static_cast<uint8_t>(i * 17 + 3);
        }
        auto r = roundtrip(payload, 8, 1, true, 62500);
        expect(r.header_valid);
        expect(r.crc_valid);
        expect(not r.ldro_cfg);
        expect(r.payload_syms_seen >= 16u);
        const double frac = static_cast<double>(r.payload_lsb0_hits) / static_cast<double>(r.payload_syms_seen);
        // No-LDRO: expect frac clearly below 0.75 detection threshold.
        // Whitening + Hamming + Gray spread LSB roughly uniformly, so
        // frac sits near 0.5 on moderate-length payloads.
        expect(frac < 0.75);
    };

    "SF12 BW125k → LDRO on, all LSB-zero"_test = [] {
        // 4096 / 125000 = 32.8 ms > 16 ms → rule-based LDRO on.
        // In the noiseless roundtrip every payload bin must have
        // adjustBin & 1 == 0 (interleaver writes a zero at bit 0).
        std::vector<uint8_t> payload(16);
        for (std::size_t i = 0; i < payload.size(); ++i) {
            payload[i] = static_cast<uint8_t>(i * 19 + 7);
        }
        auto r = roundtrip(payload, 12, 1, true, 125000);
        expect(r.header_valid);
        expect(r.crc_valid);
        expect(r.ldro_cfg);
        expect(r.payload_syms_seen >= 16u);
        expect(r.payload_lsb0_hits == r.payload_syms_seen);
    };

    "SF11 BW125k → LDRO on, all LSB-zero"_test = [] {
        // 2048 / 125000 = 16.4 ms > 16 ms → rule-based LDRO on.
        std::vector<uint8_t> payload(24);
        for (std::size_t i = 0; i < payload.size(); ++i) {
            payload[i] = static_cast<uint8_t>(i * 23 + 11);
        }
        auto r = roundtrip(payload, 11, 1, true, 125000);
        expect(r.header_valid);
        expect(r.crc_valid);
        expect(r.ldro_cfg);
        expect(r.payload_lsb0_hits == r.payload_syms_seen);
    };

    "reset clears stats"_test = [] {
        const std::vector<uint8_t> payload{'A', 'B', 'C'};
        auto                       bins = generateBins(payload, 8, 1, true, 125000);

        DecodeChain dc;
        dc.init({8, 125000, false});
        for (auto b : bins) {
            (void)dc.push_symbol(b);
        }

        dc.reset();
        // Re-run; first result must have fresh counters, not carryover.
        FrameResult last{};
        for (auto b : bins) {
            if (auto r = dc.push_symbol(b); r.has_value()) {
                last = std::move(*r);
                break;
            }
        }
        expect(last.header_valid);
        expect(last.payload_syms_seen > 0u);
        // Counter should reflect only this frame, not two frames' worth.
        expect(last.payload_syms_seen <= bins.size() - 8u);
    };
};

} // namespace

int main() { return 0; }
