// SPDX-License-Identifier: ISC
#include "test_helpers.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <numeric>
#include <random>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/algorithm/SfLane.hpp>
#include <gnuradio-4.0/lora/algorithm/tables.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>
#include <gnuradio-4.0/lora/algorithm/utilities.hpp>
#include <gnuradio-4.0/lora/algorithm/hamming.hpp>
#include <gnuradio-4.0/lora/algorithm/crc.hpp>
#include <gnuradio-4.0/lora/algorithm/GrayPartition.hpp>
#include <gnuradio-4.0/lora/algorithm/interleaving.hpp>

using namespace gr::lora::test;

const boost::ut::suite<"LoRa algorithm utilities"> utilities_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "mod positive"_test = [] {
        expect(eq(mod(5, 3), 2L));
        expect(eq(mod(6, 3), 0L));
        expect(eq(mod(0, 7), 0L));
    };

    "mod negative"_test = [] {
        expect(eq(mod(-1, 6), 5L));
        expect(eq(mod(-7, 3), 2L));
        expect(eq(mod(-6, 6), 0L));
    };

    "int2bool"_test = [] {
        auto bits = int2bool(0b1010, 4);
        expect(eq(bits.size(), 4uz));
        expect(bits[0] == true);   // MSB
        expect(bits[1] == false);
        expect(bits[2] == true);
        expect(bits[3] == false);  // LSB
    };

    "bool2int"_test = [] {
        std::vector<bool> bits = {true, false, true, false};
        expect(eq(bool2int(bits), 10u));
    };

    "int2bool/bool2int roundtrip"_test = [] {
        for (uint32_t v = 0; v < 256; v++) {
            auto bits = int2bool(v, 8);
            expect(eq(bool2int(bits), v)) << "roundtrip failed for " << v;
        }
    };
};

const boost::ut::suite<"Dechirp quality measurement"> dechirp_quality_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "dechirp_and_quality separates chirp from noise"_test = [] {

        std::vector<std::complex<float>> upchirp(N);
        std::vector<std::complex<float>> downchirp(N);
        build_ref_chirps(upchirp.data(), downchirp.data(), SF);

        std::vector<std::complex<float>> scratch(N);
        gr::algorithm::FFT<std::complex<float>> fft;

        // (a) Real upchirp — should have high PMR
        auto [bin_chirp, pmr_chirp] = dechirp_and_quality(
            upchirp.data(), downchirp.data(), scratch.data(), N, fft);
        std::printf("chirp: bin=%u PMR=%.1f\n", bin_chirp, static_cast<double>(pmr_chirp));
        expect(eq(bin_chirp, 0u)) << "upchirp(id=0) dechirps to bin 0";
        expect(gt(pmr_chirp, 10.f)) << "real chirp PMR should be >> 1";

        // (b) Gaussian noise — should have low PMR
        std::mt19937 gen(42);
        std::normal_distribution<float> dist(0.f, 1.f);
        std::vector<std::complex<float>> noise(N);
        for (auto& s : noise) s = {dist(gen), dist(gen)};

        auto [bin_noise, pmr_noise] = dechirp_and_quality(
            noise.data(), downchirp.data(), scratch.data(), N, fft);
        std::printf("noise: bin=%u PMR=%.1f\n", bin_noise, static_cast<double>(pmr_noise));
        expect(lt(pmr_noise, 4.f)) << "noise PMR should be low";
    };

    "dechirp_and_quality DC removal"_test = [] {
        // Generate upchirp with id=5 (lands at bin 5 after dechirp)
        std::vector<std::complex<float>> chirp(N);
        build_upchirp(chirp.data(), 5, SF);

        std::vector<std::complex<float>> downchirp(N);
        {
            std::vector<std::complex<float>> up(N);
            build_ref_chirps(up.data(), downchirp.data(), SF);
        }

        // Add a large DC offset
        for (auto& s : chirp) s += std::complex<float>(3.0f, 0.0f);

        std::vector<std::complex<float>> scratch(N);
        gr::algorithm::FFT<std::complex<float>> fft;

        // With DC removal, should find the real chirp
        auto [bin_dc, pmr_dc] = dechirp_and_quality(
            chirp.data(), downchirp.data(), scratch.data(), N, fft, true);
        std::printf("DC removed: bin=%u PMR=%.1f\n", bin_dc, static_cast<double>(pmr_dc));
        expect(eq(bin_dc, 5u)) << "should find chirp at bin 5 after DC removal";

        // Verify DC removal doesn't break a clean chirp at bin 0
        std::vector<std::complex<float>> clean_chirp(N);
        build_upchirp(clean_chirp.data(), 0, SF);
        auto [bin_clean, pmr_clean] = dechirp_and_quality(
            clean_chirp.data(), downchirp.data(), scratch.data(), N, fft, true);
        std::printf("Clean chirp + DC removal: bin=%u PMR=%.1f\n", bin_clean, static_cast<double>(pmr_clean));
        expect(eq(bin_clean, 0u)) << "clean upchirp(0) should still dechirp to bin 0";
        expect(gt(pmr_clean, 10.f)) << "clean chirp PMR should remain high";
    };
};

const boost::ut::suite<"LoRa whitening table"> whitening_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "table size"_test = [] {
        expect(eq(whitening_seq.size(), 255uz));
    };

    "first bytes"_test = [] {
        expect(eq(whitening_seq[0], uint8_t(0xFF)));
        expect(eq(whitening_seq[1], uint8_t(0xFE)));
        expect(eq(whitening_seq[2], uint8_t(0xFC)));
    };

    "last byte"_test = [] {
        expect(eq(whitening_seq[254], uint8_t(0x7F)));
    };

    "whitening TX stage vs test vector"_test = [] {
        auto payload_str = load_text("payload.txt");
        std::vector<uint8_t> payload(payload_str.begin(), payload_str.end());
        auto expected = load_u8("tx_01_whitened_nibbles.u8");

        // Whiten + split into nibbles (same algorithm as generate.py)
        std::vector<uint8_t> nibbles;
        for (std::size_t i = 0; i < payload.size(); i++) {
            uint8_t whitened = payload[i] ^ whitening_seq[i];
            nibbles.push_back(whitened & 0x0F);         // low nibble
            nibbles.push_back((whitened >> 4) & 0x0F);   // high nibble
        }

        expect(eq(nibbles.size(), expected.size())) << "whitened nibble count mismatch";
        for (std::size_t i = 0; i < nibbles.size() && i < expected.size(); i++) {
            expect(eq(nibbles[i], expected[i])) << "whitened nibble mismatch at index " << i;
        }
    };
};

const boost::ut::suite<"LoRa header + CRC"> header_crc_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "header build + parse roundtrip"_test = [] {
        auto header = build_explicit_header(14, CR, HAS_CRC);
        expect(eq(header[0], uint8_t(0)));   // 14 >> 4 = 0
        expect(eq(header[1], uint8_t(14)));  // 14 & 0x0F = 14

        auto info = parse_explicit_header(header[0], header[1], header[2], header[3], header[4]);
        expect(info.checksum_valid) << "header checksum should be valid";
        expect(eq(info.payload_len, uint8_t(14)));
        expect(eq(info.cr, CR));
        expect(eq(info.has_crc, HAS_CRC));
    };

    "header checksum detects corruption"_test = [] {
        auto header = build_explicit_header(14, CR, HAS_CRC);
        // Corrupt one nibble
        uint8_t bad_n1 = header[1] ^ 0x01;
        auto info = parse_explicit_header(header[0], bad_n1, header[2], header[3], header[4]);
        expect(!info.checksum_valid) << "corrupted header should fail checksum";
    };

    "header insert vs test vector"_test = [] {
        auto whitened = load_u8("tx_01_whitened_nibbles.u8");
        auto expected = load_u8("tx_02_with_header.u8");

        auto header = build_explicit_header(14, CR, HAS_CRC);
        std::vector<uint8_t> with_header;
        with_header.insert(with_header.end(), header.begin(), header.end());
        with_header.insert(with_header.end(), whitened.begin(), whitened.end());

        expect(eq(with_header.size(), expected.size())) << "with_header size mismatch";
        for (std::size_t i = 0; i < with_header.size() && i < expected.size(); i++) {
            expect(eq(with_header[i], expected[i])) << "header+whitened mismatch at index " << i;
        }
    };

    "CRC-16 known value"_test = [] {
        // CRC-16/XMODEM (poly=0x1021, init=0x0000) of "123456789" = 0x31C3
        std::array<uint8_t, 9> data = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
        auto crc = crc16(data);
        expect(eq(crc, uint16_t(0x31C3))) << "CRC-16/XMODEM of '123456789' should be 0x31C3";
    };

    "CRC add vs test vector"_test = [] {
        auto with_header = load_u8("tx_02_with_header.u8");
        auto expected = load_u8("tx_03_with_crc.u8");

        auto payload_str = load_text("payload.txt");
        std::vector<uint8_t> payload(payload_str.begin(), payload_str.end());

        uint16_t crc = lora_payload_crc(payload);
        auto crc_nibs = crc_to_nibbles(crc);

        std::vector<uint8_t> with_crc = with_header;
        with_crc.insert(with_crc.end(), crc_nibs.begin(), crc_nibs.end());

        expect(eq(with_crc.size(), expected.size())) << "with_crc size mismatch";
        for (std::size_t i = 0; i < with_crc.size() && i < expected.size(); i++) {
            expect(eq(with_crc[i], expected[i])) << "with_crc mismatch at index " << i;
        }
    };
};

const boost::ut::suite<"LoRa Hamming encoding"> hamming_enc_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "encode nibble cr=4"_test = [] {
        // 0 → all zero → all parity bits 0 → 0x00
        expect(eq(hamming_encode(0, 4), uint8_t(0)));
        // 0xF (1111) → p0=1^1^1=1, p1=1^1^1=1, p2=1^1^1=1, p3=1^1^1=1
        // full = 11111111 = 0xFF, no shift → 0xFF
        expect(eq(hamming_encode(0x0F, 4), uint8_t(0xFF)));
    };

    "encode roundtrip with hard decode"_test = [] {
        for (uint8_t nib = 0; nib < 16; nib++) {
            for (uint8_t cr = 1; cr <= 4; cr++) {
                uint8_t cw = hamming_encode(nib, cr);
                uint8_t decoded = hamming_decode_hard(cw, cr);
                expect(eq(decoded, nib)) << "encode/decode roundtrip failed for nib="
                    << static_cast<int>(nib) << " cr=" << static_cast<int>(cr);
            }
        }
    };

    "encode frame vs test vector"_test = [] {
        auto with_crc = load_u8("tx_03_with_crc.u8");
        auto expected = load_u8("tx_04_encoded.u8");

        auto encoded = hamming_encode_frame(with_crc, SF, CR);

        expect(eq(encoded.size(), expected.size())) << "encoded size mismatch";
        for (std::size_t i = 0; i < encoded.size() && i < expected.size(); i++) {
            expect(eq(encoded[i], expected[i])) << "encoded mismatch at index " << i;
        }
    };
};

const boost::ut::suite<"LoRa Hamming decoding"> hamming_dec_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "hard decode single-bit error cr=4"_test = [] {
        for (uint8_t nib = 0; nib < 16; nib++) {
            uint8_t cw = hamming_encode(nib, 4);
            // Flip each bit and verify correction
            for (int bit = 0; bit < 8; bit++) {
                auto corrupted = static_cast<uint8_t>(cw ^ (1 << bit));
                uint8_t decoded = hamming_decode_hard(corrupted, 4);
                expect(eq(decoded, nib)) << "single-bit correction failed for nib="
                    << static_cast<int>(nib) << " bit=" << bit;
            }
        }
    };

    "hard decode cr=3"_test = [] {
        for (uint8_t nib = 0; nib < 16; nib++) {
            uint8_t cw = hamming_encode(nib, 3);
            // Flip each bit in the 7-bit codeword
            for (int bit = 0; bit < 7; bit++) {
                auto corrupted = static_cast<uint8_t>(cw ^ (1 << bit));
                uint8_t decoded = hamming_decode_hard(corrupted, 3);
                expect(eq(decoded, nib)) << "cr=3 correction failed for nib="
                    << static_cast<int>(nib) << " bit=" << bit;
            }
        }
    };

    "soft decode no noise"_test = [] {
        // Encode nibble 0b1010 with cr=4, convert to strong LLRs
        uint8_t nib = 0b1010;
        uint8_t cw = hamming_encode(nib, 4);
        auto bits = int2bool(cw, 8);

        // Create LLRs: +10 for bit 1, -10 for bit 0
        std::array<double, 8> llrs;
        for (std::size_t i = 0; i < 8; i++) {
            llrs[i] = bits[i] ? 10.0 : -10.0;
        }

        uint8_t decoded = hamming_decode_soft(llrs.data(), 4);
        expect(eq(decoded, nib)) << "soft decode of clean signal should match";
    };
};

const boost::ut::suite<"LoRa interleaving"> interleaving_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "interleave frame vs test vector"_test = [] {
        auto encoded = load_u8("tx_04_encoded.u8");
        auto expected = load_u32("tx_05_interleaved.u32");

        auto interleaved = interleave_frame(encoded, SF, CR);

        expect(eq(interleaved.size(), expected.size())) << "interleaved size mismatch: got "
            << interleaved.size() << " expected " << expected.size();
        for (std::size_t i = 0; i < interleaved.size() && i < expected.size(); i++) {
            expect(eq(interleaved[i], expected[i])) << "interleaved mismatch at index " << i;
        }
    };

    "interleave/deinterleave header block roundtrip"_test = [] {
        // Create some codewords for the header block
        std::vector<uint8_t> codewords = {0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56}; // sf-2 = 6 codewords
        uint8_t sf_app = SF - 2; // 6
        uint8_t cw_len = 8;

        auto symbols = interleave_block(codewords, SF, cw_len, sf_app, true);
        expect(eq(symbols.size(), static_cast<std::size_t>(cw_len)));

        // In the real RX chain, FFT demod outputs sf_app-bit-wide symbols
        // (parity + zero bits stripped by the demodulator).
        // Simulate this by extracting only the data bits (top sf_app bits of each sf-bit symbol).
        std::vector<uint16_t> syms16;
        for (auto s : symbols) {
            uint16_t data_bits = static_cast<uint16_t>(s >> (SF - sf_app));
            syms16.push_back(data_bits);
        }

        auto decoded_cw = deinterleave_block(syms16, SF, cw_len, sf_app);
        expect(eq(decoded_cw.size(), static_cast<std::size_t>(sf_app)));

        for (std::size_t i = 0; i < sf_app; i++) {
            expect(eq(decoded_cw[i], codewords[i]))
                << "header roundtrip failed at " << i;
        }
    };

    "interleave/deinterleave payload block roundtrip"_test = [] {
        // Payload block: sf codewords, cw_len = cr + 4
        std::vector<uint8_t> codewords;
        for (int i = 0; i < SF; i++) codewords.push_back(static_cast<uint8_t>(i * 17)); // arbitrary
        uint8_t cw_len = CR + 4; // 8

        auto symbols = interleave_block(codewords, SF, cw_len, SF, false);
        expect(eq(symbols.size(), static_cast<std::size_t>(cw_len)));

        std::vector<uint16_t> syms16;
        for (auto s : symbols) syms16.push_back(static_cast<uint16_t>(s));

        auto decoded_cw = deinterleave_block(syms16, SF, cw_len, SF);
        expect(eq(decoded_cw.size(), static_cast<std::size_t>(SF)));

        for (std::size_t i = 0; i < SF; i++) {
            expect(eq(decoded_cw[i], codewords[i]))
                << "payload roundtrip failed at " << i;
        }
    };
};

const boost::ut::suite<"LoRa Gray mapping"> gray_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "gray demap vs test vector"_test = [] {
        auto interleaved = load_u32("tx_05_interleaved.u32");
        auto expected = load_u32("tx_06_gray_mapped.u32");

        // Gray demap: binary→Gray + shift by 1 (TX side)
        // out = gray(in) + 1, where gray(x) = x ^ (x>>1) ^ (x>>2) ^ ... ^ (x>>(sf-1))
        std::vector<uint32_t> gray_mapped;
        gray_mapped.reserve(interleaved.size());
        for (auto s : interleaved) {
            uint32_t g = s;
            for (int j = 1; j < SF; j++) {
                g ^= (s >> j);
            }
            gray_mapped.push_back((g + 1) % N);
        }

        expect(eq(gray_mapped.size(), expected.size())) << "gray mapped size mismatch";
        for (std::size_t i = 0; i < gray_mapped.size() && i < expected.size(); i++) {
            expect(eq(gray_mapped[i], expected[i])) << "gray mapped mismatch at index " << i;
        }
    };

    "gray mapping roundtrip (demap then map)"_test = [] {
        // TX gray_demap: gray_to_binary(s) + 1  (the +1 is chirp index offset)
        // FFT demod: recovers chirp index, subtracts 1 → gray_to_binary(s)
        // RX gray_map: binary_to_gray(gray_to_binary(s)) == s
        //
        // So the effective roundtrip (excluding the +1/-1 cancellation) is:
        //   binary_to_gray(gray_to_binary(s)) == s
        for (uint32_t sym = 0; sym < N; sym++) {
            // TX gray_demap (without +1): gray_to_binary(sym)
            uint32_t g2b = sym;
            for (int j = 1; j < SF; j++) {
                g2b ^= (sym >> j);
            }

            // RX gray_map: binary_to_gray
            uint32_t b2g = g2b ^ (g2b >> 1);

            expect(eq(b2g, sym)) << "gray roundtrip mismatch at " << sym;
        }
    };
};

const boost::ut::suite<"LoRa full TX pipeline"> pipeline_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "full TX pipeline whitening → gray matches all test vectors"_test = [] {
        // Load payload
        auto payload_str = load_text("payload.txt");
        std::vector<uint8_t> payload(payload_str.begin(), payload_str.end());

        // Stage 1: Whiten
        std::vector<uint8_t> nibbles;
        for (std::size_t i = 0; i < payload.size(); i++) {
            uint8_t w = payload[i] ^ whitening_seq[i];
            nibbles.push_back(w & 0x0F);
            nibbles.push_back((w >> 4) & 0x0F);
        }
        expect_vectors_equal(nibbles, load_u8("tx_01_whitened_nibbles.u8"), "stage 1 whitening");

        // Stage 2: Header insert
        auto header = build_explicit_header(static_cast<uint8_t>(payload.size()), CR, HAS_CRC);
        std::vector<uint8_t> with_header;
        with_header.insert(with_header.end(), header.begin(), header.end());
        with_header.insert(with_header.end(), nibbles.begin(), nibbles.end());
        expect_vectors_equal(with_header, load_u8("tx_02_with_header.u8"), "stage 2 header insert");

        // Stage 3: Add CRC
        uint16_t crc = lora_payload_crc(payload);
        auto crc_nibs = crc_to_nibbles(crc);
        std::vector<uint8_t> with_crc = with_header;
        with_crc.insert(with_crc.end(), crc_nibs.begin(), crc_nibs.end());
        expect_vectors_equal(with_crc, load_u8("tx_03_with_crc.u8"), "stage 3 add CRC");

        // Stage 4: Hamming encode
        auto encoded = hamming_encode_frame(with_crc, SF, CR);
        expect_vectors_equal(encoded, load_u8("tx_04_encoded.u8"), "stage 4 hamming encode");

        // Stage 5: Interleave
        auto interleaved = interleave_frame(encoded, SF, CR);
        expect_vectors_equal(interleaved, load_u32("tx_05_interleaved.u32"), "stage 5 interleave");

        // Stage 6: Gray demap
        std::vector<uint32_t> gray_mapped;
        for (auto s : interleaved) {
            uint32_t g = s;
            for (int j = 1; j < SF; j++) g ^= (s >> j);
            gray_mapped.push_back((g + 1) % N);
        }
        expect_vectors_equal(gray_mapped, load_u32("tx_06_gray_mapped.u32"), "stage 6 gray demap");
    };
};

// =============================================================================
// Sync algorithm tests: CFO, Xhonneux Eq.20 STO, mod-8 correction
// =============================================================================

namespace {

/// Helper: generate a preamble with known CFO applied, at os_factor=1.
std::vector<std::complex<float>> make_preamble_with_cfo(
        uint8_t sf, float cfo_frac, uint32_t n_syms, uint32_t sto_int = 0,
        std::mt19937* rng = nullptr, float noise_sigma = 0.f) {
    const uint32_t Nfft = 1u << sf;
    std::vector<std::complex<float>> upchirp(Nfft);
    gr::lora::build_upchirp(upchirp.data(), sto_int, sf, 1);

    std::vector<std::complex<float>> out;
    out.reserve(static_cast<std::size_t>(Nfft) * n_syms);
    std::normal_distribution<float> dist(0.f, noise_sigma);

    for (uint32_t s = 0; s < n_syms; s++) {
        for (uint32_t n = 0; n < Nfft; n++) {
            float phase = 2.f * static_cast<float>(std::numbers::pi) * cfo_frac
                        / static_cast<float>(Nfft) * static_cast<float>(s * Nfft + n);
            auto rot = std::complex<float>(std::cos(phase), std::sin(phase));
            auto sample = upchirp[n] * rot;
            if (rng && noise_sigma > 0.f) {
                sample += std::complex<float>(dist(*rng), dist(*rng));
            }
            out.push_back(sample);
        }
    }
    return out;
}

/// Helper: initialize an SfLane for testing.
gr::lora::SfLane make_test_lane(uint8_t sf, uint32_t bw = 62500,
                                 uint16_t sync_word = 0x12, uint16_t preamble_len = 8) {
    gr::lora::SfLane lane;
    lane.init(sf, bw, 1 /*os_factor=1*/, preamble_len, sync_word);
    return lane;
}

}  // namespace

const boost::ut::suite<"CFO estimation"> cfo_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "CFO estimate at clean SNR"_test = [] {
        constexpr uint8_t sf = 8;
        constexpr float cfo_true = 0.3f;
        auto lane = make_test_lane(sf);
        auto preamble = make_preamble_with_cfo(sf, cfo_true, lane.n_up_req);
        std::copy(preamble.begin(), preamble.end(), lane.preamble_raw.begin());

        float cfo_est = lane.estimate_CFO_frac(preamble.data());
        float err = std::abs(cfo_est - cfo_true);
        std::printf("  CFO clean: true=%.3f est=%.3f err=%.4f\n",
                    static_cast<double>(cfo_true), static_cast<double>(cfo_est),
                    static_cast<double>(err));
        expect(lt(err, 0.01f)) << "CFO error should be < 0.01 at high SNR";
    };

    "CFO estimate negative value"_test = [] {
        constexpr uint8_t sf = 8;
        constexpr float cfo_true = -0.25f;
        auto lane = make_test_lane(sf);
        auto preamble = make_preamble_with_cfo(sf, cfo_true, lane.n_up_req);
        std::copy(preamble.begin(), preamble.end(), lane.preamble_raw.begin());

        float cfo_est = lane.estimate_CFO_frac(preamble.data());
        float err = std::abs(cfo_est - cfo_true);
        std::printf("  CFO negative: true=%.3f est=%.3f err=%.4f\n",
                    static_cast<double>(cfo_true), static_cast<double>(cfo_est),
                    static_cast<double>(err));
        expect(lt(err, 0.01f)) << "negative CFO error should be < 0.01 at high SNR";
    };
};

const boost::ut::suite<"Xhonneux Eq.20 STO estimation (W3b)"> sto_eq20_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "STO frac estimate clean (sto_int=5)"_test = [] {
        constexpr uint8_t sf = 8;
        constexpr uint32_t sto_int = 5;
        auto lane = make_test_lane(sf);

        auto preamble = make_preamble_with_cfo(sf, 0.f, lane.n_up_req, sto_int);
        lane.k_hat = static_cast<int>(sto_int);
        std::copy(preamble.begin(), preamble.end(), lane.preamble_raw.begin());
        lane.estimate_CFO_frac(preamble.data());

        float sto_f = lane.estimate_STO_frac(sto_int);
        std::printf("  STO frac (integer STO=%u): est=%.4f (expect ~0)\n",
                    sto_int, static_cast<double>(sto_f));
        expect(lt(std::abs(sto_f), 0.1f)) << "fractional STO should be near 0 for integer STO";
    };

    "STO frac estimate with fractional offset"_test = [] {
        constexpr uint8_t sf = 8;
        constexpr uint32_t Nfft = 1u << sf;
        constexpr uint8_t os = 4;
        constexpr float sto_frac_true = 0.3f;
        constexpr uint32_t sto_int = 0;

        auto lane = make_test_lane(sf);
        lane.k_hat = 0;

        std::vector<std::complex<float>> upchirp_os(Nfft * os);
        build_upchirp(upchirp_os.data(), sto_int, sf, os);

        std::vector<std::complex<float>> preamble;
        preamble.reserve(static_cast<std::size_t>(Nfft) * lane.n_up_req);
        for (uint32_t s = 0; s < lane.n_up_req; s++) {
            for (uint32_t n = 0; n < Nfft; n++) {
                float pos = (static_cast<float>(n) + sto_frac_true) * static_cast<float>(os);
                auto idx0 = static_cast<uint32_t>(std::floor(pos)) % (Nfft * os);
                auto idx1 = (idx0 + 1) % (Nfft * os);
                float frac = pos - std::floor(pos);
                auto sample = upchirp_os[idx0] * (1.f - frac) + upchirp_os[idx1] * frac;
                preamble.push_back(sample);
            }
        }

        std::copy(preamble.begin(), preamble.end(), lane.preamble_raw.begin());
        lane.estimate_CFO_frac(preamble.data());
        float sto_f = lane.estimate_STO_frac(sto_int);
        std::printf("  STO frac (true=%.3f): est=%.4f\n",
                    static_cast<double>(sto_frac_true), static_cast<double>(sto_f));
        expect(lt(std::abs(sto_f - sto_frac_true), 0.2f))
            << "STO frac should be within 0.2 of true value";
    };

    "STO frac bounded output [-0.5, 0.5]"_test = [] {
        constexpr uint8_t sf = 8;
        auto lane = make_test_lane(sf);
        lane.k_hat = 0;

        auto preamble = make_preamble_with_cfo(sf, 0.f, lane.n_up_req);
        std::copy(preamble.begin(), preamble.end(), lane.preamble_raw.begin());
        lane.estimate_CFO_frac(preamble.data());

        float sto_f = lane.estimate_STO_frac(0);
        expect(ge(sto_f, -0.5f)) << "STO frac must be >= -0.5";
        expect(le(sto_f, 0.5f))  << "STO frac must be <= 0.5";
    };
};

const boost::ut::suite<"Mod-8 sync word STO correction (W3c)"> mod8_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "mod8 correction with residual +2"_test = [] {
        auto lane = make_test_lane(8);
        int delta = lane.mod8_sto_correction(10, 18);
        expect(eq(delta, 2)) << "mod-8 residual +2 should give delta=+2";
    };

    "mod8 correction with residual -1 (=7 mod 8)"_test = [] {
        auto lane = make_test_lane(8);
        int delta = lane.mod8_sto_correction(7, 15);
        expect(eq(delta, -1)) << "mod-8 residual 7 should give delta=-1";
    };

    "mod8 no correction when residuals match sync word"_test = [] {
        auto lane = make_test_lane(8);
        int delta = lane.mod8_sto_correction(8, 16);
        expect(eq(delta, 0)) << "exact match should give delta=0";
    };

    "mod8 no correction when residuals differ"_test = [] {
        auto lane = make_test_lane(8);
        int delta = lane.mod8_sto_correction(10, 19);
        expect(eq(delta, 0)) << "mismatched residuals should give delta=0";
    };

    "mod8 correction wraps for residual +4"_test = [] {
        auto lane = make_test_lane(8);
        int delta = lane.mod8_sto_correction(12, 20);
        expect(eq(delta, -4)) << "residual 4 should wrap to delta=-4";
    };

    "mod8 correction with large symbol values"_test = [] {
        auto lane = make_test_lane(8);
        int delta = lane.mod8_sto_correction(253, 245);
        expect(eq(delta, -3)) << "large values: residual 5 should wrap to delta=-3";
    };
};

// =============================================================================
// Soft Hamming decode: GrayPartition, LLR extraction, soft deinterleave
// =============================================================================

const boost::ut::suite<"GrayPartition"> gray_partition_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "partition sizes are M/2 each"_test = [] {
        GrayPartition gp;
        gp.init(8);
        expect(eq(gp.M, 256u));
        for (uint8_t k = 0; k < 8; k++) {
            expect(eq(gp.ones[k].size(), 128uz))
                << "ones[" << static_cast<int>(k) << "] should have M/2 entries";
            expect(eq(gp.zeros[k].size(), 128uz))
                << "zeros[" << static_cast<int>(k) << "] should have M/2 entries";
        }
    };

    "known bit values for SF8 symbol 42"_test = [] {
        GrayPartition gp;
        gp.init(8);
        // Gray code of 42: 42 ^ (42 >> 1) = 42 ^ 21 = 0b00101010 ^ 0b00010101 = 0b00111111 = 63
        // Binary: 63 = 0b00111111
        // Bit 0 (LSB) = 1, Bit 1 = 1, Bit 2 = 1, Bit 3 = 1, Bit 4 = 1, Bit 5 = 1, Bit 6 = 0, Bit 7 = 0
        uint32_t gray_42 = 42 ^ (42 >> 1);
        expect(eq(gray_42, 63u)) << "Gray(42) should be 63";

        // Verify symbol 42 is in the correct partition for each bit
        for (uint8_t k = 0; k < 8; k++) {
            bool bit_k = (gray_42 >> k) & 1u;
            if (bit_k) {
                bool found = false;
                for (uint32_t s : gp.ones[k]) {
                    if (s == 42) { found = true; break; }
                }
                expect(found) << "symbol 42 should be in ones[" << static_cast<int>(k) << "]";
            } else {
                bool found = false;
                for (uint32_t s : gp.zeros[k]) {
                    if (s == 42) { found = true; break; }
                }
                expect(found) << "symbol 42 should be in zeros[" << static_cast<int>(k) << "]";
            }
        }
    };

    "SF12 partition validity"_test = [] {
        GrayPartition gp;
        gp.init(12);
        expect(eq(gp.M, 4096u));
        for (uint8_t k = 0; k < 12; k++) {
            expect(eq(gp.ones[k].size(), 2048uz));
            expect(eq(gp.zeros[k].size(), 2048uz));
        }
    };
};

const boost::ut::suite<"Soft dechirp and LLR extraction"> soft_dechirp_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "LLR sign test: known symbol bin 42 at SF8"_test = [] {
        constexpr uint8_t sf = 8;
        constexpr uint32_t Nfft = 1u << sf;

        // Build chirp at bin 42 (which maps to FFT bin (42+1)%256 = 43 after +1 offset)
        std::vector<std::complex<float>> chirp(Nfft);
        build_upchirp(chirp.data(), 43, sf);  // FFT bin 43 → symbol 42 after mod(bin-1, N)

        std::vector<std::complex<float>> downchirp(Nfft), upchirp(Nfft);
        build_ref_chirps(upchirp.data(), downchirp.data(), sf);

        std::vector<std::complex<float>> scratch(Nfft);
        gr::algorithm::FFT<std::complex<float>> fft;

        auto result = dechirp_soft(chirp.data(), downchirp.data(),
                                   scratch.data(), Nfft, fft);
        expect(eq(result.bin, 43u)) << "FFT bin should be 43 for symbol 42";

        // Remap from FFT bin to symbol (subtract 1 mod N)
        std::vector<float> sym_mag_sq(Nfft);
        for (uint32_t s = 0; s < Nfft; s++) {
            sym_mag_sq[s] = result.mag_sq[(s + 1) % Nfft];
        }

        // Compute LLRs
        GrayPartition gp;
        gp.init(sf);
        std::vector<double> llrs(sf);
        SfLane::compute_symbol_llr(sym_mag_sq.data(), Nfft, sf, gp, llrs.data());

        // Gray code of 42: bits = 00111111 (MSB to LSB: k7=0 k6=0 k5=1 k4=1 k3=1 k2=1 k1=1 k0=1)
        uint32_t gray_42 = 42 ^ (42 >> 1);
        expect(eq(gray_42, 63u));

        for (uint8_t k = 0; k < sf; k++) {
            bool bit_k = (gray_42 >> k) & 1u;
            if (bit_k) {
                expect(gt(llrs[k], 0.0))
                    << "LLR[" << static_cast<int>(k) << "] should be > 0 for Gray bit 1";
            } else {
                expect(lt(llrs[k], 0.0))
                    << "LLR[" << static_cast<int>(k) << "] should be < 0 for Gray bit 0";
            }
        }
    };

    "dechirp_soft PMR matches dechirp_and_quality"_test = [] {
        constexpr uint8_t sf = 8;
        constexpr uint32_t Nfft = 1u << sf;

        std::vector<std::complex<float>> chirp(Nfft);
        build_upchirp(chirp.data(), 10, sf);

        std::vector<std::complex<float>> downchirp(Nfft), upchirp(Nfft);
        build_ref_chirps(upchirp.data(), downchirp.data(), sf);

        std::vector<std::complex<float>> scratch(Nfft);
        gr::algorithm::FFT<std::complex<float>> fft;

        auto soft_result = dechirp_soft(chirp.data(), downchirp.data(),
                                        scratch.data(), Nfft, fft);
        auto [bin_hard, pmr_hard] = dechirp_and_quality(
            chirp.data(), downchirp.data(), scratch.data(), Nfft, fft);

        expect(eq(soft_result.bin, bin_hard)) << "bin should match";
        expect(lt(std::abs(soft_result.pmr - pmr_hard), 0.01f)) << "PMR should match";
    };
};

const boost::ut::suite<"Soft decode roundtrip"> soft_decode_roundtrip_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "soft decode clean signal matches hard decode"_test = [] {
        // For all 16 nibbles and all CRs, verify soft decode gives same result
        // as hard decode when LLRs are strong (clean signal).
        for (uint8_t cr = 1; cr <= 4; cr++) {
            for (uint8_t nib = 0; nib < 16; nib++) {
                uint8_t cw = hamming_encode(nib, cr);
                auto bits = int2bool(cw, cr + 4);

                // Create strong LLRs: +10 for bit 1, -10 for bit 0
                std::vector<double> llrs(static_cast<std::size_t>(cr + 4));
                for (std::size_t i = 0; i < llrs.size(); i++) {
                    llrs[i] = bits[i] ? 10.0 : -10.0;
                }

                uint8_t soft = hamming_decode_soft(llrs.data(), cr);
                uint8_t hard = hamming_decode_hard(cw, cr);
                expect(eq(soft, hard))
                    << "soft/hard mismatch cr=" << static_cast<int>(cr)
                    << " nib=" << static_cast<int>(nib);
            }
        }
    };

    "soft deinterleave single codeword trace"_test = [] {
        // Trace through hard and soft paths for a minimal block to isolate the mismatch.
        constexpr uint8_t sf = 8;
        constexpr uint8_t sf_app = sf;
        constexpr uint8_t cr = 4;
        constexpr uint8_t cw_len = cr + 4;
        constexpr uint32_t N_fft = 1u << sf;

        // Single nibble → encode → interleave → one block of cw_len symbols
        uint8_t nib = 5;
        std::vector<uint8_t> codewords(sf_app);
        codewords[0] = hamming_encode(nib, cr);
        for (std::size_t i = 1; i < sf_app; i++) codewords[i] = hamming_encode(0, cr);

        auto interleaved = interleave_block(codewords, sf, cw_len, sf_app, false);

        // TX gray demap + shift
        std::vector<uint32_t> chirp_syms;
        for (auto s : interleaved) {
            uint32_t g = s;
            for (int j = 1; j < sf; j++) g ^= (s >> j);
            chirp_syms.push_back((g + 1) % N_fft);
        }

        // HARD RX: mod(cs-1, N) → gray_map → deinterleave → decode
        std::vector<uint16_t> rx_syms;
        for (auto cs : chirp_syms) {
            rx_syms.push_back(static_cast<uint16_t>(
                mod(static_cast<int64_t>(cs) - 1, static_cast<int64_t>(N_fft))));
        }
        std::vector<uint16_t> gray_syms;
        for (auto s : rx_syms) gray_syms.push_back(SfLane::grayMap(s));
        auto hard_cw = deinterleave_block(gray_syms, sf, cw_len, sf_app);
        uint8_t hard_nib = hamming_decode_hard(hard_cw[0], cr);
        std::printf("  hard_nib=%u expected=%u codeword=0x%02x\n",
                    hard_nib, nib, hard_cw[0]);

        // SOFT RX: create LLRs from symbol-indexed mag_sq
        GrayPartition gp;
        gp.init(sf);
        std::vector<std::vector<double>> sym_llrs;
        for (auto cs : chirp_syms) {
            auto sym = static_cast<uint32_t>(
                mod(static_cast<int64_t>(cs) - 1, static_cast<int64_t>(N_fft)));
            std::vector<float> sym_mag_sq(N_fft, 0.001f);
            sym_mag_sq[sym] = 100.f;
            std::vector<double> llrs(sf);
            SfLane::compute_symbol_llr(sym_mag_sq.data(), N_fft, sf, gp, llrs.data());
            sym_llrs.push_back(std::move(llrs));
        }
        SfLane soft_lane;
        soft_lane.sf = sf;
        soft_lane.N = N_fft;
        auto soft_nibs = soft_lane.processBlockSoft(sym_llrs, sf_app, cw_len, cr);
        std::printf("  soft_nib=%u expected=%u\n", soft_nibs[0], nib);

        expect(eq(hard_nib, nib)) << "hard decode should recover nibble";
        expect(eq(soft_nibs[0], nib)) << "soft decode should recover nibble";
    };

    "soft deinterleave matches hard deinterleave for clean signal"_test = [] {
        // Simulate the RX decode pipeline for both hard and soft paths.
        // The hard path: grayMap(symbol) → shift → deinterleave_block → hamming_decode_hard
        // The soft path: LLR from mag_sq → shift(trim) → deinterleave_block_soft → hamming_decode_soft
        //
        // Use a known set of nibbles, encode through the TX pipeline, then decode
        // both ways and compare.
        constexpr uint8_t sf = 8;
        constexpr uint8_t sf_app = sf;  // payload block
        constexpr uint8_t cr = 4;
        constexpr uint8_t cw_len = cr + 4;

        // Encode known nibbles
        std::vector<uint8_t> nibbles_in;
        for (int i = 0; i < sf_app; i++)
            nibbles_in.push_back(static_cast<uint8_t>(i % 16));
        std::vector<uint8_t> codewords;
        for (auto n : nibbles_in) codewords.push_back(hamming_encode(n, cr));

        // TX: interleave → gray_demap → +1 shift = chirp symbols
        auto interleaved = interleave_block(codewords, sf, cw_len, sf_app, false);
        std::vector<uint32_t> chirp_syms;
        for (auto s : interleaved) {
            uint32_t g = s;
            for (int j = 1; j < sf; j++) g ^= (s >> j);
            chirp_syms.push_back((g + 1) % (1u << sf));
        }

        // RX hard path: mod(sym-1, N) → grayMap → deinterleave → hamming_hard
        uint32_t N_fft = 1u << sf;
        std::vector<uint16_t> rx_hard_syms;
        for (auto cs : chirp_syms) {
            auto sym = static_cast<uint16_t>(mod(static_cast<int64_t>(cs) - 1,
                                                  static_cast<int64_t>(N_fft)));
            rx_hard_syms.push_back(sym);
        }
        // Use SfLane::processBlock hard path (via a temporary SfLane with no soft data)
        SfLane hard_lane;
        hard_lane.sf = sf;
        hard_lane.N = N_fft;
        auto hard_nibs = hard_lane.processBlock(rx_hard_syms, sf_app, cw_len, cr, false);

        // RX soft path: for each chirp symbol, create mag_sq with peak at that bin
        GrayPartition gp;
        gp.init(sf);

        std::vector<std::vector<double>> sym_llrs;
        for (auto cs : chirp_syms) {
            // The FFT would give a peak at bin = cs. After mod(bin-1, N) we get
            // the symbol. But compute_symbol_llr uses the GrayPartition which
            // indexes by symbol (not bin). The mag_sq is indexed by FFT bin.
            // In the real pipeline: mag_sq[bin] has the peak. Symbol s maps to
            // bin (s+1)%N. So mag_sq[(s+1)%N] is high for symbol s.
            // compute_symbol_llr takes mag_sq indexed by symbol s directly.
            // We need: sym_mag_sq[s] = FFT_mag_sq[(s+1)%N].
            // Since we have FFT peak at bin cs, sym_mag_sq[s] is high when (s+1)%N == cs,
            // i.e. s = (cs-1+N)%N = mod(cs-1, N).
            std::vector<float> sym_mag_sq(N_fft, 0.001f);
            auto sym = static_cast<uint32_t>(mod(static_cast<int64_t>(cs) - 1,
                                                  static_cast<int64_t>(N_fft)));
            sym_mag_sq[sym] = 100.f;

            std::vector<double> llrs(sf);
            SfLane::compute_symbol_llr(sym_mag_sq.data(), N_fft, sf, gp, llrs.data());
            sym_llrs.push_back(std::move(llrs));
        }

        // Use processBlockSoft directly
        SfLane soft_lane;
        soft_lane.sf = sf;
        soft_lane.N = N_fft;
        auto soft_nibs = soft_lane.processBlockSoft(sym_llrs, sf_app, cw_len, cr);

        expect(eq(soft_nibs.size(), hard_nibs.size())) << "nibble count mismatch";
        for (std::size_t i = 0; i < std::min(soft_nibs.size(), hard_nibs.size()); i++) {
            expect(eq(soft_nibs[i], hard_nibs[i]))
                << "soft/hard nibble mismatch at " << i
                << " soft=" << static_cast<int>(soft_nibs[i])
                << " hard=" << static_cast<int>(hard_nibs[i])
                << " expected=" << static_cast<int>(nibbles_in[i]);
        }
    };

    "soft decode with noise still recovers nibbles"_test = [] {
        // Generate a symbol at bin 42, add noise to mag_sq, verify LLR signs
        // still produce correct decode.
        constexpr uint8_t sf = 8;
        constexpr uint32_t Nfft = 1u << sf;

        GrayPartition gp;
        gp.init(sf);

        std::mt19937 gen(42);
        std::uniform_real_distribution<float> noise_dist(0.f, 5.f);

        // Repeat for several symbols
        for (uint32_t sym = 0; sym < 16; sym++) {
            std::vector<float> mag_sq(Nfft);
            for (auto& m : mag_sq) m = noise_dist(gen);
            mag_sq[sym] = 200.f;  // Strong signal peak

            std::vector<double> llrs(sf);
            SfLane::compute_symbol_llr(mag_sq.data(), Nfft, sf, gp, llrs.data());

            // Verify LLR signs match the Gray code
            uint32_t gray_sym = sym ^ (sym >> 1);
            for (uint8_t k = 0; k < sf; k++) {
                bool bit_k = (gray_sym >> k) & 1u;
                if (bit_k) {
                    expect(gt(llrs[k], 0.0))
                        << "sym=" << sym << " k=" << static_cast<int>(k) << " should be > 0";
                } else {
                    expect(lt(llrs[k], 0.0))
                        << "sym=" << sym << " k=" << static_cast<int>(k) << " should be < 0";
                }
            }
        }
    };
};

// =============================================================================
// Soft decode regression: header block (sf_app = sf-2) hard vs soft comparison
// =============================================================================

const boost::ut::suite<"Soft decode header block regression"> soft_header_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "soft decode header block matches hard decode for all SFs and nibbles"_test = [] {
        // For each SF (7-12), create a header interleaver block (sf_app = sf-2, cw_len = 8,
        // cr = 4 always for header). Encode sf_app nibbles through the TX pipeline, then
        // decode via both hard and soft paths. They must produce identical output.
        for (uint8_t test_sf = 7; test_sf <= 12; test_sf++) {
            const uint8_t sf_app = test_sf - 2;
            const uint8_t cr     = 4;
            const uint8_t cw_len = 8;  // header always uses cw_len=8
            const uint32_t N_fft = 1u << test_sf;

            // Test all 16 nibble values in each position
            for (uint8_t nib_val = 0; nib_val < 16; nib_val++) {
                // Build sf_app nibbles: [nib_val, 0, 0, ..., 0]
                std::vector<uint8_t> nibbles_in(sf_app, 0);
                nibbles_in[0] = nib_val;

                // TX: Hamming-encode → interleave → Gray demap → +1 shift
                std::vector<uint8_t> codewords;
                for (auto n : nibbles_in) codewords.push_back(hamming_encode(n, cr));
                auto interleaved = interleave_block(codewords, test_sf, cw_len, sf_app, true);

                std::vector<uint32_t> chirp_syms;
                for (auto s : interleaved) {
                    uint32_t g = s;
                    for (int j = 1; j < test_sf; j++) g ^= (s >> j);
                    chirp_syms.push_back((g + 1) % N_fft);
                }

                // RX hard path: mod(cs-1, N) → grayMap → shift → deinterleave → hamming_hard
                SfLane hard_lane;
                hard_lane.sf = test_sf;
                hard_lane.N  = N_fft;
                std::vector<uint16_t> rx_syms;
                for (auto cs : chirp_syms) {
                    rx_syms.push_back(static_cast<uint16_t>(
                        mod(static_cast<int64_t>(cs) - 1, static_cast<int64_t>(N_fft))));
                }
                auto hard_nibs = hard_lane.processBlock(rx_syms, sf_app, cw_len, cr, true);

                // RX soft path: create mag_sq peak → LLR → processBlockSoft
                GrayPartition gp;
                gp.init(test_sf);

                std::vector<std::vector<double>> sym_llrs;
                for (auto cs : chirp_syms) {
                    auto sym = static_cast<uint32_t>(
                        mod(static_cast<int64_t>(cs) - 1, static_cast<int64_t>(N_fft)));
                    std::vector<float> sym_mag_sq(N_fft, 0.001f);
                    sym_mag_sq[sym] = 100.f;
                    std::vector<double> llrs(test_sf);
                    SfLane::compute_symbol_llr(sym_mag_sq.data(), N_fft, test_sf, gp, llrs.data());
                    sym_llrs.push_back(std::move(llrs));
                }

                SfLane soft_lane;
                soft_lane.sf = test_sf;
                soft_lane.N  = N_fft;
                auto soft_nibs = soft_lane.processBlockSoft(sym_llrs, sf_app, cw_len, cr);

                // Compare
                expect(eq(soft_nibs.size(), hard_nibs.size()))
                    << "SF" << static_cast<int>(test_sf)
                    << " nib=" << static_cast<int>(nib_val) << " count mismatch";
                for (std::size_t i = 0; i < std::min(soft_nibs.size(), hard_nibs.size()); i++) {
                    expect(eq(soft_nibs[i], hard_nibs[i]))
                        << "SF" << static_cast<int>(test_sf)
                        << " nib=" << static_cast<int>(nib_val)
                        << " pos=" << i
                        << " soft=" << static_cast<int>(soft_nibs[i])
                        << " hard=" << static_cast<int>(hard_nibs[i])
                        << " expected=" << static_cast<int>(nibbles_in[i]);
                }
            }
            std::printf("  SF%u header block: all 16 nibbles passed\n",
                        static_cast<unsigned>(test_sf));
        }
    };

    "soft decode payload block matches hard decode for all SFs"_test = [] {
        // Payload blocks: sf_app = sf, cw_len = cr+4
        // This verifies soft decode for all SFs, not just SF8.
        for (uint8_t test_sf = 7; test_sf <= 12; test_sf++) {
            const uint8_t sf_app = test_sf;
            const uint8_t cr     = 4;
            const uint8_t cw_len = cr + 4;
            const uint32_t N_fft = 1u << test_sf;

            for (uint8_t nib_val = 0; nib_val < 16; nib_val++) {
                std::vector<uint8_t> nibbles_in(sf_app, 0);
                nibbles_in[0] = nib_val;

                std::vector<uint8_t> codewords;
                for (auto n : nibbles_in) codewords.push_back(hamming_encode(n, cr));
                auto interleaved = interleave_block(codewords, test_sf, cw_len, sf_app, false);

                std::vector<uint32_t> chirp_syms;
                for (auto s : interleaved) {
                    uint32_t g = s;
                    for (int j = 1; j < test_sf; j++) g ^= (s >> j);
                    chirp_syms.push_back((g + 1) % N_fft);
                }

                SfLane hard_lane;
                hard_lane.sf = test_sf;
                hard_lane.N  = N_fft;
                std::vector<uint16_t> rx_syms;
                for (auto cs : chirp_syms) {
                    rx_syms.push_back(static_cast<uint16_t>(
                        mod(static_cast<int64_t>(cs) - 1, static_cast<int64_t>(N_fft))));
                }
                auto hard_nibs = hard_lane.processBlock(rx_syms, sf_app, cw_len, cr, false);

                GrayPartition gp;
                gp.init(test_sf);
                std::vector<std::vector<double>> sym_llrs;
                for (auto cs : chirp_syms) {
                    auto sym = static_cast<uint32_t>(
                        mod(static_cast<int64_t>(cs) - 1, static_cast<int64_t>(N_fft)));
                    std::vector<float> sym_mag_sq(N_fft, 0.001f);
                    sym_mag_sq[sym] = 100.f;
                    std::vector<double> llrs(test_sf);
                    SfLane::compute_symbol_llr(sym_mag_sq.data(), N_fft, test_sf, gp, llrs.data());
                    sym_llrs.push_back(std::move(llrs));
                }

                SfLane soft_lane;
                soft_lane.sf = test_sf;
                soft_lane.N  = N_fft;
                auto soft_nibs = soft_lane.processBlockSoft(sym_llrs, sf_app, cw_len, cr);

                expect(eq(soft_nibs.size(), hard_nibs.size()))
                    << "SF" << static_cast<int>(test_sf)
                    << " nib=" << static_cast<int>(nib_val) << " count mismatch";
                for (std::size_t i = 0; i < std::min(soft_nibs.size(), hard_nibs.size()); i++) {
                    expect(eq(soft_nibs[i], hard_nibs[i]))
                        << "SF" << static_cast<int>(test_sf)
                        << " nib=" << static_cast<int>(nib_val)
                        << " pos=" << i
                        << " soft=" << static_cast<int>(soft_nibs[i])
                        << " hard=" << static_cast<int>(hard_nibs[i])
                        << " expected=" << static_cast<int>(nibbles_in[i]);
                }
            }
            std::printf("  SF%u payload block: all 16 nibbles passed\n",
                        static_cast<unsigned>(test_sf));
        }
    };
};

// =============================================================================
// Soft decode under noise: per-SF sensitivity comparison
// =============================================================================

const boost::ut::suite<"Soft vs hard decode noise sensitivity"> soft_noise_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "soft vs hard interleaver block under noise for all SFs"_test = [] {
        // Fair comparison: both hard and soft decode use the SAME noisy argmax
        // symbol (as in a real receiver). Hard decode uses the argmax bin;
        // soft decode uses the LLRs from the same mag_sq spectrum.
        std::mt19937 gen(12345);

        for (uint8_t test_sf = 7; test_sf <= 12; test_sf++) {
            const uint8_t cr = 4;
            const uint32_t N_fft = 1u << test_sf;

            for (int block_type = 0; block_type <= 1; block_type++) {
                const uint8_t sf_app = (block_type == 0)
                    ? static_cast<uint8_t>(test_sf - 2) : test_sf;
                const uint8_t cw_len = (block_type == 0) ? uint8_t(8)
                    : static_cast<uint8_t>(cr + 4);
                const char* block_name = (block_type == 0) ? "header" : "payload";
                const bool add_parity = (block_type == 0);

                int total_nibs = 0, soft_errors = 0, hard_errors = 0;
                const int N_trials = 50;

                for (int trial = 0; trial < N_trials; trial++) {
                    std::vector<uint8_t> nibbles_in(sf_app);
                    for (auto& n : nibbles_in)
                        n = static_cast<uint8_t>(gen() % 16);

                    std::vector<uint8_t> codewords;
                    for (auto n : nibbles_in) codewords.push_back(hamming_encode(n, cr));
                    auto interleaved = interleave_block(
                        codewords, test_sf, cw_len, sf_app, add_parity);

                    std::vector<uint32_t> chirp_syms;
                    for (auto s : interleaved) {
                        uint32_t g = s;
                        for (int j = 1; j < test_sf; j++) g ^= (s >> j);
                        chirp_syms.push_back((g + 1) % N_fft);
                    }

                    GrayPartition gp;
                    gp.init(test_sf);
                    std::normal_distribution<float> noise_dist(0.f, 3.f);

                    std::vector<std::vector<double>> sym_llrs;
                    std::vector<uint16_t> rx_syms;  // from argmax of noisy mag_sq

                    for (auto cs : chirp_syms) {
                        auto true_sym = static_cast<uint32_t>(
                            mod(static_cast<int64_t>(cs) - 1,
                                static_cast<int64_t>(N_fft)));

                        std::vector<float> sym_mag_sq(N_fft);
                        for (auto& m : sym_mag_sq) {
                            float n = noise_dist(gen);
                            m = n * n;
                        }
                        sym_mag_sq[true_sym] += 100.f;

                        // Find argmax (what the real receiver would use)
                        uint32_t argmax = 0;
                        float maxv = sym_mag_sq[0];
                        for (uint32_t s = 1; s < N_fft; s++) {
                            if (sym_mag_sq[s] > maxv) { maxv = sym_mag_sq[s]; argmax = s; }
                        }
                        rx_syms.push_back(static_cast<uint16_t>(argmax));

                        std::vector<double> llrs(test_sf);
                        SfLane::compute_symbol_llr(
                            sym_mag_sq.data(), N_fft, test_sf, gp, llrs.data());
                        sym_llrs.push_back(std::move(llrs));
                    }

                    SfLane hard_lane;
                    hard_lane.sf = test_sf;
                    hard_lane.N  = N_fft;
                    auto hard_nibs = hard_lane.processBlock(
                        rx_syms, sf_app, cw_len, cr, block_type == 0);

                    SfLane soft_lane;
                    soft_lane.sf = test_sf;
                    soft_lane.N  = N_fft;
                    auto soft_nibs = soft_lane.processBlockSoft(
                        sym_llrs, sf_app, cw_len, cr);

                    for (std::size_t i = 0; i < std::min(nibbles_in.size(),
                                                          soft_nibs.size()); i++) {
                        total_nibs++;
                        if (soft_nibs[i] != nibbles_in[i]) soft_errors++;
                        if (i < hard_nibs.size() && hard_nibs[i] != nibbles_in[i])
                            hard_errors++;
                    }
                }

                std::printf("  SF%u %s: %d nibs, hard_err=%d (%.1f%%), "
                            "soft_err=%d (%.1f%%)\n",
                    static_cast<unsigned>(test_sf), block_name, total_nibs,
                    hard_errors, 100.0 * hard_errors / total_nibs,
                    soft_errors, 100.0 * soft_errors / total_nibs);

                // Soft should not be significantly worse than hard
                expect(le(soft_errors, std::max(hard_errors * 3 + 5, 10)))
                    << "SF" << static_cast<int>(test_sf) << " " << block_name
                    << ": soft errors significantly worse than hard";
            }
        }
    };
};

int main() { /* boost::ut auto-runs all suites */ }
