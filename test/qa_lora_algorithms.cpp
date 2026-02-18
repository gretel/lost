// SPDX-License-Identifier: ISC
#include "test_helpers.hpp"

#include <array>
#include <cstring>

#include <gnuradio-4.0/lora/algorithm/tables.hpp>
#include <gnuradio-4.0/lora/algorithm/utilities.hpp>
#include <gnuradio-4.0/lora/algorithm/hamming.hpp>
#include <gnuradio-4.0/lora/algorithm/crc.hpp>
#include <gnuradio-4.0/lora/algorithm/interleaving.hpp>

using namespace gr::lora::test;

// ---- Tests ----

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

int main() { /* boost::ut auto-runs all suites */ }
