// SPDX-License-Identifier: ISC
//
// tx_chain.hpp — Complete LoRa TX chain as pure algorithm functions.
//
// This header provides the full LoRa transmitter pipeline from payload bytes
// to baseband IQ samples, without any GR4 graph or block dependencies. Each
// stage is exposed as a separate function for testability, and the top-level
// generate_frame_iq() ties them together.
//
// Pipeline:
//   payload bytes -> whiten -> insert_header -> add_crc -> hamming_encode
//   -> interleave -> gray_demap -> modulate -> IQ samples

#ifndef GNURADIO_LORA_TX_CHAIN_HPP
#define GNURADIO_LORA_TX_CHAIN_HPP

#include <complex>
#include <cstdint>
#include <span>
#include <vector>

#include <gnuradio-4.0/lora/algorithm/crc.hpp>
#include <gnuradio-4.0/lora/algorithm/hamming.hpp>
#include <gnuradio-4.0/lora/algorithm/interleaving.hpp>
#include <gnuradio-4.0/lora/algorithm/tables.hpp>
#include <gnuradio-4.0/lora/algorithm/utilities.hpp>

namespace gr::lora {

/// Whitening: each payload byte -> 2 nibbles XORed with LFSR sequence.
[[nodiscard]] inline std::vector<uint8_t> whiten(
        std::span<const uint8_t> payload) {
    std::vector<uint8_t> nibbles;
    nibbles.reserve(payload.size() * 2);
    for (std::size_t i = 0; i < payload.size(); i++) {
        uint8_t ws = whitening_seq[i % whitening_seq.size()];
        uint8_t low_nib  = (payload[i] & 0x0F) ^ (ws & 0x0F);
        uint8_t high_nib = ((payload[i] >> 4) & 0x0F) ^ ((ws >> 4) & 0x0F);
        nibbles.push_back(low_nib);
        nibbles.push_back(high_nib);
    }
    return nibbles;
}

/// Insert explicit header (5 nibbles) before whitened payload nibbles.
[[nodiscard]] inline std::vector<uint8_t> insert_header(
        std::span<const uint8_t> whitened_nibbles,
        uint8_t payload_len, uint8_t cr, bool has_crc) {
    auto hdr = build_explicit_header(payload_len, cr, has_crc);
    std::vector<uint8_t> result;
    result.reserve(5 + whitened_nibbles.size());
    result.insert(result.end(), hdr.begin(), hdr.end());
    result.insert(result.end(), whitened_nibbles.begin(),
                  whitened_nibbles.end());
    return result;
}

/// Append CRC-16/XMODEM as 4 nibbles (LoRa CRC uses the last-2-byte XOR
/// variant per Tapparel & Burg Section III-E).
[[nodiscard]] inline std::vector<uint8_t> add_crc(
        std::span<const uint8_t> with_header,
        std::span<const uint8_t> payload, bool has_crc) {
    std::vector<uint8_t> result(with_header.begin(), with_header.end());
    if (has_crc && payload.size() >= 2) {
        uint16_t crc = crc16(payload.subspan(0, payload.size() - 2));
        crc ^= static_cast<uint16_t>(payload[payload.size() - 1]);
        crc ^= static_cast<uint16_t>(payload[payload.size() - 2]) << 8;
        result.push_back(static_cast<uint8_t>(crc & 0x0F));
        result.push_back(static_cast<uint8_t>((crc >> 4) & 0x0F));
        result.push_back(static_cast<uint8_t>((crc >> 8) & 0x0F));
        result.push_back(static_cast<uint8_t>((crc >> 12) & 0x0F));
    }
    return result;
}

/// Hamming-encode a full frame of nibbles. The first (sf - 2) nibbles
/// (header region) always use cr=4; the rest use the given CR.
[[nodiscard]] inline std::vector<uint8_t> hamming_enc_frame(
        std::span<const uint8_t> nibbles, uint8_t sf, uint8_t cr) {
    std::vector<uint8_t> encoded;
    encoded.reserve(nibbles.size());
    for (std::size_t i = 0; i < nibbles.size(); i++) {
        uint8_t cr_app = (static_cast<int>(i) < sf - 2) ? uint8_t(4) : cr;
        encoded.push_back(hamming_encode(nibbles[i], cr_app));
    }
    return encoded;
}

/// Gray de-mapping: binary -> Gray code + 1 cyclic offset.
[[nodiscard]] inline std::vector<uint32_t> gray_demap(
        std::span<const uint32_t> symbols, uint8_t sf) {
    const auto n = static_cast<int64_t>(1u << sf);
    std::vector<uint32_t> result;
    result.reserve(symbols.size());
    for (auto sym : symbols) {
        uint32_t g = sym;
        for (int j = 1; j < sf; j++) g ^= (sym >> j);
        result.push_back(static_cast<uint32_t>(mod(
            static_cast<int64_t>(g) + 1, n)));
    }
    return result;
}

/// Modulate symbols into baseband IQ chirps with preamble, sync word,
/// and SFD.
[[nodiscard]] inline std::vector<std::complex<float>> modulate_frame(
        std::span<const uint32_t> symbols,
        uint8_t sf, uint8_t os_factor,
        uint16_t sync_word, uint16_t preamble_len,
        uint32_t zero_pad = 0) {
    const uint32_t sps = (1u << sf) * os_factor;
    std::vector<std::complex<float>> iq;

    std::vector<std::complex<float>> upchirp(sps), downchirp(sps);
    build_ref_chirps(upchirp.data(), downchirp.data(), sf, os_factor);

    auto sw0 = static_cast<uint16_t>(((sync_word & 0xF0) >> 4) << 3);
    auto sw1 = static_cast<uint16_t>((sync_word & 0x0F) << 3);

    // Preamble
    for (uint16_t i = 0; i < preamble_len; i++) {
        iq.insert(iq.end(), upchirp.begin(), upchirp.end());
    }

    // 2 sync word upchirps
    {
        std::vector<std::complex<float>> sw_chirp(sps);
        build_upchirp(sw_chirp.data(), sw0, sf, os_factor);
        iq.insert(iq.end(), sw_chirp.begin(), sw_chirp.end());
    }
    {
        std::vector<std::complex<float>> sw_chirp(sps);
        build_upchirp(sw_chirp.data(), sw1, sf, os_factor);
        iq.insert(iq.end(), sw_chirp.begin(), sw_chirp.end());
    }

    // SFD: 2 full downchirps + quarter downchirp
    iq.insert(iq.end(), downchirp.begin(), downchirp.end());
    iq.insert(iq.end(), downchirp.begin(), downchirp.end());
    iq.insert(iq.end(), downchirp.begin(),
              downchirp.begin() + static_cast<std::ptrdiff_t>(sps / 4));

    // Payload chirps
    for (auto sym : symbols) {
        std::vector<std::complex<float>> chirp(sps);
        build_upchirp(chirp.data(), sym, sf, os_factor);
        iq.insert(iq.end(), chirp.begin(), chirp.end());
    }

    // Zero padding
    if (zero_pad > 0) {
        iq.resize(iq.size() + zero_pad, std::complex<float>(0.f, 0.f));
    }

    return iq;
}

/// Full TX chain: payload bytes -> baseband IQ samples.
///
/// Ties together the complete LoRa TX pipeline:
///   whiten -> insert_header -> add_crc -> hamming_encode -> interleave
///   -> gray_demap -> modulate
[[nodiscard]] inline std::vector<std::complex<float>> generate_frame_iq(
        std::span<const uint8_t> payload,
        uint8_t sf, uint8_t cr, uint8_t os_factor,
        uint16_t sync_word, uint16_t preamble_len,
        bool has_crc = true,
        uint32_t zero_pad = 0) {
    auto whitened    = whiten(payload);
    auto with_header = insert_header(whitened,
                                     static_cast<uint8_t>(payload.size()),
                                     cr, has_crc);
    auto with_crc    = add_crc(with_header, payload, has_crc);
    auto encoded     = hamming_enc_frame(with_crc, sf, cr);
    auto interleaved = interleave_frame(encoded, sf, cr);
    auto gray_mapped = gray_demap(interleaved, sf);
    return modulate_frame(gray_mapped, sf, os_factor,
                          sync_word, preamble_len, zero_pad);
}

}  // namespace gr::lora

#endif  // GNURADIO_LORA_TX_CHAIN_HPP
