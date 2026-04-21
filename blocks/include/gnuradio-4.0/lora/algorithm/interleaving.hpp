// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_INTERLEAVING_HPP
#define GNURADIO_LORA_INTERLEAVING_HPP

#include <cstdint>
#include <vector>

#include <gnuradio-4.0/lora/algorithm/utilities.hpp>

namespace gr::lora {

/// Diagonal block interleaver (TX side).
///
/// Processes one interleaver block at a time:
///   Input:  sf_app codewords, each cw_len bits wide
///   Output: cw_len symbols, each sf bits wide (sf_app bits data + optional parity + zero-pad)
///
/// For the header block (is_header=true or ldro=true): sf_app = sf-2, cw_len = 8,
/// and a parity bit + zero are appended to make each symbol sf bits wide.
///
/// For payload blocks: sf_app = sf, cw_len = cr+4, no parity/padding.
///
/// The interleaving pattern: inter[i][j] = cw_bin[(i-j-1) mod sf_app][i]
/// (Tapparel & Burg, Section III-C)
[[nodiscard]] inline std::vector<uint32_t> interleave_block(const std::vector<uint8_t>& codewords, // sf_app codewords
    uint8_t sf, uint8_t cw_len, uint8_t sf_app, bool add_parity) {
    // Build binary matrix: sf_app rows x cw_len columns
    std::vector<std::vector<bool>> cw_bin(sf_app);
    for (std::size_t i = 0; i < sf_app; i++) {
        if (i < codewords.size()) {
            cw_bin[i] = int2bool(codewords[i], cw_len);
        } else {
            cw_bin[i] = std::vector<bool>(cw_len, false);
        }
    }

    // Output: cw_len symbols, each sf bits wide
    std::vector<std::vector<bool>> inter_bin(cw_len, std::vector<bool>(sf, false));

    // Diagonal interleaving
    for (std::size_t i = 0; i < cw_len; i++) {
        for (std::size_t j = 0; j < sf_app; j++) {
            auto row        = static_cast<std::size_t>(mod(static_cast<int>(i) - static_cast<int>(j) - 1, sf_app));
            inter_bin[i][j] = cw_bin[row][i];
        }
        // For header / LDRO: append parity bit at position sf_app
        if (add_parity) {
            bool parity = false;
            for (std::size_t j = 0; j < sf_app; j++) {
                parity ^= inter_bin[i][j];
            }
            inter_bin[i][sf_app] = parity;
            // Remaining bits (sf_app+1 .. sf-1) stay 0
        }
    }

    // Convert binary rows to uint32 symbols
    std::vector<uint32_t> symbols;
    symbols.reserve(cw_len);
    for (std::size_t i = 0; i < cw_len; i++) {
        symbols.push_back(bool2int(inter_bin[i]));
    }
    return symbols;
}

/// Interleave an entire frame of codewords (TX).
/// Handles the header block (first sf-2 codewords with cw_len=8 and parity)
/// and subsequent payload blocks (sf codewords with cw_len=cr+4).
/// When ldro=true (Low Data Rate Optimization), payload blocks also use
/// sf_app=sf-2 with parity, matching the header block's reduced rate.
[[nodiscard]] inline std::vector<uint32_t> interleave_frame(const std::vector<uint8_t>& codewords, uint8_t sf, uint8_t cr, bool ldro = false) {
    std::vector<uint32_t> all_symbols;
    std::size_t           idx      = 0;
    bool                  is_first = true;

    while (idx < codewords.size()) {
        bool    reduced_rate = is_first || ldro;
        uint8_t sf_app       = reduced_rate ? static_cast<uint8_t>(sf - 2) : sf;
        uint8_t cw_len       = is_first ? uint8_t(8) : static_cast<uint8_t>(4 + cr);

        // Gather sf_app codewords for this block
        std::vector<uint8_t> block;
        for (std::size_t i = 0; i < sf_app && idx < codewords.size(); i++) {
            block.push_back(codewords[idx++]);
        }
        // Pad with zeros if needed (incomplete last block)
        while (block.size() < sf_app) {
            block.push_back(0);
        }

        auto syms = interleave_block(block, sf, cw_len, sf_app, reduced_rate);
        all_symbols.insert(all_symbols.end(), syms.begin(), syms.end());
        is_first = false;
    }

    return all_symbols;
}

/// Diagonal block deinterleaver (RX side).
///
/// Inverse of interleave_block. Takes cw_len symbols (sf_app significant bits each)
/// and produces sf_app codewords (cw_len bits each).
///
/// deinter[mod(i-j-1, sf_app)][i] = inter[i][j]
/// (inverse of Tapparel & Burg, Section III-C)
[[nodiscard]] inline std::vector<uint8_t> deinterleave_block(const std::vector<uint16_t>& symbols, // cw_len symbols
    uint8_t /*sf*/, uint8_t cw_len, uint8_t sf_app) {
    // Convert symbols to binary: inter_bin[i] = int2bool(symbols[i], sf_app).
    // Each symbol is treated as an sf_app-bit value (parity/padding bits are ignored).
    std::vector<std::vector<bool>> inter_bin(cw_len);
    for (std::size_t i = 0; i < cw_len; i++) {
        inter_bin[i] = int2bool(symbols[i], sf_app);
    }

    // Deinterleave
    std::vector<std::vector<bool>> deinter_bin(sf_app, std::vector<bool>(cw_len, false));
    for (std::size_t i = 0; i < cw_len; i++) {
        for (std::size_t j = 0; j < sf_app; j++) {
            auto row            = static_cast<std::size_t>(mod(static_cast<int>(i) - static_cast<int>(j) - 1, sf_app));
            deinter_bin[row][i] = inter_bin[i][j];
        }
    }

    // Convert to uint8 codewords
    std::vector<uint8_t> codewords;
    codewords.reserve(sf_app);
    for (std::size_t i = 0; i < sf_app; i++) {
        codewords.push_back(static_cast<uint8_t>(bool2int(deinter_bin[i])));
    }
    return codewords;
}

} // namespace gr::lora

#endif // GNURADIO_LORA_INTERLEAVING_HPP
