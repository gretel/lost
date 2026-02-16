// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_SYMBOL_DEMODULATOR_HPP
#define GNURADIO_LORA_SYMBOL_DEMODULATOR_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/algorithm/crc.hpp>
#include <gnuradio-4.0/lora/algorithm/hamming.hpp>
#include <gnuradio-4.0/lora/algorithm/interleaving.hpp>
#include <gnuradio-4.0/lora/algorithm/tables.hpp>
#include <gnuradio-4.0/lora/algorithm/utilities.hpp>

namespace gr::lora {

/// SymbolDemodulator: aligned IQ symbol blocks -> decoded payload bytes.
///
/// Full LoRa RX decode pipeline in a single block:
///   dechirp + FFT -> Gray mapping -> deinterleave -> Hamming decode
///   -> header parse -> dewhiten -> CRC verify
///
/// Processes symbols in groups: first 8 symbols are the header block
/// (sf_app=sf-2, cw_len=8), then payload blocks of cw_len=(4+cr) each.
///
/// Input:  N (= 2^sf) complex samples per symbol, from BurstDetector.
/// Output: decoded payload bytes.
/// Message port: publishes decoded payload + CRC status.
///
/// State machine:
///   IDLE    -> waiting for burst_start tag
///   HEADER  -> accumulating 8 header symbols
///   PAYLOAD -> accumulating payload symbols, emitting decoded bytes at end
struct SymbolDemodulator : gr::Block<SymbolDemodulator, gr::NoDefaultTagForwarding> {
    gr::PortIn<std::complex<float>>  in;
    gr::PortOut<uint8_t>             out;
    gr::MsgPortOut                   msg_out;  ///< decoded payload message

    // Configuration
    uint8_t  sf           = 8;
    uint32_t bandwidth    = 62500;     ///< LoRa bandwidth in Hz (for LDRO auto-detect)
    bool     impl_head    = false;
    uint8_t  impl_cr      = 4;        ///< CR for implicit header mode
    uint8_t  impl_pay_len = 0;        ///< payload len for implicit header mode
    bool     impl_has_crc = true;     ///< CRC flag for implicit header mode
    uint8_t  ldro_mode    = 2;        ///< 0=off, 1=on, 2=auto

    GR_MAKE_REFLECTABLE(SymbolDemodulator, in, out, msg_out,
                        sf, bandwidth, impl_head, impl_cr, impl_pay_len, impl_has_crc, ldro_mode);

    // --- Internal state ---
    enum State : uint8_t { IDLE, HEADER, PAYLOAD };

    State    _state = IDLE;
    uint32_t _N     = 0;   ///< 2^sf

    // CFO parameters (from burst_start tag)
    int      _cfo_int  = 0;
    float    _cfo_frac = 0.f;
    bool     _is_downchirp = false;  ///< true if burst uses downchirp preamble

    // Reference downchirp (built with CFO baked in)
    std::vector<std::complex<float>> _downchirp;
    std::vector<std::complex<float>> _dechirped;

    // Symbol accumulation
    std::vector<uint16_t> _symbol_buffer;  ///< accumulated symbols for current block
    uint32_t _total_symbols_rx = 0;        ///< total symbols received in this frame

    // Header decode results
    uint8_t  _cr       = 4;
    uint32_t _pay_len  = 0;
    bool     _has_crc  = true;
    bool     _ldro     = false;
    uint32_t _symb_numb = 0;  ///< total payload symbols expected

    // Nibble accumulation (across deinterleave blocks)
    std::vector<uint8_t> _nibbles;

    // FFT engine
    gr::algorithm::FFT<std::complex<float>> _fft;

    void start() { recalculate(); }

    void recalculate() {
        _N = 1u << sf;
        _downchirp.resize(_N);
        _dechirped.resize(_N);

        // Default downchirp (will be rebuilt with CFO on burst_start)
        std::vector<std::complex<float>> upchirp(_N);
        build_ref_chirps(upchirp.data(), _downchirp.data(), sf, 1);

        _fft = gr::algorithm::FFT<std::complex<float>>{};
        _state = IDLE;
    }

    /// Build the reference downchirp with CFO baked in.
    void buildDownchirpWithCFO() {
        std::vector<std::complex<float>> upchirp(_N);
        build_upchirp(upchirp.data(),
                      static_cast<uint32_t>(mod(_cfo_int, static_cast<int64_t>(_N))),
                      sf, 1);
        for (uint32_t n = 0; n < _N; n++) {
            _downchirp[n] = std::conj(upchirp[n]);
        }
        // Apply CFO_frac correction
        for (uint32_t n = 0; n < _N; n++) {
            float phase = -2.f * static_cast<float>(std::numbers::pi) * _cfo_frac
                        / static_cast<float>(_N) * static_cast<float>(n);
            _downchirp[n] *= std::complex<float>(std::cos(phase), std::sin(phase));
        }
    }

    /// Dechirp one symbol -> raw symbol value (argmax - 1).
    /// Gray mapping and header division happen in processBlock().
    [[nodiscard]] uint16_t demodSymbol(const std::complex<float>* samples) {
        for (uint32_t i = 0; i < _N; i++) {
            _dechirped[i] = samples[i] * _downchirp[i];
        }

        auto fft_out = _fft.compute(_dechirped);

        float max_val = 0.f;
        uint32_t max_idx = 0;
        for (uint32_t i = 0; i < _N; i++) {
            float mag_sq = fft_out[i].real() * fft_out[i].real()
                         + fft_out[i].imag() * fft_out[i].imag();
            if (mag_sq > max_val) {
                max_val = mag_sq;
                max_idx = i;
            }
        }

        // Subtract 1 (cancel TX +1 offset)
        return static_cast<uint16_t>(
            mod(static_cast<int64_t>(max_idx) - 1, static_cast<int64_t>(_N)));
    }

    /// Gray demapping: x ^ (x >> 1)
    [[nodiscard]] static uint16_t grayMap(uint16_t symbol) {
        return static_cast<uint16_t>(symbol ^ (symbol >> 1));
    }

    /// Process a complete interleaver block of symbols -> nibbles.
    /// Order: GrayMapping -> header_divide -> Deinterleave -> HammingDec
    [[nodiscard]] std::vector<uint8_t> processBlock(
            const std::vector<uint16_t>& symbols,
            uint8_t sf_app, uint8_t cw_len, uint8_t cr_app,
            bool is_header_block) {
        // Step 1: Gray demapping (BEFORE divide-by-4)
        std::vector<uint16_t> gray_syms;
        gray_syms.reserve(symbols.size());
        for (auto s : symbols) {
            gray_syms.push_back(grayMap(s));
        }

        // Step 2: For header/LDRO blocks, divide by 4 (right-shift by SF - sf_app)
        if (is_header_block) {
            for (auto& s : gray_syms) {
                s >>= (sf - sf_app);
            }
        }

        // Step 3: Deinterleave
        auto codewords = deinterleave_block(gray_syms, sf, cw_len, sf_app);

        // Hamming decode
        std::vector<uint8_t> nibbles;
        nibbles.reserve(codewords.size());
        for (auto cw : codewords) {
            nibbles.push_back(hamming_decode_hard(cw, cr_app));
        }

        return nibbles;
    }

    /// Decode the accumulated nibbles into payload bytes.
    /// Handles header parsing, dewhitening, CRC verification, and output.
    void decodeFrame(std::span<uint8_t> out_span, std::size_t& out_idx) {
        if (_nibbles.size() < 5 && !impl_head) return;

        // --- Header decode ---
        std::size_t data_start = 0;
        if (!impl_head) {
            auto info = parse_explicit_header(
                _nibbles[0], _nibbles[1], _nibbles[2], _nibbles[3], _nibbles[4]);

            if (!info.checksum_valid || info.payload_len == 0) {
                // Header error -- discard
                return;
            }

            _pay_len = info.payload_len;
            _cr      = info.cr;
            _has_crc = info.has_crc;
            data_start = 5;
        } else {
            _pay_len = impl_pay_len;
            _cr      = impl_cr;
            _has_crc = impl_has_crc;
            data_start = 0;
        }

        // --- Dewhitening ---
        std::vector<uint8_t> decoded_bytes;
        uint32_t total_data_nibs = _pay_len * 2 + (_has_crc ? 4 : 0);

        for (uint32_t i = 0; i < total_data_nibs / 2; i++) {
            std::size_t nib_idx = data_start + 2 * i;
            if (nib_idx + 1 >= _nibbles.size()) break;

            uint8_t low_nib  = _nibbles[nib_idx];
            uint8_t high_nib = _nibbles[nib_idx + 1];

            if (i < _pay_len) {
                uint8_t ws = whitening_seq[i % whitening_seq.size()];
                low_nib  ^= (ws & 0x0F);
                high_nib ^= ((ws >> 4) & 0x0F);
            }
            decoded_bytes.push_back(static_cast<uint8_t>((high_nib << 4) | low_nib));
        }

        // --- CRC verification ---
        bool crc_valid = true;
        if (_has_crc && _pay_len >= 2
            && decoded_bytes.size() >= _pay_len + 2u) {
            crc_valid = lora_verify_crc(
                std::span<const uint8_t>(decoded_bytes.data(), _pay_len),
                decoded_bytes[_pay_len],
                decoded_bytes[_pay_len + 1]);
        }

        // --- Output payload bytes ---
        for (uint32_t i = 0; i < _pay_len && out_idx < out_span.size(); i++) {
            if (i < decoded_bytes.size()) {
                out_span[out_idx++] = decoded_bytes[i];
            }
        }

        // --- Publish tag ---
        gr::property_map out_tag;
        out_tag["pay_len"]       = pmtv::pmt(static_cast<int64_t>(_pay_len));
        out_tag["cr"]            = pmtv::pmt(static_cast<int64_t>(_cr));
        out_tag["crc_valid"]     = pmtv::pmt(crc_valid);
        out_tag["is_downchirp"]  = pmtv::pmt(_is_downchirp);
        this->publishTag(out_tag, 0UZ);

        // --- Publish message ---
        std::string payload_str;
        for (uint32_t i = 0; i < _pay_len && i < decoded_bytes.size(); i++) {
            payload_str.push_back(static_cast<char>(decoded_bytes[i]));
        }
        gr::property_map msg_data;
        msg_data["payload"]       = pmtv::pmt(payload_str);
        msg_data["crc_valid"]     = pmtv::pmt(crc_valid);
        msg_data["is_downchirp"]  = pmtv::pmt(_is_downchirp);
        gr::sendMessage<gr::message::Command::Notify>(msg_out, "", "payload", msg_data);
    }

    [[nodiscard]] gr::work::Status processBulk(
            gr::InputSpanLike auto& input,
            gr::OutputSpanLike auto& output) noexcept {
        auto in_span  = std::span(input);
        auto out_span = std::span(output);

        if (_N == 0) recalculate();

        // Only check for burst_start when IDLE: tags are re-delivered when
        // processBulk produces 0 output, so checking in other states would
        // incorrectly reset the state machine mid-frame.
        if (_state == IDLE && this->inputTagsPresent()) {
            const auto& tag = this->mergedInputTag();
            if (auto it = tag.map.find("burst_start"); it != tag.map.end()) {
                if (pmtv::cast<bool>(it->second)) {
                    // New burst — reset and start header accumulation
                    if (auto sf_it = tag.map.find("sf"); sf_it != tag.map.end()) {
                        auto new_sf = static_cast<uint8_t>(pmtv::cast<int64_t>(sf_it->second));
                        if (new_sf != sf) {
                            sf = new_sf;
                            recalculate();
                        }
                    }
                    if (auto it2 = tag.map.find("cfo_int"); it2 != tag.map.end()) {
                        _cfo_int = static_cast<int>(pmtv::cast<int64_t>(it2->second));
                    }
                    if (auto it2 = tag.map.find("cfo_frac"); it2 != tag.map.end()) {
                        _cfo_frac = static_cast<float>(pmtv::cast<double>(it2->second));
                    }
                    if (auto it2 = tag.map.find("is_downchirp"); it2 != tag.map.end()) {
                        _is_downchirp = pmtv::cast<bool>(it2->second);
                    } else {
                        _is_downchirp = false;
                    }

                    buildDownchirpWithCFO();

                    // Determine LDRO mode
                    if (ldro_mode == 2) {  // AUTO
                        _ldro = (static_cast<float>(1u << sf) * 1e3f
                                / static_cast<float>(bandwidth)) > LDRO_MAX_DURATION_MS;
                    } else {
                        _ldro = (ldro_mode != 0);
                    }

                    _state = HEADER;
                    _symbol_buffer.clear();
                    _nibbles.clear();
                    _total_symbols_rx = 0;
                    _symb_numb = 0;
                    _cr = 4;  // header always uses CR=4
                }
            }
        }

        if (_state == IDLE) {
            // Consume all input to prevent scheduler deadlock.
            std::ignore = input.consume(in_span.size());
            output.publish(0UZ);
            return gr::work::Status::OK;
        }
        if (in_span.size() < _N) {
            // Not enough for a full symbol -- consume to allow EOS
            std::ignore = input.consume(in_span.size());
            output.publish(0UZ);
            return gr::work::Status::OK;
        }

        std::size_t in_idx  = 0;
        std::size_t out_idx = 0;

        while (in_idx + _N <= in_span.size()) {
            // Demodulate one symbol (raw argmax-1, no divide-by-4)
            uint16_t symbol = demodSymbol(&in_span[in_idx]);
            _symbol_buffer.push_back(symbol);
            _total_symbols_rx++;
            in_idx += _N;

            if (_state == HEADER) {
                // Header block: 8 symbols, sf_app = sf-2, cw_len = 8, cr_app = 4
                if (_symbol_buffer.size() == 8) {
                    auto header_nibbles = processBlock(
                        _symbol_buffer,
                        static_cast<uint8_t>(sf - 2),  // sf_app
                        8,                              // cw_len
                        4,                              // cr_app (always 4 for header)
                        true);
                    _nibbles.insert(_nibbles.end(), header_nibbles.begin(), header_nibbles.end());
                    _symbol_buffer.clear();

                    // Parse header to determine payload parameters
                    if (!impl_head && _nibbles.size() >= 5) {
                        auto info = parse_explicit_header(
                            _nibbles[0], _nibbles[1], _nibbles[2], _nibbles[3], _nibbles[4]);

                        if (!info.checksum_valid || info.payload_len == 0) {
                            _state = IDLE;
                            continue;
                        }

                        _pay_len = info.payload_len;
                        _cr      = info.cr;
                        _has_crc = info.has_crc;

                        // Calculate total payload symbols
                        _symb_numb = 8 + static_cast<uint32_t>(
                            std::ceil(static_cast<double>(
                                2 * _pay_len - sf + 2 + 5 + (_has_crc ? 4 : 0))
                                / (sf - 2 * static_cast<int>(_ldro))))
                            * (4 + _cr);
                    }

                    _state = PAYLOAD;
                }
            } else if (_state == PAYLOAD) {
                // Payload blocks: cw_len = 4+cr symbols each
                uint8_t cw_len = 4 + _cr;
                uint8_t sf_app = _ldro ? static_cast<uint8_t>(sf - 2) : sf;

                if (_symbol_buffer.size() == cw_len) {
                    auto payload_nibbles = processBlock(
                        _symbol_buffer,
                        sf_app,
                        cw_len,
                        _cr,
                        false);
                    _nibbles.insert(_nibbles.end(),
                                    payload_nibbles.begin(), payload_nibbles.end());
                    _symbol_buffer.clear();
                }

                // Check if we have all symbols
                if (_symb_numb > 0 && _total_symbols_rx >= _symb_numb) {
                    // Process any remaining symbols in buffer
                    if (!_symbol_buffer.empty()) {
                        uint8_t remaining_cw_len = 4 + _cr;
                        uint8_t remaining_sf_app = _ldro ? static_cast<uint8_t>(sf - 2) : sf;
                        // Pad with zeros if needed
                        while (_symbol_buffer.size() < remaining_cw_len) {
                            _symbol_buffer.push_back(0);
                        }
                        auto last_nibbles = processBlock(
                            _symbol_buffer,
                            remaining_sf_app,
                            remaining_cw_len,
                            _cr,
                            false);
                        _nibbles.insert(_nibbles.end(),
                                        last_nibbles.begin(), last_nibbles.end());
                        _symbol_buffer.clear();
                    }

                    // Decode the complete frame
                    decodeFrame(out_span, out_idx);

                    _state = IDLE;
                    break;  // stop consuming more symbols for this frame
                }
            }
        }

        std::ignore = input.consume(in_idx);
        output.publish(out_idx);
        return gr::work::Status::OK;
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_SYMBOL_DEMODULATOR_HPP
