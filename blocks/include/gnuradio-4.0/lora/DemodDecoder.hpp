// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_DEMOD_DECODER_HPP
#define GNURADIO_LORA_DEMOD_DECODER_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/algorithm/crc.hpp>
#include <gnuradio-4.0/lora/algorithm/hamming.hpp>
#include <gnuradio-4.0/lora/algorithm/interleaving.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>
#include <gnuradio-4.0/lora/algorithm/utilities.hpp>
#include <gnuradio-4.0/lora/log.hpp>

namespace gr::lora {

/// DemodDecoder: aligned IQ symbol blocks -> decoded payload bytes.
///
/// Full LoRa RX decode pipeline in a single block:
///   dechirp+FFT -> gray demap -> deinterleave -> Hamming decode
///   -> header parse -> dewhiten -> CRC verify
///
/// Interface contract:
///   Input:  cf32 stream, N (= 2^sf) samples per symbol, from FrameSync.
///   Output: uint8 decoded payload bytes.
///   msg_out: publishes {payload, crc_valid, is_downchirp} per decoded frame.
///
/// Tag protocol:
///   Reads `burst_start` tag with {sf, cfo_int, cfo_frac, is_downchirp,
///   snr_db, rx_channel}. Publishes output tag with {pay_len, cr,
///   crc_valid, is_downchirp, snr_db, rx_channel} at the first output byte.
///
/// Rate: variable (N*num_symbols cf32 -> pay_len uint8), manual consume/publish.
GR_REGISTER_BLOCK("gr::lora::DemodDecoder", gr::lora::DemodDecoder)
struct DemodDecoder : gr::Block<DemodDecoder, gr::NoDefaultTagForwarding> {
    gr::PortIn<std::complex<float>>  in;
    gr::PortOut<uint8_t>             out;
    gr::MsgPortOut                   msg_out;

    uint8_t  sf           = 8;
    uint32_t bandwidth    = 62500;
    bool     impl_head    = false;
    uint8_t  impl_cr      = 4;
    uint8_t  impl_pay_len = 0;
    bool     impl_has_crc = true;
    uint8_t  ldro_mode    = 2;  // 0=off, 1=on, 2=auto
    bool     debug        = false;

    GR_MAKE_REFLECTABLE(DemodDecoder, in, out, msg_out,
                        sf, bandwidth, impl_head, impl_cr, impl_pay_len, impl_has_crc, ldro_mode, debug);

    enum State : uint8_t { IDLE, HEADER, PAYLOAD };

    State    _state = IDLE;
    bool     _header_failed = false;  // suppress tag re-delivery after header checksum fail
    uint32_t _N     = 0;

    int      _cfo_int  = 0;
    float    _cfo_frac = 0.f;
    bool     _is_downchirp = false;
    double   _snr_db = 0.0;
    double   _noise_floor_db = -999.0;
    double   _peak_db = -999.0;
    int64_t  _rx_channel = -1;

    std::vector<std::complex<float>> _downchirp;
    std::vector<std::complex<float>> _dechirped;

    std::vector<uint16_t> _symbol_buffer;
    uint32_t _total_symbols_rx = 0;

    uint8_t  _cr       = 4;
    uint32_t _pay_len  = 0;
    bool     _has_crc  = true;
    bool     _ldro     = false;
    uint32_t _symb_numb = 0;

    std::vector<uint8_t> _nibbles;

    gr::algorithm::FFT<std::complex<float>> _fft;

    void start() { recalculate(); }

    void settingsChanged(const gr::property_map& /*oldSettings*/,
                         const gr::property_map& newSettings) {
        if (_N > 0 && (newSettings.contains("sf") || newSettings.contains("bandwidth"))) {
            recalculate();
        }
    }

    void recalculate() {
        _N = 1u << sf;
        _downchirp.resize(_N);
        _dechirped.resize(_N);

        std::vector<std::complex<float>> upchirp(_N);
        build_ref_chirps(upchirp.data(), _downchirp.data(), sf, 1);

        _fft = gr::algorithm::FFT<std::complex<float>>{};
        _state = IDLE;
    }

    void buildDownchirpWithCFO() {
        std::vector<std::complex<float>> upchirp(_N);
        build_upchirp(upchirp.data(),
                      static_cast<uint32_t>(mod(_cfo_int, static_cast<int64_t>(_N))),
                      sf, 1);
        for (uint32_t n = 0; n < _N; n++) {
            _downchirp[n] = std::conj(upchirp[n]);
        }
        for (uint32_t n = 0; n < _N; n++) {
            float phase = -2.f * static_cast<float>(std::numbers::pi) * _cfo_frac
                        / static_cast<float>(_N) * static_cast<float>(n);
            _downchirp[n] *= std::complex<float>(std::cos(phase), std::sin(phase));
        }
    }

    [[nodiscard]] uint16_t demodSymbol(const std::complex<float>* samples) {
        auto max_idx = dechirp_argmax(samples, _downchirp.data(),
                                      _dechirped.data(), _N, _fft);
        return static_cast<uint16_t>(
            mod(static_cast<int64_t>(max_idx) - 1, static_cast<int64_t>(_N)));
    }

    [[nodiscard]] static uint16_t grayMap(uint16_t symbol) {
        return static_cast<uint16_t>(symbol ^ (symbol >> 1));
    }

    [[nodiscard]] std::vector<uint8_t> processBlock(
            const std::vector<uint16_t>& symbols,
            uint8_t sf_app, uint8_t cw_len, uint8_t cr_app,
            bool is_header_block) {
        std::vector<uint16_t> gray_syms;
        gray_syms.reserve(symbols.size());
        for (auto s : symbols) {
            gray_syms.push_back(grayMap(s));
        }

        if (sf_app != sf) {
            for (auto& s : gray_syms) {
                s >>= (sf - sf_app);
            }
        }

        auto codewords = deinterleave_block(gray_syms, sf, cw_len, sf_app);

        std::vector<uint8_t> nibbles;
        nibbles.reserve(codewords.size());
        for (auto cw : codewords) {
            nibbles.push_back(hamming_decode_hard(cw, cr_app));
        }

        return nibbles;
    }

    void decodeFrame(std::span<uint8_t> out_span, std::size_t& out_idx) {
        if (_nibbles.size() < 5 && !impl_head) return;

        std::size_t data_start = 0;
        if (!impl_head) {
            auto info = parse_explicit_header(
                _nibbles[0], _nibbles[1], _nibbles[2], _nibbles[3], _nibbles[4]);

            if (!info.checksum_valid || info.payload_len == 0) {
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

        uint32_t total_data_nibs = _pay_len * 2 + (_has_crc ? 4 : 0);
        std::size_t avail_nibs = (_nibbles.size() > data_start)
                               ? _nibbles.size() - data_start : 0;
        std::size_t use_nibs = std::min(static_cast<std::size_t>(total_data_nibs), avail_nibs);
        use_nibs &= ~std::size_t{1};
        auto decoded_bytes = dewhiten(
            std::span<const uint8_t>(&_nibbles[data_start], use_nibs), _pay_len);

        bool crc_valid = true;
        if (_has_crc && _pay_len >= 2
            && decoded_bytes.size() >= _pay_len + 2u) {
            crc_valid = lora_verify_crc(
                std::span<const uint8_t>(decoded_bytes.data(), _pay_len),
                decoded_bytes[_pay_len],
                decoded_bytes[_pay_len + 1]);
        }

        if (debug) {
            log_ts("debug", "DemodDecoder", "frame: pay_len=%u cr=%u crc=%s nibbles=%zu",
                   _pay_len, _cr, crc_valid ? "OK" : "FAIL", _nibbles.size());
        }

        for (uint32_t i = 0; i < _pay_len && out_idx < out_span.size(); i++) {
            if (i < decoded_bytes.size()) {
                out_span[out_idx++] = decoded_bytes[i];
            }
        }

        gr::property_map out_tag;
        out_tag["pay_len"]       = gr::pmt::Value(static_cast<int64_t>(_pay_len));
        out_tag["cr"]            = gr::pmt::Value(static_cast<int64_t>(_cr));
        out_tag["crc_valid"]     = gr::pmt::Value(crc_valid);
        out_tag["is_downchirp"]  = gr::pmt::Value(_is_downchirp);
        out_tag["snr_db"]        = gr::pmt::Value(_snr_db);
        if (_noise_floor_db > -999.0) {
            out_tag["noise_floor_db"] = gr::pmt::Value(_noise_floor_db);
        }
        if (_peak_db > -999.0) {
            out_tag["peak_db"] = gr::pmt::Value(_peak_db);
        }
        if (_rx_channel >= 0) {
            out_tag["rx_channel"] = gr::pmt::Value(_rx_channel);
        }
        this->publishTag(out_tag, 0UZ);

        std::string payload_str;
        for (uint32_t i = 0; i < _pay_len && i < decoded_bytes.size(); i++) {
            payload_str.push_back(static_cast<char>(decoded_bytes[i]));
        }
        gr::property_map msg_data;
        msg_data["payload"]       = gr::pmt::Value(payload_str);
        msg_data["crc_valid"]     = gr::pmt::Value(crc_valid);
        msg_data["is_downchirp"]  = gr::pmt::Value(_is_downchirp);
        gr::sendMessage<gr::message::Command::Notify>(msg_out, "", "payload", msg_data);
    }

    [[nodiscard]] gr::work::Status processBulk(
            gr::InputSpanLike auto& input,
            gr::OutputSpanLike auto& output) {
        auto in_span  = std::span(input);
        auto out_span = std::span(output);

        if (_N == 0) recalculate();

        // Reset header-fail suppression when no tag is present (burst is over,
        // next burst_start tag will be a genuinely new burst).
        if (!this->inputTagsPresent()) {
            _header_failed = false;
        }

        // Only check for burst_start when IDLE: tags are re-delivered when
        // processBulk produces 0 output, so checking in other states would
        // incorrectly reset the state machine mid-frame.
        if (_state == IDLE && this->inputTagsPresent()) {
            const auto& tag = this->mergedInputTag();
            if (auto it = tag.map.find("burst_start"); it != tag.map.end()) {
                if (it->second.value_or<bool>(false) && !_header_failed) {
                    if (auto sf_it = tag.map.find("sf"); sf_it != tag.map.end()) {
                        auto new_sf = static_cast<uint8_t>(sf_it->second.value_or<int64_t>(sf));
                        if (new_sf != sf) {
                            sf = new_sf;
                            recalculate();
                        }
                    }
                    if (auto it2 = tag.map.find("cfo_int"); it2 != tag.map.end()) {
                        _cfo_int = static_cast<int>(it2->second.value_or<int64_t>(0));
                    }
                    if (auto it2 = tag.map.find("cfo_frac"); it2 != tag.map.end()) {
                        _cfo_frac = static_cast<float>(it2->second.value_or<double>(0.0));
                    }
                    if (auto it2 = tag.map.find("is_downchirp"); it2 != tag.map.end()) {
                        _is_downchirp = it2->second.value_or<bool>(false);
                    } else {
                        _is_downchirp = false;
                    }
                    if (auto it2 = tag.map.find("snr_db"); it2 != tag.map.end()) {
                        _snr_db = it2->second.value_or<double>(0.0);
                    } else {
                        _snr_db = 0.0;
                    }
                    if (auto it2 = tag.map.find("noise_floor_db"); it2 != tag.map.end()) {
                        _noise_floor_db = it2->second.value_or<double>(-999.0);
                    } else {
                        _noise_floor_db = -999.0;
                    }
                    if (auto it2 = tag.map.find("peak_db"); it2 != tag.map.end()) {
                        _peak_db = it2->second.value_or<double>(-999.0);
                    } else {
                        _peak_db = -999.0;
                    }
                    if (auto it2 = tag.map.find("rx_channel"); it2 != tag.map.end()) {
                        _rx_channel = it2->second.value_or<int64_t>(-1);
                    } else {
                        _rx_channel = -1;
                    }

                    if (debug) {
                        log_ts("info ", "DemodDecoder",
                               "IDLE->HEADER: sf=%u cfo_int=%d cfo_frac=%.3f snr=%.1f dB noise=%.1f dBFS%s",
                               sf, _cfo_int, static_cast<double>(_cfo_frac),
                               static_cast<double>(_snr_db),
                               _noise_floor_db,
                               _is_downchirp ? " (downchirp)" : "");
                    }

                    buildDownchirpWithCFO();

                    if (ldro_mode == 2) {
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
                    _cr = 4;
                }
            }
        }

        if (_state == IDLE) {
            std::ignore = input.consume(in_span.size());
            output.publish(0UZ);
            return gr::work::Status::OK;
        }
        if (in_span.size() < _N) {
            std::ignore = input.consume(in_span.size());
            output.publish(0UZ);
            return gr::work::Status::OK;
        }

        std::size_t in_idx  = 0;
        std::size_t out_idx = 0;

        while (in_idx + _N <= in_span.size()) {
            uint16_t symbol = demodSymbol(&in_span[in_idx]);
            _symbol_buffer.push_back(symbol);
            _total_symbols_rx++;
            in_idx += _N;

            if (_state == HEADER) {
                if (_symbol_buffer.size() == 8) {
                    auto header_nibbles = processBlock(
                        _symbol_buffer,
                        static_cast<uint8_t>(sf - 2),
                        8, 4, true);
                    _nibbles.insert(_nibbles.end(), header_nibbles.begin(), header_nibbles.end());
                    _symbol_buffer.clear();

                    if (!impl_head && _nibbles.size() >= 5) {
                        auto info = parse_explicit_header(
                            _nibbles[0], _nibbles[1], _nibbles[2], _nibbles[3], _nibbles[4]);

                        if (debug) {
                            log_ts("debug", "DemodDecoder", "header: pay_len=%u cr=%u has_crc=%d checksum=%s",
                                   info.payload_len, info.cr, info.has_crc,
                                   info.checksum_valid ? "OK" : "FAIL");
                        }

                        if (!info.checksum_valid || info.payload_len == 0) {
                            _state = IDLE;
                            _header_failed = true;
                            break;
                        }

                        _pay_len = info.payload_len;
                        _cr      = info.cr;
                        _has_crc = info.has_crc;

                        _symb_numb = 8 + static_cast<uint32_t>(
                            std::ceil(static_cast<double>(
                                2 * _pay_len - sf + 2 + 5 + (_has_crc ? 4 : 0))
                                / (sf - 2 * static_cast<int>(_ldro))))
                            * (4 + _cr);
                    }

                    _state = PAYLOAD;
                }
            } else if (_state == PAYLOAD) {
                uint8_t cw_len = 4 + _cr;
                uint8_t sf_app = _ldro ? static_cast<uint8_t>(sf - 2) : sf;

                if (_symbol_buffer.size() == cw_len) {
                    auto payload_nibbles = processBlock(
                        _symbol_buffer, sf_app, cw_len, _cr, false);
                    _nibbles.insert(_nibbles.end(),
                                    payload_nibbles.begin(), payload_nibbles.end());
                    _symbol_buffer.clear();
                }

                if (_symb_numb > 0 && _total_symbols_rx >= _symb_numb) {
                    if (!_symbol_buffer.empty()) {
                        uint8_t remaining_cw_len = 4 + _cr;
                        uint8_t remaining_sf_app = _ldro ? static_cast<uint8_t>(sf - 2) : sf;
                        while (_symbol_buffer.size() < remaining_cw_len) {
                            _symbol_buffer.push_back(0);
                        }
                        auto last_nibbles = processBlock(
                            _symbol_buffer, remaining_sf_app, remaining_cw_len, _cr, false);
                        _nibbles.insert(_nibbles.end(),
                                        last_nibbles.begin(), last_nibbles.end());
                        _symbol_buffer.clear();
                    }

                    decodeFrame(out_span, out_idx);

                    _state = IDLE;
                    break;
                }
            }
        }

        std::ignore = input.consume(in_idx);
        output.publish(out_idx);
        return gr::work::Status::OK;
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_DEMOD_DECODER_HPP
