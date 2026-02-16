// SPDX-License-Identifier: ISC
/// Microbenchmarks for individual LoRa PHY algorithm steps.
///
/// Measures the hot-path functions that dominate encode/decode time.
/// Run with: ./bm_lora_algorithms
/// Extra precision: BM_DIGITS=3 ./bm_lora_algorithms

#include <benchmark.hpp>

#include <complex>
#include <cstdint>
#include <numbers>
#include <vector>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/algorithm/crc.hpp>
#include <gnuradio-4.0/lora/algorithm/hamming.hpp>
#include <gnuradio-4.0/lora/algorithm/interleaving.hpp>
#include <gnuradio-4.0/lora/algorithm/tables.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>
#include <gnuradio-4.0/lora/algorithm/utilities.hpp>

namespace {

// MeshCore LoRa configuration
constexpr uint8_t  SF           = 8;
constexpr uint32_t N            = 1u << SF;  // 256
constexpr uint8_t  CR           = 4;
constexpr uint16_t SYNC_WORD    = 0x12;
constexpr uint16_t PREAMBLE_LEN = 8;
constexpr uint8_t  OS_FACTOR    = 1;  // no oversampling for algorithm benchmarks

// Test payload: "Hello MeshCore" (14 bytes, typical ADVERT-class size)
const std::vector<uint8_t> PAYLOAD = {
    'H', 'e', 'l', 'l', 'o', ' ', 'M', 'e', 's', 'h', 'C', 'o', 'r', 'e'};

// Volatile sink to prevent dead-code elimination without inline asm issues.
volatile std::size_t g_sink = 0;

/// Generate a dechirped symbol (simulates the multiply step of dechirp).
std::vector<std::complex<float>> make_dechirped_symbol(uint16_t sym_val) {
    std::vector<std::complex<float>> upchirp(N), downchirp(N);
    gr::lora::build_ref_chirps(upchirp.data(), downchirp.data(), SF, 1);

    std::vector<std::complex<float>> chirp(N);
    gr::lora::build_upchirp(chirp.data(), sym_val, SF, 1);

    std::vector<std::complex<float>> dechirped(N);
    for (uint32_t i = 0; i < N; i++) {
        dechirped[i] = chirp[i] * downchirp[i];
    }
    return dechirped;
}

}  // namespace

// ============================================================================
// RX hot-path benchmarks
// ============================================================================

const boost::ut::suite rx_benchmarks = [] {
    using namespace benchmark;

    // --- FFT: the single most expensive per-symbol operation ---
    {
        gr::algorithm::FFT<std::complex<float>> fft;
        auto dechirped = make_dechirped_symbol(42);
        auto warmup    = fft.compute(dechirped);  // warm up twiddle cache

        constexpr int REPS = 10'000;
        ::benchmark::benchmark<REPS>(std::string_view("FFT (dechirp+argmax) N=256"), N) =
            [&fft, &dechirped] {
                auto result = fft.compute(dechirped);
                g_sink = result.size();
            };
    }

    ::benchmark::results::add_separator();

    // --- Dechirp: element-wise complex multiply ---
    {
        std::vector<std::complex<float>> upchirp(N), downchirp(N);
        gr::lora::build_ref_chirps(upchirp.data(), downchirp.data(), SF, 1);

        std::vector<std::complex<float>> chirp(N);
        gr::lora::build_upchirp(chirp.data(), 42, SF, 1);

        std::vector<std::complex<float>> out(N);

        constexpr int REPS = 50'000;
        ::benchmark::benchmark<REPS>(std::string_view("dechirp (complex mul) N=256"), N) =
            [&out, &chirp, &downchirp] {
                for (uint32_t i = 0; i < N; i++) {
                    out[i] = chirp[i] * downchirp[i];
                }
                g_sink = static_cast<std::size_t>(out[0].real());
            };
    }

    // --- Gray mapping: trivial per-symbol ---
    {
        constexpr int REPS = 100'000;
        uint16_t sym = 142;
        ::benchmark::benchmark<REPS>(std::string_view("gray_map (x^(x>>1))"), 1) =
            [&sym] {
                uint16_t g = static_cast<uint16_t>(sym ^ (sym >> 1));
                g_sink = g;
            };
    }

    ::benchmark::results::add_separator();

    // --- Deinterleave: one block (header block: 8 symbols, sf_app=6) ---
    {
        auto whitened    = gr::lora::whiten(PAYLOAD);
        auto with_header = gr::lora::insert_header(whitened,
                                                    static_cast<uint8_t>(PAYLOAD.size()),
                                                    CR, true);
        auto with_crc    = gr::lora::add_crc(with_header, PAYLOAD, true);
        auto encoded     = gr::lora::hamming_enc_frame(with_crc, SF, CR);
        auto interleaved = gr::lora::interleave_frame(encoded, SF, CR);

        // Take first 8 symbols (header block)
        std::vector<uint16_t> header_syms;
        for (int i = 0; i < 8; i++) {
            header_syms.push_back(static_cast<uint16_t>(interleaved[static_cast<std::size_t>(i)]));
        }

        // Take next cw_len symbols for payload block
        std::vector<uint16_t> payload_syms;
        uint8_t cw_len = 4 + CR;
        for (std::size_t i = 8; i < 8 + cw_len && i < interleaved.size(); i++) {
            payload_syms.push_back(static_cast<uint16_t>(interleaved[i]));
        }
        while (payload_syms.size() < cw_len) payload_syms.push_back(0);

        constexpr int REPS = 10'000;
        ::benchmark::benchmark<REPS>(std::string_view("deinterleave (header, 8 sym)"), 8) =
            [&header_syms] {
                auto cw = gr::lora::deinterleave_block(header_syms, SF, 8, SF - 2);
                g_sink = cw.size();
            };

        ::benchmark::benchmark<REPS>(std::string_view("deinterleave (payload, 8 sym)"), cw_len) =
            [&payload_syms, cw_len] {
                auto cw = gr::lora::deinterleave_block(payload_syms, SF, cw_len, SF);
                g_sink = cw.size();
            };
    }

    ::benchmark::results::add_separator();

    // --- Hamming decode (hard): per codeword ---
    {
        uint8_t cw = gr::lora::hamming_encode(0x0A, CR);

        constexpr int REPS = 100'000;
        ::benchmark::benchmark<REPS>(std::string_view("hamming_decode_hard (cr=4)"), 1) =
            [cw] {
                auto nib = gr::lora::hamming_decode_hard(cw, CR);
                g_sink = nib;
            };
    }

    // --- Hamming encode: per nibble ---
    {
        constexpr int REPS = 100'000;
        ::benchmark::benchmark<REPS>(std::string_view("hamming_encode (cr=4)"), 1) =
            [] {
                auto cw = gr::lora::hamming_encode(0x0A, CR);
                g_sink = cw;
            };
    }

    ::benchmark::results::add_separator();

    // --- CRC-16/XMODEM ---
    {
        constexpr int REPS = 50'000;
        ::benchmark::benchmark<REPS>(std::string_view("crc16_xmodem (14 bytes)"), PAYLOAD.size()) =
            [] {
                auto crc = gr::lora::crc16(PAYLOAD);
                g_sink = crc;
            };
    }

    // --- Header checksum ---
    {
        constexpr int REPS = 100'000;
        ::benchmark::benchmark<REPS>(std::string_view("header_checksum"), 1) =
            [] {
                auto ck = gr::lora::header_checksum(
                    static_cast<uint8_t>(PAYLOAD.size()), CR, true);
                g_sink = ck;
            };
    }

    // --- Dewhitening (14 bytes -> 28 nibble XORs) ---
    {
        constexpr int REPS = 50'000;
        ::benchmark::benchmark<REPS>(std::string_view("whiten/dewhiten (14 bytes)"), PAYLOAD.size()) =
            [] {
                auto result = gr::lora::whiten(PAYLOAD);
                g_sink = result.size();
            };
    }

    ::benchmark::results::add_separator();

    // --- Build upchirp: chirp generation ---
    {
        std::vector<std::complex<float>> chirp(N);

        constexpr int REPS = 10'000;
        ::benchmark::benchmark<REPS>(std::string_view("build_upchirp N=256"), N) =
            [&chirp] {
                gr::lora::build_upchirp(chirp.data(), 42, SF, 1);
                g_sink = static_cast<std::size_t>(chirp[0].real());
            };
    }

    // --- Interleave: one frame ---
    {
        auto whitened    = gr::lora::whiten(PAYLOAD);
        auto with_header = gr::lora::insert_header(whitened,
                                                    static_cast<uint8_t>(PAYLOAD.size()),
                                                    CR, true);
        auto with_crc    = gr::lora::add_crc(with_header, PAYLOAD, true);
        auto encoded     = gr::lora::hamming_enc_frame(with_crc, SF, CR);

        constexpr int REPS = 5'000;
        ::benchmark::benchmark<REPS>(std::string_view("interleave_frame (37 cw)"), encoded.size()) =
            [&encoded] {
                auto syms = gr::lora::interleave_frame(encoded, SF, CR);
                g_sink = syms.size();
            };
    }
};

// ============================================================================
// TX chain benchmarks
// ============================================================================

const boost::ut::suite tx_benchmarks = [] {
    using namespace benchmark;

    ::benchmark::results::add_separator();

    // --- Full TX chain: payload -> IQ ---
    {
        constexpr int REPS = 1'000;
        ::benchmark::benchmark<REPS>(std::string_view("generate_frame_iq (14B, os=1)"), PAYLOAD.size()) =
            [] {
                auto iq = gr::lora::generate_frame_iq(
                    PAYLOAD, SF, CR, OS_FACTOR, SYNC_WORD, PREAMBLE_LEN, true, 0);
                g_sink = iq.size();
            };
    }

    // --- Full TX chain with 4x oversampling (hardware rate) ---
    {
        constexpr int REPS = 500;
        ::benchmark::benchmark<REPS>(std::string_view("generate_frame_iq (14B, os=4)"), PAYLOAD.size()) =
            [] {
                auto iq = gr::lora::generate_frame_iq(
                    PAYLOAD, SF, CR, 4, SYNC_WORD, PREAMBLE_LEN, true, 0);
                g_sink = iq.size();
            };
    }
};

int main() { /* boost::ut runs suites automatically */ }
