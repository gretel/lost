// SPDX-License-Identifier: ISC
/// End-to-end LoRa decode chain throughput benchmark.
///
/// Generates frames via TX chain, then measures RX decode throughput
/// at the algorithm level (no GR4 graph overhead).
///
/// Reports: frames/second, symbols/second, real-time margin.
/// Run with: ./bm_lora_decode

#include <benchmark.hpp>

#include <chrono>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <string>
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
constexpr uint32_t BW           = 62500;
constexpr uint16_t SYNC_WORD    = 0x12;
constexpr uint16_t PREAMBLE_LEN = 8;

// Test payload: "Hello MeshCore" (14 bytes)
const std::vector<uint8_t> PAYLOAD = {
    'H', 'e', 'l', 'l', 'o', ' ', 'M', 'e', 's', 'h', 'C', 'o', 'r', 'e'};

// Volatile sink to prevent dead-code elimination without inline asm issues.
volatile std::size_t g_sink = 0;

/// Standalone RX decode of a frame's payload symbols (algorithm-level).
///
/// This mirrors SymbolDemodulator's internal pipeline but without the
/// GR4 block machinery, measuring pure decode throughput.
struct AlgorithmRxDecoder {
    gr::algorithm::FFT<std::complex<float>> fft;
    std::vector<std::complex<float>> downchirp;
    std::vector<std::complex<float>> dechirped;

    AlgorithmRxDecoder() : downchirp(N), dechirped(N) {
        std::vector<std::complex<float>> upchirp(N);
        gr::lora::build_ref_chirps(upchirp.data(), downchirp.data(), SF, 1);
    }

    /// Dechirp + FFT + argmax for one symbol.
    [[nodiscard]] uint16_t demodSymbol(const std::complex<float>* samples) {
        for (uint32_t i = 0; i < N; i++) {
            dechirped[i] = samples[i] * downchirp[i];
        }
        auto fft_out = fft.compute(dechirped);

        float max_val = 0.f;
        uint32_t max_idx = 0;
        for (uint32_t i = 0; i < N; i++) {
            float mag_sq = fft_out[i].real() * fft_out[i].real()
                         + fft_out[i].imag() * fft_out[i].imag();
            if (mag_sq > max_val) {
                max_val = mag_sq;
                max_idx = i;
            }
        }
        return static_cast<uint16_t>(
            gr::lora::mod(static_cast<int64_t>(max_idx) - 1, static_cast<int64_t>(N)));
    }

    /// Gray demapping.
    [[nodiscard]] static uint16_t grayMap(uint16_t s) {
        return static_cast<uint16_t>(s ^ (s >> 1));
    }

    /// Decode a complete frame from payload IQ symbols (after preamble/SFD).
    /// Returns decoded payload bytes.
    [[nodiscard]] std::vector<uint8_t> decodePayloadSymbols(
            const std::complex<float>* payload_iq,
            std::size_t n_symbols) {
        // Step 1: Demodulate all symbols
        std::vector<uint16_t> raw_symbols;
        raw_symbols.reserve(n_symbols);
        for (std::size_t s = 0; s < n_symbols; s++) {
            raw_symbols.push_back(demodSymbol(&payload_iq[s * N]));
        }

        // Step 2: Header block (first 8 symbols)
        std::vector<uint8_t> all_nibbles;

        // Header: 8 symbols, sf_app=SF-2, cw_len=8, cr_app=4
        {
            std::vector<uint16_t> hdr_syms(raw_symbols.begin(),
                                            raw_symbols.begin() + 8);
            // Gray map + divide by 4
            for (auto& s : hdr_syms) {
                s = static_cast<uint16_t>(grayMap(s) >> 2);
            }
            auto codewords = gr::lora::deinterleave_block(hdr_syms, SF, 8, SF - 2);
            for (auto cw : codewords) {
                all_nibbles.push_back(gr::lora::hamming_decode_hard(cw, 4));
            }
        }

        // Parse header
        if (all_nibbles.size() < 5) return {};
        auto hdr = gr::lora::parse_explicit_header(
            all_nibbles[0], all_nibbles[1], all_nibbles[2],
            all_nibbles[3], all_nibbles[4]);
        if (!hdr.checksum_valid || hdr.payload_len == 0) return {};

        // Step 3: Payload blocks
        uint8_t cw_len = 4 + hdr.cr;
        std::size_t sym_idx = 8;
        while (sym_idx + cw_len <= n_symbols) {
            std::vector<uint16_t> blk_syms;
            for (uint8_t j = 0; j < cw_len; j++) {
                uint16_t s = grayMap(raw_symbols[sym_idx + j]);
                blk_syms.push_back(s);
            }
            auto codewords = gr::lora::deinterleave_block(blk_syms, SF, cw_len, SF);
            for (auto cw : codewords) {
                all_nibbles.push_back(gr::lora::hamming_decode_hard(cw, hdr.cr));
            }
            sym_idx += cw_len;
        }

        // Step 4: Dewhitening + CRC
        std::vector<uint8_t> decoded_bytes;
        uint32_t total_data_nibs = hdr.payload_len * 2 + (hdr.has_crc ? 4 : 0);
        std::size_t data_start = 5;  // skip header nibbles

        for (uint32_t i = 0; i < total_data_nibs / 2; i++) {
            std::size_t nib_idx = data_start + 2 * i;
            if (nib_idx + 1 >= all_nibbles.size()) break;

            uint8_t low  = all_nibbles[nib_idx];
            uint8_t high = all_nibbles[nib_idx + 1];
            if (i < hdr.payload_len) {
                uint8_t ws = gr::lora::whitening_seq[i % gr::lora::whitening_seq.size()];
                low  ^= (ws & 0x0F);
                high ^= ((ws >> 4) & 0x0F);
            }
            decoded_bytes.push_back(static_cast<uint8_t>((high << 4) | low));
        }

        return decoded_bytes;
    }
};

/// Count payload symbols in a frame (after preamble+SFD).
std::size_t countPayloadSymbols(std::size_t payload_len, uint8_t sf, uint8_t cr, bool has_crc) {
    bool ldro = (static_cast<float>(1u << sf) * 1e3f / static_cast<float>(BW)) > 16.f;
    int sf_app = sf - 2 * static_cast<int>(ldro);
    auto n_payload_syms = static_cast<std::size_t>(
        std::ceil(static_cast<double>(
            2 * static_cast<int>(payload_len) - sf + 2 + 5 + (has_crc ? 4 : 0))
            / sf_app))
        * (4 + cr);
    return 8 + n_payload_syms;  // header block + payload symbols
}

}  // namespace

// ============================================================================
// Full decode chain benchmarks
// ============================================================================

const boost::ut::suite decode_benchmarks = [] {
    using namespace benchmark;
    using namespace boost::ut;

    // Generate a clean frame via TX chain (os_factor=1, no oversampling)
    auto iq = gr::lora::generate_frame_iq(
        PAYLOAD, SF, CR, 1, SYNC_WORD, PREAMBLE_LEN, true, 0);

    // Calculate preamble+SFD length to skip to payload symbols
    // Preamble: 8 upchirps + 2 sync word upchirps + 2.25 downchirps = 12.25 symbols
    std::size_t preamble_samples = static_cast<std::size_t>(
        (PREAMBLE_LEN + 2) * N + N * 2 + N / 4);

    std::size_t n_payload_symbols = countPayloadSymbols(PAYLOAD.size(), SF, CR, true);
    const std::complex<float>* payload_iq = iq.data() + preamble_samples;

    // Verify the decoder works before benchmarking
    AlgorithmRxDecoder decoder;
    auto decoded = decoder.decodePayloadSymbols(payload_iq, n_payload_symbols);

    std::string expected(PAYLOAD.begin(), PAYLOAD.end());
    std::string got(decoded.begin(), decoded.end());
    expect(eq(got, expected)) << "Sanity check: decode must produce correct payload";

    // --- Benchmark: full RX decode (per-frame) ---
    {
        constexpr int REPS = 2'000;
        ::benchmark::benchmark<REPS>(std::string_view("full RX decode (14B frame)"), 1) =
            [&decoder, payload_iq, n_payload_symbols] {
                auto result = decoder.decodePayloadSymbols(payload_iq, n_payload_symbols);
                g_sink = result.size();
            };
    }

    ::benchmark::results::add_separator();

    // --- Benchmark: full TX+RX loopback ---
    {
        constexpr int REPS = 500;
        ::benchmark::benchmark<REPS>(std::string_view("TX+RX loopback (14B)"), 1) =
            [&decoder] {
                // TX
                auto frame_iq = gr::lora::generate_frame_iq(
                    PAYLOAD, SF, CR, 1, SYNC_WORD, PREAMBLE_LEN, true, 0);

                // RX (skip preamble)
                std::size_t pre_samps = static_cast<std::size_t>(
                    (PREAMBLE_LEN + 2) * N + N * 2 + N / 4);
                std::size_t n_syms = countPayloadSymbols(PAYLOAD.size(), SF, CR, true);

                auto result = decoder.decodePayloadSymbols(
                    frame_iq.data() + pre_samps, n_syms);
                g_sink = result.size();
            };
    }

    ::benchmark::results::add_separator();

    // --- Real-time margin calculation ---
    // Measure how many frames/second we can decode and compare to LoRa airtime
    {
        constexpr int N_FRAMES = 1000;
        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < N_FRAMES; i++) {
            auto result = decoder.decodePayloadSymbols(payload_iq, n_payload_symbols);
            g_sink = result.size();
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double elapsed_s = std::chrono::duration<double>(t1 - t0).count();
        double frames_per_sec = static_cast<double>(N_FRAMES) / elapsed_s;

        // LoRa airtime for this frame: n_total_symbols * T_sym
        // T_sym = 2^SF / BW = 256 / 62500 = 4.096 ms
        double t_sym_s = static_cast<double>(N) / static_cast<double>(BW);
        std::size_t total_syms = PREAMBLE_LEN + 2 + 2 + n_payload_symbols;  // +SFD
        double airtime_s = static_cast<double>(total_syms) * t_sym_s + t_sym_s * 0.25;
        double max_realtime_fps = 1.0 / airtime_s;
        double margin = frames_per_sec / max_realtime_fps;

        std::printf("\n--- Real-time margin ---\n");
        std::printf("  Decode rate:     %.0f frames/s\n", frames_per_sec);
        std::printf("  Symbol rate:     %.0f symbols/s\n",
                    frames_per_sec * static_cast<double>(n_payload_symbols));
        std::printf("  Frame airtime:   %.1f ms (%zu symbols)\n",
                    airtime_s * 1e3, total_syms);
        std::printf("  Max real-time:   %.1f frames/s\n", max_realtime_fps);
        std::printf("  Margin:          %.0fx real-time\n", margin);
        std::printf("  Concurrent ch:   ~%.0f (at 100%% CPU)\n", margin);
        std::fflush(stdout);
    }
};

int main() { /* boost::ut runs suites automatically */ }
