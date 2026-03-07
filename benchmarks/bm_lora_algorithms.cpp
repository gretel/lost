// SPDX-License-Identifier: ISC
/// Microbenchmarks for individual LoRa PHY algorithm steps.
///
/// Measures the hot-path functions that dominate encode/decode time.
/// Run with: ./bm_lora_algorithms
/// Extra precision: BM_DIGITS=3 ./bm_lora_algorithms

#include <benchmark.hpp>

#include <chrono>
#include <complex>
#include <cstdint>
#include <format>
#include <numbers>
#include <vector>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/algorithm/crc.hpp>
#include <gnuradio-4.0/lora/algorithm/firdes.hpp>
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

    // --- dechirp_argmax: combined dechirp+FFT+argmax (production hot path) ---
    {
        gr::algorithm::FFT<std::complex<float>> fft;
        std::vector<std::complex<float>> upchirp(N), downchirp(N);
        gr::lora::build_ref_chirps(upchirp.data(), downchirp.data(), SF, 1);

        std::vector<std::complex<float>> chirp(N);
        gr::lora::build_upchirp(chirp.data(), 42, SF, 1);

        std::vector<std::complex<float>> scratch(N);

        g_sink = gr::lora::dechirp_argmax(chirp.data(), downchirp.data(), scratch.data(), N, fft);

        constexpr int REPS = 10'000;
        ::benchmark::benchmark<REPS>(std::string_view("dechirp_argmax N=256"), N) =
            [&fft, &chirp, &downchirp, &scratch] {
                g_sink = gr::lora::dechirp_argmax(chirp.data(), downchirp.data(),
                                                   scratch.data(), N, fft);
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
        auto encoded     = gr::lora::hamming_encode_frame(with_crc, SF, CR);
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

    // --- Build ref chirps: upchirp + downchirp pair ---
    {
        std::vector<std::complex<float>> up(N), down(N);

        constexpr int REPS = 10'000;
        ::benchmark::benchmark<REPS>(std::string_view("build_ref_chirps N=256"), N) =
            [&up, &down] {
                gr::lora::build_ref_chirps(up.data(), down.data(), SF, 1);
                g_sink = static_cast<std::size_t>(up[0].real());
            };
    }

    // --- Interleave: one frame ---
    {
        auto whitened    = gr::lora::whiten(PAYLOAD);
        auto with_header = gr::lora::insert_header(whitened,
                                                    static_cast<uint8_t>(PAYLOAD.size()),
                                                    CR, true);
        auto with_crc    = gr::lora::add_crc(with_header, PAYLOAD, true);
        auto encoded     = gr::lora::hamming_encode_frame(with_crc, SF, CR);

        constexpr int REPS = 5'000;
        ::benchmark::benchmark<REPS>(std::string_view("interleave_frame (37 cw)"), encoded.size()) =
            [&encoded] {
                auto syms = gr::lora::interleave_frame(encoded, SF, CR);
                g_sink = syms.size();
            };
    }
};

const boost::ut::suite tx_benchmarks = [] {
    using namespace benchmark;

    ::benchmark::results::add_separator();

    // --- Gray demap: per-symbol TX path ---
    {
        auto whitened    = gr::lora::whiten(PAYLOAD);
        auto with_header = gr::lora::insert_header(whitened,
                                                    static_cast<uint8_t>(PAYLOAD.size()),
                                                    CR, true);
        auto with_crc    = gr::lora::add_crc(with_header, PAYLOAD, true);
        auto encoded     = gr::lora::hamming_encode_frame(with_crc, SF, CR);
        auto interleaved = gr::lora::interleave_frame(encoded, SF, CR);

        constexpr int REPS = 5'000;
        ::benchmark::benchmark<REPS>(std::string_view("gray_demap (20 sym)"), interleaved.size()) =
            [&interleaved] {
                auto result = gr::lora::gray_demap(interleaved, SF);
                g_sink = result.size();
            };
    }

    // --- Modulate frame: chirp IQ generation from symbols ---
    {
        auto whitened    = gr::lora::whiten(PAYLOAD);
        auto with_header = gr::lora::insert_header(whitened,
                                                    static_cast<uint8_t>(PAYLOAD.size()),
                                                    CR, true);
        auto with_crc    = gr::lora::add_crc(with_header, PAYLOAD, true);
        auto encoded     = gr::lora::hamming_encode_frame(with_crc, SF, CR);
        auto interleaved = gr::lora::interleave_frame(encoded, SF, CR);
        auto gray_mapped = gr::lora::gray_demap(interleaved, SF);

        constexpr int REPS = 1'000;
        ::benchmark::benchmark<REPS>(std::string_view("modulate_frame (14B, os=1)"), gray_mapped.size()) =
            [&gray_mapped] {
                auto iq = gr::lora::modulate_frame(gray_mapped, SF, OS_FACTOR,
                                                    SYNC_WORD, PREAMBLE_LEN);
                g_sink = iq.size();
            };
    }

    ::benchmark::results::add_separator();

    {
        constexpr int REPS = 1'000;
        ::benchmark::benchmark<REPS>(std::string_view("generate_frame_iq (14B, os=1)"), PAYLOAD.size()) =
            [] {
                auto iq = gr::lora::generate_frame_iq(
                    PAYLOAD, SF, CR, OS_FACTOR, SYNC_WORD, PREAMBLE_LEN, true, 0);
                g_sink = iq.size();
            };
    }

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

const boost::ut::suite misc_benchmarks = [] {
    using namespace benchmark;

    ::benchmark::results::add_separator();

    // --- Hamming soft decode (ML-LUT): per codeword ---
    {
        // Create realistic LLR values from a known codeword
        uint8_t cw = gr::lora::hamming_encode(0x0A, CR);
        constexpr uint8_t cw_len = CR + 4;
        std::array<gr::lora::LLR, cw_len> llr{};
        for (uint8_t j = 0; j < cw_len; j++) {
            bool bit = (cw >> (cw_len - 1 - j)) & 1;
            llr[j] = bit ? 3.5 : -3.5;
        }

        constexpr int REPS = 100'000;
        ::benchmark::benchmark<REPS>(std::string_view("hamming_decode_soft (cr=4)"), 1) =
            [&llr] {
                auto nib = gr::lora::hamming_decode_soft(llr.data(), CR);
                g_sink = nib;
            };
    }

    // --- FIR filter design: channelizer initialization ---
    {
        constexpr uint32_t NUM_TAPS    = 101;
        constexpr double   SAMPLE_RATE = 250000.0;
        constexpr double   CUTOFF      = 31250.0;  // BW/2

        auto warmup = gr::lora::firdes_low_pass(NUM_TAPS, SAMPLE_RATE, CUTOFF);
        g_sink = warmup.size();

        constexpr int REPS = 5'000;
        ::benchmark::benchmark<REPS>(std::string_view("firdes_low_pass (101 taps)"), NUM_TAPS) =
            [] {
                auto taps = gr::lora::firdes_low_pass(NUM_TAPS, SAMPLE_RATE, CUTOFF);
                g_sink = taps.size();
            };
    }
};

// Measure best-of-N batches to avoid OS scheduling jitter in the mean.
// Each batch runs BATCH_REPS calls; we take the minimum batch time.
// This gives a reliable lower bound unaffected by preemptions.
template<typename Fn>
double min_of_batches_ns(Fn&& fn, int batch_reps, int n_batches) {
    double best = std::numeric_limits<double>::max();
    for (int b = 0; b < n_batches; ++b) {
        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < batch_reps; ++i) { fn(); }
        auto t1 = std::chrono::high_resolution_clock::now();
        double ns = std::chrono::duration<double, std::nano>(t1 - t0).count() / batch_reps;
        if (ns < best) { best = ns; }
    }
    return best;
}

const boost::ut::suite performance_assertions = [] {
    using namespace boost::ut;

    "FFT must complete under 10 us per symbol"_test = [] {
        gr::algorithm::FFT<std::complex<float>> fft;
        auto dechirped = make_dechirped_symbol(42);
        fft.compute(dechirped); // warm up

        double best_ns = min_of_batches_ns([&] {
            auto result = fft.compute(dechirped);
            g_sink = result.size();
        }, 500, 20);
        double best_us = best_ns / 1000.0;
        expect(lt(best_us, 10.0)) << std::format("FFT took {:.1f} us (limit: 10 us)", best_us);
    };

    "TX chain (os=4) must complete under 500 us"_test = [] {
        // warm up
        auto warmup = gr::lora::generate_frame_iq(PAYLOAD, SF, CR, 4, SYNC_WORD, PREAMBLE_LEN, true, 0);
        g_sink = warmup.size();

        double best_ns = min_of_batches_ns([&] {
            auto iq = gr::lora::generate_frame_iq(PAYLOAD, SF, CR, 4, SYNC_WORD, PREAMBLE_LEN, true, 0);
            g_sink = iq.size();
        }, 20, 10);
        double best_us = best_ns / 1000.0;
        expect(lt(best_us, 500.0)) << std::format("TX chain took {:.1f} us (limit: 500 us)", best_us);
    };

    "Hamming decode must complete under 1 us"_test = [] {
        uint8_t cw = gr::lora::hamming_encode(0x0A, CR);
        // warm up
        for (int i = 0; i < 1000; ++i) { g_sink = gr::lora::hamming_decode_hard(cw, CR); }

        double best_ns = min_of_batches_ns([&] {
            g_sink = gr::lora::hamming_decode_hard(cw, CR);
        }, 10'000, 20);
        expect(lt(best_ns, 1000.0)) << std::format("Hamming decode took {:.1f} ns (limit: 1 us)", best_ns);
    };

    "CRC-16 (14 bytes) must complete under 1 us"_test = [] {
        gr::lora::crc16(PAYLOAD); // warm up

        double best_ns = min_of_batches_ns([&] {
            g_sink = gr::lora::crc16(PAYLOAD);
        }, 5'000, 20);
        double best_us = best_ns / 1000.0;
        expect(lt(best_us, 1.0)) << std::format("CRC-16 took {:.3f} us (limit: 1 us)", best_us);
    };

    "dechirp_argmax must complete under 10 us"_test = [] {
        gr::algorithm::FFT<std::complex<float>> fft;
        std::vector<std::complex<float>> upchirp(N), downchirp(N);
        gr::lora::build_ref_chirps(upchirp.data(), downchirp.data(), SF, 1);
        std::vector<std::complex<float>> chirp(N), scratch(N);
        gr::lora::build_upchirp(chirp.data(), 42, SF, 1);
        // warm up
        g_sink = gr::lora::dechirp_argmax(chirp.data(), downchirp.data(), scratch.data(), N, fft);

        double best_ns = min_of_batches_ns([&] {
            g_sink = gr::lora::dechirp_argmax(chirp.data(), downchirp.data(), scratch.data(), N, fft);
        }, 500, 20);
        double best_us = best_ns / 1000.0;
        expect(lt(best_us, 10.0)) << std::format("dechirp_argmax took {:.1f} us (limit: 10 us)", best_us);
    };

    "modulate_frame (os=4) must complete under 500 us"_test = [] {
        auto whitened    = gr::lora::whiten(PAYLOAD);
        auto with_header = gr::lora::insert_header(whitened,
                                                    static_cast<uint8_t>(PAYLOAD.size()),
                                                    CR, true);
        auto with_crc    = gr::lora::add_crc(with_header, PAYLOAD, true);
        auto encoded     = gr::lora::hamming_encode_frame(with_crc, SF, CR);
        auto interleaved = gr::lora::interleave_frame(encoded, SF, CR);
        auto gray_mapped = gr::lora::gray_demap(interleaved, SF);
        // warm up
        auto warmup = gr::lora::modulate_frame(gray_mapped, SF, 4, SYNC_WORD, PREAMBLE_LEN);
        g_sink = warmup.size();

        double best_ns = min_of_batches_ns([&] {
            auto iq = gr::lora::modulate_frame(gray_mapped, SF, 4, SYNC_WORD, PREAMBLE_LEN);
            g_sink = iq.size();
        }, 20, 10);
        double best_us = best_ns / 1000.0;
        expect(lt(best_us, 500.0)) << std::format("modulate_frame took {:.1f} us (limit: 500 us)", best_us);
    };
};

int main() { /* boost::ut runs suites automatically */ }
