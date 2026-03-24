// SPDX-License-Identifier: ISC
/// TX->RX loopback tests: algorithm-level and graph-level.
/// Per-stage TX tests are in qa_lora_tx.cpp.

#include "test_helpers.hpp"

#include <chrono>
#include <cstdio>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/lora/MultiSfDecoder.hpp>
#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>

using namespace gr::lora::test;

// ============================================================================
// Full Loopback: GR4 TX -> GR4 RX (algorithm-level)
// ============================================================================

const boost::ut::suite<"Full loopback (algorithm-level)"> full_loopback_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "TX -> RX loopback recovers payload"_test = [] {
        auto payload_str = load_text("payload.txt");
        std::vector<uint8_t> payload(payload_str.begin(), payload_str.end());

        // === TX ===
        auto whitened    = whiten(payload);
        auto with_header = insert_header(whitened, static_cast<uint8_t>(payload.size()), CR, HAS_CRC);
        auto with_crc    = add_crc(with_header, payload, HAS_CRC);
        auto encoded     = hamming_encode_frame(with_crc, SF, CR);
        auto interleaved = interleave_frame(encoded, SF, CR);
        auto gray_mapped = gray_demap(interleaved, SF);

        // === RX (reverse the TX chain, no IQ/FrameSync/FFTDemod) ===
        // We skip modulate/demodulate since that's tested separately.
        // Start from gray_mapped (= what FFTDemod would produce after subtracting 1 + the argmax)

        // Stage 1: FFTDemod simulation (subtract 1)
        std::vector<uint16_t> fft_output;
        for (auto gm : gray_mapped) {
            fft_output.push_back(static_cast<uint16_t>((gm + N - 1) % N));
        }

        // Stage 2: GrayMapping
        std::vector<uint16_t> rx_interleaved;
        for (auto sym : fft_output) {
            rx_interleaved.push_back(static_cast<uint16_t>(sym ^ (sym >> 1)));
        }

        // Verify RX interleaved matches TX interleaved
        expect(eq(rx_interleaved.size(), interleaved.size())) << "RX interleaved size";
        for (std::size_t i = 0; i < rx_interleaved.size(); i++) {
            expect(eq(static_cast<uint32_t>(rx_interleaved[i]), interleaved[i]))
                << "RX interleaved mismatch at " << i;
        }

        // Stage 3: Deinterleave
        std::vector<uint8_t> all_codewords;
        std::size_t sym_idx = 0;
        bool is_first = true;
        while (sym_idx < rx_interleaved.size()) {
            uint8_t sf_app = is_first ? static_cast<uint8_t>(SF - 2) : SF;
            uint8_t cw_len = is_first ? uint8_t(8) : static_cast<uint8_t>(4 + CR);
            if (sym_idx + cw_len > rx_interleaved.size()) break;

            std::vector<uint16_t> block_syms;
            for (std::size_t j = 0; j < cw_len; j++) {
                uint16_t sym = rx_interleaved[sym_idx + j];
                if (is_first) sym >>= (SF - sf_app);
                block_syms.push_back(sym);
            }
            auto cw = deinterleave_block(block_syms, SF, cw_len, sf_app);
            all_codewords.insert(all_codewords.end(), cw.begin(), cw.end());
            sym_idx += cw_len;
            is_first = false;
        }

        // Stage 4: Hamming decode
        std::vector<uint8_t> nibbles;
        for (std::size_t i = 0; i < all_codewords.size(); i++) {
            uint8_t cr_app = (static_cast<int>(i) < SF - 2) ? uint8_t(4) : CR;
            nibbles.push_back(hamming_decode_hard(all_codewords[i], cr_app));
        }

        // Stage 5: Header decode
        expect(ge(nibbles.size(), std::size_t{5})) << "Not enough nibbles for header";
        auto hdr = parse_explicit_header(nibbles[0], nibbles[1], nibbles[2],
                                          nibbles[3], nibbles[4]);
        expect(hdr.checksum_valid) << "Header checksum invalid";
        expect(eq(static_cast<int>(hdr.payload_len), static_cast<int>(payload.size())))
            << "Header payload_len";
        expect(eq(static_cast<int>(hdr.cr), static_cast<int>(CR))) << "Header CR";
        expect(hdr.has_crc == HAS_CRC) << "Header CRC flag";

        // Stage 6: Dewhitening
        std::size_t payload_nibs = hdr.payload_len * 2;
        std::size_t crc_nibs = hdr.has_crc ? 4 : 0;
        std::size_t data_start = 5;

        std::vector<uint8_t> decoded_bytes;
        for (std::size_t i = 0; i < (payload_nibs + crc_nibs) / 2; i++) {
            std::size_t nib_idx = data_start + 2 * i;
            if (nib_idx + 1 >= nibbles.size()) break;
            uint8_t low_nib  = nibbles[nib_idx];
            uint8_t high_nib = nibbles[nib_idx + 1];
            if (i < hdr.payload_len) {
                uint8_t ws = whitening_seq[i % whitening_seq.size()];
                low_nib  ^= (ws & 0x0F);
                high_nib ^= ((ws >> 4) & 0x0F);
            }
            decoded_bytes.push_back(static_cast<uint8_t>((high_nib << 4) | low_nib));
        }

        // Stage 7: CRC verify
        if (hdr.has_crc && hdr.payload_len >= 2 && decoded_bytes.size() >= hdr.payload_len + 2u) {
            uint16_t computed_crc = crc16(std::span<const uint8_t>(
                decoded_bytes.data(), hdr.payload_len - 2));
            computed_crc ^= static_cast<uint16_t>(decoded_bytes[hdr.payload_len - 1]);
            computed_crc ^= static_cast<uint16_t>(static_cast<unsigned>(decoded_bytes[hdr.payload_len - 2]) << 8);
            uint16_t received_crc = static_cast<uint16_t>(
                                      static_cast<unsigned>(decoded_bytes[hdr.payload_len])
                                    | (static_cast<unsigned>(decoded_bytes[hdr.payload_len + 1]) << 8));
            expect(eq(computed_crc, received_crc))
                << "CRC mismatch: computed=0x" << std::format("{:04x}", computed_crc)
                << " received=0x" << std::format("{:04x}", received_crc);
        }

        // Final: compare payload
        std::string decoded_str(decoded_bytes.begin(),
                                decoded_bytes.begin() + hdr.payload_len);
        expect(eq(decoded_str, payload_str))
            << "Loopback payload: got \"" << decoded_str
            << "\" expected \"" << payload_str << "\"";
    };
};

// ============================================================================
// Multi-SF loopback: verify TX->RX roundtrip at SF7 and SF12 (boundary cases)
// ============================================================================

const boost::ut::suite<"Multi-SF loopback"> multi_sf_loopback_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    auto do_loopback = [](uint8_t test_sf, uint8_t test_cr,
                          uint32_t test_bw, const std::string& payload_str) {
        std::vector<uint8_t> payload(payload_str.begin(), payload_str.end());
        // os_factor=4 is implied by generate_frame() default

        // === TX ===
        auto whitened    = whiten(payload);
        auto with_header = insert_header(whitened,
                                         static_cast<uint8_t>(payload.size()),
                                         test_cr, true);
        auto with_crc    = add_crc(with_header, payload, true);
        auto encoded     = hamming_encode_frame(with_crc, test_sf, test_cr);

        bool ldro = needs_ldro(test_sf, test_bw);
        auto interleaved = interleave_frame(encoded, test_sf, test_cr, ldro);
        auto gray_mapped = gray_demap(interleaved, test_sf);

        uint32_t N_test = 1u << test_sf;

        // === RX (reverse the TX chain, no IQ) ===
        // Stage 1: FFTDemod simulation (subtract 1)
        std::vector<uint16_t> fft_output;
        for (auto gm : gray_mapped) {
            fft_output.push_back(static_cast<uint16_t>((gm + N_test - 1) % N_test));
        }

        // Stage 2: GrayMapping
        std::vector<uint16_t> rx_interleaved;
        for (auto sym : fft_output) {
            rx_interleaved.push_back(static_cast<uint16_t>(sym ^ (sym >> 1)));
        }

        // Stage 3: Deinterleave
        std::vector<uint8_t> all_codewords;
        std::size_t sym_idx = 0;
        bool is_first = true;
        while (sym_idx < rx_interleaved.size()) {
            bool reduced = is_first || ldro;
            uint8_t sf_app = reduced ? static_cast<uint8_t>(test_sf - 2) : test_sf;
            uint8_t cw_len = is_first ? uint8_t(8) : static_cast<uint8_t>(4 + test_cr);
            if (sym_idx + cw_len > rx_interleaved.size()) break;

            std::vector<uint16_t> block_syms;
            for (std::size_t j = 0; j < cw_len; j++) {
                uint16_t sym = rx_interleaved[sym_idx + j];
                if (reduced) sym >>= (test_sf - sf_app);
                block_syms.push_back(sym);
            }
            auto cw = deinterleave_block(block_syms, test_sf, cw_len, sf_app);
            all_codewords.insert(all_codewords.end(), cw.begin(), cw.end());
            sym_idx += cw_len;
            is_first = false;
        }

        // Stage 4: Hamming decode
        std::vector<uint8_t> nibbles;
        for (std::size_t i = 0; i < all_codewords.size(); i++) {
            uint8_t cr_app = (static_cast<int>(i) < test_sf - 2) ? uint8_t(4) : test_cr;
            nibbles.push_back(hamming_decode_hard(all_codewords[i], cr_app));
        }

        // Stage 5: Header decode
        expect(ge(nibbles.size(), std::size_t{5})) << "Not enough nibbles";
        auto hdr = parse_explicit_header(nibbles[0], nibbles[1], nibbles[2],
                                          nibbles[3], nibbles[4]);
        expect(hdr.checksum_valid) << "Header checksum";
        expect(eq(static_cast<int>(hdr.payload_len), static_cast<int>(payload.size())))
            << "Header pay_len";

        // Stage 6: Dewhitening
        std::vector<uint8_t> decoded_bytes;
        std::size_t data_start = 5;
        uint32_t total_nibs = hdr.payload_len * 2 + (hdr.has_crc ? 4u : 0u);
        for (uint32_t i = 0; i < total_nibs / 2; i++) {
            std::size_t nib_idx = data_start + 2 * i;
            if (nib_idx + 1 >= nibbles.size()) break;
            uint8_t low_nib  = nibbles[nib_idx];
            uint8_t high_nib = nibbles[nib_idx + 1];
            if (i < hdr.payload_len) {
                uint8_t ws = whitening_seq[i % whitening_seq.size()];
                low_nib  ^= (ws & 0x0F);
                high_nib ^= ((ws >> 4) & 0x0F);
            }
            decoded_bytes.push_back(static_cast<uint8_t>((high_nib << 4) | low_nib));
        }

        // Stage 7: CRC verify
        if (hdr.has_crc && hdr.payload_len >= 2
            && decoded_bytes.size() >= hdr.payload_len + 2u) {
            bool crc_ok = lora_verify_crc(
                std::span<const uint8_t>(decoded_bytes.data(), hdr.payload_len),
                decoded_bytes[hdr.payload_len],
                decoded_bytes[hdr.payload_len + 1]);
            expect(crc_ok) << "CRC mismatch at SF=" << static_cast<int>(test_sf);
        }

        // Compare payload
        std::string decoded_str(decoded_bytes.begin(),
                                decoded_bytes.begin() +
                                static_cast<int64_t>(hdr.payload_len));
        expect(eq(decoded_str, payload_str))
            << "SF=" << static_cast<int>(test_sf) << " payload mismatch";
    };

    "SF7 loopback"_test = [&do_loopback] {
        do_loopback(7, 4, 125000, "Hello SF7");
    };

    "SF12 loopback"_test = [&do_loopback] {
        do_loopback(12, 4, 125000, "Hello SF12");
    };

    "SF11 BW=62.5kHz LDRO loopback"_test = [&do_loopback] {
        // SF11 @ 62.5kHz: symbol duration = 32.77ms > 16ms -> LDRO active
        do_loopback(11, 4, 62500, "LDRO test");
    };

    "SF10 BW=62.5kHz LDRO boundary loopback"_test = [&do_loopback] {
        // SF10 @ 62.5kHz: symbol duration = 16.38ms > 16ms -> LDRO just active
        do_loopback(10, 4, 62500, "LDRO boundary");
    };

    "SF9 CR=2 loopback"_test = [&do_loopback] {
        do_loopback(9, 2, 125000, "CR2 test");
    };

    "SF8 CR=1 loopback"_test = [&do_loopback] {
        do_loopback(8, 1, 62500, "CR1 test");
    };
};

// ============================================================================
// Performance timing: measure TX→RX algorithm throughput
// ============================================================================

const boost::ut::suite<"Loopback timing"> timing_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "TX algorithm chain throughput"_test = [] {
        auto payload_str = load_text("payload.txt");
        std::vector<uint8_t> payload(payload_str.begin(), payload_str.end());

        constexpr int REPS = 500;
        auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < REPS; i++) {
            auto iq = generate_frame_iq(payload, SF, CR, 1, 0x12, 8, true, N * 2, 2);
            expect(gt(iq.size(), std::size_t{0})) << "IQ generation failed";
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double elapsed_s = std::chrono::duration<double>(t1 - t0).count();
        double fps = static_cast<double>(REPS) / elapsed_s;

        std::fprintf(stderr, "\n--- TX chain timing ---\n");
        std::fprintf(stderr, "  %d iterations in %.3f s = %.0f frames/s\n",
                     REPS, elapsed_s, fps);

        // Sanity: TX chain should be fast (>100 frames/s at os_factor=1)
        expect(gt(fps, 50.0)) << "TX chain too slow: " << fps << " fps";
    };

    "Full algorithm loopback throughput"_test = [] {
        auto payload_str = load_text("payload.txt");
        std::vector<uint8_t> payload(payload_str.begin(), payload_str.end());

        // Pre-generate the IQ frame once (os_factor=1 for fastest decode)
        auto iq = generate_frame_iq(payload, SF, CR, 1, 0x12, 8, true, N * 2, 2);

        // The RX algorithm pipeline: gray_map → deinterleave → hamming → header → dewhiten → CRC
        // We simulate the FFT demod step (dechirp + argmax) inline.
        // Generate reference chirp for dechirp
        std::vector<std::complex<float>> ref_downchirp(N);
        for (uint32_t n = 0; n < N; n++) {
            float phase = -static_cast<float>(M_PI) * static_cast<float>(n * n) / static_cast<float>(N);
            ref_downchirp[n] = {std::cos(phase), std::sin(phase)};
        }

        constexpr int REPS = 200;
        auto t0 = std::chrono::high_resolution_clock::now();
        for (int rep = 0; rep < REPS; rep++) {
            // Extract payload symbols from the IQ (skip preamble + SFD)
            // Preamble: 8 upchirps + 2 sync + 2.25 downchirps = 12.25 symbols
            // Start at sample offset 12.25 * N
            std::size_t payload_start = static_cast<std::size_t>(12.25 * N);

            // Count symbols: for "Hello MeshCore" (14 bytes) at SF8/CR4:
            //   header: 8 symbols, payload: ceil((14+2)*2 * (4+4) / 8) = 32 codewords
            //   -> 32 / 8 = 4 payload blocks -> 4*8 = 32 symbols
            //   total payload symbols = 8 (header) + 32 = 40... but let's just demod all
            std::size_t n_symbols = (iq.size() - payload_start) / N;

            // Dechirp + FFT + argmax for each symbol
            std::vector<uint16_t> demod_symbols;
            demod_symbols.reserve(n_symbols);
            for (std::size_t s = 0; s < n_symbols; s++) {
                std::size_t offset = payload_start + s * N;
                // Dechirp
                std::vector<std::complex<float>> dechirped(N);
                for (uint32_t n = 0; n < N; n++) {
                    dechirped[n] = iq[offset + n] * ref_downchirp[n];
                }
                // FFT (simple DFT for N=256 — adequate for timing test)
                float max_mag = 0;
                uint32_t max_bin = 0;
                for (uint32_t k = 0; k < N; k++) {
                    std::complex<float> sum{0, 0};
                    for (uint32_t n = 0; n < N; n++) {
                        float angle = -2.0f * static_cast<float>(M_PI)
                                      * static_cast<float>(k * n) / static_cast<float>(N);
                        sum += dechirped[n] * std::complex<float>{std::cos(angle), std::sin(angle)};
                    }
                    float mag = std::abs(sum);
                    if (mag > max_mag) { max_mag = mag; max_bin = k; }
                }
                demod_symbols.push_back(static_cast<uint16_t>((max_bin + N - 1) % N));
            }

            // Gray map
            for (auto& sym : demod_symbols) {
                sym = static_cast<uint16_t>(sym ^ (sym >> 1));
            }

            // Deinterleave + Hamming decode + header parse + dewhiten
            std::vector<uint8_t> all_codewords;
            std::size_t sym_idx = 0;
            bool is_first = true;
            while (sym_idx < demod_symbols.size()) {
                uint8_t sf_app = is_first ? static_cast<uint8_t>(SF - 2) : SF;
                uint8_t cw_len = is_first ? uint8_t(8) : static_cast<uint8_t>(4 + CR);
                if (sym_idx + cw_len > demod_symbols.size()) break;

                std::vector<uint16_t> block_syms;
                for (std::size_t j = 0; j < cw_len; j++) {
                    uint16_t sym = demod_symbols[sym_idx + j];
                    if (is_first) sym >>= (SF - sf_app);
                    block_syms.push_back(sym);
                }
                auto cw = deinterleave_block(block_syms, SF, cw_len, sf_app);
                all_codewords.insert(all_codewords.end(), cw.begin(), cw.end());
                sym_idx += cw_len;
                is_first = false;
            }

            std::vector<uint8_t> nibbles;
            for (std::size_t i = 0; i < all_codewords.size(); i++) {
                uint8_t cr_app = (static_cast<int>(i) < SF - 2) ? uint8_t(4) : CR;
                nibbles.push_back(hamming_decode_hard(all_codewords[i], cr_app));
            }

            if (nibbles.size() >= 5) {
                auto hdr = parse_explicit_header(nibbles[0], nibbles[1], nibbles[2],
                                                  nibbles[3], nibbles[4]);
                // Just touch the result to prevent dead-code elimination
                if (!hdr.checksum_valid) { expect(false) << "Header checksum failed in timing loop"; }
            }
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double elapsed_s = std::chrono::duration<double>(t1 - t0).count();
        double fps = static_cast<double>(REPS) / elapsed_s;

        // LoRa airtime: (8 preamble + 2 SFD + 2.25 downchirp + ~40 payload) * 4.096ms
        double t_sym_s = static_cast<double>(N) / 62500.0;
        double airtime_s = 52.25 * t_sym_s;
        double realtime_fps = 1.0 / airtime_s;
        double margin = fps / realtime_fps;

        std::fprintf(stderr, "\n--- Full loopback timing ---\n");
        std::fprintf(stderr, "  %d iterations in %.3f s = %.1f frames/s\n",
                     REPS, elapsed_s, fps);
        std::fprintf(stderr, "  Frame airtime: %.1f ms\n", airtime_s * 1e3);
        std::fprintf(stderr, "  Real-time margin: %.1fx\n", margin);

        // Must be at least 1x real-time (decode faster than airtime)
        expect(gt(margin, 1.0))
            << "Decode slower than real-time: " << margin << "x";
    };
};

// ============================================================================
// Graph-level loopback: TX IQ -> MultiSfDecoder -> payload
// This exercises the same production decode pipeline as lora_trx.
// ============================================================================

const boost::ut::suite<"Graph-level loopback"> graph_loopback_tests = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::lora;

    auto run_graph_loopback = [](const std::vector<uint8_t>& payload,
                                 uint8_t test_sf, uint8_t test_cr,
                                 uint32_t test_bw, uint8_t os_factor,
                                 uint16_t sync_word, uint16_t preamble_len,
                                 const std::string& label) {
        uint32_t sps = (1u << test_sf) * os_factor;
        auto iq = generate_frame_iq(payload, test_sf, test_cr, os_factor,
                                    sync_word, preamble_len,
                                    true, sps * 5, 2, test_bw, false);

        Graph graph;
        auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(iq.size())},
            {"repeat_tags", false},
            {"mark_tag", false}
        });
        src.values = iq;

        auto& decoder = graph.emplaceBlock<MultiSfDecoder>();
        decoder.center_freq  = CENTER_FREQ;
        decoder.bandwidth    = test_bw;
        decoder.sync_word    = sync_word;
        decoder.os_factor    = os_factor;
        decoder.preamble_len = preamble_len;
        decoder.sf_min       = test_sf;
        decoder.sf_max       = test_sf;

        auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", true}
        });

        (void)graph.connect<"out", "in">(src, decoder);
        (void)graph.connect<"out", "in">(decoder, sink);

        scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            throw std::runtime_error(std::format("failed to init scheduler: {}", ret.error()));
        }
        sched.runAndWait();

        std::printf("  %s: %zu bytes output (expected %zu)\n",
                    label.c_str(), sink._samples.size(), payload.size());

        expect(ge(sink._samples.size(), payload.size()))
            << label << " output too small: " << sink._samples.size();

        if (sink._samples.size() >= payload.size()) {
            std::string decoded(sink._samples.begin(),
                                sink._samples.begin()
                                + static_cast<std::ptrdiff_t>(payload.size()));
            std::string expected(payload.begin(), payload.end());
            expect(eq(decoded, expected))
                << label << " payload mismatch: \"" << decoded << "\"";
        }

        bool found_crc = false;
        for (const auto& t : sink._tags) {
            if (auto it = t.map.find("crc_valid"); it != t.map.end()) {
                expect(it->second.value_or<bool>(false))
                    << label << " CRC should be valid";
                found_crc = true;
            }
        }
        expect(found_crc) << label << " should have crc_valid tag";
    };

    "MeshCore ADVERT-like payload through graph"_test = [&run_graph_loopback] {
        // MeshCore ADVERT header: version + route + payload_type + flags + 32B pubkey + name
        // Use 48 bytes — realistic ADVERT size (truncated name)
        std::vector<uint8_t> advert(48);
        advert[0] = 0x01;  // version
        advert[1] = 0x00;  // route: FLOOD
        advert[2] = 0x01;  // payload: ADVERT
        for (std::size_t i = 3; i < advert.size(); i++) {
            advert[i] = static_cast<uint8_t>(i & 0xFF);
        }
        run_graph_loopback(advert, SF, CR, BW, OS_FACTOR,
                           SYNC_WORD, PREAMBLE_LEN, "MeshCore ADVERT (48B)");
    };

    "MeshCore TXT_MSG-like payload through graph"_test = [&run_graph_loopback] {
        // Typical TXT_MSG: ~22 bytes
        std::vector<uint8_t> msg = {
            0x01,       // version
            0x00,       // route: FLOOD
            0x06,       // payload: TXT_MSG
            'H', 'e', 'l', 'l', 'o', ' ', 'f', 'r', 'o', 'm', ' ',
            'l', 'o', 'r', 'a', '_', 't', 'r', 'x'
        };
        run_graph_loopback(msg, SF, CR, BW, OS_FACTOR,
                           SYNC_WORD, PREAMBLE_LEN, "MeshCore TXT_MSG (22B)");
    };

    "Short payload (5B) through graph at os_factor=1"_test = [&run_graph_loopback] {
        std::vector<uint8_t> payload = {'H', 'e', 'l', 'l', 'o'};
        run_graph_loopback(payload, SF, CR, BW, 1,
                           SYNC_WORD, PREAMBLE_LEN, "Hello (5B, os=1)");
    };

    "Empty-ish payload (1B) through graph"_test = [&run_graph_loopback] {
        std::vector<uint8_t> payload = {0x42};
        run_graph_loopback(payload, SF, CR, BW, OS_FACTOR,
                           SYNC_WORD, PREAMBLE_LEN, "Single byte (1B)");
    };

    // Zero-byte payload: valid per LoRa spec (header-only frame, no payload
    // data, no CRC bytes transmitted because add_crc() requires payload >= 2).
    // The TX chain produces a well-formed header-only frame. However,
    // MultiSfDecoder explicitly rejects payload_len==0, treating it as an
    // invalid frame. The test verifies the pipeline does not crash and
    // documents this limitation.
    "zero-byte payload loopback"_test = [] {
        using namespace gr;
        using namespace gr::lora;

        std::vector<uint8_t> payload{};  // empty
        constexpr uint8_t  test_sf   = SF;
        constexpr uint8_t  test_cr   = CR;
        constexpr uint32_t test_bw   = BW;
        constexpr uint8_t  os        = OS_FACTOR;
        constexpr uint16_t sw        = SYNC_WORD;
        constexpr uint16_t plen      = PREAMBLE_LEN;
        uint32_t sps = (1u << test_sf) * os;

        // --- TX: generate_frame_iq must not crash on empty payload ---
        auto iq = generate_frame_iq(payload, test_sf, test_cr, os,
                                    sw, plen, true, sps * 5, 2, test_bw, false);
        // Header-only frame: preamble(8) + sync(2) + SFD(2.25) + header(8 symbols)
        // + zero_pad(sps*5) — must have non-trivial length.
        expect(boost::ut::gt(iq.size(), std::size_t{0}))
            << "empty-payload IQ should be non-empty";

        // --- Graph: run without crash ---
        Graph graph;
        auto& src = graph.emplaceBlock<testing::TagSource<std::complex<float>,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_max", static_cast<gr::Size_t>(iq.size())},
            {"repeat_tags", false},
            {"mark_tag", false}
        });
        src.values = iq;

        auto& decoder = graph.emplaceBlock<MultiSfDecoder>();
        decoder.center_freq  = CENTER_FREQ;
        decoder.bandwidth    = test_bw;
        decoder.sync_word    = sw;
        decoder.os_factor    = os;
        decoder.preamble_len = plen;
        decoder.sf_min       = test_sf;
        decoder.sf_max       = test_sf;

        auto& sink = graph.emplaceBlock<testing::TagSink<uint8_t,
            testing::ProcessFunction::USE_PROCESS_BULK>>({
            {"log_samples", true},
            {"log_tags", true}
        });

        (void)graph.connect<"out", "in">(src, decoder);
        (void)graph.connect<"out", "in">(decoder, sink);

        scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            throw std::runtime_error(
                std::format("failed to init scheduler: {}", ret.error()));
        }
        sched.runAndWait();

        std::printf("  zero-byte payload: %zu bytes output\n", sink._samples.size());

        // MultiSfDecoder rejects payload_len==0 as invalid — no output expected.
        // If a future change enables 0-byte decode, update this assertion to
        // check for crc_valid=true instead.
        expect(boost::ut::eq(sink._samples.size(), std::size_t{0}))
            << "MultiSfDecoder rejects 0-byte payload (pay_len==0 treated as invalid)";

        // No crc_valid tag should be emitted since the frame is dropped.
        bool found_crc = false;
        for (const auto& t : sink._tags) {
            if (t.map.find("crc_valid") != t.map.end()) {
                found_crc = true;
            }
        }
        expect(!found_crc)
            << "no crc_valid tag expected (frame dropped by MultiSfDecoder)";
    };

    // LDRO configs — graph-level decode with reduced-rate payload blocks
    "SF10/BW62.5k LDRO through graph"_test = [&run_graph_loopback] {
        std::vector<uint8_t> payload = {'H', 'e', 'l', 'l', 'o', ' ', 'L', 'D', 'R', 'O'};
        run_graph_loopback(payload, 10, 4, 62500, 4,
                           SYNC_WORD, PREAMBLE_LEN, "SF10/BW62.5k LDRO (10B)");
    };

    "SF11/BW62.5k LDRO through graph"_test = [&run_graph_loopback] {
        std::vector<uint8_t> payload = {'H', 'e', 'l', 'l', 'o', ' ', 'L', 'D', 'R', 'O'};
        run_graph_loopback(payload, 11, 4, 62500, 4,
                           SYNC_WORD, PREAMBLE_LEN, "SF11/BW62.5k LDRO (10B)");
    };

    "SF12/BW125k LDRO through graph"_test = [&run_graph_loopback] {
        std::vector<uint8_t> payload = {'H', 'e', 'l', 'l', 'o', ' ', 'L', 'D', 'R', 'O'};
        run_graph_loopback(payload, 12, 4, 125000, 4,
                           SYNC_WORD, PREAMBLE_LEN, "SF12/BW125k LDRO (10B)");
    };
};

int main() { /* boost::ut auto-runs all suites */ }
