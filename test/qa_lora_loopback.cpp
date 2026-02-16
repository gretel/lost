// SPDX-License-Identifier: ISC
/// Full TX pipeline and TX->RX loopback tests (algorithm-level only).

#include <boost/ut.hpp>

#include <cmath>
#include <complex>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>

// ---- Test vector loading helpers ----

namespace {

const std::filesystem::path& testVectorDir() {
    static const std::filesystem::path dir = TEST_VECTORS_DIR;
    return dir;
}

std::vector<uint8_t> load_u8(const std::string& filename) {
    auto path = testVectorDir() / filename;
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open test vector: " + path.string());
    return {std::istreambuf_iterator<char>(f), {}};
}

std::vector<uint32_t> load_u32(const std::string& filename) {
    auto path = testVectorDir() / filename;
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open test vector: " + path.string());
    std::vector<uint32_t> data;
    uint32_t val;
    while (f.read(reinterpret_cast<char*>(&val), sizeof(val))) {
        data.push_back(val);
    }
    return data;
}

std::vector<std::complex<float>> load_cf32(const std::string& filename) {
    auto path = testVectorDir() / filename;
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open test vector: " + path.string());
    std::vector<std::complex<float>> data;
    float re, im;
    while (f.read(reinterpret_cast<char*>(&re), sizeof(re)) &&
           f.read(reinterpret_cast<char*>(&im), sizeof(im))) {
        data.emplace_back(re, im);
    }
    return data;
}

std::string load_text(const std::string& filename) {
    auto path = testVectorDir() / filename;
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open test vector: " + path.string());
    return {std::istreambuf_iterator<char>(f), {}};
}

// MeshCore test configuration (must match test_vectors/config.json)
constexpr uint8_t  SF           = 8;
constexpr uint32_t N            = 1u << SF; // 256
constexpr uint8_t  CR           = 4;
constexpr uint32_t BW           = 62500;
constexpr uint32_t SAMP_RATE    = 250000;  // 4 * BW
constexpr uint8_t  OS_FACTOR    = 4;
constexpr uint16_t SYNC_WORD    = 0x12;
constexpr uint16_t PREAMBLE_LEN = 8;
constexpr bool     HAS_CRC      = true;
constexpr uint32_t SPS          = N * OS_FACTOR; // 1024 samples per symbol

} // namespace

// ============================================================================
// TX Integration Test
// ============================================================================

const boost::ut::suite<"Full TX pipeline (algorithm-level)"> full_tx_pipeline_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "Full TX pipeline vs GR3 test vector"_test = [] {
        auto payload_str = load_text("payload.txt");
        std::vector<uint8_t> payload(payload_str.begin(), payload_str.end());

        // Step 1: Whitening
        auto whitened = whiten(payload);
        auto expected_whitened = load_u8("tx_01_whitened_nibbles.u8");
        expect(eq(whitened.size(), expected_whitened.size())) << "Whitened size";
        for (std::size_t i = 0; i < whitened.size(); i++) {
            expect(eq(whitened[i], expected_whitened[i])) << "Whitened[" << i << "]";
        }

        // Step 2: Header insert
        auto with_header = insert_header(whitened, static_cast<uint8_t>(payload.size()), CR, HAS_CRC);
        auto expected_header = load_u8("tx_02_with_header.u8");
        expect(eq(with_header.size(), expected_header.size())) << "Header size";
        for (std::size_t i = 0; i < with_header.size(); i++) {
            expect(eq(with_header[i], expected_header[i])) << "Header[" << i << "]";
        }

        // Step 3: Add CRC
        auto with_crc = add_crc(with_header, payload, HAS_CRC);
        auto expected_crc = load_u8("tx_03_with_crc.u8");
        expect(eq(with_crc.size(), expected_crc.size())) << "CRC size";
        for (std::size_t i = 0; i < with_crc.size(); i++) {
            expect(eq(with_crc[i], expected_crc[i])) << "CRC[" << i << "]";
        }

        // Step 4: Hamming encode
        auto encoded = hamming_enc_frame(with_crc, SF, CR);
        auto expected_encoded = load_u8("tx_04_encoded.u8");
        expect(eq(encoded.size(), expected_encoded.size())) << "Encoded size";
        for (std::size_t i = 0; i < encoded.size(); i++) {
            expect(eq(encoded[i], expected_encoded[i])) << "Encoded[" << i << "]";
        }

        // Step 5: Interleave
        auto interleaved = interleave_frame(encoded, SF, CR);
        auto expected_interleaved = load_u32("tx_05_interleaved.u32");
        expect(eq(interleaved.size(), expected_interleaved.size())) << "Interleaved size";
        for (std::size_t i = 0; i < interleaved.size(); i++) {
            expect(eq(interleaved[i], expected_interleaved[i])) << "Interleaved[" << i << "]";
        }

        // Step 6: Gray demap
        auto gray_mapped = gray_demap(interleaved, SF);
        auto expected_gray = load_u32("tx_06_gray_mapped.u32");
        expect(eq(gray_mapped.size(), expected_gray.size())) << "Gray mapped size";
        for (std::size_t i = 0; i < gray_mapped.size(); i++) {
            expect(eq(gray_mapped[i], expected_gray[i])) << "GrayDemap[" << i << "]";
        }

        // Step 7: Modulate -> IQ (with same zero padding as GR3 test vector generator)
        constexpr uint32_t ZERO_PAD = N * OS_FACTOR * 5; // 5 symbol durations of silence
        auto iq = modulate_frame(gray_mapped, SF, OS_FACTOR,
                                 SYNC_WORD, PREAMBLE_LEN, ZERO_PAD);
        auto expected_iq = load_cf32("tx_07_iq_frame.cf32");

        expect(eq(iq.size(), expected_iq.size()))
            << "IQ frame size: got " << iq.size()
            << " expected " << expected_iq.size();

        // Compare IQ samples with tolerance (floating point)
        // First check preamble+sync+SFD (symbol-by-symbol)
        std::size_t preamble_end = static_cast<std::size_t>((PREAMBLE_LEN + 4.25) * SPS);
        {
            float max_err_pre = 0.f;
            for (std::size_t i = 0; i < preamble_end && i < iq.size() && i < expected_iq.size(); i++) {
                float err = std::max(std::abs(iq[i].real() - expected_iq[i].real()),
                                     std::abs(iq[i].imag() - expected_iq[i].imag()));
                if (err > max_err_pre) max_err_pre = err;
            }
            expect(lt(max_err_pre, 1e-4f))
                << "Preamble+SFD max error: " << max_err_pre;
        }

        // Check each payload symbol individually
        auto gray_expected = load_u32("tx_06_gray_mapped.u32");
        std::size_t n_payload_syms = gray_expected.size();
        std::size_t first_bad_sym = SIZE_MAX;
        for (std::size_t s = 0; s < n_payload_syms; s++) {
            std::size_t sym_start = preamble_end + s * SPS;
            float sym_max_err = 0.f;
            for (std::size_t j = 0; j < SPS && sym_start + j < iq.size() && sym_start + j < expected_iq.size(); j++) {
                float err = std::max(
                    std::abs(iq[sym_start + j].real() - expected_iq[sym_start + j].real()),
                    std::abs(iq[sym_start + j].imag() - expected_iq[sym_start + j].imag()));
                if (err > sym_max_err) sym_max_err = err;
            }
            if (sym_max_err > 1e-4f && first_bad_sym == SIZE_MAX) {
                first_bad_sym = s;
                // Print the first few samples of mismatching symbol
                std::size_t sym_start2 = preamble_end + s * SPS;
                std::printf("Symbol %zu (value=%u) max_err=%.6f\n",
                    s, gray_mapped[s], sym_max_err);
                std::printf("  GR4[0]: (%.6f, %.6f)  GR3[0]: (%.6f, %.6f)\n",
                    iq[sym_start2].real(), iq[sym_start2].imag(),
                    expected_iq[sym_start2].real(), expected_iq[sym_start2].imag());
            }
            expect(lt(sym_max_err, 1e-4f))
                << "Payload symbol " << s << " (value=" << gray_mapped[s]
                << ") max error: " << sym_max_err;
        }
    };
};

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
        auto encoded     = hamming_enc_frame(with_crc, SF, CR);
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
            for (int j = 0; j < cw_len; j++) {
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
            computed_crc ^= static_cast<uint16_t>(decoded_bytes[hdr.payload_len - 2]) << 8;
            uint16_t received_crc = static_cast<uint16_t>(decoded_bytes[hdr.payload_len])
                                  | (static_cast<uint16_t>(decoded_bytes[hdr.payload_len + 1]) << 8);
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

int main() { /* boost::ut auto-runs all suites */ }
