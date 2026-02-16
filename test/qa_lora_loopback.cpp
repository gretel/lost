// SPDX-License-Identifier: ISC
/// TX->RX loopback test (algorithm-level only).
/// Per-stage TX tests are in qa_lora_tx.cpp.

#include <boost/ut.hpp>

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
constexpr bool     HAS_CRC      = true;

} // namespace

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
