// SPDX-License-Identifier: ISC
#include <boost/ut.hpp>

#include <cmath>
#include <complex>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>

#include <gnuradio-4.0/lora/algorithm/utilities.hpp>
#include <gnuradio-4.0/lora/algorithm/interleaving.hpp>

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

// MeshCore test configuration (must match test_vectors/config.json)
constexpr uint8_t  SF           = 8;
constexpr uint32_t N            = 1u << SF; // 256
constexpr uint8_t  OS_FACTOR    = 4;
constexpr uint16_t PREAMBLE_LEN = 8;

} // namespace

// ---- Interleaver Algorithm Tests (no scheduler) ----

const boost::ut::suite<"Interleaver algorithm"> interleaver_algo_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    // NOTE: interleave vs test vector test is in qa_lora_algorithms.cpp

    "Interleave → Deinterleave roundtrip (algorithm-level)"_test = [] {
        auto encoded    = load_u8("tx_04_encoded.u8");
        auto interleaved = load_u32("tx_05_interleaved.u32");

        // Deinterleave the header block (first 8 symbols → 6 codewords).
        // The interleaved symbols are SF bits wide (with parity+zero in MSBs).
        // In the real RX chain, FFT demod divides header symbols by 2^(SF-sf_app)
        // to strip parity/zero bits, producing sf_app-bit values.
        uint8_t sf_app = SF - 2; // header uses sf-2
        uint8_t cw_len = 8;      // header always cw_len=8

        std::vector<uint16_t> header_syms;
        for (std::size_t i = 0; i < cw_len && i < interleaved.size(); i++) {
            // Strip parity+zero bits (same as FFT demod divide-by-4 for header)
            uint16_t stripped = static_cast<uint16_t>(interleaved[i] >> (SF - sf_app));
            header_syms.push_back(stripped);
        }

        auto codewords = deinterleave_block(header_syms, SF, cw_len, sf_app);

        expect(eq(codewords.size(), std::size_t{sf_app}))
            << "Deinterleave header block size: " << codewords.size() << " expected " << sf_app;
        for (std::size_t i = 0; i < codewords.size() && i < sf_app; i++) {
            expect(eq(codewords[i], encoded[i]))
                << "Deinterleave header mismatch at index " << i;
        }
    };
};

// ---- FFTDemod Basic Tests ----

const boost::ut::suite<"FFTDemod block"> fft_demod_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "FFTDemod dechirps a known upchirp"_test = [] {
        // Create an upchirp with symbol value 42, dechirp + FFT should give argmax=42
        // (after +1 offset cancellation, i.e. argmax will be 43 and -1 gives 42)
        constexpr uint32_t test_symbol = 42;

        std::vector<std::complex<float>> chirp(N);
        build_upchirp(chirp.data(), test_symbol, SF, 1);

        // Build downchirp
        std::vector<std::complex<float>> downchirp(N);
        std::vector<std::complex<float>> upchirp_ref(N);
        build_ref_chirps(upchirp_ref.data(), downchirp.data(), SF, 1);

        // Dechirp
        std::vector<std::complex<float>> dechirped(N);
        for (uint32_t i = 0; i < N; i++) {
            dechirped[i] = chirp[i] * downchirp[i];
        }

        // FFT
        gr::algorithm::FFT<std::complex<float>> fft{};
        auto fft_out = fft.compute(dechirped);

        // Argmax
        float max_val = 0.f;
        uint32_t max_idx = 0;
        for (uint32_t i = 0; i < N; i++) {
            float mag_sq = fft_out[i].real() * fft_out[i].real() + fft_out[i].imag() * fft_out[i].imag();
            if (mag_sq > max_val) { max_val = mag_sq; max_idx = i; }
        }

        // Raw dechirp argmax should directly be the symbol value
        // (the -1 offset is only in the full RX chain after GrayDemap adds +1 on TX side)
        expect(eq(static_cast<uint16_t>(max_idx), static_cast<uint16_t>(test_symbol)))
            << "FFT dechirp: expected argmax " << test_symbol << " got " << max_idx;
    };

    "FFTDemod dechirps symbol 0"_test = [] {
        std::vector<std::complex<float>> chirp(N);
        build_upchirp(chirp.data(), 0, SF, 1);

        std::vector<std::complex<float>> downchirp(N);
        std::vector<std::complex<float>> upchirp_ref(N);
        build_ref_chirps(upchirp_ref.data(), downchirp.data(), SF, 1);

        std::vector<std::complex<float>> dechirped(N);
        for (uint32_t i = 0; i < N; i++) {
            dechirped[i] = chirp[i] * downchirp[i];
        }

        gr::algorithm::FFT<std::complex<float>> fft{};
        auto fft_out = fft.compute(dechirped);

        float max_val = 0.f;
        uint32_t max_idx = 0;
        for (uint32_t i = 0; i < N; i++) {
            float mag_sq = fft_out[i].real() * fft_out[i].real() + fft_out[i].imag() * fft_out[i].imag();
            if (mag_sq > max_val) { max_val = mag_sq; max_idx = i; }
        }

        expect(eq(static_cast<uint16_t>(max_idx), uint16_t{0}))
            << "FFT dechirp symbol 0: argmax=" << max_idx;
    };

    "FFTDemod dechirps all symbols"_test = [] {
        std::vector<std::complex<float>> downchirp(N);
        std::vector<std::complex<float>> upchirp_ref(N);
        build_ref_chirps(upchirp_ref.data(), downchirp.data(), SF, 1);

        gr::algorithm::FFT<std::complex<float>> fft{};

        uint32_t mismatches = 0;
        for (uint32_t sym = 0; sym < N; sym++) {
            std::vector<std::complex<float>> chirp(N);
            build_upchirp(chirp.data(), sym, SF, 1);

            std::vector<std::complex<float>> dechirped(N);
            for (uint32_t i = 0; i < N; i++) {
                dechirped[i] = chirp[i] * downchirp[i];
            }

            auto fft_out = fft.compute(dechirped);

            float max_val = 0.f;
            uint32_t max_idx = 0;
            for (uint32_t i = 0; i < N; i++) {
                float mag_sq = fft_out[i].real() * fft_out[i].real() + fft_out[i].imag() * fft_out[i].imag();
                if (mag_sq > max_val) { max_val = mag_sq; max_idx = i; }
            }

            if (max_idx != sym) mismatches++;
        }
        expect(eq(mismatches, uint32_t{0}))
            << "FFT dechirp all symbols: " << mismatches << " mismatches out of " << N;
    };
};

// ---- Modulate Algorithm Test (no scheduler) ----

const boost::ut::suite<"Modulate chirp generation"> modulate_chirp_tests = [] {
    using namespace boost::ut;
    using namespace gr::lora;

    "Upchirp generation is unit magnitude"_test = [] {
        std::vector<std::complex<float>> chirp(N * OS_FACTOR);
        build_upchirp(chirp.data(), 42, SF, OS_FACTOR);

        float max_err = 0.f;
        for (auto& c : chirp) {
            float mag = std::abs(c);
            float err = std::abs(mag - 1.0f);
            if (err > max_err) max_err = err;
        }
        expect(lt(max_err, 1e-5f))
            << "Upchirp magnitude error: " << max_err;
    };

    "Downchirp is conjugate of upchirp"_test = [] {
        std::vector<std::complex<float>> up(N * OS_FACTOR);
        std::vector<std::complex<float>> down(N * OS_FACTOR);
        build_ref_chirps(up.data(), down.data(), SF, OS_FACTOR);

        float max_err = 0.f;
        for (std::size_t i = 0; i < up.size(); i++) {
            auto expected = std::conj(up[i]);
            float err_re = std::abs(down[i].real() - expected.real());
            float err_im = std::abs(down[i].imag() - expected.imag());
            float err = std::max(err_re, err_im);
            if (err > max_err) max_err = err;
        }
        expect(lt(max_err, 1e-6f))
            << "Downchirp != conj(upchirp), max error: " << max_err;
    };

    "IQ test vector exists and has correct size"_test = [] {
        auto iq = load_cf32("tx_07_iq_frame.cf32");
        // Expected: (preamble_len=8 + 2 sync + 2.25 SFD + payload_symbols) * sps
        // payload_symbols from tx_06_gray_mapped.u32
        auto gray_mapped = load_u32("tx_06_gray_mapped.u32");
        uint32_t sps = N * OS_FACTOR;
        std::size_t expected_min = static_cast<std::size_t>((PREAMBLE_LEN + 4.25) * sps)
                                 + gray_mapped.size() * sps;
        expect(gt(iq.size(), std::size_t{0})) << "IQ test vector is empty";
        expect(ge(iq.size(), expected_min))
            << "IQ test vector too small: " << iq.size() << " < " << expected_min;
    };
};

int main() { /* boost::ut auto-runs all suites */ }
