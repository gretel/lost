// SPDX-License-Identifier: ISC
/// Shared test helpers: file loaders, config constants, vector comparison.
/// Included by all qa_lora_*.cpp test suites.
#pragma once

#include <complex>
#include <cstdint>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <string>
#include <vector>

#include <boost/ut.hpp>

namespace gr::lora::test {

// ---- Test vector file loaders ----

inline const std::filesystem::path& testVectorDir() {
    static const std::filesystem::path dir = TEST_VECTORS_DIR;
    return dir;
}

inline std::vector<uint8_t> load_u8(const std::string& filename) {
    auto path = testVectorDir() / filename;
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open test vector: " + path.string());
    return {std::istreambuf_iterator<char>(f), {}};
}

inline std::vector<uint32_t> load_u32(const std::string& filename) {
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

inline std::vector<std::complex<float>> load_cf32(const std::string& filename) {
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

inline std::string load_text(const std::string& filename) {
    auto path = testVectorDir() / filename;
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open test vector: " + path.string());
    return {std::istreambuf_iterator<char>(f), {}};
}

/// Compare two vectors element-by-element using boost::ut assertions.
template<typename T>
void expect_vectors_equal(const std::vector<T>& actual, const std::vector<T>& expected,
                          const std::string& label) {
    using boost::ut::eq;      // NOLINT(build/namespaces)
    using boost::ut::expect;  // NOLINT(build/namespaces)
    expect(eq(actual.size(), expected.size())) << label << " size mismatch";
    for (std::size_t i = 0; i < actual.size() && i < expected.size(); i++) {
        expect(eq(actual[i], expected[i])) << label << " mismatch at index " << i;
    }
}

// ---- Default LoRa test configuration ----
// SF8 / BW62.5k / CR4/8, must match test_vectors/config.json.

constexpr uint8_t  SF           = 8;
constexpr uint32_t N            = 1u << SF;         // 256
constexpr uint8_t  CR           = 4;
constexpr uint32_t BW           = 62500;
constexpr uint8_t  OS_FACTOR    = 4;
constexpr uint16_t SYNC_WORD    = 0x12;
constexpr uint16_t PREAMBLE_LEN = 8;
constexpr uint32_t CENTER_FREQ  = 866000000;
constexpr bool     HAS_CRC      = true;
constexpr uint32_t SPS          = N * OS_FACTOR;     // 1024

}  // namespace gr::lora::test
