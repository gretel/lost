// SPDX-License-Identifier: ISC

// Phase 6 unit tests: AntennaCombiner pass-through scaffold.
//
// For N_ant=1, combine() must be bit-identical to memcpy. The multi-
// antenna path is explicitly NOT tested here because it's not implemented
// yet; a separate test file will cover MALoRa once real multi-antenna
// hardware is in play.

#include <boost/ut.hpp>

#include <gnuradio-4.0/lora/phy/AntennaCombiner.hpp>

#include <array>
#include <complex>
#include <cstring>
#include <span>
#include <vector>

using namespace boost::ut;
using gr::lora::phy::AntennaCombiner;
using cf32 = std::complex<float>;

namespace {

suite<"AntennaCombiner N=1"> _passthrough = [] {
    "128 sample memcpy"_test = [] {
        AntennaCombiner c;
        c.init({7, 125000, 1});

        std::vector<cf32> in(128);
        for (std::size_t i = 0; i < in.size(); ++i) {
            in[i] = cf32{static_cast<float>(i), static_cast<float>(i) * 0.5f};
        }

        std::vector<cf32>                    out(128);
        std::array<std::span<const cf32>, 1> ants{std::span<const cf32>(in)};
        c.combine(std::span<const std::span<const cf32>>(ants), std::span<cf32>(out));

        for (std::size_t i = 0; i < in.size(); ++i) {
            expect(out[i] == in[i]);
        }
    };

    "zero length span no crash"_test = [] {
        AntennaCombiner c;
        c.init({7, 125000, 1});

        std::vector<cf32>                    empty;
        std::array<std::span<const cf32>, 1> ants{std::span<const cf32>(empty)};
        std::vector<cf32>                    out;
        c.combine(std::span<const std::span<const cf32>>(ants), std::span<cf32>(out));
        // (no assertion other than no crash)
    };

    "empty antenna list no crash"_test = [] {
        AntennaCombiner c;
        c.init({7, 125000, 1});

        std::array<std::span<const cf32>, 0> ants{};
        std::vector<cf32>                    out(16);
        c.combine(std::span<const std::span<const cf32>>(ants), std::span<cf32>(out));
        // (no assertion — out may be untouched)
    };

    "truncates to out.size"_test = [] {
        // If in is larger than out, combine() truncates. Verifies the
        // min() guard path.
        AntennaCombiner c;
        c.init({7, 125000, 1});

        std::vector<cf32>                    in(256, cf32{1.f, 0.f});
        std::vector<cf32>                    out(128);
        std::array<std::span<const cf32>, 1> ants{std::span<const cf32>(in)};
        c.combine(std::span<const std::span<const cf32>>(ants), std::span<cf32>(out));
        for (auto v : out) {
            expect(v == cf32{1.f, 0.f});
        }
    };

    "config reports n_antennas"_test = [] {
        AntennaCombiner c;
        c.init({7, 125000, 1});
        expect(c.n_antennas() == 1_u);
    };
};

} // namespace

int main() { return 0; }
