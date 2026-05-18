// SPDX-License-Identifier: ISC
// Generate implicit-header LoRa IQ samples and transmit via Pluto IIO TX.
// Cross-build via cmake with libiio, deploy to tezuka or use from macOS:
//
//   tx_implicit [--uri URI] [--freq HZ] [--gain DB] [--loop]
//
//   --uri URI    IIO context URI (default: local:)
//                local:  = on-device Pluto (tezuka)
//                ip:HOST = remote Pluto via network
//   --freq HZ    TX LO frequency in Hz (default: 869618000)
//   --gain DB    TX gain in dB, 0-89 (default: 89)
//   --loop       repeat frame continuously until Ctrl-C
//
// Examples:
//   tx_implicit                           # one-shot, local device
//   tx_implicit --uri ip:10.0.23.149      # one-shot, remote Pluto
//   tx_implicit --loop --freq 868000000   # continuous, custom freq

#include <algorithm>
#include <cerrno>
#include <cfenv>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/lora/algorithm/tx_chain.hpp>

#include <iio.h>

namespace {
    constexpr float SCALE = 20000.0f;

    // Pluto TX device
    constexpr auto TX_DEVICE = "cf-ad9361-dds-core-lpc";

    // TX PHY defaults (MeshCore 868 MHz)
    constexpr long long TX_LO_FREQ  = 869618000LL; // Hz
    constexpr long long TX_GAIN_DB  = 89LL;        // dB (max)

    // 28-byte payload matching config-tezuka-tx-test.toml implicit_pay_len
    const std::vector<uint8_t> PAYLOAD(28, 0x42);

    [[nodiscard]] std::vector<std::complex<float>> generate_frame() {
        return gr::lora::generate_frame_iq(
            PAYLOAD,    // payload bytes
            8,          // SF
            1,          // CR 4/5 (cr=1)
            40,         // OS factor (BW62.5k @ 2.5MS/s = 40)
            0x12,       // sync_word
            16,         // preamble_len
            true,       // has_crc
            0,          // zero_pad
            2,          // ldro_mode (auto)
            62500,      // bandwidth
            false,      // inverted_iq
            true        // implicit_header ← KEY: skip explicit header
        );
    }

    // Convert cf32 to S16 interleaved (Pluto TX format le:S16/16>>0)
    [[nodiscard]] std::vector<int16_t> to_s16(std::span<const std::complex<float>> iq) {
        std::vector<int16_t> out;
        out.reserve(iq.size() * 2);
        for (auto& s : iq) {
            out.push_back(static_cast<int16_t>(std::clamp(s.real(), -1.0f, 1.0f) * SCALE));
            out.push_back(static_cast<int16_t>(std::clamp(s.imag(), -1.0f, 1.0f) * SCALE));
        }
        return out;
    }

    // Configure AD9361 PHY for TX: frequency, power-up LO, set gain.
    // Without this, TX LO stays powered down (lora_trx enable_tx=false).
    bool configure_phy(iio_context* ctx, long long freq_hz, long long gain_db) {
        auto* phy = iio_context_find_device(ctx, "ad9361-phy");
        if (!phy) {
            std::fprintf(stderr, "tx_implicit: ad9361-phy not found\n");
            return false;
        }

        // TX LO (altvoltage1 = TX synthesizer)
        auto* tx_lo = iio_device_find_channel(phy, "altvoltage1", true);
        if (!tx_lo) {
            std::fprintf(stderr, "tx_implicit: altvoltage1 channel not found\n");
            return false;
        }
        iio_channel_attr_write_longlong(tx_lo, "frequency", freq_hz);
        iio_channel_attr_write(tx_lo, "powerdown", "0");
        std::fprintf(stderr, "tx_implicit: TX LO freq=%lld Hz, powerdown=0\n", freq_hz);

        // TX gain (voltage0 output = TX1)
        auto* tx_gain_ch = iio_device_find_channel(phy, "voltage0", true);
        if (!tx_gain_ch) {
            std::fprintf(stderr, "tx_implicit: TX gain channel not found\n");
            return false;
        }
        iio_channel_attr_write_longlong(tx_gain_ch, "hardwaregain", gain_db);
        std::fprintf(stderr, "tx_implicit: TX gain=%lld dB\n", gain_db);

        return true;
    }

    bool transmit_frame(iio_context* ctx, std::span<const int16_t> samples, int loop, long long freq_hz, long long gain_db) {
        // Configure AD9361 PHY before touching DDS device
        if (!configure_phy(ctx, freq_hz, gain_db)) {
            return false;
        }

        auto* dev = iio_context_find_device(ctx, TX_DEVICE);
        if (!dev) {
            std::fprintf(stderr, "tx_implicit: device '%s' not found\n", TX_DEVICE);
            return false;
        }

        // Enable TX I+Q channels before creating buffer (libiio requirement)
        auto* ch_i = iio_device_find_channel(dev, "voltage0", true);
        auto* ch_q = iio_device_find_channel(dev, "voltage1", true);
        if (!ch_i || !ch_q) {
            std::fprintf(stderr, "tx_implicit: TX channels not found\n");
            return false;
        }
        iio_channel_enable(ch_i);
        iio_channel_enable(ch_q);

        iio_device_set_kernel_buffers_count(dev, 4);

        const auto nbsamples = static_cast<std::size_t>(samples.size() / 2);
        auto* buf = iio_device_create_buffer(dev, nbsamples, false);
        if (!buf) {
            std::fprintf(stderr, "tx_implicit: create_buffer(%zu) failed: %s\n", nbsamples, std::strerror(errno));
            return false;
        }

        for (int i = 0; i < loop; ++i) {
            std::memcpy(iio_buffer_start(buf), samples.data(), static_cast<std::size_t>(samples.size()) * sizeof(int16_t));
            ssize_t ret = iio_buffer_push(buf);
            if (ret < 0) {
                std::fprintf(stderr, "tx_implicit: push failed: %s\n", std::strerror(static_cast<int>(-ret)));
                iio_buffer_destroy(buf);
                return false;
            }
            if (loop > 1) {
                std::fprintf(stderr, "tx_implicit: frame %d/%d\r", i + 1, loop);
            }
        }

        iio_buffer_destroy(buf);
        return true;
    }
}

int main(int argc, char** argv) {
    // Defaults
    const char* uri    = "local:";
    long long   freq   = TX_LO_FREQ;
    long long   gain   = TX_GAIN_DB;
    int         loop   = 1;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--loop") == 0) {
            loop = -1;
        } else if (std::strcmp(argv[i], "--uri") == 0 && i + 1 < argc) {
            uri = argv[++i];
        } else if (std::strcmp(argv[i], "--freq") == 0 && i + 1 < argc) {
            freq = std::atoll(argv[++i]);
        } else if (std::strcmp(argv[i], "--gain") == 0 && i + 1 < argc) {
            gain = std::atoll(argv[++i]);
        } else {
            std::fprintf(stderr, "usage: tx_implicit [--uri URI] [--freq HZ] [--gain DB] [--loop]\n");
            return 1;
        }
    }

    auto cf32 = generate_frame();
    auto s16  = to_s16(cf32);

    std::fprintf(stderr, "tx_implicit: frame %zu cf32 samples -> %zu S16 bytes (uri=%s freq=%lld gain=%lld)\n",
        cf32.size(), s16.size() * sizeof(int16_t), uri, freq, gain);

    // Create IIO context: local: on-device, ip:HOST for remote.
    iio_context* ctx = nullptr;
    if (std::strcmp(uri, "local:") == 0) {
        ctx = iio_create_local_context();
    } else {
        ctx = iio_create_context_from_uri(uri);
    }
    if (!ctx) {
        std::fprintf(stderr, "tx_implicit: iio_create_context(%s) failed\n", uri);
        return 1;
    }

    bool ok = transmit_frame(ctx, s16, loop, freq, gain);
    iio_context_destroy(ctx);

    if (!ok) {
        return 1;
    }
    std::fprintf(stderr, "tx_implicit: done\n");
    return 0;
}
