// SPDX-License-Identifier: ISC
//
// SpectrumTap — shared state for periodic spectrum output.
//
// MultiSfDecoder writes raw IQ into a ring buffer during DETECT state.
// The main loop polls for new FFT results and broadcasts as CBOR.

#ifndef GNURADIO_LORA_ALGORITHM_SPECTRUM_TAP_HPP
#define GNURADIO_LORA_ALGORITHM_SPECTRUM_TAP_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <mutex>
#include <numbers>
#include <vector>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>

namespace gr::lora {

struct SpectrumState {
    static constexpr uint32_t kDefaultFFTSize = 1024;

    /// Configuration (set once before start, read-only after).
    uint32_t fft_size    = kDefaultFFTSize;
    float    sample_rate = 250000.f;
    double   center_freq = 869618000.0;

    /// Ring buffer for raw IQ samples (written by MultiSfDecoder, lock-free).
    std::vector<std::complex<float>> ring;
    std::atomic<uint64_t>            write_pos{0};

    /// FFT output (written by compute(), read by main loop under lock).
    std::mutex         result_mutex;
    std::vector<float> magnitude_db; ///< fft_size bins, dBFS, DC-centered
    std::atomic<bool>  result_ready{false};
    uint64_t           last_compute_pos = 0;

    /// FFT engine + scratch (used only by compute()).
    gr::algorithm::FFT<std::complex<float>> fft;
    std::vector<float>                      window;
    std::vector<std::complex<float>>        windowed;

    void init() {
        // 64× fft_size gives ~16ms headroom at 16 MS/s, enough for FFT copy + scheduler jitter
        static constexpr uint32_t kRingMultiplier = 64;
        ring.resize(fft_size * kRingMultiplier, {0.f, 0.f});
        magnitude_db.resize(fft_size, -120.f);
        windowed.resize(fft_size);
        fft = gr::algorithm::FFT<std::complex<float>>{};

        // Pre-compute Hann window
        window.resize(fft_size);
        for (uint32_t i = 0; i < fft_size; i++) {
            window[i] = 0.5f * (1.f - std::cos(2.f * std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(fft_size)));
        }
    }

    /// Write IQ samples into ring buffer (called from MultiSfDecoder graph thread).
    void push(const std::complex<float>* data, std::size_t count) {
        auto ring_size = ring.size();
        auto pos       = write_pos.load(std::memory_order_relaxed);
        for (std::size_t i = 0; i < count; i++) {
            ring[(pos + i) % ring_size] = data[i];
        }
        write_pos.store(pos + count, std::memory_order_release);
    }

    /// Compute FFT if enough new samples have arrived.
    /// Called from the main loop thread.  Returns true if new result available.
    bool compute() {
        auto pos = write_pos.load(std::memory_order_acquire);
        if (pos - last_compute_pos < fft_size) {
            return false;
        }

        // Read the last fft_size samples from ring buffer + apply window.
        // Epoch guard: re-check write_pos after reading to detect if push()
        // overwrote any of the samples we just read (ring wrap-around race).
        auto ring_size = ring.size();
        auto start     = pos - fft_size;
        for (uint32_t i = 0; i < fft_size; i++) {
            windowed[i] = ring[(start + i) % ring_size] * window[i];
        }
        auto pos_after = write_pos.load(std::memory_order_acquire);
        if (pos_after - start >= ring_size) {
            // Writer wrapped around during our read — data is stale, discard.
            last_compute_pos = pos_after;
            return false;
        }

        auto fft_out = fft.compute(std::span<const std::complex<float>>(windowed.data(), fft_size));

        // Compute dBFS with fftshift (DC in center)
        auto half = fft_size / 2;
        {
            std::lock_guard lock(result_mutex);
            for (uint32_t i = 0; i < fft_size; i++) {
                auto  src       = fft_out[(i + half) % fft_size];
                float mag_sq    = src.real() * src.real() + src.imag() * src.imag();
                float norm      = mag_sq / static_cast<float>(fft_size * fft_size);
                magnitude_db[i] = (norm > 1e-20f) ? 10.f * std::log10(norm) : -120.f;
            }
            result_ready.store(true, std::memory_order_release);
        }
        last_compute_pos = pos;
        return true;
    }
};

} // namespace gr::lora

#endif // GNURADIO_LORA_ALGORITHM_SPECTRUM_TAP_HPP
