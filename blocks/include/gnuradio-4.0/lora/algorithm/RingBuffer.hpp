// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_ALGORITHM_RING_BUFFER_HPP
#define GNURADIO_LORA_ALGORITHM_RING_BUFFER_HPP

// Circular IQ ring buffer with bulk push and windowed extraction.
// Extracted from ScanController for reuse by WidebandDecoder.

#include <algorithm>
#include <complex>
#include <cstddef>
#include <span>
#include <vector>

namespace gr::lora {

using cf32 = std::complex<float>;

struct RingBuffer {
    std::vector<cf32> data;
    std::size_t       writeIdx{0};
    std::size_t       count{0};    // total samples written (monotonic)

    void resize(std::size_t capacity) {
        data.assign(capacity, {0.f, 0.f});
        writeIdx = 0;
        count    = 0;
    }

    void push(std::span<const cf32> samples) noexcept {
        const auto cap = data.size();
        const auto n   = samples.size();
        if (n == 0 || cap == 0) return;
        const auto firstChunk = std::min(n, cap - writeIdx);
        std::copy_n(samples.data(), firstChunk, data.data() + writeIdx);
        if (firstChunk < n) {
            std::copy_n(samples.data() + firstChunk, n - firstChunk, data.data());
        }
        writeIdx = (writeIdx + n) % cap;
        count += n;
    }

    /// Zero-copy view of the most recent N samples as 1 or 2 contiguous spans.
    /// When the data wraps around the circular boundary, two spans are returned;
    /// otherwise the second span is empty.  The spans point into the ring buffer
    /// — callers must NOT modify the data through them.
    [[nodiscard]] std::pair<std::span<const cf32>, std::span<const cf32>>
    recentView(std::size_t n) const noexcept {
        n = std::min(n, count);
        if (n == 0 || data.empty()) return {{}, {}};

        const auto cap = data.size();
        const std::size_t start = (writeIdx + cap - n) % cap;
        if (start + n <= cap) {
            // No wrap — single contiguous span
            return {std::span<const cf32>(data.data() + start, n), {}};
        }
        // Wraps around — two spans
        const std::size_t firstLen = cap - start;
        return {
            std::span<const cf32>(data.data() + start, firstLen),
            std::span<const cf32>(data.data(), n - firstLen)
        };
    }

    [[nodiscard]] std::vector<cf32> recent(std::size_t n) const {
        if (count < n || n > data.size()) return {};
        std::vector<cf32> out(n);
        const auto        cap   = data.size();
        const std::size_t start = (writeIdx + cap - n) % cap;
        const auto firstChunk = std::min(n, cap - start);
        std::copy_n(data.data() + start, firstChunk, out.data());
        if (firstChunk < n) {
            std::copy_n(data.data(), n - firstChunk, out.data() + firstChunk);
        }
        return out;
    }

    void reset() noexcept {
        writeIdx = 0;
        count    = 0;
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_ALGORITHM_RING_BUFFER_HPP
