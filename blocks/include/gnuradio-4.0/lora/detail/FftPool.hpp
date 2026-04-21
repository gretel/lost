// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_DETAIL_FFT_POOL_HPP
#define GNURADIO_LORA_DETAIL_FFT_POOL_HPP

// FftPool: thread-local cache of reusable gr::algorithm::FFT<cf32> instances
// keyed by SF (FFT size = 2^sf). Avoids per-symbol FFT reconstruction and
// internal plan setup overhead on hot paths.
//
// Lifetime: the thread-local map lives for the duration of the owning
// thread. Callers hold references only transiently during a single
// dechirp+FFT round; they do NOT store references across calls.

#include <cstdint>
#include <unordered_map>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/phy/Types.hpp>

namespace gr::lora::detail {

class FftPool {
public:
    using Fft = gr::algorithm::FFT<phy::cf32>;

    /// Return a reference to the thread-local FFT for the given SF. The
    /// instance is created on first use and reused thereafter. Not thread-
    /// safe across threads (each thread has its own cache).
    [[nodiscard]] static Fft& acquire(uint8_t sf) {
        thread_local std::unordered_map<uint8_t, Fft> cache;
        auto                                          it = cache.find(sf);
        if (it == cache.end()) {
            it = cache.emplace(sf, Fft{}).first;
        }
        return it->second;
    }
};

} // namespace gr::lora::detail

#endif // GNURADIO_LORA_DETAIL_FFT_POOL_HPP
