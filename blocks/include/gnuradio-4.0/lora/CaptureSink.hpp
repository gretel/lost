// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_CAPTURE_SINK_HPP
#define GNURADIO_LORA_CAPTURE_SINK_HPP

#include <atomic>
#include <complex>
#include <cstdint>
#include <memory>
#include <vector>

#include <gnuradio-4.0/Block.hpp>

namespace gr::lora {

using cf32 = std::complex<float>;

/// Shared state between CaptureSink (graph thread) and scan orchestrator.
///
/// Protocol:
///   1. Orchestrator calls request(N) — sets request_count, resets counters.
///   2. CaptureSink copies incoming samples into buffer until N are captured.
///   3. CaptureSink sets done=true.
///   4. Orchestrator polls done, reads buffer[0..N-1], calls reset().
struct CaptureState {
    std::vector<cf32>     buffer;
    std::atomic<uint32_t> request_count{0};
    std::atomic<uint32_t> captured{0};
    std::atomic<bool>     done{false};

    /// Start a capture of exactly `count` samples.
    /// Caller must ensure buffer.size() >= count.
    void request(uint32_t count) {
        captured.store(0, std::memory_order_relaxed);
        done.store(false, std::memory_order_release);
        request_count.store(count, std::memory_order_release);
    }

    /// Reset to idle state.  Call after reading the captured data.
    void reset() {
        request_count.store(0, std::memory_order_release);
    }
};

/// On-demand cf32 capture sink for the scan orchestrator.
///
/// Normally discards all input (pure sink, no backpressure).
/// When CaptureState::request_count > 0, copies samples into
/// the shared buffer until the requested count is reached.
struct CaptureSink : gr::Block<CaptureSink> {
    using Description = Doc<"On-demand cf32 capture sink for scan orchestrator">;

    gr::PortIn<cf32> in;

    GR_MAKE_REFLECTABLE(CaptureSink, in);

    std::shared_ptr<CaptureState> _state;

    gr::work::Status processBulk(gr::InputSpanLike auto& input) noexcept {
        auto inSpan = std::span(input);
        const auto n = inSpan.size();

        if (_state) {
            auto req = _state->request_count.load(std::memory_order_acquire);
            auto cap = _state->captured.load(std::memory_order_relaxed);

            if (req > 0 && cap < req) {
                auto remaining = req - cap;
                auto toCopy = std::min(static_cast<uint32_t>(n), remaining);
                std::copy_n(inSpan.begin(), toCopy,
                            _state->buffer.begin()
                                + static_cast<std::ptrdiff_t>(cap));
                cap += toCopy;
                _state->captured.store(cap, std::memory_order_release);
                if (cap >= req) {
                    _state->done.store(true, std::memory_order_release);
                }
            }
        }

        std::ignore = input.consume(n);
        return gr::work::Status::OK;
    }
};

} // namespace gr::lora

#endif // GNURADIO_LORA_CAPTURE_SINK_HPP
