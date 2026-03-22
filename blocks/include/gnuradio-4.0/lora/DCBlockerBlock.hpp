// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_DC_BLOCKER_BLOCK_HPP
#define GNURADIO_LORA_DC_BLOCKER_BLOCK_HPP

#include <complex>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

#include <gnuradio-4.0/lora/algorithm/DCBlocker.hpp>

namespace gr::lora {

// GR4 block wrapper for the DCBlocker algorithm.
//
// Inserts as a 1:1 cf32 passthrough with optional DC spur removal.
// When disabled, copies input to output with zero overhead.
// Forwards all input tags (including application-specific tags like sf, cfo_int, etc.)
// via NoDefaultTagForwarding + manual publishTag.
GR_REGISTER_BLOCK("gr::lora::DCBlockerBlock", gr::lora::DCBlockerBlock)
struct DCBlockerBlock : gr::Block<DCBlockerBlock, gr::NoDefaultTagForwarding> {
    using Description = Doc<"2nd-order Butterworth DC blocker — removes DC spur from IQ stream">;

    gr::PortIn<std::complex<float>>  in;
    gr::PortOut<std::complex<float>> out;

    Annotated<bool, "dc_blocker_enabled", Visible, Doc<"enable DC spur removal">>
        dc_blocker_enabled = true;
    Annotated<float, "dc_blocker_cutoff", Unit<"Hz">, Visible, Doc<"high-pass cutoff frequency">>
        dc_blocker_cutoff = 1000.f;
    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"input sample rate">>
        sample_rate = 250'000.f;

    GR_MAKE_REFLECTABLE(DCBlockerBlock, in, out, dc_blocker_enabled, dc_blocker_cutoff, sample_rate);

    DCBlocker _dc;

    void start() { rebuildFilter(); }

    void settingsChanged(const gr::property_map& /*oldSettings*/,
                         const gr::property_map& newSettings) {
        if (newSettings.contains("dc_blocker_enabled")
            || newSettings.contains("dc_blocker_cutoff")
            || newSettings.contains("sample_rate")) {
            rebuildFilter();
        }
    }

    gr::work::Status processBulk(gr::InputSpanLike auto& input,
                                 gr::OutputSpanLike auto& output) noexcept {
        auto inSpan  = std::span(input);
        auto outSpan = std::span(output);
        auto n = std::min(inSpan.size(), outSpan.size());

        // forward all tags unchanged
        if (this->inputTagsPresent()) {
            this->publishTag(this->mergedInputTag().map, 0UZ);
        }

        if (dc_blocker_enabled && _dc.initialised()) {
            _dc.processBlock(std::span<const cf32>(inSpan.data(), n),
                             std::span<cf32>(outSpan.data(), n));
        } else {
            std::copy_n(inSpan.begin(), n, outSpan.begin());
        }

        std::ignore = input.consume(n);
        output.publish(n);
        return gr::work::Status::OK;
    }

    void rebuildFilter() {
        if (dc_blocker_enabled && sample_rate > 0.f && dc_blocker_cutoff > 0.f) {
            _dc.init(sample_rate, dc_blocker_cutoff);
        }
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_DC_BLOCKER_BLOCK_HPP
