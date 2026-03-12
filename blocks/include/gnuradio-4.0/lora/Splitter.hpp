// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_SPLITTER_HPP
#define GNURADIO_LORA_SPLITTER_HPP

#include <algorithm>
#include <complex>
#include <cstdint>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

namespace gr::lora {

// 1-to-N signal splitter: copies input samples and tags to all outputs.
//
// Used to fan out a single radio channel to multiple decode chains (e.g.
// different SF or sync word configurations). When n_outputs=1, the block
// is a trivial passthrough with no copy overhead.
GR_REGISTER_BLOCK("gr::lora::Splitter", gr::lora::Splitter)
struct Splitter : gr::Block<Splitter, gr::NoDefaultTagForwarding> {
    using Description = Doc<"1-to-N signal splitter — copies samples and tags to all outputs">;

    gr::PortIn<std::complex<float>>                    in;
    std::vector<gr::PortOut<std::complex<float>>>       out{};

    Annotated<gr::Size_t, "n_outputs", gr::Visible, gr::Limits<1U, 8U>> n_outputs = 2U;
    bool debug = false;

    GR_MAKE_REFLECTABLE(Splitter, in, out, n_outputs, debug);

    void start() {
        out.resize(n_outputs);
    }

    void settingsChanged(const gr::property_map& /*oldSettings*/,
                         const gr::property_map& newSettings) {
        if (newSettings.contains("n_outputs")) {
            out.resize(n_outputs);
        }
    }

    template<gr::InputSpanLike TInSpan, gr::OutputSpanLike TOutSpan>
    gr::work::Status processBulk(TInSpan& inSpan, std::span<TOutSpan>& outs) {
        const auto inData = std::span(inSpan);
        const auto nSamples = inData.size();

        // Forward tags to all outputs
        if (this->inputTagsPresent()) {
            const auto& tag = this->mergedInputTag();
            for (std::size_t i = 0; i < outs.size(); i++) {
                outs[i].publishTag(tag.map, 0UZ);
            }
        }

        // Copy samples to all outputs
        for (std::size_t i = 0; i < outs.size(); i++) {
            auto outData = std::span(outs[i]);
            const auto nCopy = std::min(nSamples, outData.size());
            std::copy_n(inData.begin(), nCopy, outData.begin());
            outs[i].publish(nCopy);
        }

        std::ignore = inSpan.consume(nSamples);
        return gr::work::Status::OK;
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_SPLITTER_HPP
