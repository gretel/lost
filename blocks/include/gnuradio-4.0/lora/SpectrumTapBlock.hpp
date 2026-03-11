// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_SPECTRUM_TAP_BLOCK_HPP
#define GNURADIO_LORA_SPECTRUM_TAP_BLOCK_HPP

#include <algorithm>
#include <complex>
#include <memory>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/lora/algorithm/SpectrumTap.hpp>

namespace gr::lora {

struct SpectrumTapBlock : gr::Block<SpectrumTapBlock> {
    using Description = Doc<"1:1 cf32 passthrough feeding a SpectrumState ring buffer.">;

    gr::PortIn<std::complex<float>>  in;
    gr::PortOut<std::complex<float>> out;

    GR_MAKE_REFLECTABLE(SpectrumTapBlock, in, out);

    std::shared_ptr<SpectrumState> _spectrum;

    gr::work::Status processBulk(gr::InputSpanLike auto& input,
                                 gr::OutputSpanLike auto& output) noexcept {
        auto inSpan  = std::span(input);
        auto outSpan = std::span(output);
        auto n = std::min(inSpan.size(), outSpan.size());

        if (_spectrum) {
            _spectrum->push(inSpan.data(), n);
        }

        std::copy_n(inSpan.begin(), n, outSpan.begin());
        std::ignore = input.consume(n);
        output.publish(n);
        return gr::work::Status::OK;
    }
};

} // namespace gr::lora

#endif // GNURADIO_LORA_SPECTRUM_TAP_BLOCK_HPP
