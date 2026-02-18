// SPDX-License-Identifier: ISC
//
// RadioSource: GR4 source block that reads IQ from an SDR device via
// a pure-C bridge, avoiding the dual-libc++ ABI crash on macOS.
//
// The bridge implementation (uhd_radio_bridge.c or soapy_radio_bridge.c)
// is compiled as pure C. This header is compiled with LLVM 19 (GR4
// toolchain) and calls only C functions across the ABI boundary.

#ifndef GR4_LORA_RADIO_SOURCE_HPP
#define GR4_LORA_RADIO_SOURCE_HPP

#include <complex>
#include <cstdint>
#include <cstdio>
#include <string>

#include <gnuradio-4.0/Block.hpp>

#include "radio_bridge.h"

namespace gr::lora {

struct RadioSource : public gr::Block<RadioSource> {
    using Description = Doc<R""(
SDR source block using a pure-C bridge to avoid libc++ ABI issues.
Produces complex float IQ samples from an SDR device (e.g. USRP B210).
)"">;

    gr::PortOut<std::complex<float>> out;

    // Device configuration (set before start, not runtime-changeable)
    gr::Annotated<std::string, "device args", gr::Visible>     device_args{""};
    gr::Annotated<std::string, "clock source", gr::Visible>    clock_source{""};
    gr::Annotated<float, "sample rate", gr::Unit<"Hz">, gr::Visible> sample_rate{250000.f};
    gr::Annotated<double, "center frequency", gr::Unit<"Hz">, gr::Visible> center_freq{869618000.0};
    gr::Annotated<double, "gain", gr::Unit<"dB">, gr::Visible> gain{30.0};
    gr::Annotated<double, "bandwidth", gr::Unit<"Hz">, gr::Visible> bandwidth{0.0};
    gr::Annotated<uint32_t, "RX channel index", gr::Visible> channel{0U};
    gr::Annotated<uint32_t, "max chunk size"> max_chunk_size{8192U};

    GR_MAKE_REFLECTABLE(RadioSource, out, device_args, clock_source,
                        sample_rate, center_freq, gain, bandwidth, channel,
                        max_chunk_size);

    radio_bridge_t* _bridge{nullptr};
    bool _skip_free{false};  ///< Set true before stop() to skip device free

    void start() {
        radio_bridge_config_t config{};
        config.device_args  = device_args.value.c_str();
        config.clock_source = clock_source.value.empty() ? nullptr : clock_source.value.c_str();
        config.sample_rate  = static_cast<double>(sample_rate.value);
        config.center_freq  = center_freq.value;
        config.gain_db      = gain.value;
        config.bandwidth    = bandwidth.value;
        config.channel      = channel.value;
        config.antenna      = nullptr;
        config.direction    = RADIO_BRIDGE_RX;

        _bridge = radio_bridge_create(&config);
        if (!_bridge) {
            throw gr::exception(
                std::format("RadioSource: failed to create device: {}",
                            radio_bridge_last_error()));
        }

        std::fprintf(stderr, "[RadioSource] Device ready: ch=%u rate=%.0f freq=%.0f gain=%.1f\n",
                     channel.value,
                     radio_bridge_get_sample_rate(_bridge),
                     radio_bridge_get_center_freq(_bridge),
                     radio_bridge_get_gain(_bridge));
    }

    void stop() {
        if (_bridge) {
            radio_bridge_destroy_ex(_bridge, _skip_free ? 1 : 0);
        }
        _bridge = nullptr;
    }

    work::Status processBulk(OutputSpanLike auto& output) {
        if (!_bridge) {
            output.publish(0UZ);
            return work::Status::ERROR;
        }

        const auto maxSamples = std::min(
            static_cast<std::size_t>(max_chunk_size.value), output.size());

        auto* buf = reinterpret_cast<float*>(output.data());
        int ret = radio_bridge_read(_bridge, buf, maxSamples, 0.1);

        if (ret > 0) {
            output.publish(static_cast<std::size_t>(ret));
            return work::Status::OK;
        }

        // Timeout or overflow: publish nothing, keep running.
        output.publish(0UZ);
        return work::Status::OK;
    }
};

}  // namespace gr::lora

#endif  // GR4_LORA_RADIO_SOURCE_HPP
