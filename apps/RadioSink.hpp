// SPDX-License-Identifier: ISC
//
// RadioSink: GR4 sink block that writes IQ to an SDR device via a
// pure-C bridge, avoiding the dual-libc++ ABI crash on macOS.
//
// Accepts complex float samples from the GR4 graph and writes them
// to the SDR hardware via radio_bridge_write().

#ifndef GR4_LORA_RADIO_SINK_HPP
#define GR4_LORA_RADIO_SINK_HPP

#include <complex>
#include <cstdint>
#include <cstdio>
#include <string>

#include <gnuradio-4.0/Block.hpp>

#include "radio_bridge.h"

namespace gr::lora {

struct RadioSink : public gr::Block<RadioSink> {
    using Description = Doc<R""(
SDR sink block using a pure-C bridge to avoid libc++ ABI issues.
Consumes complex float IQ samples and writes them to an SDR device
(e.g. USRP B210) for transmission.
)"">;

    gr::PortIn<std::complex<float>> in;

    // Device configuration (set before start, not runtime-changeable)
    gr::Annotated<std::string, "device args", gr::Visible>     device_args{""};
    gr::Annotated<std::string, "clock source", gr::Visible>    clock_source{""};
    gr::Annotated<float, "sample rate", gr::Unit<"Hz">, gr::Visible> sample_rate{250000.f};
    gr::Annotated<double, "center frequency", gr::Unit<"Hz">, gr::Visible> center_freq{869618000.0};
    gr::Annotated<double, "gain", gr::Unit<"dB">, gr::Visible> gain{30.0};
    gr::Annotated<double, "bandwidth", gr::Unit<"Hz">, gr::Visible> bandwidth{0.0};
    gr::Annotated<uint32_t, "max chunk size"> max_chunk_size{8192U};

    GR_MAKE_REFLECTABLE(RadioSink, in, device_args, clock_source,
                        sample_rate, center_freq, gain, bandwidth,
                        max_chunk_size);

    radio_bridge_t* _bridge{nullptr};
    bool _skip_free{false};  ///< Set true before stop() to skip device free

    void start() {
        radio_bridge_config_t config{};
        config.device_args  = device_args.value.c_str();
        config.clock_source = clock_source.value.empty()
                              ? nullptr : clock_source.value.c_str();
        config.sample_rate  = static_cast<double>(sample_rate.value);
        config.center_freq  = center_freq.value;
        config.gain_db      = gain.value;
        config.bandwidth    = bandwidth.value;
        config.channel      = 0;
        config.antenna      = nullptr;
        config.direction    = RADIO_BRIDGE_TX;

        _bridge = radio_bridge_create(&config);
        if (!_bridge) {
            throw gr::exception(
                std::format("RadioSink: failed to create device: {}",
                            radio_bridge_last_error()));
        }

        std::fprintf(stderr, "[RadioSink] Device ready: rate=%.0f freq=%.0f gain=%.1f\n",
                     radio_bridge_get_sample_rate(_bridge),
                     radio_bridge_get_center_freq(_bridge),
                     radio_bridge_get_gain(_bridge));
    }

    void stop() {
        if (_bridge) {
            radio_bridge_destroy_ex(_bridge, _skip_free ? 1 : 0);
            _bridge = nullptr;
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& input) {
        if (!_bridge) {
            if (!input.consume(input.size())) {
                throw gr::exception("RadioSink: could not consume input");
            }
            return work::Status::ERROR;
        }

        const auto nSamples = std::min(
            static_cast<std::size_t>(max_chunk_size.value), input.size());

        if (nSamples == 0) {
            return work::Status::OK;
        }

        auto* buf = reinterpret_cast<const float*>(input.data());
        int ret = radio_bridge_write(_bridge, buf, nSamples, 0.1, 0);

        if (ret > 0) {
            if (!input.consume(static_cast<std::size_t>(ret))) {
                throw gr::exception("RadioSink: could not consume input");
            }
            return work::Status::OK;
        }

        // Timeout or underflow: consume nothing, retry on next work() call.
        return work::Status::OK;
    }
};

}  // namespace gr::lora

#endif  // GR4_LORA_RADIO_SINK_HPP
