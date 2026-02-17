// SPDX-License-Identifier: ISC
//
// SoapySink: GR4 sink block that writes IQ to a SoapySDR device via a
// pure-C bridge, avoiding the dual-libc++ ABI crash.
//
// Mirrors the SoapySource pattern but in the TX direction. Accepts
// complex float samples from the GR4 graph and writes them to the SDR
// hardware via soapy_bridge_write().

#ifndef GR4_LORA_SOAPY_SINK_HPP
#define GR4_LORA_SOAPY_SINK_HPP

#include <complex>
#include <cstdint>
#include <cstdio>
#include <string>

#include <gnuradio-4.0/Block.hpp>

#include "soapy_c_bridge.h"

namespace gr::lora {

struct SoapySink : public gr::Block<SoapySink> {
    using Description = Doc<R""(
SDR sink block using SoapySDR via a pure-C bridge to avoid libc++ ABI issues.
Consumes complex float IQ samples and writes them to a SoapySDR-compatible
device (e.g. USRP B210) for transmission.
)"">;

    gr::PortIn<std::complex<float>> in;

    // Device configuration (set before start, not runtime-changeable)
    gr::Annotated<std::string, "device driver", gr::Visible>   device{"uhd"};
    gr::Annotated<std::string, "clock source", gr::Visible>    clock_source{"external"};
    gr::Annotated<std::string, "extra device args", gr::Visible> device_args{""};
    gr::Annotated<float, "sample rate", gr::Unit<"Hz">, gr::Visible> sample_rate{250000.f};
    gr::Annotated<double, "center frequency", gr::Unit<"Hz">, gr::Visible> center_freq{869618000.0};
    gr::Annotated<double, "gain", gr::Unit<"dB">, gr::Visible> gain{30.0};
    gr::Annotated<double, "bandwidth", gr::Unit<"Hz">, gr::Visible> bandwidth{0.0};
    gr::Annotated<uint32_t, "max chunk size"> max_chunk_size{8192U};

    GR_MAKE_REFLECTABLE(SoapySink, in, device, clock_source, device_args,
                        sample_rate, center_freq, gain, bandwidth,
                        max_chunk_size);

    soapy_bridge_t* _bridge{nullptr};
    bool _skip_unmake{false};  ///< Set true before stop() to skip SoapySDRDevice_unmake

    void start() {
        soapy_bridge_config_t config{};
        config.driver       = device.value.c_str();
        config.clock_source = clock_source.value.empty()
                              ? nullptr : clock_source.value.c_str();
        config.device_args  = device_args.value.empty()
                              ? nullptr : device_args.value.c_str();
        config.sample_rate  = static_cast<double>(sample_rate.value);
        config.center_freq  = center_freq.value;
        config.gain_db      = gain.value;
        config.bandwidth    = bandwidth.value;
        config.channel      = 0;
        config.antenna      = nullptr;
        config.direction    = SOAPY_BRIDGE_TX;

        _bridge = soapy_bridge_create(&config);
        if (!_bridge) {
            throw gr::exception(
                std::format("SoapySink: failed to create device: {}",
                            soapy_bridge_last_error()));
        }

        std::fprintf(stderr, "[SoapySink] Device ready: rate=%.0f freq=%.0f gain=%.1f\n",
                     soapy_bridge_get_sample_rate(_bridge),
                     soapy_bridge_get_center_freq(_bridge),
                     soapy_bridge_get_gain(_bridge));
    }

    void stop() {
        if (_bridge) {
            soapy_bridge_destroy_ex(_bridge, _skip_unmake ? 1 : 0);
            _bridge = nullptr;
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& input) {
        if (!_bridge) {
            if (!input.consume(input.size())) {
                throw gr::exception("SoapySink: could not consume input");
            }
            return work::Status::ERROR;
        }

        const auto nSamples = std::min(
            static_cast<std::size_t>(max_chunk_size.value), input.size());

        if (nSamples == 0) {
            // No data available — keep running, don't signal DONE.
            return work::Status::OK;
        }

        auto* buf = reinterpret_cast<const float*>(input.data());
        int ret = soapy_bridge_write(_bridge, buf, nSamples,
                                     100000,  // 100ms timeout
                                     0);      // not end-of-burst mid-stream

        if (ret > 0) {
            if (!input.consume(static_cast<std::size_t>(ret))) {
                throw gr::exception("SoapySink: could not consume input");
            }
            return work::Status::OK;
        }

        // Timeout or underflow: consume nothing, retry on next work() call.
        // Underflow is logged by the bridge.
        return work::Status::OK;
    }
};

}  // namespace gr::lora

#endif  // GR4_LORA_SOAPY_SINK_HPP
