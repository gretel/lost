// SPDX-License-Identifier: ISC
//
// SoapySource: GR4 source block that reads IQ from a SoapySDR device via a
// pure-C bridge, avoiding the dual-libc++ ABI crash.
//
// The C bridge (soapy_c_bridge.c) is compiled with the system compiler and
// links against system libc++ (matching Homebrew SoapySDR/UHD). This header
// is compiled with LLVM 19 (GR4 toolchain) and calls only C functions.

#ifndef GR4_LORA_SOAPY_SOURCE_HPP
#define GR4_LORA_SOAPY_SOURCE_HPP

#include <complex>
#include <cstdint>
#include <cstdio>
#include <string>

#include <gnuradio-4.0/Block.hpp>

#include "soapy_c_bridge.h"

namespace gr::lora {

struct SoapySource : public gr::Block<SoapySource> {
    using Description = Doc<R""(
SDR source block using SoapySDR via a pure-C bridge to avoid libc++ ABI issues.
Produces complex float IQ samples from a SoapySDR-compatible device (e.g. USRP B210).
)"">;

    gr::PortOut<std::complex<float>> out;

    // Device configuration (set before start, not runtime-changeable)
    gr::Annotated<std::string, "device driver", gr::Visible>   device{"uhd"};
    gr::Annotated<std::string, "clock source", gr::Visible>    clock_source{"external"};
    gr::Annotated<std::string, "extra device args", gr::Visible> device_args{""};
    gr::Annotated<float, "sample rate", gr::Unit<"Hz">, gr::Visible> sample_rate{250000.f};
    gr::Annotated<double, "center frequency", gr::Unit<"Hz">, gr::Visible> center_freq{869525000.0};
    gr::Annotated<double, "gain", gr::Unit<"dB">, gr::Visible> gain{30.0};
    gr::Annotated<double, "bandwidth", gr::Unit<"Hz">, gr::Visible> bandwidth{0.0};
    gr::Annotated<uint32_t, "max chunk size"> max_chunk_size{8192U};

    GR_MAKE_REFLECTABLE(SoapySource, out, device, clock_source, device_args,
                        sample_rate, center_freq, gain, bandwidth, max_chunk_size);

    soapy_bridge_t* _bridge{nullptr};
    bool _skip_unmake{false};  ///< Set true before stop() to skip SoapySDRDevice_unmake

    void start() {
        soapy_bridge_config_t config{};
        config.driver       = device.value.c_str();
        config.clock_source = clock_source.value.empty() ? nullptr : clock_source.value.c_str();
        config.device_args  = device_args.value.empty() ? nullptr : device_args.value.c_str();
        config.sample_rate  = static_cast<double>(sample_rate.value);
        config.center_freq  = center_freq.value;
        config.gain_db      = gain.value;
        config.bandwidth    = bandwidth.value;
        config.channel      = 0;
        config.antenna      = nullptr;
        config.direction    = SOAPY_BRIDGE_RX;

        _bridge = soapy_bridge_create(&config);
        if (!_bridge) {
            throw gr::exception(
                std::format("SoapySource: failed to create device: {}",
                            soapy_bridge_last_error()));
        }

        // Update with actual values from the device
        std::fprintf(stderr, "[SoapySource] Device ready: rate=%.0f freq=%.0f gain=%.1f\n",
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

    work::Status processBulk(OutputSpanLike auto& output) {
        if (!_bridge) {
            output.publish(0UZ);
            return work::Status::ERROR;
        }

        const auto maxSamples = std::min(
            static_cast<std::size_t>(max_chunk_size.value), output.size());

        // Read IQ samples. The bridge returns complex float as interleaved
        // float pairs, which is the same memory layout as std::complex<float>.
        auto* buf = reinterpret_cast<float*>(output.data());
        int ret = soapy_bridge_read(_bridge, buf, maxSamples, 100000);  // 100ms timeout

        if (ret > 0) {
            output.publish(static_cast<std::size_t>(ret));
            return work::Status::OK;
        }

        // Timeout (-1) or overflow (-4): publish nothing, keep running.
        // Overflow is logged by the bridge. Other errors are also non-fatal
        // for a streaming source — we just retry on the next work() call.
        output.publish(0UZ);
        return work::Status::OK;
    }
};

}  // namespace gr::lora

#endif  // GR4_LORA_SOAPY_SOURCE_HPP
