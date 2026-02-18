// SPDX-License-Identifier: ISC
//
// DualRadioSource: GR4 source block that reads IQ from 2 RX channels
// simultaneously using a single multi-channel streamer.
//
// One stream carries both channels. The bridge's multi-channel recv
// handles time alignment internally.
//
// Outputs: out0 (channel 0 IQ) and out1 (channel 1 IQ).

#ifndef GR4_LORA_DUAL_RADIO_SOURCE_HPP
#define GR4_LORA_DUAL_RADIO_SOURCE_HPP

#include <algorithm>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>

#include "radio_bridge.h"

namespace gr::lora {

struct DualRadioSource : public gr::Block<DualRadioSource> {
    using Description = Doc<R""(
Dual-channel SDR source using a single multi-channel streamer.
Produces complex float IQ samples on two output ports (out0, out1),
one per RX channel. Uses one stream for both channels.
)"">;

    gr::PortOut<std::complex<float>> out0;  ///< channel 0 IQ
    gr::PortOut<std::complex<float>> out1;  ///< channel 1 IQ

    // Device configuration
    gr::Annotated<std::string, "device args", gr::Visible>     device_args{""};
    gr::Annotated<std::string, "clock source", gr::Visible>    clock_source{""};
    gr::Annotated<float, "sample rate", gr::Unit<"Hz">, gr::Visible> sample_rate{5000000.f};

    // Channel 0
    gr::Annotated<double, "ch0 center frequency", gr::Unit<"Hz">, gr::Visible> center_freq_0{866000000.0};
    gr::Annotated<double, "ch0 gain", gr::Unit<"dB">, gr::Visible> gain_0{50.0};
    gr::Annotated<double, "ch0 bandwidth", gr::Unit<"Hz">, gr::Visible> bandwidth_0{0.0};

    // Channel 1
    gr::Annotated<double, "ch1 center frequency", gr::Unit<"Hz">, gr::Visible> center_freq_1{869000000.0};
    gr::Annotated<double, "ch1 gain", gr::Unit<"dB">, gr::Visible> gain_1{50.0};
    gr::Annotated<double, "ch1 bandwidth", gr::Unit<"Hz">, gr::Visible> bandwidth_1{0.0};

    gr::Annotated<uint32_t, "max chunk size"> max_chunk_size{65536U};

    GR_MAKE_REFLECTABLE(DualRadioSource, out0, out1, device_args, clock_source,
                        sample_rate,
                        center_freq_0, gain_0, bandwidth_0,
                        center_freq_1, gain_1, bandwidth_1,
                        max_chunk_size);

    radio_bridge_multi_t* _bridge{nullptr};
    bool _skip_free{false};
    std::vector<float> _buf0;  ///< temp buffer for ch0 (interleaved I/Q floats)
    std::vector<float> _buf1;  ///< temp buffer for ch1

    void start() {
        radio_bridge_multi_config_t config{};
        config.device_args   = device_args.value.c_str();
        config.clock_source  = clock_source.value.empty() ? nullptr : clock_source.value.c_str();
        config.sample_rate   = static_cast<double>(sample_rate.value);
        config.center_freq_0 = center_freq_0.value;
        config.gain_db_0     = gain_0.value;
        config.bandwidth_0   = bandwidth_0.value;
        config.antenna_0     = nullptr;
        config.center_freq_1 = center_freq_1.value;
        config.gain_db_1     = gain_1.value;
        config.bandwidth_1   = bandwidth_1.value;
        config.antenna_1     = nullptr;

        _bridge = radio_bridge_multi_create(&config);
        if (!_bridge) {
            throw gr::exception(
                std::format("DualRadioSource: failed to create device: {}",
                            radio_bridge_last_error()));
        }

        _buf0.resize(max_chunk_size.value * 2);
        _buf1.resize(max_chunk_size.value * 2);

        std::fprintf(stderr, "[DualRadioSource] Device ready: rate=%.0f\n",
                     radio_bridge_multi_get_sample_rate(_bridge));
        std::fprintf(stderr, "[DualRadioSource]   ch0: freq=%.0f gain=%.1f\n",
                     radio_bridge_multi_get_center_freq(_bridge, 0),
                     radio_bridge_multi_get_gain(_bridge, 0));
        std::fprintf(stderr, "[DualRadioSource]   ch1: freq=%.0f gain=%.1f\n",
                     radio_bridge_multi_get_center_freq(_bridge, 1),
                     radio_bridge_multi_get_gain(_bridge, 1));
    }

    void stop() {
        if (_bridge) {
            radio_bridge_multi_destroy_ex(_bridge, _skip_free ? 1 : 0);
            _bridge = nullptr;
        }
    }

    work::Status processBulk(OutputSpanLike auto& output0,
                             OutputSpanLike auto& output1) {
        if (!_bridge) {
            output0.publish(0UZ);
            output1.publish(0UZ);
            return work::Status::ERROR;
        }

        const auto maxSamples = std::min({
            static_cast<std::size_t>(max_chunk_size.value),
            output0.size(),
            output1.size()
        });

        int ret = radio_bridge_multi_read(
            _bridge, _buf0.data(), _buf1.data(), maxSamples, 0.1);

        if (ret > 0) {
            auto n = static_cast<std::size_t>(ret);
            std::copy_n(reinterpret_cast<const std::complex<float>*>(_buf0.data()),
                        n, output0.data());
            std::copy_n(reinterpret_cast<const std::complex<float>*>(_buf1.data()),
                        n, output1.data());
            output0.publish(n);
            output1.publish(n);
            return work::Status::OK;
        }

        output0.publish(0UZ);
        output1.publish(0UZ);
        return work::Status::OK;
    }
};

}  // namespace gr::lora

#endif  // GR4_LORA_DUAL_RADIO_SOURCE_HPP
