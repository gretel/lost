// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_SCAN_SINK_HPP
#define GNURADIO_LORA_SCAN_SINK_HPP

#include <chrono>
#include <complex>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/lora/log.hpp>

namespace gr::lora {

struct ScanSink : gr::Block<ScanSink> {
    using Description = Doc<"Sink for scan detection results and spectrum data. Logs detections to stderr.">;

    gr::PortIn<gr::DataSet<float>, gr::Async> detections;
    gr::PortIn<gr::DataSet<float>, gr::Async> spectrum;

    uint32_t _detectionCount{0};
    uint32_t _sweepCount{0};

    GR_MAKE_REFLECTABLE(ScanSink, detections, spectrum);

    gr::work::Status processBulk(gr::InputSpanLike auto& detectSpan,
                                 gr::InputSpanLike auto& spectrumSpan) noexcept {
        auto detSpan = std::span(detectSpan);
        for (std::size_t i = 0; i < detSpan.size(); ++i) {
            processDetection(detSpan[i]);
        }
        if (!detSpan.empty()) {
            std::ignore = detectSpan.consume(detSpan.size());
        }

        auto specSpan = std::span(spectrumSpan);
        for (std::size_t i = 0; i < specSpan.size(); ++i) {
            processSpectrum(specSpan[i]);
        }
        if (!specSpan.empty()) {
            std::ignore = spectrumSpan.consume(specSpan.size());
        }

        return gr::work::Status::OK;
    }

    void processDetection(const gr::DataSet<float>& ds) noexcept {
        ++_detectionCount;

        if (ds.meta_information.empty()) return;
        const auto& meta = ds.meta_information[0];

        auto getInt = [&](const std::string& key) -> int64_t {
            auto it = meta.find(key);
            if (it == meta.end()) return 0;
            return std::get<int64_t>(it->second);
        };
        auto getFloat = [&](const std::string& key) -> float {
            auto it = meta.find(key);
            if (it == meta.end()) return 0.f;
            return std::get<float>(it->second);
        };
        auto getDouble = [&](const std::string& key) -> double {
            auto it = meta.find(key);
            if (it == meta.end()) return 0.0;
            return std::get<double>(it->second);
        };

        gr::lora::log_ts("det  ", "scan",
            "SF%lld BW%.0fk freq=%.3f MHz ratio=%.1f sweep=%lld",
            static_cast<long long>(getInt("sf")),
            static_cast<double>(getFloat("bw")) / 1e3,
            getDouble("freq") / 1e6,
            static_cast<double>(getFloat("ratio")),
            static_cast<long long>(getInt("sweep")));
    }

    void processSpectrum(const gr::DataSet<float>& ds) noexcept {
        if (ds.meta_information.empty()) return;
        const auto& meta = ds.meta_information[0];

        auto it = meta.find("sweep");
        if (it != meta.end()) {
            _sweepCount = static_cast<uint32_t>(std::get<int64_t>(it->second));
        }
    }
};

} // namespace gr::lora

#endif // GNURADIO_LORA_SCAN_SINK_HPP
