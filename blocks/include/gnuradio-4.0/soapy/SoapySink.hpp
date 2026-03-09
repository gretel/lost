#ifndef SOAPYSINK_HPP
#define SOAPYSINK_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-W#warnings"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#endif
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.h>
#include <SoapySDR/Formats.h>
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <map>
#include <string>
#include <vector>

#include <gnuradio-4.0/meta/formatter.hpp>

#include <gnuradio-4.0/soapy/SoapyRaiiWrapper.hpp>

namespace gr::blocks::soapy {

GR_REGISTER_BLOCK("gr::blocks::soapy::SoapySimpleSink", gr::blocks::soapy::SoapySinkBlock, ([T], 1UZ), [std::complex<float>])
GR_REGISTER_BLOCK("gr::blocks::soapy::SoapyDualSimpleSink", gr::blocks::soapy::SoapySinkBlock, ([T], 2UZ), [std::complex<float>])

template<typename T, std::size_t nPorts = std::dynamic_extent>
struct SoapySinkBlock : public Block<SoapySinkBlock<T, nPorts>> {
    using Description = Doc<R""(A Soapy sink block that transmits samples to SDR hardware using the SoapySDR library.
Supports timed burst transmission with end-of-burst signalling and BURST_ACK confirmation.)"">;

    template<typename U, gr::meta::fixed_string description = "", typename... Arguments>
    using A = Annotated<U, description, Arguments...>;

    using TSizeChecker = Limits<0U, std::numeric_limits<std::uint32_t>::max(), [](std::uint32_t x) { return std::has_single_bit(x); }>;
    using TBasePort    = PortIn<T>;
    using TPortType    = std::conditional_t<nPorts == 1U, TBasePort, std::conditional_t<nPorts == std::dynamic_extent, std::vector<TBasePort>, std::array<TBasePort, nPorts>>>;

    TPortType in;

    A<std::string, "device driver name", Visible>                                                        device;
    A<std::string, "add. device parameter", Visible>                                                     device_parameter;
    A<float, "sample rate", Unit<"samples/s">, Doc<"sampling rate in samples per second (Hz)">, Visible> sample_rate = 1'000'000.f;
    A<Tensor<gr::Size_t>, "TX channel ID mapping vector", Visible>                                       tx_channels = initDefaultValues<true>(gr::Size_t(0U));
    A<Tensor<pmt::Value>, "TX channel antenna mapping", Visible>                                         tx_antennae;
    A<Tensor<double>, "TX center frequency", Unit<"Hz">, Doc<"TX-RF center frequency">, Visible>         tx_center_frequency = initDefaultValues(107'000'000.);
    A<Tensor<double>, "TX bandwidth", Unit<"Hz">, Doc<"TX-RF bandwidth">, Visible>                       tx_bandwidth        = initDefaultValues(double(sample_rate / 2));
    A<Tensor<double>, "TX gain", Unit<"dB">, Doc<"TX channel gain">, Visible>                            tx_gains            = initDefaultValues(10.);

    A<std::string, "clock source", Doc<"e.g. 'internal', 'external', 'gpsdo'">>            clock_source;
    A<double, "frequency correction", Unit<"ppm">, Doc<"oscillator frequency correction">> frequency_correction_ppm = 0.0;
    A<std::string, "stream args", Doc<"comma-separated key=value pairs for setupStream">>  stream_args;

    A<std::uint32_t, "max write chunk size", Doc<"ideally N x 512">, Visible, TSizeChecker>  max_chunk_size         = 512U << 4U;
    A<std::uint32_t, "write time out", Unit<"us">, Doc<"soapy write time-out">>              max_time_out_us        = 1'000'000;
    A<double, "TX start delay", Unit<"s">, Doc<"delay before first sample for PA settling">> tx_start_delay_s       = 0.1;
    A<bool, "timed TX", Doc<"use hardware time for burst start">>                            timed_tx               = true;
    A<bool, "wait for BURST_ACK", Doc<"wait for BURST_ACK on stop">>                         wait_burst_ack         = true;
    A<std::uint32_t, "BURST_ACK timeout", Unit<"us">, Doc<"timeout for BURST_ACK polling">>  burst_ack_timeout      = 5'000'000;
    A<gr::Size_t, "max underflow count", Doc<"0: disable">>                                  max_underflow_count    = 10U;
    A<gr::Size_t, "max stream error count", Doc<"0: disable">>                               max_stream_error_count = 10U;
    A<gr::Size_t, "max time error count", Doc<"0: disable">>                                 max_time_error_count   = 10U;
    A<gr::Size_t, "max overflow count", Doc<"0: disable">>                                   max_overflow_count     = 10U;

    GR_MAKE_REFLECTABLE(SoapySinkBlock, in, device, device_parameter, sample_rate, tx_channels, tx_antennae, tx_center_frequency, tx_bandwidth, tx_gains, clock_source, frequency_correction_ppm, stream_args, max_chunk_size, max_time_out_us, tx_start_delay_s, timed_tx, wait_burst_ack, burst_ack_timeout, max_underflow_count, max_stream_error_count, max_time_error_count, max_overflow_count);

    Device                          _device{};
    Device::Stream<T, SOAPY_SDR_TX> _txStream{};
    bool                            _firstWrite       = true;
    gr::Size_t                      _underflowCount   = 0U;
    gr::Size_t                      _streamErrorCount = 0U;
    gr::Size_t                      _timeErrorCount   = 0U;
    gr::Size_t                      _overflowCount    = 0U;
    bool                            _eobSent          = false;

    void settingsChanged(const property_map& oldSettings, const property_map& newSettings) {
        if (_device.get() == nullptr) {
            return; // device not yet initialised (called during init before start)
        }

        bool needReinit = false;
        if ((newSettings.contains("device") && (oldSettings.at("device") != newSettings.at("device")))                   //
            || (newSettings.contains("tx_channels") && (oldSettings.at("tx_channels") != newSettings.at("tx_channels"))) //
            || (newSettings.contains("sample_rate"))) {
            needReinit = true;
        }

        if (newSettings.contains("tx_antennae")) {
            setAntennae();
        }
        if (newSettings.contains("tx_center_frequency") || newSettings.contains("sample_rate")) {
            setCenterFrequency();
        }
        if (newSettings.contains("tx_gains")) {
            setGains();
        }
        if (newSettings.contains("frequency_correction_ppm")) {
            setFrequencyCorrection();
        }

        if (needReinit && lifecycle::isActive(this->state())) {
            reinitDevice();
        }
    }

    void start() { reinitDevice(); }

    void stop() {
        if (wait_burst_ack && _txStream.get() != nullptr) {
            // GR4's EoS mechanism transitions the sink to STOPPED without a final
            // processBulk call, so END_BURST may never have been sent. Send it now
            // to ensure the FPGA generates BURST_ACK and completes the burst.
            if (!_eobSent) {
                int       flags  = SOAPY_SDR_END_BURST;
                long long timeNs = 0;
                T         zero{};
                auto      zeroSpan = std::span<const T>(&zero, 1);
                _txStream.writeStream(flags, timeNs, max_time_out_us, zeroSpan);
            }
            waitForBurstAck();
        }
        _txStream.reset();
        _device.reset();
    }

    void pause() { _txStream.deactivate(); }
    void resume() {
        _txStream.activate();
        _firstWrite = true;
    }

    [[nodiscard]] constexpr work::Status processBulk(InputSpanLike auto& input)
    requires(nPorts == 1)
    {
        if (_device.get() == nullptr) {
            std::ignore = input.consume(0UZ);
            return work::Status::ERROR;
        }
        if (input.size() == 0UZ) {
            if (!input.consume(0UZ)) {
                return work::Status::ERROR;
            }
            return this->state() == lifecycle::State::REQUESTED_STOP ? work::Status::DONE : work::Status::OK;
        }

        const auto nSamples  = std::min(std::size_t{max_chunk_size.value}, input.size());
        auto       inputSpan = std::span<const T>(input.data(), nSamples);
        int        flags     = 0;
        long long  txTimeNs  = 0;

        if (_firstWrite && timed_tx) {
            flags       = SOAPY_SDR_HAS_TIME;
            txTimeNs    = static_cast<long long>(_device.getHardwareTime()) + static_cast<long long>(tx_start_delay_s * 1e9);
            _firstWrite = false;
        } else {
            _firstWrite = false;
        }

        if (nSamples == input.size() && this->state() == lifecycle::State::REQUESTED_STOP) {
            flags |= SOAPY_SDR_END_BURST;
            _eobSent = true;
        }

        int ret = _txStream.writeStream(flags, txTimeNs, max_time_out_us, inputSpan);

        if (ret < 0) {
            auto status = handleStreamingErrors(ret);
            if (status != work::Status::OK) {
                return status;
            }
            if (!input.consume(0UZ)) {
                return work::Status::ERROR;
            }
            return work::Status::OK;
        }

        resetStreamingErrorCounts();
        if (!input.consume(static_cast<std::size_t>(ret))) {
            return work::Status::ERROR;
        }
        return work::Status::OK;
    }

    template<InputSpanLike TInputBuffer>
    [[nodiscard]] constexpr work::Status processBulk(std::span<TInputBuffer>& inputs)
    requires(nPorts > 1)
    {
        if (_device.get() == nullptr) {
            std::ranges::for_each(inputs, [](auto& input) { std::ignore = input.consume(0UZ); });
            return work::Status::ERROR;
        }
        auto nSamples = static_cast<std::uint32_t>(inputs[0].size());
        nSamples      = std::min(nSamples, max_chunk_size.value);

        if (nSamples == 0U) {
            std::ranges::for_each(inputs, [](auto& input) { std::ignore = input.consume(0UZ); });
            return this->state() == lifecycle::State::REQUESTED_STOP ? work::Status::DONE : work::Status::OK;
        }

        int       flags    = 0;
        long long txTimeNs = 0;

        if (_firstWrite && timed_tx) {
            flags       = SOAPY_SDR_HAS_TIME;
            txTimeNs    = static_cast<long long>(_device.getHardwareTime()) + static_cast<long long>(tx_start_delay_s * 1e9);
            _firstWrite = false;
        } else {
            _firstWrite = false;
        }

        if (nSamples == static_cast<std::uint32_t>(inputs[0].size()) && this->state() == lifecycle::State::REQUESTED_STOP) {
            flags |= SOAPY_SDR_END_BURST;
            _eobSent = true;
        }

        std::vector<std::span<const T>> txBuffers(tx_channels->size());
        for (std::size_t i = 0UZ; i < tx_channels->size(); ++i) {
            txBuffers[i] = std::span<const T>(inputs[i].data(), nSamples);
        }
        int ret = _txStream.writeStreamFromBufferList(flags, txTimeNs, static_cast<long int>(max_time_out_us), txBuffers);

        if (ret < 0) {
            auto status = handleStreamingErrors(ret);
            if (status != work::Status::OK) {
                return status;
            }
            std::ranges::for_each(inputs, [](auto& input) { std::ignore = input.consume(0UZ); });
            return work::Status::OK;
        }

        resetStreamingErrorCounts();
        std::ranges::for_each(inputs, [ret](auto& input) { std::ignore = input.consume(static_cast<std::size_t>(ret)); });
        return work::Status::OK;
    }

    void reinitDevice() {
        _txStream.reset();
        _device = Device(detail::buildDeviceArgs(device.value, device_parameter.value));

        if (!clock_source.value.empty()) {
            _device.setClockSource(clock_source.value);
        }

        std::size_t nChannelMax = _device.getNumChannels(SOAPY_SDR_TX);
        if (nChannelMax < tx_channels->size()) {
            throw gr::exception(std::format("device {} TX channel count exceeded: requested {} but device has {}", device.value, tx_channels->size(), nChannelMax));
        }

        setSampleRate();
        setAntennae();
        setCenterFrequency();
        setGains();
        setFrequencyCorrection();
        _txStream = _device.setupStream<T, SOAPY_SDR_TX>(tx_channels.value, detail::parseKwargs(stream_args.value));
        _txStream.activate();
        _firstWrite = true;
        _eobSent    = false;
        resetStreamingErrorCounts();
    }

    void setSampleRate() {
        std::size_t nChannels = tx_channels->size();
        for (std::size_t i = 0UZ; i < nChannels; i++) {
            _device.setSampleRate(SOAPY_SDR_TX, tx_channels->at(i), static_cast<double>(sample_rate));
        }

        std::vector<double> actualSampleRates;
        for (std::size_t i = 0UZ; i < nChannels; i++) {
            actualSampleRates.push_back(_device.getSampleRate(SOAPY_SDR_TX, tx_channels->at(i)));
        }

        if (!detail::equalWithinOnePercent(actualSampleRates, std::vector<double>(nChannels, static_cast<double>(sample_rate)))) {
            throw gr::exception(std::format("TX sample rate mismatch:\nSet: {} vs. Actual: {}\n", sample_rate, gr::join(actualSampleRates, ", ")));
        }
    }

    void setAntennae() {
        if (tx_antennae->empty()) {
            return;
        }
        std::size_t nChannels = tx_channels->size();
        std::size_t nAntennae = tx_antennae->size();
        for (std::size_t i = 0UZ; i < nChannels; i++) {
            std::string antenna = tx_antennae->at(std::min(i, nAntennae - 1UZ)).value_or(std::string());
            if (!antenna.empty()) {
                _device.setAntenna(SOAPY_SDR_TX, tx_channels->at(i), antenna);
            }
        }
    }

    void setCenterFrequency() {
        std::size_t nChannels  = tx_channels->size();
        std::size_t nFrequency = tx_center_frequency->size();
        for (std::size_t i = 0UZ; i < nChannels; i++) {
            _device.setCenterFrequency(SOAPY_SDR_TX, tx_channels->at(i), tx_center_frequency->at(std::min(i, nFrequency - 1UZ)));
        }
        std::vector<double> tx_center_frequency_actual;
        for (std::size_t i = 0UZ; i < nChannels; i++) {
            tx_center_frequency_actual.push_back(_device.getCenterFrequency(SOAPY_SDR_TX, tx_channels->at(i)));
        }

        if (!detail::equalWithinOnePercent(tx_center_frequency_actual, tx_center_frequency.value)) {
            throw gr::exception(std::format("TX center frequency mismatch:\nSet: {} vs. Actual: {}\n", gr::join(tx_center_frequency.value, ", "), gr::join(tx_center_frequency_actual, ", ")));
        }
    }

    void setBandwidth() {
        std::size_t nChannels  = tx_channels->size();
        std::size_t nBandwidth = tx_bandwidth->size();
        for (std::size_t i = 0UZ; i < nChannels; i++) {
            _device.setBandwidth(SOAPY_SDR_TX, tx_channels->at(i), tx_bandwidth->at(std::min(i, nBandwidth - 1UZ)));
        }
    }

    void setGains() {
        std::size_t nChannels = tx_channels->size();
        std::size_t nGains    = tx_gains->size();
        for (std::size_t i = 0UZ; i < nChannels; i++) {
            _device.setGain(SOAPY_SDR_TX, tx_channels->at(i), tx_gains->at(std::min(i, nGains - 1UZ)));
        }
    }

    void setFrequencyCorrection() {
        if (frequency_correction_ppm == 0.0) {
            return;
        }
        std::size_t nChannels = tx_channels->size();
        for (std::size_t i = 0UZ; i < nChannels; i++) {
            if (_device.hasFrequencyCorrection(SOAPY_SDR_TX, tx_channels->at(i))) {
                _device.setFrequencyCorrection(SOAPY_SDR_TX, tx_channels->at(i), frequency_correction_ppm);
            }
        }
    }

    void resetStreamingErrorCounts() {
        _underflowCount   = 0U;
        _streamErrorCount = 0U;
        _timeErrorCount   = 0U;
        _overflowCount    = 0U;
    }

    work::Status handleStreamingErrors(int ret) {
        switch (ret) {
        case SOAPY_SDR_TIMEOUT: return work::Status::OK;
        case SOAPY_SDR_UNDERFLOW: {
            _underflowCount++;
            if (max_underflow_count > 0 && _underflowCount > max_underflow_count) {
                throw gr::exception(std::format("SOAPY_SDR_UNDERFLOW for Block {} Device {}: reached {} of requested max {}", this->name, device, _underflowCount, max_underflow_count));
            }
            return work::Status::OK;
        }
        case SOAPY_SDR_STREAM_ERROR: {
            _streamErrorCount++;
            std::println(stderr, "[{}] SOAPY_SDR_STREAM_ERROR #{} ('{}')", this->name, _streamErrorCount, SoapySDR_errToStr(ret));
            if (max_stream_error_count > 0 && _streamErrorCount > max_stream_error_count) {
                throw gr::exception(std::format("SOAPY_SDR_STREAM_ERROR for Block {} Device {}: reached {} of requested max {}", this->name, device, _streamErrorCount, max_stream_error_count));
            }
            return work::Status::OK;
        }
        case SOAPY_SDR_CORRUPTION: {
            throw gr::exception(std::format("SOAPY_SDR_CORRUPTION for Block {} Device {}", this->name, device));
        }
        case SOAPY_SDR_OVERFLOW: {
            _overflowCount++;
            std::println(stderr, "[{}] SOAPY_SDR_OVERFLOW #{} ('{}')", this->name, _overflowCount, SoapySDR_errToStr(ret));
            if (max_overflow_count > 0 && _overflowCount > max_overflow_count) {
                throw gr::exception(std::format("SOAPY_SDR_OVERFLOW for Block {} Device {}: reached {} of requested max {}", this->name, device, _overflowCount, max_overflow_count));
            }
            return work::Status::OK;
        }
        case SOAPY_SDR_NOT_SUPPORTED: {
            throw gr::exception(std::format("SOAPY_SDR_NOT_SUPPORTED for Block {} Device {}", this->name, device));
        }
        case SOAPY_SDR_TIME_ERROR: {
            _timeErrorCount++;
            std::println(stderr, "[{}] SOAPY_SDR_TIME_ERROR #{} ('{}')", this->name, _timeErrorCount, SoapySDR_errToStr(ret));
            if (max_time_error_count > 0 && _timeErrorCount > max_time_error_count) {
                throw gr::exception(std::format("SOAPY_SDR_TIME_ERROR for Block {} Device {}: reached {} of requested max {}", this->name, device, _timeErrorCount, max_time_error_count));
            }
            return work::Status::OK;
        }
        default: throw gr::exception(std::format("unknown SoapySDR return type: {} ('{}')", ret, SoapySDR_errToStr(ret)));
        }
    }

    void waitForBurstAck() {
        constexpr std::uint32_t kPollIntervalUs = 10'000; // 10 ms
        std::uint32_t           elapsed         = 0U;

        while (elapsed < burst_ack_timeout) {
            std::size_t chanMask = 0UZ;
            int         flags    = 0;
            long long   timeNs   = 0;
            int         ret      = _txStream.readStreamStatus(chanMask, flags, timeNs, kPollIntervalUs);

            if (ret == 0 && (flags & SOAPY_SDR_END_BURST)) {
                std::println(stderr, "SoapySink: BURST_ACK received after {} ms", elapsed / 1000);
                return;
            }
            // SoapyUHD returns UNDERFLOW (-7) for non-ACK async events
            if (ret == SOAPY_SDR_TIMEOUT || ret == SOAPY_SDR_UNDERFLOW) {
                elapsed += kPollIntervalUs;
                continue;
            }
            if (ret != 0) {
                std::println(stderr, "SoapySink: readStreamStatus error: {} - '{}'", ret, SoapySDR_errToStr(ret));
                return;
            }
            elapsed += kPollIntervalUs;
        }
        std::println(stderr, "SoapySink: BURST_ACK timeout after {} us", burst_ack_timeout.value);
    }

    template<bool increment = false, typename U>
    static std::vector<U> initDefaultValues(U initialValue) {
        std::vector<U> values;
        values.resize(nPorts);
        if constexpr (increment) {
            std::ranges::generate(values, [i = U(0), initialValue]() mutable { return initialValue + i++; });
        } else {
            std::fill(values.begin(), values.end(), initialValue);
        }
        return values;
    }
};

template<typename T>
using SoapySimpleSink = SoapySinkBlock<T, 1UZ>;
template<typename T>
using SoapyDualSimpleSink = SoapySinkBlock<T, 2UZ>;

} // namespace gr::blocks::soapy

#endif // SOAPYSINK_HPP
