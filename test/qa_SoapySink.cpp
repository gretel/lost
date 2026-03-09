#include <boost/ut.hpp>

#include <array>
#include <complex>
#include <vector>

#include <gnuradio-4.0/meta/formatter.hpp>

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-W#warnings"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#endif
#include <SoapySDR/Device.h>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Modules.h>
#include <SoapySDR/Modules.hpp>
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/soapy/SoapySink.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>

namespace gr::blocks::soapy {
static_assert(std::is_constructible_v<SoapySinkBlock<std::complex<float>>, gr::property_map>, "SoapySinkBlock not constructible with property_map");
static_assert(std::is_constructible_v<SoapySimpleSink<std::complex<float>>, gr::property_map>, "SoapySimpleSink not constructible with property_map");
} // namespace gr::blocks::soapy

const boost::ut::suite<"basic SoapySDR TX API"> basicSoapyTxAPI = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::soapy;

    std::set<std::string> availableDeviceDriver;
    "enumerate TX-capable devices"_test = [&availableDeviceDriver] {
        SoapySDR::KwargsList devices = Device::enumerate(SoapySDR::Kwargs{{"driver", "uhd"}, {"type", "b200"}});

        std::println("Detected B210 devices with TX support:");
        for (const auto& deviceKwargs : devices) {
            std::string driver = deviceKwargs.at("driver");
            try {
                Device dev(SoapySDR::Kwargs{{"driver", driver}, {"type", "b200"}});
                if (dev.getNumChannels(SOAPY_SDR_TX) > 0UZ) {
                    availableDeviceDriver.insert(driver);
                    std::println("  {} ({} TX channels)", driver, dev.getNumChannels(SOAPY_SDR_TX));
                }
            } catch (...) {
                // device may not be openable
            }
        }

        if (availableDeviceDriver.empty()) {
            std::println(stderr, "no TX-capable devices found - aborting TX tests");
            exit(0);
        }
    };

    "TX stream setup and teardown"_test =
        [](std::string deviceDriver) {
            std::println("TX stream setup test - deviceDriver '{}'", deviceDriver);

            Device device(SoapySDR::Kwargs{{"driver", deviceDriver}, {"type", "b200"}});
            expect(gt(device.getNumChannels(SOAPY_SDR_TX), 0UZ));

            device.setSampleRate(SOAPY_SDR_TX, 0, 1'000'000.);
            expect(approx(device.getSampleRate(SOAPY_SDR_TX, 0), 1e6, 1e2));

            device.setCenterFrequency(SOAPY_SDR_TX, 0, 434'000'000.);
            expect(approx(device.getCenterFrequency(SOAPY_SDR_TX, 0), 434e6, 1e4));

            using TValueType        = std::complex<float>;
            Device::Stream txStream = device.setupStream<TValueType, SOAPY_SDR_TX>();
            txStream.activate();

            std::vector<TValueType> zeros(1024, TValueType{0.f, 0.f});
            int                     flags    = SOAPY_SDR_END_BURST;
            long long               txTimeNs = 0;
            int                     ret      = txStream.writeStream(flags, txTimeNs, 1'000'000, std::span<const TValueType>(zeros));
            std::println("writeStream returned: {}", ret);
            expect(gt(ret, 0)) << std::format("writeStream failed for driver={}\n", deviceDriver);

            txStream.deactivate();
            std::println("TX stream setup test - deviceDriver '{}' -- DONE", deviceDriver);
        } //
        | availableDeviceDriver;
};

const boost::ut::suite<"SoapySink Block API"> soapySinkBlockAPI = [] {
    using namespace boost::ut;

    using TDuration = std::chrono::duration<std::chrono::steady_clock::rep, std::chrono::steady_clock::period>;
    using namespace std::chrono_literals;
    auto createWatchdog = [](auto& sched, TDuration timeOut = 5s, TDuration pollingPeriod = 40ms) {
        auto        externalInterventionNeeded = std::make_shared<std::atomic_bool>(false);
        std::thread watchdogThread([&sched, &externalInterventionNeeded, timeOut, pollingPeriod]() {
            auto timeout = std::chrono::steady_clock::now() + timeOut;
            while (std::chrono::steady_clock::now() < timeout) {
                if (sched.state() == gr::lifecycle::State::STOPPED) {
                    return;
                }
                std::this_thread::sleep_for(pollingPeriod);
            }
            std::println(stderr, "watchdog kicked in");
            externalInterventionNeeded->store(true, std::memory_order_relaxed);
            sched.requestStop();
            std::println(stderr, "requested scheduler to stop");
        });

        return std::make_pair(std::move(watchdogThread), externalInterventionNeeded);
    };

    "B210 TX burst via SoapySink"_test = [&createWatchdog] {
        using namespace gr;
        using namespace gr::blocks::soapy;
        using namespace gr::testing;
        using scheduler = gr::scheduler::Simple<>;
        gr::Graph flow;
        using ValueType = std::complex<float>;

        constexpr gr::Size_t nSamples = 10'000;

        auto& source = flow.emplaceBlock<ConstantSource<ValueType>>({
            {"n_samples_max", nSamples},
        });
        auto& sink   = flow.emplaceBlock<SoapySinkBlock<ValueType, 1UZ>>({
            {"device", "uhd"},                              //
            {"device_parameter", "type=b200"},              // avoid X300 PCIe crash
            {"sample_rate", float(1e6)},                    //
            {"tx_center_frequency", Tensor<double>{434e6}}, // ISM band
            {"tx_gains", Tensor<double>{30.}},              //
            {"timed_tx", true},                             //
            {"wait_burst_ack", true},                       //
        });
        expect(eq(gr::ConnectionResult::SUCCESS, flow.connect<"out">(source).to<"in">(sink)));

        scheduler sched;
        if (auto ret = sched.exchange(std::move(flow)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        auto [watchdogThread, externalInterventionNeeded] = createWatchdog(sched, 10s);

        auto retVal = sched.runAndWait();
        expect(retVal.has_value()) << std::format("scheduler execution error: {}", retVal.error());

        if (watchdogThread.joinable()) {
            watchdogThread.join();
        }
        expect(!externalInterventionNeeded->load(std::memory_order_relaxed));

        std::println("N.B. 'B210 TX burst via SoapySink' test finished");
    };

    "unload soapy modules"_test = [] { expect(nothrow([] { SoapySDR_unloadModules(); })) << "WARNING: unload SoapyModules - FAILED"; };
};

int main() { /* not needed for UT */ }
