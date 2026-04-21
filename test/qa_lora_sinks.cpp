// SPDX-License-Identifier: ISC
//
// Construction + state-initialisation smoke tests for the small sink/passthrough
// blocks (CaptureSink, ScanSink, SpectrumTapBlock).  Their full processBulk
// behaviour is exercised end-to-end through qa_lora_scan; this file ensures
// each block instantiates with sane defaults and the supporting state types
// initialise correctly.

#include <boost/ut.hpp>

#include <atomic>
#include <complex>
#include <cstdint>
#include <memory>

#include <gnuradio-4.0/lora/CaptureSink.hpp>
#include <gnuradio-4.0/lora/ScanSink.hpp>
#include <gnuradio-4.0/lora/SpectrumTapBlock.hpp>
#include <gnuradio-4.0/lora/algorithm/SpectrumTap.hpp>

using namespace boost::ut;

const suite<"CaptureSink"> captureSink = [] {
    "default state is null"_test = [] {
        gr::lora::CaptureSink sink;
        expect(sink._state == nullptr) << "freshly-constructed sink has no shared state";
    };

    "request count semantics on shared state"_test = [] {
        auto state = std::make_shared<gr::lora::CaptureState>();
        state->buffer.resize(1024);
        state->request_count.store(512, std::memory_order_release);

        expect(state->request_count.load() == 512_u);
        expect(state->captured.load() == 0_u);
        expect(not state->done.load());
    };

    "request restarts the counters"_test = [] {
        // request(N) is the orchestrator-side primitive; reset() only clears
        // request_count after the orchestrator has read the buffer.  Verify
        // the request->capture handshake.
        auto state = std::make_shared<gr::lora::CaptureState>();
        state->buffer.resize(64);
        state->captured.store(32, std::memory_order_release);
        state->done.store(true, std::memory_order_release);

        state->request(48);

        expect(state->captured.load() == 0_u);
        expect(state->request_count.load() == 48_u);
        expect(not state->done.load());
    };
};

const suite<"ScanSink"> scanSink = [] {
    "default counters are zero"_test = [] {
        gr::lora::ScanSink sink;
        expect(sink._detectionCount == 0_u);
        expect(sink._sweepCount == 0_u);
        expect(not sink._onDataSet) << "no callback by default";
    };

    "callback can be installed"_test = [] {
        gr::lora::ScanSink sink;
        bool               fired = false;
        sink._onDataSet          = [&fired](const gr::DataSet<float>&) { fired = true; };
        expect(static_cast<bool>(sink._onDataSet));
        // synthesise a minimal DataSet and route it through processDetection
        gr::DataSet<float> ds;
        ds.meta_information.emplace_back();
        sink.processDetection(ds);
        expect(fired) << "_onDataSet should fire on processed detection";
        expect(sink._detectionCount == 1_u);
    };
};

const suite<"SpectrumTapBlock"> spectrumTap = [] {
    "default push interval is 1"_test = [] {
        gr::lora::SpectrumTapBlock block;
        expect(block._pushInterval == 1_u);
        expect(block._pushCount == 0_u);
        expect(block._spectrum == nullptr);
    };

    "shared spectrum state can be attached"_test = [] {
        gr::lora::SpectrumTapBlock block;
        auto                       state = std::make_shared<gr::lora::SpectrumState>(/*ringSize=*/4096);
        block._spectrum                  = state;
        expect(block._spectrum != nullptr);
        expect(block._spectrum.use_count() >= 2); // state + block hold refs
    };
};

int main() { return 0; }
