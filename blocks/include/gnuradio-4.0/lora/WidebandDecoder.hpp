// SPDX-License-Identifier: ISC
#ifndef GNURADIO_LORA_WIDEBAND_DECODER_HPP
#define GNURADIO_LORA_WIDEBAND_DECODER_HPP

// WidebandDecoder: single GR4 block that fuses wideband capture (16 MS/s)
// + L1 energy gating + streaming digital channelization + multi-SF decode.
//
// Graph: SoapySource (16 MS/s) → WidebandDecoder → FrameSink
//
// M2: ChannelSlot — phase-continuous streaming channelizer
// M3: Block skeleton — L1 energy + channel pool management
// M4: Fused SfLane decode (pending)

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <numeric>
#include <ranges>
#include <span>
#include <sstream>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/lora/algorithm/HalfBandDecimator.hpp>
#include <gnuradio-4.0/lora/algorithm/RingBuffer.hpp>
#include <gnuradio-4.0/lora/algorithm/SfLane.hpp>
#include <gnuradio-4.0/lora/algorithm/Telemetry.hpp>
#include <gnuradio-4.0/lora/log.hpp>

namespace gr::lora {

using cf32 = std::complex<float>;

// ─── ChannelSlot (M2) ───────────────────────────────────────────────────────

struct ChannelSlot {
    enum class State : uint8_t { Idle, Active, Draining };

    State    state{State::Idle};
    double   channelFreq{0.0};   // channel center frequency (Hz)
    double   centerFreq{0.0};    // wideband center frequency (Hz)
    double   sampleRate{0.0};    // wideband sample rate (Hz)
    uint32_t osFactor{1};        // decimation factor = sample_rate / bw
    uint32_t channelIdx{0};      // L1 channel index that activated this slot
    uint32_t activeSweeps{0};    // sweeps since activation (for min hold time)

    // Phase-continuous NCO state
    // Double-precision phaseInc kept for computing the single-precision step phasor
    // in activate(). Hot path uses complex-multiply recurrence (ncoRot *= ncoStep)
    // with renormalization every 1024 samples.
    double ncoPhaseInc{0.0};     // radians per sample = 2*pi * shift_hz / sample_rate
    cf32   ncoRot{1.f, 0.f};     // current NCO phasor (rotated each sample)
    cf32   ncoStep{1.f, 0.f};    // per-sample rotation phasor (computed once in activate)

    // Cascaded half-band FIR decimator
    CascadedDecimator decimator;
    uint32_t decodeBw{62500};

    // Narrowband output samples (accumulated between drain calls)
    std::vector<cf32> nbAccum;

    // Pre-allocated zero buffer for overflow gap filling
    std::vector<cf32> _gapBuf;

    // Per-SF decode lanes (SF7-12, os_factor=1 since channelizer already decimated)
    std::vector<SfLane> sfLanes;

    /// Activate the slot for a given channel.  Resets all internal state.
    /// @param sfs  spreading factors to decode (empty = all SF7-12)
    void activate(double channel_freq, double center_freq,
                  double sample_rate, uint32_t os_factor, uint32_t ch_idx = 0,
                  uint32_t bw = 62500, uint16_t sync = 0x12, uint16_t preamble = 8,
                  std::span<const uint8_t> sfs = {}) {
        state        = State::Active;
        activeSweeps = 0;
        channelFreq  = channel_freq;
        centerFreq  = center_freq;
        sampleRate  = sample_rate;
        osFactor    = os_factor;
        channelIdx  = ch_idx;

        // NCO shift: negate to bring channel down to baseband
        const double shift_hz = -(channel_freq - center_freq);
        ncoPhaseInc = 2.0 * std::numbers::pi * shift_hz / sample_rate;
        ncoStep = cf32(static_cast<float>(std::cos(ncoPhaseInc)),
                       static_cast<float>(std::sin(ncoPhaseInc)));
        ncoRot  = cf32(1.f, 0.f);  // start at 0 phase

        // Cascaded half-band FIR: os_factor must be power of 2 and >= 1
        if (os_factor < 1 || (os_factor & (os_factor - 1)) != 0) {
            std::fprintf(stderr, "[wideband] activate: invalid os_factor %u, must be power of 2\n",
                         os_factor);
            state = State::Idle;
            return;
        }
        uint32_t nStages = 0;
        for (uint32_t v = os_factor; v > 1; v >>= 1) ++nStages;
        decimator.init(nStages);
        decodeBw = bw;
        nbAccum.clear();
        _gapBuf.assign(8192, cf32{0.f, 0.f});

        // Initialize SfLanes at os_factor=1 (configurable SF range)
        sfLanes.clear();
        if (sfs.empty()) {
            sfLanes.reserve(6);
            for (uint8_t sf = 7; sf <= 12; ++sf) {
                sfLanes.emplace_back();
                sfLanes.back().init(sf, bw, 1, preamble, sync);
            }
        } else {
            sfLanes.reserve(sfs.size());
            for (uint8_t sf : sfs) {
                sfLanes.emplace_back();
                sfLanes.back().init(sf, bw, 1, preamble, sync);
            }
        }
    }

    /// Deactivate the slot and release buffered output.
    void deactivate() {
        state = State::Idle;
        decimator.reset();
        nbAccum.clear();
        nbAccum.shrink_to_fit();
        sfLanes.clear();
    }

    /// Mix wideband samples through the NCO and cascaded half-band FIR
    /// decimator in a single fused pass. Each wideband sample is read once
    /// and never materialized as an intermediate mixed result, eliminating
    /// 128 KB of memory traffic per call (8192 × 16 bytes).
    ///
    /// Phase is continuous across calls. Output appended to nbAccum.
    void pushWideband(std::span<const cf32> wbSamples, std::size_t gapSamples = 0) {
        if (wbSamples.empty() && gapSamples > 0) {
            // Overflow: no IQ data but time advanced. Insert zeros to preserve
            // symbol boundary alignment. The zeros produce noise in the
            // narrowband output but prevent timing shifts that corrupt dechirp.
            if (_gapBuf.size() < gapSamples) {
                _gapBuf.assign(gapSamples, cf32{0.f, 0.f});
            } else {
                // Always zero the used span — buffer may have stale data
                std::fill_n(_gapBuf.begin(), gapSamples, cf32{0.f, 0.f});
            }
            decimator.processWithNcoBatch(
                std::span<const cf32>(_gapBuf.data(), gapSamples),
                nbAccum, ncoRot, ncoStep);
            return;
        }
        decimator.processWithNcoBatch(wbSamples, nbAccum, ncoRot, ncoStep);
    }
};

// ─── WidebandDecoder block (M3) ─────────────────────────────────────────────

GR_REGISTER_BLOCK("gr::lora::WidebandDecoder", gr::lora::WidebandDecoder)
struct WidebandDecoder
    : gr::Block<WidebandDecoder, gr::NoDefaultTagForwarding> {

    using Description = Doc<"Wideband LoRa decoder: L1 energy gating + streaming channelization + multi-SF decode.">;

    // --- ports ---
    gr::PortIn<cf32>     in;
    gr::PortOut<uint8_t> out;
    gr::MsgPortOut       msg_out;

    // --- settings ---
    float       sample_rate{16000000.f};   // wideband sample rate (Hz)
    float       center_freq{866500000.f};  // wideband center frequency (Hz)
    float       channel_bw{62500.f};       // L1 channel grid spacing (Hz)
    float       decode_bw{62500.f};        // decode bandwidth for channelization (Hz)
    std::string decode_bw_str{"62500"};    // comma-separated BWs for multi-BW decode
    std::string decode_sfs_str{""};        // comma-separated SFs to decode, empty = all (7-12)
    float       min_ratio{8.0f};           // min L2 CAD peak ratio (unused in M3)
    float       buffer_ms{512.f};          // ring buffer duration (ms)
    uint32_t    max_channels{24};          // max simultaneous decode channels
    uint32_t    l1_interval{64};           // processBulk calls between L1 snapshots
    uint32_t    l1_snapshots{16};          // snapshots to accumulate before probing
    uint32_t    l1_fft_size{4096};         // L1 FFT size
    uint16_t    sync_word{0x12};           // LoRa sync word
    uint16_t    preamble_len{8};           // preamble length
    float       energy_thresh{1e-6f};      // per-symbol energy threshold
    float       min_snr_db{-10.0f};        // minimum SNR for decode
    uint32_t    max_symbols{600};          // max symbols per frame
    uint32_t    overflow_max_per_sweep{5}; // overflows tolerated per sweep before tainting
    bool        debug{false};              // verbose logging

    GR_MAKE_REFLECTABLE(WidebandDecoder, in, out, msg_out,
        sample_rate, center_freq, channel_bw, decode_bw, decode_bw_str,
        decode_sfs_str, min_ratio,
        buffer_ms, max_channels, l1_interval, l1_snapshots, l1_fft_size,
        sync_word, preamble_len, energy_thresh, min_snr_db, max_symbols,
        overflow_max_per_sweep, debug);

    // --- internal state ---

    RingBuffer                _ring;
    std::vector<ChannelSlot>  _slots;

    // L1 energy detection
    gr::algorithm::FFT<cf32>  _fft;
    std::vector<float>        _window;       // Hann window for L1 FFT
    std::vector<float>        _channelEnergy;
    uint32_t                  _nChannels{0};
    uint32_t                  _callCount{0};
    uint32_t                  _snapshotCount{0};
    uint32_t                  _sweepCount{0};
    uint32_t                  _overflowsInSweep{0};
    uint32_t                  _overflowLogSuppress{0}; // suppress overflow log for N calls after first
    uint32_t                  _zeroCalls{0};   // processBulk calls with 0 input samples (per sweep)
    uint32_t                  _totalCalls{0};  // total processBulk calls (per sweep)

    // Hot channel tracking
    std::vector<uint32_t>     _hotChannels;
    std::vector<uint32_t>                _decodeBws;
    std::vector<uint8_t>                 _decodeSfs;   // SF range (empty = all SF7-12)
    std::vector<std::vector<uint32_t>>   _activeChannelMap;  // channelIdx → list of slotIdx

    // DC removal (IIR high-pass: y[n] = x[n] - x[n-1] + alpha * y[n-1])
    cf32                      _dc_prev_in{0.f, 0.f};
    cf32                      _dc_prev_out{0.f, 0.f};
    static constexpr float    _dc_alpha{0.999f};
    std::vector<cf32>         _dcBuf;   // persistent buffer (avoids per-call heap alloc)
    std::vector<cf32>         _fftBuf;  // persistent L1 FFT buffer (avoids ~30/s heap allocs)
    std::vector<float>        _fftBinMag; // per-bin |FFT|^2 from last snapshot (for peak-bin NCO refinement)

    // Noise floor tracking
    float                     _noise_floor_db{-999.f};

    // Telemetry callback (set by app, fires on sweep completion and slot changes)
    std::function<void(const gr::property_map&)> _telemetry;
    std::chrono::steady_clock::time_point        _sweepStartTime;

    // --- lifecycle ---

    void start() {
        // Ring buffer
        const auto bufferSamples = static_cast<std::size_t>(
            buffer_ms * sample_rate / 1000.f);
        _ring.resize(bufferSamples);

        // L1 channel grid
        const float usableBw = sample_rate * 0.8f;
        _nChannels = static_cast<uint32_t>(usableBw / channel_bw);
        _channelEnergy.assign(_nChannels, 0.f);

        // Parse decode_bw_str into _decodeBws
        _decodeBws.clear();
        {
            std::istringstream iss(decode_bw_str);
            std::string token;
            while (std::getline(iss, token, ',')) {
                token.erase(0, token.find_first_not_of(" \t"));
                token.erase(token.find_last_not_of(" \t") + 1);
                if (token.empty()) continue;
                uint32_t bw = 0;
                try { bw = static_cast<uint32_t>(std::stoul(token)); }
                catch (...) {
                    log_ts("warn ", "wideband", "invalid BW token '%s', skipping", token.c_str());
                    continue;
                }
                uint32_t os = static_cast<uint32_t>(
                    std::round(static_cast<double>(sample_rate) / static_cast<double>(bw)));
                if (os == 0 || (os & (os - 1)) != 0) {
                    log_ts("error", "wideband",
                        "BW %u gives non-power-of-2 os=%u at %.0f MS/s, skipping",
                        bw, os, static_cast<double>(sample_rate) / 1e6);
                    continue;
                }
                _decodeBws.push_back(bw);
            }
        }
        std::ranges::sort(_decodeBws);  // narrowest first
        if (_decodeBws.empty()) {
            _decodeBws.push_back(static_cast<uint32_t>(decode_bw));
        }

        // Parse decode_sfs_str into _decodeSfs
        _decodeSfs.clear();
        if (!decode_sfs_str.empty()) {
            std::istringstream iss2(decode_sfs_str);
            std::string token;
            while (std::getline(iss2, token, ',')) {
                token.erase(0, token.find_first_not_of(" \t"));
                token.erase(token.find_last_not_of(" \t") + 1);
                if (token.empty()) continue;
                uint8_t sf = 0;
                try { sf = static_cast<uint8_t>(std::stoul(token)); }
                catch (...) {
                    log_ts("warn ", "wideband", "invalid SF token '%s', skipping", token.c_str());
                    continue;
                }
                if (sf >= 7 && sf <= 12) {
                    _decodeSfs.push_back(sf);
                }
            }
            std::ranges::sort(_decodeSfs);
        }

        _activeChannelMap.assign(_nChannels, {});

        // Channel slots
        _slots.resize(max_channels);

        // L1 FFT Hann window
        _window.resize(l1_fft_size);
        for (uint32_t i = 0; i < l1_fft_size; ++i) {
            _window[i] = 0.5f * (1.f - std::cos(2.f * std::numbers::pi_v<float>
                                                  * static_cast<float>(i)
                                                  / static_cast<float>(l1_fft_size)));
        }
        _fft = gr::algorithm::FFT<cf32>{};

        _dcBuf.reserve(8192);           // typical chunk size
        _fftBuf.reserve(l1_fft_size);   // persistent L1 FFT buffer
        _fftBinMag.assign(l1_fft_size, 0.f);
        _callCount      = 0;
        _snapshotCount  = 0;
        _sweepCount     = 0;
        _sweepStartTime = std::chrono::steady_clock::now();

        {
            std::string sf_desc;
            if (_decodeSfs.empty()) {
                sf_desc = "SF7-12";
            } else {
                for (std::size_t i = 0; i < _decodeSfs.size(); ++i) {
                    if (i > 0) sf_desc += ',';
                    sf_desc += "SF" + std::to_string(_decodeSfs[i]);
                }
            }
            log_ts("info ", "wideband",
                "started: %.1f MS/s, %.3f MHz center, %u ch (%.1f kHz), %u slots, buffer %u ms, %s",
                static_cast<double>(sample_rate) / 1e6,
                static_cast<double>(center_freq) / 1e6,
                _nChannels,
                static_cast<double>(channel_bw) / 1e3,
                max_channels,
                static_cast<unsigned>(buffer_ms),
                sf_desc.c_str());
        }
    }

    // --- processBulk ---

    [[nodiscard]] gr::work::Status processBulk(
        gr::InputSpanLike auto& input,
        gr::OutputSpanLike auto& output) noexcept
    {
        auto in_span = std::span(input);
        ++_totalCalls;
        if (in_span.empty()) ++_zeroCalls;

        // 1. Count overflow tags (informational — do NOT deactivate slots).
        //    At 16 MS/s the B210 produces ~1 overflow per processBulk call.
        //    Deactivating slots would destroy all in-progress decode.
        //    L1 energy detection and SfLane decode are tolerant of occasional
        //    sample discontinuities (missing samples look like noise bursts).
        if (this->inputTagsPresent()) {
            const auto& tag = this->mergedInputTag();
            if (tag.map.contains("overflow")) {
                ++_overflowsInSweep;
            }
        }

        // 2. DC removal (IIR high-pass) — needed when signal is at/near center
        //    frequency where B210 DC spur corrupts the channel.
        _dcBuf.resize(in_span.size());
        for (std::size_t i = 0; i < in_span.size(); ++i) {
            cf32 s = in_span[i];
            cf32 out_val = s - _dc_prev_in + cf32(_dc_alpha, 0.f) * _dc_prev_out;
            _dc_prev_in  = s;
            _dc_prev_out = out_val;
            _dcBuf[i] = out_val;
        }
        auto clean_span = std::span<const cf32>(_dcBuf);

        // 3. Push DC-removed input through all active ChannelSlots.
        //    On overflow (0 input samples), insert zeros to preserve timing.
        constexpr std::size_t kExpectedChunk = 8192;
        const std::size_t gapSize = in_span.empty() ? kExpectedChunk : 0;
        for (auto& slot : _slots) {
            if (slot.state == ChannelSlot::State::Active) {
                slot.pushWideband(clean_span, gapSize);
            }
        }

        // 4. Periodic L1 energy snapshot (from DC-removed span)
        ++_callCount;
        if (_callCount >= l1_interval) {
            _callCount = 0;
            computeEnergySnapshotFromSpan(clean_span);
        }

        // 5. After accumulating enough snapshots, update active channels
        std::size_t out_idx = 0;
        if (_snapshotCount >= l1_snapshots) {
            // Always run hot channel detection — L1 energy is statistical
            // and tolerant of sample discontinuities from overflows.
            // Overflow tags only affect active decode slots (reset in step 1).
            findHotChannels();
            updateActiveChannels();
            _channelEnergy.assign(_nChannels, 0.f);
            _fftBinMag.assign(l1_fft_size, 0.f);
            _snapshotCount = 0;
            uint32_t sweepZeroCalls  = _zeroCalls;
            uint32_t sweepTotalCalls = _totalCalls;
            _zeroCalls = 0;
            _totalCalls = 0;
            uint32_t sweepOverflows = _overflowsInSweep;
            bool wasTainted = (sweepOverflows > overflow_max_per_sweep);
            _overflowsInSweep = 0;
            _sweepCount++;

            // sweep timing
            auto now = std::chrono::steady_clock::now();
            auto durMs = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - _sweepStartTime).count());
            _sweepStartTime = now;

            // active slot count
            uint32_t nActive = 0;
            for (const auto& slot : _slots) {
                if (slot.state != ChannelSlot::State::Idle) ++nActive;
            }

            // telemetry event (every sweep, ~2/s)
            if (_telemetry) {
                try {
                    gr::property_map evt;
                    evt["type"]        = std::pmr::string("wideband_sweep");
                    evt["sweep"]       = _sweepCount;
                    evt["tainted"]     = wasTainted;
                    evt["overflows"]   = sweepOverflows;
                    evt["zero_calls"]  = sweepZeroCalls;
                    evt["total_calls"] = sweepTotalCalls;
                    evt["duration_ms"] = durMs;
                    evt["n_snapshots"] = static_cast<uint32_t>(l1_snapshots);
                    evt["n_hot"]       = static_cast<uint32_t>(_hotChannels.size());
                    evt["n_active"]    = nActive;
                    evt["max_slots"]   = max_channels;
                    _telemetry(evt);
                } catch (...) {}
            }

            // periodic console status (every 10 sweeps)
            if (_sweepCount % 10 == 0) {
                log_ts("info ", "wideband",
                    "sweep %u  hot %zu  active %u/%u  dur %ums  ovf %u%s",
                    _sweepCount,
                    _hotChannels.size(),
                    nActive,
                    static_cast<uint32_t>(max_channels),
                    durMs,
                    sweepOverflows,
                    wasTainted ? " TAINTED" : "");
            }
        }

        // 6. Process SfLane decode from active slot nbAccum buffers
        {
            auto out_span = std::span(output);
            for (auto& slot : _slots) {
                if (slot.state != ChannelSlot::State::Active) continue;
                if (slot.nbAccum.empty()) continue;

                // Feed narrowband samples to each SF lane's accum
                for (auto& lane : slot.sfLanes) {
                    lane.accum.insert(lane.accum.end(),
                                      slot.nbAccum.begin(), slot.nbAccum.end());

                    // Process symbols (os_factor=1: sps == N)
                    std::size_t pos = 0;
                    while (pos + lane.N <= lane.accum.size()) {
                        // At os_factor=1: in_down IS the accum data (no downsampling)
                        std::copy_n(&lane.accum[pos], lane.N, lane.in_down.data());
                        lane.items_to_consume = static_cast<int>(lane.N);

                        switch (lane.state) {
                        case SfLane::DETECT:
                            processDetect(lane, slot);
                            break;
                        case SfLane::SYNC:
                            lane.stall_count++;
                            if (lane.stall_count > SfLane::kMaxStallSymbols) {
                                lane.resetToDetect();
                                break;
                            }
                            processSync(lane, slot, lane.accum, pos);
                            break;
                        case SfLane::OUTPUT:
                            lane.stall_count++;
                            if (lane.stall_count > SfLane::kMaxStallSymbols) {
                                lane.resetToDetect();
                                break;
                            }
                            processOutput(lane, slot, out_span, out_idx);
                            break;
                        }

                        auto consumed = static_cast<std::size_t>(
                            std::max(lane.items_to_consume, 1));
                        pos += consumed;
                    }

                    // Erase consumed samples
                    if (pos > 0) {
                        lane.accum.erase(lane.accum.begin(),
                                          lane.accum.begin()
                                          + static_cast<std::ptrdiff_t>(pos));
                    }
                }

                slot.nbAccum.clear();
            }
        }

        // Always consume all input (scheduler fast path)
        std::ignore = input.consume(in_span.size());
        output.publish(out_idx);
        return gr::work::Status::OK;
    }

    // --- L1 energy detection (adapted from ScanController) ---

    /// Compute L1 energy snapshot directly from the input span (no ring buffer).
    /// Uses the last l1_fft_size samples from the input span.
    void computeEnergySnapshotFromSpan(std::span<const cf32> span) noexcept {
        if (span.size() < l1_fft_size) return;

        // Use the last l1_fft_size samples from the span
        auto src = span.subspan(span.size() - l1_fft_size, l1_fft_size);
        _fftBuf.resize(l1_fft_size);
        std::copy(src.begin(), src.end(), _fftBuf.begin());

        // Apply Hann window
        for (uint32_t i = 0; i < l1_fft_size; ++i) {
            _fftBuf[i] *= _window[i];
        }

        auto fftOut = _fft.compute(
            std::span<const cf32>(_fftBuf.data(), l1_fft_size));

        // Compute |FFT|^2 per bin and store for peak-bin NCO refinement.
        // Bins are in fftshift domain: bin 0 = most negative freq.
        const auto half    = l1_fft_size / 2;
        const float binBw  = sample_rate / static_cast<float>(l1_fft_size);
        const auto binsPerCh = static_cast<uint32_t>(channel_bw / binBw);

        const float usableBw = sample_rate * 0.8f;
        const auto  startBin = static_cast<uint32_t>(std::round(
            (static_cast<float>(l1_fft_size) - usableBw / binBw) / 2.f));

        if (_fftBinMag.size() < l1_fft_size) {
            _fftBinMag.assign(l1_fft_size, 0.f);
        }
        for (uint32_t i = 0; i < l1_fft_size; ++i) {
            const auto fftIdx = (i + half) % l1_fft_size;
            const auto& s = fftOut[fftIdx];
            _fftBinMag[i] += s.real() * s.real() + s.imag() * s.imag();
        }

        // Accumulate into channel bins (from fftOut directly, not _fftBinMag,
        // to avoid quadratic growth — _fftBinMag accumulates across snapshots)
        for (uint32_t ch = 0; ch < _nChannels
             && ch * binsPerCh + startBin + binsPerCh <= l1_fft_size; ++ch) {
            float energy = 0.f;
            for (uint32_t b = 0; b < binsPerCh; ++b) {
                const auto fftIdx = (startBin + ch * binsPerCh + b + half) % l1_fft_size;
                const auto& s = fftOut[fftIdx];
                energy += s.real() * s.real() + s.imag() * s.imag();
            }
            _channelEnergy[ch] += energy;
        }

        ++_snapshotCount;
    }

    // --- Hot channel detection (adapted from ScanController) ---

    void findHotChannels() noexcept {
        _hotChannels.clear();
        if (_nChannels == 0 || _snapshotCount == 0) return;

        // Threshold: channels with energy > 6x median
        std::vector<float> sorted(_channelEnergy.begin(),
                                  _channelEnergy.begin()
                                  + static_cast<std::ptrdiff_t>(_nChannels));
        std::ranges::sort(sorted);
        const float median = sorted[sorted.size() / 2];

        constexpr float kHotMultiplier = 6.0f;
        const float threshold = median * kHotMultiplier;

        std::vector<uint32_t> raw;
        for (uint32_t ch = 0; ch < _nChannels; ++ch) {
            if (_channelEnergy[ch] > threshold) {
                raw.push_back(ch);
            }
        }

        // Cluster dedup: merge adjacent hot channels, keep peak per cluster
        std::size_t i = 0;
        while (i < raw.size()) {
            uint32_t bestCh = raw[i];
            float    bestE  = _channelEnergy[bestCh];
            std::size_t j = i + 1;
            while (j < raw.size() && raw[j] == raw[j - 1] + 1) {
                if (_channelEnergy[raw[j]] > bestE) {
                    bestCh = raw[j];
                    bestE  = _channelEnergy[raw[j]];
                }
                ++j;
            }
            _hotChannels.push_back(bestCh);
            i = j;
        }
    }

    // --- Channel pool management ---

    void updateActiveChannels() noexcept {
        // 1. Deactivate slots for channels no longer hot (with minimum hold time)
        constexpr uint32_t kMinHoldSweeps = 20;  // ~660ms at 33ms/sweep, ~10s at 524ms
        for (uint32_t ch = 0; ch < _nChannels; ++ch) {
            if (_activeChannelMap[ch].empty()) continue;
            // Increment sweep counter for all active slots on this channel
            for (uint32_t slotIdx : _activeChannelMap[ch]) {
                ++_slots[slotIdx].activeSweeps;
            }
            // Keep slot alive if nearby channel is hot OR minimum hold not reached
            constexpr uint32_t kSlotKeepRadius = 2;
            bool nearbyHot = false;
            for (uint32_t h : _hotChannels) {
                uint32_t dist = (h > ch) ? (h - ch) : (ch - h);
                if (dist <= kSlotKeepRadius) { nearbyHot = true; break; }
            }
            bool holdActive = false;
            for (uint32_t slotIdx : _activeChannelMap[ch]) {
                if (_slots[slotIdx].activeSweeps < kMinHoldSweeps) {
                    holdActive = true; break;
                }
            }
            if (!nearbyHot && !holdActive) {
                for (uint32_t slotIdx : _activeChannelMap[ch]) {
                    if (_telemetry) {
                        try {
                            gr::property_map evt;
                            evt["type"]    = std::pmr::string("wideband_slot");
                            evt["action"]  = std::pmr::string("deactivate");
                            evt["slot"]    = slotIdx;
                            evt["channel"] = ch;
                            evt["freq"]    = _slots[slotIdx].channelFreq;
                            evt["bw"]      = _slots[slotIdx].decodeBw;
                            _telemetry(evt);
                        } catch (...) {}
                    }
                    _slots[slotIdx].deactivate();
                }
                _activeChannelMap[ch].clear();
            }
        }

        // 2. Activate slots for new hot channels (one per BW per channel)
        for (uint32_t ch : _hotChannels) {
            if (ch >= _nChannels) continue;
            for (uint32_t bw : _decodeBws) {
                // Check if a nearby channel already has an active slot at this BW
                constexpr uint32_t kSlotActivateRadius = 2;
                bool alreadyActive = false;
                uint32_t lo = (ch > kSlotActivateRadius) ? (ch - kSlotActivateRadius) : 0;
                uint32_t hi = std::min(ch + kSlotActivateRadius, _nChannels - 1);
                for (uint32_t adj = lo; adj <= hi; ++adj) {
                    for (uint32_t slotIdx : _activeChannelMap[adj]) {
                        if (_slots[slotIdx].decodeBw == bw) { alreadyActive = true; break; }
                    }
                    if (alreadyActive) break;
                }
                if (alreadyActive) continue;

                uint32_t freeSlot = UINT32_MAX;
                for (uint32_t s = 0; s < static_cast<uint32_t>(_slots.size()); ++s) {
                    if (_slots[s].state == ChannelSlot::State::Idle) { freeSlot = s; break; }
                }
                if (freeSlot == UINT32_MAX) {
                    if (debug) log_ts("debug", "wideband", "no free slot for ch %u BW %u", ch, bw);
                    break;
                }

                double freq = channelCenterFreq(ch);
                uint32_t os = static_cast<uint32_t>(
                    std::round(static_cast<double>(sample_rate) / static_cast<double>(bw)));

                _slots[freeSlot].activate(freq, static_cast<double>(center_freq),
                    static_cast<double>(sample_rate), os, ch, bw, sync_word, preamble_len,
                    std::span<const uint8_t>(_decodeSfs));
                _activeChannelMap[ch].push_back(freeSlot);

                // No ring buffer replay — start decode from the live stream.
                // The L1 sweep fires every ~500ms; a LoRa preamble lasts
                // 8 symbols (~33ms at SF8/BW62.5k), so the next ADVERT will
                // be caught from the start.

                if (_telemetry) {
                    try {
                        gr::property_map evt;
                        evt["type"]    = std::pmr::string("wideband_slot");
                        evt["action"]  = std::pmr::string("activate");
                        evt["slot"]    = freeSlot;
                        evt["channel"] = ch;
                        evt["freq"]    = freq;
                        evt["bw"]      = bw;
                        _telemetry(evt);
                    } catch (...) {}
                }
            }
        }
    }

    // --- Helpers ---

    [[nodiscard]] double channelCenterFreq(uint32_t ch) const noexcept {
        const double binBw   = static_cast<double>(sample_rate) / l1_fft_size;
        const double usableBins = static_cast<double>(sample_rate) * 0.8 / binBw;
        const auto   startBin   = static_cast<uint32_t>(std::round(
            (static_cast<double>(l1_fft_size) - usableBins) / 2.0));
        const auto   bpC = static_cast<uint32_t>(static_cast<double>(channel_bw) / binBw);
        const double centerBin = startBin + ch * bpC + bpC / 2.0;
        return static_cast<double>(center_freq)
             + (centerBin - static_cast<double>(l1_fft_size) / 2.0) * binBw;
    }

    // =========================================================================
    // DETECT state: look for consecutive upchirps matching the preamble
    // Adapted from MultiSfDecoder for os_factor=1 (no downsampling needed)
    // =========================================================================
    void processDetect(SfLane& lane, ChannelSlot& /*slot*/) {
        auto [dechirp_bin, dechirp_pmr] = dechirp_and_quality(
            lane.in_down.data(), lane.downchirp.data(),
            lane.scratch_N.data(), lane.N, lane.fft, /*remove_dc=*/true);
        lane.bin_idx_new = static_cast<int32_t>(dechirp_bin);

        if (!lane.has_energy(lane.in_down.data(), lane.N, energy_thresh)) {
            lane.symbol_cnt = 1;
            lane.bin_idx = -1;
            return;
        }

        if (dechirp_pmr < sflane_detail::kMinPreamblePMR) {
            lane.symbol_cnt = 1;
            lane.bin_idx = -1;
            return;
        }

        // Consecutive upchirp check
        if (std::abs(mod(std::abs(lane.bin_idx_new - lane.bin_idx) + 1,
                         static_cast<int64_t>(lane.N)) - 1)
                <= sflane_detail::kPreambleBinTolerance && lane.bin_idx_new != -1) {
            if (lane.symbol_cnt == 1 && lane.bin_idx != -1) {
                lane.preamb_up_vals[0] = lane.bin_idx;
            }
            auto sc = static_cast<uint32_t>(lane.symbol_cnt);
            if (sc < lane.preamb_up_vals.size()) {
                lane.preamb_up_vals[sc] = lane.bin_idx_new;
            }
            if (static_cast<std::size_t>(sc) * lane.N < lane.preamble_raw.size()) {
                std::copy_n(lane.in_down.data(), lane.N,
                            &lane.preamble_raw[static_cast<std::size_t>(sc) * lane.N]);
            }
            // os_factor=1: preamble_raw_up gets same data as preamble_raw
            if (static_cast<std::size_t>(sc) * lane.N + lane.N <= lane.preamble_raw_up.size()) {
                std::copy_n(lane.in_down.data(), lane.N,
                            &lane.preamble_raw_up[static_cast<std::size_t>(sc) * lane.N]);
            }
            lane.symbol_cnt++;
        } else {
            std::copy_n(lane.in_down.data(), lane.N, &lane.preamble_raw[0]);
            std::copy_n(lane.in_down.data(), lane.N, &lane.preamble_raw_up[0]);
            lane.symbol_cnt = 1;
        }
        lane.bin_idx = lane.bin_idx_new;

        if (static_cast<uint32_t>(lane.symbol_cnt) == lane.n_up_req) {
            lane.additional_upchirps = 0;
            lane.state = SfLane::SYNC;
            lane.sync_phase = SfLane::NET_ID1;
            lane.symbol_cnt = 0;
            lane.cfo_frac_sto_frac_est = false;
            lane.k_hat = sflane_detail::most_frequent(lane.preamb_up_vals);

            if (debug) {
                log_ts("debug", "wideband", "SF%u DETECT->SYNC: k_hat=%d",
                       lane.sf, lane.k_hat);
            }

            // At os_factor=1, alignment is N - k_hat samples
            lane.items_to_consume = static_cast<int>(lane.N)
                                  - lane.k_hat;
        }
    }

    // =========================================================================
    // SYNC state: CFO/STO estimation, sync word verification
    // Adapted from MultiSfDecoder for os_factor=1
    // =========================================================================
    void processSync(SfLane& lane, ChannelSlot& slot,
                     std::span<const cf32> accum, std::size_t pos) {
        if (!lane.cfo_frac_sto_frac_est) {
            auto cfo_offset = static_cast<std::size_t>(lane.N)
                + static_cast<std::size_t>(std::max(static_cast<int>(lane.N) - lane.k_hat, 0));
            if (cfo_offset + lane.up_symb_to_use * lane.N > lane.preamble_raw.size()) {
                cfo_offset = static_cast<std::size_t>(lane.N);
            }
            lane.cfo_frac = lane.estimate_CFO_frac_Bernier(&lane.preamble_raw[cfo_offset]);
            lane.sto_frac = lane.estimate_STO_frac();

            for (uint32_t n = 0; n < lane.N; n++) {
                float phase = -2.f * static_cast<float>(std::numbers::pi) * lane.cfo_frac
                            / static_cast<float>(lane.N) * static_cast<float>(n);
                lane.CFO_frac_correc[n] = cf32(std::cos(phase), std::sin(phase));
            }
            lane.cfo_frac_sto_frac_est = true;
        }

        // Apply CFO frac correction
        sflane_detail::complex_multiply(lane.symb_corr.data(), lane.in_down.data(),
                                  lane.CFO_frac_correc.data(), lane.N);
        int32_t bin_idx_sync = static_cast<int32_t>(
            lane.get_symbol_val(lane.symb_corr.data(), lane.downchirp.data()));

        switch (lane.sync_phase) {
        case SfLane::NET_ID1: {
            if (bin_idx_sync == 0 || bin_idx_sync == 1
                || static_cast<uint32_t>(bin_idx_sync) == lane.N - 1) {
                // Additional upchirp — at os_factor=1, just copy in_down to net_id_samp
                uint32_t copy_len = std::min(lane.N / 4,
                                              static_cast<uint32_t>(lane.net_id_samp.size()));
                if (copy_len <= lane.N) {
                    std::copy_n(&lane.in_down[lane.N * 3 / 4], std::min(copy_len, lane.N / 4),
                                lane.net_id_samp.data());
                }

                if (lane.additional_upchirps >= sflane_detail::kMaxAdditionalUpchirps) {
                    if (lane.preamble_rotations >= sflane_detail::kMaxPreambleRotations) {
                        lane.resetToDetect();
                        lane.items_to_consume = 0;
                        return;
                    }
                    std::rotate(lane.preamble_raw_up.begin(),
                                lane.preamble_raw_up.begin() + static_cast<std::ptrdiff_t>(lane.N),
                                lane.preamble_raw_up.end());
                    // Store new upchirp at end of preamble_raw_up
                    // (at os_factor=1, alignment already handled by items_to_consume)
                    {
                        std::size_t dst = static_cast<std::size_t>(lane.n_up_req + 3) * lane.N;
                        if (dst + lane.N <= lane.preamble_raw_up.size()) {
                            std::copy_n(lane.in_down.data(), lane.N, &lane.preamble_raw_up[dst]);
                        }
                    }
                    lane.preamble_rotations++;
                } else {
                    // Store additional upchirp in preamble_raw_up
                    {
                        std::size_t dst = static_cast<std::size_t>(
                            lane.n_up_req + lane.additional_upchirps) * lane.N;
                        if (dst + lane.N <= lane.preamble_raw_up.size()) {
                            std::copy_n(lane.in_down.data(), lane.N, &lane.preamble_raw_up[dst]);
                        }
                    }
                    lane.additional_upchirps++;
                }
            } else {
                lane.sync_phase = SfLane::NET_ID2;
                // At os_factor=1: copy net ID samples from in_down
                std::size_t dst_off = lane.N / 4;
                std::size_t copy_len = std::min(static_cast<std::size_t>(lane.N),
                                                  lane.net_id_samp.size() - dst_off);
                std::copy_n(lane.in_down.data(),
                            std::min(copy_len, static_cast<std::size_t>(lane.N)),
                            &lane.net_id_samp[dst_off]);
            }
            break;
        }
        case SfLane::NET_ID2: {
            lane.sync_phase = SfLane::DOWNCHIRP1;
            std::size_t dst_off = lane.N + lane.N / 4;
            if (dst_off < lane.net_id_samp.size()) {
                std::size_t copy_len = std::min(static_cast<std::size_t>(lane.N),
                                                  lane.net_id_samp.size() - dst_off);
                std::copy_n(lane.in_down.data(),
                            std::min(copy_len, static_cast<std::size_t>(lane.N)),
                            &lane.net_id_samp[dst_off]);
            }
            break;
        }
        case SfLane::DOWNCHIRP1: {
            // Copy quarter-down samples to net_id_samp for sync word verification
            // (MultiSfDecoder stores sps/4 samples at offset sps*2 + sps/4)
            {
                std::size_t dst_off = lane.N * 2 + lane.N / 4;
                if (dst_off < lane.net_id_samp.size()) {
                    std::size_t copy_len = std::min(
                        static_cast<std::size_t>(lane.N / 4),
                        lane.net_id_samp.size() - dst_off);
                    std::copy_n(lane.in_down.data(),
                                std::min(copy_len, static_cast<std::size_t>(lane.N)),
                                &lane.net_id_samp[dst_off]);
                }
            }
            lane.sync_phase = SfLane::DOWNCHIRP2;
            break;
        }
        case SfLane::DOWNCHIRP2: {
            lane.down_val = static_cast<int>(
                lane.get_symbol_val(lane.symb_corr.data(), lane.upchirp.data()));
            lane.sync_phase = SfLane::QUARTER_DOWN;
            break;
        }
        case SfLane::QUARTER_DOWN: {
            // Integer CFO from downchirp
            if (static_cast<uint32_t>(lane.down_val) < lane.N / 2) {
                lane.cfo_int = lane.down_val / 2;
            } else {
                lane.cfo_int = (lane.down_val - static_cast<int>(lane.N)) / 2;
            }

            // Correct STOint and CFOint in preamble upchirps
            auto rot_off = static_cast<std::size_t>(
                mod(static_cast<int64_t>(lane.cfo_int), static_cast<int64_t>(lane.N)));
            std::rotate(lane.preamble_upchirps.begin(),
                        lane.preamble_upchirps.begin() + static_cast<std::ptrdiff_t>(rot_off),
                        lane.preamble_upchirps.end());

            uint32_t n_corr_syms = lane.n_up_req + lane.additional_upchirps;
            std::size_t corr_len = static_cast<std::size_t>(n_corr_syms) * lane.N;
            for (std::size_t n = 0; n < corr_len; n++) {
                float phase = -2.f * static_cast<float>(std::numbers::pi)
                            * static_cast<float>(lane.cfo_int)
                            / static_cast<float>(lane.N) * static_cast<float>(n);
                lane.corr_vec[n] = cf32(std::cos(phase), std::sin(phase));
            }
            sflane_detail::complex_multiply(lane.preamble_upchirps.data(), lane.preamble_upchirps.data(),
                                      lane.corr_vec.data(), lane.up_symb_to_use * lane.N);

            // SFO estimation: use slot's channel freq as center for SFO calc
            lane.sfo_hat = (static_cast<float>(lane.cfo_int) + lane.cfo_frac)
                          * static_cast<float>(lane.bandwidth)
                          / static_cast<float>(slot.channelFreq);

            // SFO correction in preamble
            double clk_off = static_cast<double>(lane.sfo_hat) / lane.N;
            double fs   = static_cast<double>(lane.bandwidth);
            double fs_p = static_cast<double>(lane.bandwidth) * (1.0 - clk_off);
            int N_int = static_cast<int>(lane.N);
            for (std::size_t n = 0; n < corr_len; n++) {
                double n_mod = static_cast<double>(
                    mod(static_cast<int64_t>(n), static_cast<int64_t>(N_int)));
                double floor_n_N = std::floor(static_cast<double>(n) / N_int);
                double phase_d = -2.0 * std::numbers::pi * (
                    n_mod * n_mod / (2.0 * N_int)
                    * (lane.bandwidth / fs_p * lane.bandwidth / fs_p - lane.bandwidth / fs * lane.bandwidth / fs)
                    + (floor_n_N * (lane.bandwidth / fs_p * lane.bandwidth / fs_p - lane.bandwidth / fs_p)
                       + lane.bandwidth / 2.0 * (1.0 / fs - 1.0 / fs_p)) * n_mod);
                lane.corr_vec[n] = cf32(static_cast<float>(std::cos(phase_d)),
                                         static_cast<float>(std::sin(phase_d)));
            }
            sflane_detail::complex_multiply(lane.preamble_upchirps.data(), lane.preamble_upchirps.data(),
                                      lane.corr_vec.data(), lane.up_symb_to_use * lane.N);

            // Re-estimate STO frac after SFO correction
            float tmp_sto_frac = lane.estimate_STO_frac();
            // At os_factor=1, no STO threshold filtering needed
            lane.sto_frac = tmp_sto_frac;

            // Update sto_frac to net_id position
            lane.sto_frac += lane.sfo_hat * static_cast<float>(preamble_len);
            if (std::abs(lane.sto_frac) > 0.5f) {
                lane.sto_frac += (lane.sto_frac > 0 ? -1.f : 1.f);
            }

            // Decimate net_id for sync word verification (os_factor=1: no decimation)
            std::fill(lane.scratch_2N.begin(), lane.scratch_2N.end(), cf32(0.f, 0.f));
            int start_off = -sflane_detail::my_roundf(lane.sto_frac)
                          + static_cast<int>(lane.N / 4) + lane.cfo_int;
            for (uint32_t i = 0; i < lane.N * 2; i++) {
                int idx = start_off + static_cast<int>(i);
                if (idx >= 0 && static_cast<std::size_t>(idx) < lane.net_id_samp.size()) {
                    lane.scratch_2N[i] = lane.net_id_samp[static_cast<std::size_t>(idx)];
                }
            }

            // Apply CFO corrections to net ID
            for (uint32_t n = 0; n < 2 * lane.N; n++) {
                float phase = -2.f * static_cast<float>(std::numbers::pi)
                            * static_cast<float>(lane.cfo_int)
                            / static_cast<float>(lane.N) * static_cast<float>(n);
                lane.corr_vec[n] = cf32(std::cos(phase), std::sin(phase));
            }
            sflane_detail::complex_multiply(lane.scratch_2N.data(), lane.scratch_2N.data(),
                                      lane.corr_vec.data(), 2 * lane.N);
            sflane_detail::complex_multiply(lane.scratch_2N.data(), lane.scratch_2N.data(),
                                      lane.CFO_frac_correc.data(), lane.N);
            sflane_detail::complex_multiply(&lane.scratch_2N[lane.N], &lane.scratch_2N[lane.N],
                                      lane.CFO_frac_correc.data(), lane.N);

            int netid1 = static_cast<int>(
                lane.get_symbol_val(lane.scratch_2N.data(), lane.downchirp.data()));
            int netid2 = static_cast<int>(
                lane.get_symbol_val(&lane.scratch_2N[lane.N], lane.downchirp.data()));

            // Sync word verification
            bool sync_ok = false;
            int net_id_off = 0;
            if (lane.sw0 == 0) {
                sync_ok = true;
            } else if (std::abs(netid1 - static_cast<int>(lane.sw0)) > sflane_detail::kSyncWordBinTolerance) {
                // wrong
            } else {
                net_id_off = netid1 - static_cast<int>(lane.sw0);
                if (lane.sw1 != 0 && mod(netid2 - net_id_off, static_cast<int64_t>(lane.N))
                    != static_cast<int64_t>(lane.sw1)) {
                    // wrong
                } else {
                    sync_ok = true;
                }
            }

            if (!sync_ok) {
                lane.resetToDetect();
                lane.items_to_consume = 0;
                return;
            }

            // Apply sync word timing offset (at os_factor=1)
            lane.items_to_consume = -net_id_off;

            auto [snr, sig] = lane.determine_snr();
            lane.snr_db    = snr;
            lane.signal_db = sig;
            if (lane.snr_db < min_snr_db) {
                lane.resetToDetect();
                lane.items_to_consume = 0;
                return;
            }

            // Transition to OUTPUT
            lane.state = SfLane::OUTPUT;
            lane.output_symb_cnt = 0;
            lane.symbol_buffer.clear();
            lane.nibbles.clear();
            lane.total_symbols_rx = 0;
            lane.symb_numb = 0;
            lane.cr = 4;

            // Update sto_frac to payload beginning
            lane.sto_frac += lane.sfo_hat * 4.25f;
            lane.sfo_cum = lane.sto_frac
                          - static_cast<float>(sflane_detail::my_roundf(lane.sto_frac));

            // Adjust items_to_consume for quarter-down skip + CFO alignment
            lane.items_to_consume += static_cast<int>(lane.N / 4) + lane.cfo_int;

            // Build CFO-corrected downchirp for demodulation
            lane.buildDownchirpWithCFO();

            // Compute LDRO
            lane.ldro = (static_cast<float>(1u << lane.sf) * 1e3f
                        / static_cast<float>(lane.bandwidth)) > LDRO_MAX_DURATION_MS;

            if (debug) {
                log_ts("debug", "wideband",
                    "SF%u SYNC->OUTPUT: cfo_int=%d cfo_frac=%.3f snr=%.1f dB freq=%.3f MHz",
                    lane.sf, lane.cfo_int, static_cast<double>(lane.cfo_frac),
                    static_cast<double>(lane.snr_db), slot.channelFreq / 1e6);
            }
            break;
        }
        }  // switch sync_phase
    }

    // =========================================================================
    // OUTPUT state: demodulate symbols + decode
    // Adapted from MultiSfDecoder for os_factor=1
    // =========================================================================
    void processOutput(SfLane& lane, ChannelSlot& slot,
                       std::span<uint8_t> out_span, std::size_t& out_idx) {
        // Termination
        uint32_t symb_limit = (lane.frame_symb_numb > 0) ? lane.frame_symb_numb : max_symbols;
        if (!lane.has_energy(lane.in_down.data(), lane.N, energy_thresh)
            || lane.output_symb_cnt >= symb_limit) {
            if (lane.symb_numb > 0 && lane.total_symbols_rx >= lane.symb_numb) {
                finishFrame(lane, slot, out_span, out_idx);
            }
            lane.resetToDetect();
            return;
        }

        // Buffer first 8 symbols for early header decode
        if (lane.output_symb_cnt < 8 && !lane.header_decoded) {
            std::copy_n(lane.in_down.data(), lane.N,
                        &lane.header_symbols[static_cast<std::size_t>(lane.output_symb_cnt) * lane.N]);
        }

        // Demodulate this symbol
        uint16_t symbol = lane.demodSymbol(lane.in_down.data());
        lane.symbol_buffer.push_back(symbol);
        lane.total_symbols_rx++;
        lane.output_symb_cnt++;

        // SFO tracking (os_factor=1: simpler threshold)
        if (std::abs(lane.sfo_cum) > 0.5f) {
            int sign_val = (lane.sfo_cum > 0) ? 1 : -1;
            lane.items_to_consume -= sign_val;
            lane.sfo_cum -= static_cast<float>(sign_val);
        }
        lane.sfo_cum += lane.sfo_hat;

        // After 8th symbol: decode header
        if (lane.output_symb_cnt == 8 && !lane.header_decoded) {
            if (debug) {
                log_ts("debug", "wideband",
                    "SF%u header symbols: %u %u %u %u %u %u %u %u",
                    lane.sf,
                    lane.symbol_buffer.size() > 0 ? lane.symbol_buffer[0] : 0,
                    lane.symbol_buffer.size() > 1 ? lane.symbol_buffer[1] : 0,
                    lane.symbol_buffer.size() > 2 ? lane.symbol_buffer[2] : 0,
                    lane.symbol_buffer.size() > 3 ? lane.symbol_buffer[3] : 0,
                    lane.symbol_buffer.size() > 4 ? lane.symbol_buffer[4] : 0,
                    lane.symbol_buffer.size() > 5 ? lane.symbol_buffer[5] : 0,
                    lane.symbol_buffer.size() > 6 ? lane.symbol_buffer[6] : 0,
                    lane.symbol_buffer.size() > 7 ? lane.symbol_buffer[7] : 0);
            }
            lane.decodeHeader();
        }

        // Decode pipeline
        if (lane.total_symbols_rx <= 8) {
            // HEADER block
            if (lane.symbol_buffer.size() == 8) {
                auto header_nibbles = lane.processBlock(
                    lane.symbol_buffer, static_cast<uint8_t>(lane.sf - 2), 8, 4, true);
                lane.nibbles.insert(lane.nibbles.end(),
                                     header_nibbles.begin(), header_nibbles.end());
                lane.symbol_buffer.clear();

                if (lane.nibbles.size() >= 5) {
                    auto info = parse_explicit_header(
                        lane.nibbles[0], lane.nibbles[1], lane.nibbles[2],
                        lane.nibbles[3], lane.nibbles[4]);

                    if (debug) {
                        log_ts("debug", "wideband",
                            "SF%u header: pay_len=%u cr=%u crc=%u chk=%s nibs=%x %x %x %x %x",
                            lane.sf, info.payload_len, info.cr, info.has_crc ? 1 : 0,
                            info.checksum_valid ? "OK" : "FAIL",
                            lane.nibbles[0], lane.nibbles[1], lane.nibbles[2],
                            lane.nibbles[3], lane.nibbles[4]);
                    }
                    if (!info.checksum_valid || info.payload_len == 0) {
                        lane.resetToDetect();
                        return;
                    }

                    lane.pay_len = info.payload_len;
                    lane.cr      = info.cr;
                    lane.has_crc = info.has_crc;
                    lane.symb_numb = 8 + static_cast<uint32_t>(
                        std::ceil(static_cast<double>(
                            2 * lane.pay_len - lane.sf + 2 + 5 + (lane.has_crc ? 4 : 0))
                            / (lane.sf - 2 * static_cast<int>(lane.ldro))))
                        * (4 + lane.cr);
                }
            }
        } else {
            // PAYLOAD blocks
            uint8_t cw_len = 4 + lane.cr;
            if (lane.symbol_buffer.size() == cw_len) {
                uint8_t sf_app = lane.ldro ? static_cast<uint8_t>(lane.sf - 2) : lane.sf;
                auto payload_nibbles = lane.processBlock(
                    lane.symbol_buffer, sf_app, cw_len, lane.cr, false);
                lane.nibbles.insert(lane.nibbles.end(),
                                     payload_nibbles.begin(), payload_nibbles.end());
                lane.symbol_buffer.clear();
            }

            // Frame complete
            if (lane.symb_numb > 0 && lane.total_symbols_rx >= lane.symb_numb) {
                finishFrame(lane, slot, out_span, out_idx);
                lane.resetToDetect();
            }
        }
    }

    /// Flush remaining symbols, decode frame, emit output bytes + tags + message.
    void finishFrame(SfLane& lane, ChannelSlot& slot,
                     std::span<uint8_t> out_span, std::size_t& out_idx) {
        // Flush remaining symbol buffer
        if (!lane.symbol_buffer.empty()) {
            uint8_t cw_len = 4 + lane.cr;
            uint8_t sf_app = lane.ldro ? static_cast<uint8_t>(lane.sf - 2) : lane.sf;
            while (lane.symbol_buffer.size() < cw_len) {
                lane.symbol_buffer.push_back(0);
            }
            auto last_nibbles = lane.processBlock(
                lane.symbol_buffer, sf_app, cw_len, lane.cr, false);
            lane.nibbles.insert(lane.nibbles.end(),
                                 last_nibbles.begin(), last_nibbles.end());
            lane.symbol_buffer.clear();
        }

        auto frame = lane.decodeFrame();
        if (frame.payload.empty()) return;

        // Copy to output
        for (uint32_t i = 0; i < frame.pay_len && out_idx < out_span.size(); i++) {
            if (i < frame.payload.size()) {
                out_span[out_idx++] = frame.payload[i];
            }
        }

        // Publish output tag with channel info
        gr::property_map out_tag;
        out_tag["sf"]            = gr::pmt::Value(static_cast<int64_t>(lane.sf));
        out_tag["pay_len"]       = gr::pmt::Value(static_cast<int64_t>(frame.pay_len));
        out_tag["cr"]            = gr::pmt::Value(static_cast<int64_t>(frame.cr));
        out_tag["crc_valid"]     = gr::pmt::Value(frame.crc_valid);
        out_tag["is_downchirp"]  = gr::pmt::Value(false);
        out_tag["snr_db"]        = gr::pmt::Value(static_cast<double>(lane.snr_db));
        out_tag["channel_freq"]  = gr::pmt::Value(slot.channelFreq);
        out_tag["decode_bw"]     = gr::pmt::Value(static_cast<double>(slot.decodeBw));
        out_tag["cfo_int"]       = gr::pmt::Value(static_cast<int64_t>(lane.cfo_int));
        out_tag["cfo_frac"]      = gr::pmt::Value(static_cast<double>(lane.cfo_frac));
        out_tag["sfo_hat"]       = gr::pmt::Value(static_cast<double>(lane.sfo_hat));
        if (_noise_floor_db > -999.f) {
            out_tag["noise_floor_db"] = gr::pmt::Value(static_cast<double>(_noise_floor_db));
        }
        this->publishTag(out_tag, 0UZ);

        // Publish message
        std::string payload_str;
        for (uint32_t i = 0; i < frame.pay_len && i < frame.payload.size(); i++) {
            payload_str.push_back(static_cast<char>(frame.payload[i]));
        }
        gr::property_map msg_data;
        msg_data["payload"]       = gr::pmt::Value(payload_str);
        msg_data["crc_valid"]     = gr::pmt::Value(frame.crc_valid);
        msg_data["is_downchirp"]  = gr::pmt::Value(false);
        gr::sendMessage<gr::message::Command::Notify>(msg_out, "", "payload", msg_data);

        if (debug) {
            log_ts("info ", "wideband",
                   "SF%u frame @ %.3f MHz BW%u: pay_len=%u cr=%u crc=%s snr=%.1f dB",
                   lane.sf, slot.channelFreq / 1e6, slot.decodeBw,
                   frame.pay_len, frame.cr,
                   frame.crc_valid ? "OK" : "FAIL",
                   static_cast<double>(lane.snr_db));
        }

        // Narrowest-BW-first dedup: suppress wider-BW slots on same channel
        if (frame.crc_valid) {
            for (uint32_t peerSlotIdx : _activeChannelMap[slot.channelIdx]) {
                auto& peer = _slots[peerSlotIdx];
                if (peer.decodeBw > slot.decodeBw
                    && peer.state == ChannelSlot::State::Active) {
                    peer.state = ChannelSlot::State::Draining;
                    for (auto& peerLane : peer.sfLanes) {
                        peerLane.resetToDetect();
                    }
                    if (debug) {
                        log_ts("debug", "wideband",
                            "dedup: draining slot BW %u (ch %u) after BW %u decode",
                            peer.decodeBw, slot.channelIdx, slot.decodeBw);
                    }
                }
            }
        }
    }

    // --- Helpers ---

    [[nodiscard]] uint32_t activeSlotCount() const noexcept {
        uint32_t count = 0;
        for (const auto& slot : _slots) {
            if (slot.state != ChannelSlot::State::Idle) ++count;
        }
        return count;
    }
};

}  // namespace gr::lora

#endif  // GNURADIO_LORA_WIDEBAND_DECODER_HPP
