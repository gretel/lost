// SPDX-License-Identifier: ISC
/// Tests for TOML configuration parsing (apps/config.cpp).
///
/// Suite 1: load_config — TRX mode parsing, field values, error cases
/// Suite 2: load_scan_config — scan mode parsing, validation, error cases

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <boost/ut.hpp>

#include "../apps/config.hpp"

#ifndef CONFIG_TOML_PATH
#define CONFIG_TOML_PATH "apps/config.toml"
#endif

using namespace boost::ut;

namespace {

const std::filesystem::path kTmpDir{"tmp"};

/// Write a string to a temp TOML file and return its path.
std::string write_tmp_toml(const std::string& name, const std::string& content) {
    std::filesystem::create_directories(kTmpDir);
    auto path = (kTmpDir / name).string();
    std::ofstream ofs(path);
    ofs << content;
    return path;
}

}  // namespace

// ── Suite 1: config load_config ────────────────────────────────────────────

const boost::ut::suite configLoadConfig = [] {

    "reference config loads"_test = [] {
        auto cfgs = lora_config::load_config(CONFIG_TOML_PATH);
        expect(cfgs.size() == 1_u) << "expected 1 TrxConfig";
        const auto& c = cfgs[0];

        // Device
        expect(eq(c.device, std::string{"uhd"}));
        expect(eq(c.device_param, std::string{"type=b200"}));
        expect(eq(c.clock, std::string{"external"}));

        // Radio
        expect(c.freq == 869'618'000.0) << "freq";
        expect(c.gain_rx == 35.0) << "gain_rx";
        expect(c.gain_tx == 70.0) << "gain_tx";
        expect(c.rx_channels.size() == 2_u) << "rx_channels count";
        expect(c.rx_channels[0] == 1_u) << "rx_channels[0]";
        expect(c.rx_channels[1] == 2_u) << "rx_channels[1]";
        expect(c.tx_channel == 0_u) << "tx_channel";

        // Codec
        expect(c.sf == 8_uc) << "sf";
        expect(c.bw == 62'500_u) << "bw";
        expect(c.cr == 4_uc) << "cr (8 - 4 = 4)";
        expect(that % c.sync == uint16_t{0x12}) << "sync_word";
        expect(c.preamble == 8_u) << "preamble";

        // Rate & name
        expect(std::abs(c.rate - 250'000.f) < 1.f) << "rate";
        expect(eq(c.name, std::string{"MeshCore 868 MHz"}));

        // Network (from [trx.network])
        expect(eq(c.listen, std::string{"127.0.0.1"}));
        expect(c.port == 5556_u) << "port";
        expect(c.lbt == true) << "lbt";
    };

    "decode chain parsed"_test = [] {
        auto cfgs = lora_config::load_config(CONFIG_TOML_PATH);
        expect(cfgs.size() == 1_u);
        const auto& dc = cfgs[0].decode_configs;
        expect(dc.size() == 1_u) << "1 decode entry";
        expect(eq(dc[0].label, std::string{"meshcore"}));
        expect(that % dc[0].sync_word == uint16_t{0x12}) << "decode sync_word";
    };

    "decode_bws parsed"_test = [] {
        auto cfgs = lora_config::load_config(CONFIG_TOML_PATH);
        expect(cfgs.size() == 1_u);
        const auto& bws = cfgs[0].decode_bws;
        expect(bws.size() == 3_u) << "3 decode BWs";
        expect(bws[0] == 62'500_u);
        expect(bws[1] == 125'000_u);
        expect(bws[2] == 250'000_u);
    };

    "network parsed"_test = [] {
        auto cfgs = lora_config::load_config(CONFIG_TOML_PATH);
        expect(cfgs.size() == 1_u);
        const auto& c = cfgs[0];
        expect(eq(c.listen, std::string{"127.0.0.1"}));
        expect(c.port == 5556_u);
        expect(c.lbt == true);
        expect(c.status_interval == 10_u);
    };

    "raw passthrough"_test = [] {
        auto cfgs = lora_config::load_config(CONFIG_TOML_PATH);
        expect(cfgs.size() == 1_u);
        const auto& r = cfgs[0].raw;
        expect(!r.meshcore.name.empty()) << "meshcore.name non-empty";
        expect(r.aggregator.upstream.find("5556") != std::string::npos)
            << "aggregator.upstream contains 5556";
    };

    "cr validation"_test = [] {
        auto path = write_tmp_toml("bad_cr.toml", R"(
[device]
driver = "uhd"

[radio_test]
freq = 868000000

[codec_test]
sf = 8
bw = 125000
cr = 3
sync_word = 0x12
preamble_len = 8

[trx]
radio = "radio_test"
codec = "codec_test"

[[trx.decode]]
label = "test"
sync_word = 0x12
)");
        auto cfgs = lora_config::load_config(path);
        expect(cfgs.empty()) << "cr=3 should fail validation";
    };

    "missing codec"_test = [] {
        auto path = write_tmp_toml("missing_codec.toml", R"(
[device]
driver = "uhd"

[radio_test]
freq = 868000000

[trx]
radio = "radio_test"
codec = "codec_nonexistent"

[[trx.decode]]
label = "test"
sync_word = 0x12
)");
        auto cfgs = lora_config::load_config(path);
        expect(cfgs.empty()) << "missing codec should fail";
    };

    "missing radio"_test = [] {
        auto path = write_tmp_toml("missing_radio.toml", R"(
[device]
driver = "uhd"

[codec_test]
sf = 8
bw = 125000
cr = 8
sync_word = 0x12
preamble_len = 8

[trx]
radio = "radio_nonexistent"
codec = "codec_test"

[[trx.decode]]
label = "test"
sync_word = 0x12
)");
        auto cfgs = lora_config::load_config(path);
        expect(cfgs.empty()) << "missing radio should fail";
    };

    "missing trx section"_test = [] {
        auto path = write_tmp_toml("no_trx.toml", R"(
[device]
driver = "uhd"
)");
        auto cfgs = lora_config::load_config(path);
        expect(cfgs.empty()) << "no [trx] should fail";
    };

    "missing decode entry"_test = [] {
        auto path = write_tmp_toml("no_decode.toml", R"(
[device]
driver = "uhd"

[radio_test]
freq = 868000000

[codec_test]
sf = 8
bw = 125000
cr = 8
sync_word = 0x12
preamble_len = 8

[trx]
radio = "radio_test"
codec = "codec_test"
)");
        auto cfgs = lora_config::load_config(path);
        expect(cfgs.empty()) << "no [[trx.decode]] should fail";
    };
};

// ── Suite 2: config load_scan_config ───────────────────────────────────────

const boost::ut::suite configLoadScan = [] {

    "reference scan config loads"_test = [] {
        auto cfgs = lora_config::load_scan_config(CONFIG_TOML_PATH);
        expect(cfgs.size() == 1_u) << "expected 1 ScanSetConfig";
        const auto& s = cfgs[0];

        expect(s.freq_start == 863'000'000.0) << "freq_start";
        expect(s.freq_stop == 870'000'000.0) << "freq_stop";
        expect(s.bws.size() >= 1_u);
        expect(std::ranges::find(s.bws, uint32_t{62500}) != s.bws.end())
            << "bws contains 62500";
        expect(s.os_factor == 4_u) << "os_factor";
        expect(std::abs(s.min_ratio - 8.0F) < 0.01F) << "min_ratio";
        expect(s.l1_rate == 16'000'000.0) << "l1_rate";
        expect(s.master_clock == 32'000'000.0) << "master_clock";
        expect(s.udp_port == 5557_u) << "udp_port";
    };

    "missing scan section"_test = [] {
        auto path = write_tmp_toml("no_scan.toml", R"(
[device]
driver = "uhd"
)");
        auto cfgs = lora_config::load_scan_config(path);
        expect(cfgs.empty()) << "no [scan] should fail";
    };

    "bad decimation"_test = [] {
        auto path = write_tmp_toml("bad_decim.toml", R"(
[device]
driver = "uhd"

[radio_test]
freq = 868000000

[scan]
radio      = "radio_test"
freq_start = 863000000
freq_stop  = 870000000
bws        = [62500]
l1_rate    = 10000000
os_factor  = 3
)");
        // 10e6 / (62500 * 3) = 10e6 / 187500 = 53.33 — not integer
        auto cfgs = lora_config::load_scan_config(path);
        expect(cfgs.empty()) << "non-integer decimation should fail";
    };

    "freq_start >= freq_stop"_test = [] {
        auto path = write_tmp_toml("bad_freq.toml", R"(
[device]
driver = "uhd"

[radio_test]
freq = 868000000

[scan]
radio      = "radio_test"
freq_start = 870000000
freq_stop  = 863000000
bws        = [125000]
l1_rate    = 16000000
os_factor  = 4
)");
        auto cfgs = lora_config::load_scan_config(path);
        expect(cfgs.empty()) << "freq_start >= freq_stop should fail";
    };

    "bws sorted"_test = [] {
        auto path = write_tmp_toml("unsorted_bws.toml", R"(
[device]
driver = "uhd"

[radio_test]
freq = 868000000

[scan]
radio      = "radio_test"
freq_start = 863000000
freq_stop  = 870000000
bws        = [250000, 62500]
l1_rate    = 16000000
os_factor  = 4
)");
        auto cfgs = lora_config::load_scan_config(path);
        expect(cfgs.size() == 1_u) << "should parse OK";
        expect(cfgs[0].bws.size() == 2_u);
        expect(cfgs[0].bws[0] == 62'500_u) << "bws[0] should be smallest";
        expect(cfgs[0].bws[1] == 250'000_u) << "bws[1] should be largest";
    };
};

int main() { }
