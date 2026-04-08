# gr4-lora armv7 deploy on Tezuka fishball7020 (ADALM-Pluto / FISH Ball)

This document describes the end-to-end procedure for taking the
GitHub Actions `ARMv7 Cross CI` artifact from this repository and
running `lora_trx` / `lora_scan` on a Tezuka-based PlutoSDR.

## Prerequisites

- Target device running a Tezuka rootfs built with the Bootlin gcc 15.1
  toolchain **and** carrying a SoapySDR + SoapyPlutoPAPR runtime
  (`gretel/miyazaki` branch `dev`, commit cd857c0 or later).
- Host SSH access to the target (default: `root@<ip>`, password `analog`).
- The SD card mounted on the target at `/mnt/sd/` with FAT32 r/w
  permissions.

### Verify the target is runnable

```
ssh root@<pluto-ip> 'strings /usr/lib/libstdc++.so.6 | grep GLIBCXX_3.4.34'
ssh root@<pluto-ip> 'ls /usr/lib/libSoapySDR.so.0.8*'
ssh root@<pluto-ip> 'ls /usr/lib/SoapySDR/modules0.8/libTezukaSupport.so'
ssh root@<pluto-ip> 'ls /sys/bus/iio/devices/'
```

Expected IIO devices: `ad9361-phy`, `xadc`, `cf-ad9361-lpc`,
`cf-ad9361-dds-core-lpc`. If any of these fail, stop and re-flash
the Tezuka rootfs before continuing — the gr4-lora binaries will not
run.

Note: `SoapySDRUtil` is not shipped (BR2_PACKAGE_SOAPY_SDR builds the
library only, not the CLI). The Tezuka SoapySDR module is named
`libTezukaSupport.so` and registers itself as the `"tezuka"` driver
(not `"plutosdr"` — see config-pluto.toml).

## Download the CI artifact

```
gh run list --workflow ci-armv7.yml --limit 5
gh run download <run_id> --repo gretel/gr4-lora --dir tmp/phase2-artifact
```

The artifact layout is:

```
gr4-lora-armv7-eabihf-<sha>/
├── BUILD_INFO.txt
├── README-DEPLOY.md              (this file)
├── bin/
│   ├── lora_trx                  hardware app — full-duplex TRX
│   ├── lora_scan                 hardware app — band scanner
│   └── qa_lora_*                 17 portability test binaries
├── etc/
│   └── config-pluto.toml         TOML config template for Pluto
└── lib/
    ├── libGrLoraBlocksShared.so
    └── libgnuradio-*.so
```

Every `bin/*` binary has RPATH `$ORIGIN/../lib`, so it resolves shared
libraries from `<deploy-root>/lib/` regardless of the install path.

## Deploy

```
STAGE=tmp/phase2-artifact/gr4-lora-armv7-eabihf-*
ssh root@<pluto-ip> 'mkdir -p /root/gr4-lora'
scp -r "$STAGE"/{bin,lib,etc} root@<pluto-ip>:/root/gr4-lora/
```

The total deploy is ~5 MiB including the 17 qa tests and both apps.

## Smoke tests

### 1. Check RPATH + dynamic linker

```
ssh root@<pluto-ip> '/root/gr4-lora/bin/lora_trx --version'
ssh root@<pluto-ip> '/root/gr4-lora/bin/lora_scan --version'
```

Both should print their git rev. If you see

```
error while loading shared libraries: libgnuradio-blocklib-core.so: cannot open shared object file
```

then the `lib/` tree is not present alongside `bin/`.

### 2. Run a synthetic qa binary

```
ssh root@<pluto-ip> '/root/gr4-lora/bin/qa_lora_cad'
```

This is a CAD (Channel Activity Detection) unit test. No hardware
access; it exercises the scheduler + FFT + block registry paths on
the target CPU.

### 3. Probe Pluto via lora_trx

```
ssh root@<pluto-ip> '/root/gr4-lora/bin/lora_trx --config /root/gr4-lora/etc/config-pluto.toml --log-level DEBUG'
```

Expected startup sequence:

1. Config parse log line from `lora_config`
2. SoapySDR banner showing `driver=tezuka` + `uri=local:`
3. SoapyPlutoPAPR enumerates IIO devices on `local:` backend, finds
   `ad9361-phy` + `cf-ad9361-lpc` + `cf-ad9361-dds-core-lpc`
4. `rx_antennae = ["TX/RX"]` warning (see below)
5. Ad9361 tune to 868.5 MHz
6. Scheduler goes running and starts emitting frames

## Known warnings

### Antenna name mismatch

`lora_trx`'s channel map hard-codes B210 antenna labels (`"TX/RX"`,
`"RX2"`). On Pluto the SoapyPlutoPAPR driver silently ignores
unknown antenna names and keeps its constructor-time defaults
(`A_BALANCED` for RX, `A` for TX), so the effective RX/TX signal
path is correct even though the strings don't match. A future PR
should make the channel map driver-aware.

### DC offset mode

Pluto's SoapySDR driver returns `hasDCOffsetMode() = false`, so
`dc_offset_auto = true` is a no-op. The DSP-level `dc_blocker = true`
still runs and handles the residual DC spur.

## Troubleshooting

### `Pluto not found`

Check `SoapySDRUtil --probe=driver=plutosdr` directly on the target.
If that fails, the SoapyPluto module is missing from the rootfs or
the iio local backend cannot find `ad9361-phy`. Re-flash Tezuka.

### Sample rate errors

SoapyPluto's minimum sample rate is 260 kHz without libad9361 FIR
(65 kHz with). The config template uses 500 kHz which is safely
above both thresholds.

### Overflows in log

Pluto's AD9361 via libiio local backend is noticeably more sensitive
to scheduler stalls than a USB B210. If you see repeated
`SoapySource: overflow` lines, reduce `rate` in `config-pluto.toml`
or the incoming sample load.
