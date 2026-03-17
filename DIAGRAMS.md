# gr4-lora flow graphs

## lora_trx RX — single BW

```mermaid
graph TD
    SoapySource["SoapySimpleSource&lt;cf32&gt;<br/><i>250 kHz, 1 channel</i>"]
    MSfDec["MultiSfDecoder<br/><i>SF7-12, CR auto, LDRO auto</i><br/><i>LBT + spectrum tap</i>"]
    FSink["FrameSink<br/><i>console + CBOR/UDP</i>"]

    SoapySource -- "cf32" --> MSfDec
    MSfDec -- "uint8_t + tags<br/>{sf, cr, pay_len, crc_valid, snr_db}" --> FSink
```

## lora_trx RX — multi-BW

`decode_bws = [62500, 125000, 250000]`

```mermaid
graph TD
    SoapySource["SoapySimpleSource&lt;cf32&gt;<br/><i>250 kHz</i>"]
    Splitter["Splitter<br/><i>n_outputs=3</i>"]
    MSf62["MultiSfDecoder<br/><i>BW=62.5k, os=4</i><br/><i>SF7-12</i>"]
    MSf125["MultiSfDecoder<br/><i>BW=125k, os=2</i><br/><i>SF7-12</i>"]
    MSf250["MultiSfDecoder<br/><i>BW=250k, os=1</i><br/><i>SF7-12</i>"]
    FS1["FrameSink<br/><i>ch=N00</i>"]
    FS2["FrameSink<br/><i>ch=N01</i>"]
    FS3["FrameSink<br/><i>ch=N02</i>"]

    SoapySource -- "cf32" --> Splitter
    Splitter -- "out#0" --> MSf62
    Splitter -- "out#1" --> MSf125
    Splitter -- "out#2" --> MSf250
    MSf62 -- "uint8_t" --> FS1
    MSf125 -- "uint8_t" --> FS2
    MSf250 -- "uint8_t" --> FS3
```

## lora_trx RX — 2-radio multi-BW

`rx_channel = [1, 2]`, `decode_bws = [62500, 125000, 250000]`

6 MultiSfDecoder x 6 SfLanes = 36 simultaneous decode paths.

```mermaid
graph TD
    SoapyDual["SoapyDualSimpleSource&lt;cf32&gt;<br/><i>250 kHz, 2 channels (MIMO)</i>"]

    SplA["Splitter<br/><i>RX_A, n=3</i>"]
    SplB["Splitter<br/><i>RX_B, n=3</i>"]

    A62["MultiSfDecoder<br/><i>BW62k</i>"]
    A125["MultiSfDecoder<br/><i>BW125k</i>"]
    A250["MultiSfDecoder<br/><i>BW250k</i>"]
    B62["MultiSfDecoder<br/><i>BW62k</i>"]
    B125["MultiSfDecoder<br/><i>BW125k</i>"]
    B250["MultiSfDecoder<br/><i>BW250k</i>"]

    FA1["FrameSink ch=100"]
    FA2["FrameSink ch=101"]
    FA3["FrameSink ch=102"]
    FB1["FrameSink ch=200"]
    FB2["FrameSink ch=201"]
    FB3["FrameSink ch=202"]

    SoapyDual -- "out#0 (RX_A)" --> SplA
    SoapyDual -- "out#1 (RX_B)" --> SplB

    SplA --> A62 --> FA1
    SplA --> A125 --> FA2
    SplA --> A250 --> FA3
    SplB --> B62 --> FB1
    SplB --> B125 --> FB2
    SplB --> B250 --> FB3
```

## lora_trx TX

```mermaid
graph LR
    UDP["UDP lora_tx<br/>CBOR request"]
    TXQ["tx_worker thread<br/><i>TxRequestQueue (depth=8)</i><br/><i>LBT: poll channel_busy</i><br/><i>generate_iq()</i>"]
    TxSrc["TxQueueSource<br/><i>gr::lora</i>"]
    SoapySink["SoapySinkBlock&lt;cf32, 2&gt;<br/><i>gr::blocks::soapy</i>"]

    UDP --> TXQ
    TXQ -- "cf32 IQ burst" --> TxSrc
    TxSrc -- "out0 (IQ) → in#0" --> SoapySink
    TxSrc -. "out1 (zeros) → in#1" .-> SoapySink
```

## lora_scan

```mermaid
graph TD
    SoapySource["SoapySimpleSource&lt;cf32&gt;<br/><i>16 MS/s wideband</i>"]
    Splitter["Splitter<br/><i>n_outputs=2</i>"]
    Tap["SpectrumTapBlock<br/><i>ring → FFT</i>"]
    Cap["CaptureSink<br/><i>on-demand sample grab</i>"]
    Null["NullSink&lt;cf32&gt;"]
    Orch["Scan orchestrator<br/><i>(main thread, not a GR4 block)</i><br/><br/><i>L1: FFT energy snapshots</i><br/><i>L2: ChannelActivityDetector.detectMultiSf()</i>"]

    SoapySource -- "cf32" --> Splitter
    Splitter -- "out#0" --> Tap
    Splitter -- "out#1" --> Cap
    Tap --> Null
    Cap -. "CaptureState<br/>(atomic req/done)" .-> Orch
```

## MultiSfDecoder internals

```mermaid
graph LR
    subgraph MultiSfDecoder
        direction TB
        Accum["per-lane<br/>accum buffer"]
        L7["SfLane SF7<br/><i>DETECT→SYNC→OUTPUT</i>"]
        L8["SfLane SF8"]
        L9["SfLane SF9"]
        L10["SfLane SF10"]
        L11["SfLane SF11"]
        L12["SfLane SF12"]
    end

    IQ["cf32 in"] --> Accum
    Accum --> L7 & L8 & L9 & L10 & L11 & L12
    L7 & L8 & L9 & L10 & L11 & L12 --> Out["uint8_t out<br/>+ per-frame tags"]

    L7 & L8 & L9 & L10 & L11 & L12 -. "any lane in SYNC/OUTPUT" .-> LBT["channel_busy<br/>(atomic)"]
    IQ -. "raw IQ push" .-> Spec["SpectrumState<br/>(waterfall)"]
```
