# LoRa Protocol Landscape

Reference documentation for LoRa-based protocols detectable by this SDR.
The FrameSink block identifies protocols by their LoRa sync word.

## MeshCore (sync_word = 0x12)

- **Spec**: https://github.com/meshcore-dev/MeshCore
- **Sync word**: 0x12 (shared with Reticulum)
- **Typical config**: SF8, BW 62.5 kHz, CR 4/8, 8-symbol preamble
- **Payload**: MeshCore v1 packet format (header byte + optional transport
  codes + path + encrypted payload). ADVERT payloads are cleartext.
- **Encryption**: X25519 + ChaCha20-Poly1305

## Reticulum (sync_word = 0x12)

- **Spec**: https://reticulum.network/manual/
- **Sync word**: 0x12 (same as MeshCore — cannot distinguish at PHY layer)
- **Typical config**: SF8, BW 62.5 kHz, CR 4/8
- **Payload**: Reticulum uses its own packet format with link-layer headers.
  Differentiation from MeshCore requires application-layer parsing.

## Meshtastic (sync_word = 0x2B)

- **Spec**: https://meshtastic.org/docs/overview/radio-settings/
- **Sync word**: 0x2B
- **Typical configs** (presets):
  - Short/Fast: SF7, BW 250 kHz, CR 4/5
  - Short/Slow: SF7, BW 250 kHz, CR 4/8
  - Medium/Slow: SF11, BW 250 kHz, CR 4/8
  - Long/Fast: SF11, BW 250 kHz, CR 4/5
  - Long/Moderate: SF11, BW 125 kHz, CR 4/8
  - Long/Slow: SF12, BW 125 kHz, CR 4/8
  - Very Long/Slow: SF12, BW 62.5 kHz, CR 4/8
- **Preamble**: 16 symbols
- **Payload**: Protobuf-encoded, AES-128-CTR or AES-256-CTR encrypted.
  Channel/key derived from channel name + PSK.

## LoRaWAN (sync_word = 0x34)

- **Spec**: https://lora-alliance.org/about-lorawan/
- **Sync word**: 0x34
- **Typical config**: SF7-12, BW 125/250/500 kHz, CR 4/5
- **Preamble**: 8 symbols
- **Payload**: LoRaWAN MAC frames (MHDR + MACPayload + MIC).
  AES-128-CMAC integrity, AES-128-CTR encryption.
  Uplink/downlink asymmetric keys (NwkSKey, AppSKey).
- **ADR**: Adaptive Data Rate adjusts SF/BW dynamically.

## MeshCom (sync_word = varies)

- **Spec**: https://meshcom.at/
- **Notes**: Austrian amateur radio mesh network. Uses custom LoRa
  parameters. Sync word and modulation settings depend on regional
  configuration. Limited public documentation.

## Detection Strategy

The PHY layer can only distinguish protocols by sync word. MeshCore
and Reticulum share sync_word=0x12 — they are indistinguishable at the
PHY layer. FrameSink outputs the raw sync word value; application-layer
parsers (Python decoder scripts) classify based on payload structure.

For a multi-protocol scanner, run parallel FrameSync instances with
different sync_word settings (0x12, 0x2B, 0x34).
