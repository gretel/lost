# SPDX-License-Identifier: ISC
#
# Hardware applications — three binaries:
#   lora_rx  — single-channel receiver using native GR4 SoapySDR blocks
#   lora_tx  — batch/CBOR-streaming transmitter using native GR4 SoapySDR blocks
#   lora_trx — full-duplex transceiver (RX + TX) with CBOR/UDP interface
#
# All use GR4's gr-soapy blocks directly. No C radio_bridge needed.

# SoapySDR target may already exist from the GR4 subdirectory
if(NOT TARGET SoapySDR)
    find_package(SoapySDR CONFIG QUIET)
endif()

if(NOT TARGET SoapySDR)
    message(STATUS "SoapySDR not found -- skipping hardware apps")
    return()
endif()

# --- lora_rx: single-channel RX using SoapySimpleSource ---
add_executable(lora_rx lora_rx.cpp)
target_link_libraries(lora_rx PRIVATE gnuradio-blocklib-core gr-lora gr-soapy)

# --- lora_tx: batch TX / CBOR streaming using SoapySinkBlock ---
add_executable(lora_tx lora_tx.cpp)
target_link_libraries(lora_tx PRIVATE gnuradio-blocklib-core gr-lora gr-soapy gr-testing)

# --- lora_trx: full-duplex transceiver with CBOR/UDP interface ---
add_executable(lora_trx lora_trx.cpp)
target_link_libraries(lora_trx PRIVATE gnuradio-blocklib-core gr-lora gr-soapy gr-testing)
