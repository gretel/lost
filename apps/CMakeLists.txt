# SPDX-License-Identifier: ISC
#
# lora_trx — full-duplex LoRa transceiver (RX + TX) with CBOR/UDP interface.
# All I/O is CBOR over UDP. See docs/cbor-schemas.md.

# SoapySDR target may already exist from the GR4 subdirectory
if(NOT TARGET SoapySDR)
    find_package(SoapySDR CONFIG QUIET)
endif()

if(NOT TARGET SoapySDR)
    message(STATUS "SoapySDR not found -- skipping hardware apps")
    return()
endif()

add_executable(lora_trx lora_trx.cpp)
target_link_libraries(lora_trx PRIVATE gnuradio-blocklib-core gr-lora gr-soapy gr-testing)
