# SPDX-License-Identifier: ISC
#
# lora_trx — full-duplex LoRa transceiver (RX + TX) with CBOR/UDP interface.
# All I/O is CBOR over UDP. See docs/cbor-schemas.md.

if(NOT TARGET gr-soapy)
    message(STATUS "gr-soapy not available (SoapySDR not found) -- skipping hardware apps")
    return()
endif()

# SoapySink.hpp lives in blocks/include/ — add it to the include path via gr-lora

add_executable(lora_trx lora_trx.cpp config.cpp)
target_link_libraries(lora_trx PRIVATE gnuradio-blocklib-core gr-lora gr-soapy gr-testing tomlplusplus::tomlplusplus)

add_executable(lora_scan lora_scan.cpp)
target_link_libraries(lora_scan PRIVATE gnuradio-blocklib-core gr-lora gr-soapy gr-testing)

# Version: pass -DGIT_REV=<hash> at configure time, or auto-detect from git.
if(NOT DEFINED GIT_REV)
    execute_process(
        COMMAND git rev-parse --short HEAD
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_REV
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE _git_result)
    if(NOT _git_result EQUAL 0)
        set(GIT_REV "dev")
    endif()
endif()

target_compile_definitions(lora_trx PRIVATE GIT_REV="${GIT_REV}")
target_compile_definitions(lora_scan PRIVATE GIT_REV="${GIT_REV}")
