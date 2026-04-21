# SPDX-License-Identifier: ISC
#
# lora_trx — full-duplex LoRa transceiver (RX + TX) with CBOR/UDP interface.
# All I/O is CBOR over UDP. See CBOR-SCHEMA.md at the repo root.

if(NOT TARGET gr-sdr)
    message(STATUS "gr-sdr not available (SoapySDR not found) -- skipping hardware apps")
    return()
endif()

add_executable(lora_trx lora_trx.cpp config.cpp tx_worker.cpp)
target_link_libraries(lora_trx PRIVATE gnuradio-core-group gr-lora gr-sdr gr-testing tomlplusplus::tomlplusplus)

add_executable(lora_scan lora_scan.cpp config.cpp)
target_link_libraries(lora_scan PRIVATE gnuradio-core-group gr-lora gr-sdr gr-testing tomlplusplus::tomlplusplus)

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
