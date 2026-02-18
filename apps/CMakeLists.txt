# SPDX-License-Identifier: ISC
#
# Hardware applications using the unified radio_bridge API.
#
# Two binaries:
#   lora_rx  — single-channel receiver (default) or dual-RX scanner (--scan)
#   lora_tx  — batch/CBOR-streaming transmitter or TX→RX loopback (--loopback)
#
# The radio_bridge API has two backends: UHD (preferred) and SoapySDR (fallback).
# Both are compiled as pure C to isolate the dual-libc++ ABI boundary on macOS
# (LLVM 19 for GR4 C++23 vs system libc++ for UHD/SoapySDR).
# Only one backend is linked into each binary — they export identical symbols.

# --- Detect available SDR backends ---
find_package(UHD QUIET)
# SoapySDR target may already exist from the GR4 subdirectory
if(NOT TARGET SoapySDR)
    find_package(SoapySDR CONFIG QUIET)
endif()

if(NOT UHD_FOUND AND NOT TARGET SoapySDR)
    message(STATUS "Neither UHD nor SoapySDR found -- skipping hardware apps")
    return()
endif()

enable_language(C)

# --- Build radio_bridge static library (one backend) ---
if(UHD_FOUND)
    message(STATUS "Using UHD backend for radio_bridge (preferred)")
    add_library(radio_bridge STATIC uhd_radio_bridge.c)
    target_include_directories(radio_bridge PRIVATE ${UHD_INCLUDE_DIRS})
    target_link_libraries(radio_bridge PRIVATE ${UHD_LIBRARIES})
elseif(TARGET SoapySDR)
    message(STATUS "Using SoapySDR backend for radio_bridge (fallback)")
    add_library(radio_bridge STATIC soapy_radio_bridge.c)
    target_include_directories(radio_bridge PRIVATE ${SoapySDR_INCLUDE_DIRS})
    target_link_libraries(radio_bridge PRIVATE SoapySDR)
endif()

set_target_properties(radio_bridge PROPERTIES LANGUAGE C)
target_compile_options(radio_bridge PRIVATE -fstack-protector-strong)
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(radio_bridge PRIVATE _FORTIFY_SOURCE=2)
endif()

# --- lora_rx: single-channel RX (default) + dual-RX scanner (--scan) ---
add_executable(lora_rx lora_rx.cpp)
target_link_libraries(lora_rx PRIVATE gnuradio-blocklib-core gr-lora radio_bridge)

# --- lora_tx: batch TX / CBOR streaming / loopback (--loopback) ---
add_executable(lora_tx lora_tx.cpp)
target_link_libraries(lora_tx PRIVATE gnuradio-blocklib-core gr-lora gr-testing radio_bridge)
