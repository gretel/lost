# SPDX-License-Identifier: ISC
cmake_minimum_required(VERSION 3.27)
project(gr4-lora CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)

# Build type: default to RelWithAssert (optimised, assertions enabled)
if(CMAKE_CXX_COMPILER_ID MATCHES "(Clang|GNU|Intel)")
    set(CMAKE_CXX_FLAGS_RELWITHASSERT "-O2 -DASSERT_ENABLED -ffunction-sections -fdata-sections"
        CACHE INTERNAL "Flags for RelWithAssert builds.")
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "RelWithAssert" CACHE STRING "Build type" FORCE)
    endif()
endif()

# Clang / GCC colour output + libc++
if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    add_compile_options(-stdlib=libc++ -fcolor-diagnostics)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libc++")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-fdiagnostics-color=always)
endif()

# --- Reproducible builds ---
add_compile_options(-Werror=date-time)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(-ffile-prefix-map=${PROJECT_SOURCE_DIR}=.)
endif()

# --- Security hardening ---
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(-ftrivial-auto-var-init=zero)
endif()

# --- GR4 (provides gnuradio-core, gnuradio-algorithm, gr-testing, ut, …) ---
add_subdirectory(gnuradio4)

# --- LoRa block library ---
add_subdirectory(blocks)

# --- Tests ---
option(ENABLE_LORA_TESTING "Enable gr4-lora test suite" ON)
if(ENABLE_LORA_TESTING)
    enable_testing()
    add_subdirectory(test)
endif()

# --- Benchmarks ---
option(ENABLE_LORA_BENCHMARKS "Enable gr4-lora benchmarks" ON)
if(ENABLE_LORA_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

# --- Hardware applications (require SoapySDR) ---
option(ENABLE_LORA_APPS "Enable gr4-lora hardware applications" ON)
if(ENABLE_LORA_APPS)
    # SoapySDR may already be found by GR4 subdirectory; if not, search common paths
    if(NOT TARGET SoapySDR)
        list(APPEND CMAKE_PREFIX_PATH /opt/homebrew)
        find_package(SoapySDR CONFIG QUIET)
    endif()
    add_subdirectory(apps)
endif()
