# SPDX-License-Identifier: ISC
if(TARGET SoapySDR)
  # --- C bridge: compiled with SYSTEM compiler to match SoapySDR/UHD ABI ---
  # The GR4 project uses Homebrew LLVM 19 with its own libc++, but SoapySDR and
  # UHD link against Apple's system libc++. Loading both libc++ in one process
  # causes a malloc crash. The C bridge isolates the ABI boundary: it's compiled
  # as pure C with the system compiler, so only C types cross between the two worlds.
  enable_language(C)

  add_library(soapy_c_bridge STATIC soapy_c_bridge.c)
  # Force system C compiler (Apple Clang) for this target, overriding toolchain.cmake
  set_target_properties(soapy_c_bridge PROPERTIES
    C_COMPILER_LAUNCHER ""
    LANGUAGE C
  )
  target_include_directories(soapy_c_bridge PRIVATE ${SoapySDR_INCLUDE_DIRS})
  target_link_libraries(soapy_c_bridge PRIVATE SoapySDR)
  target_compile_options(soapy_c_bridge PRIVATE -fstack-protector-strong)
  if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
      target_compile_definitions(soapy_c_bridge PRIVATE _FORTIFY_SOURCE=2)
  endif()

  # --- lora_rx_soapy: compiled with LLVM 19 (GR4 toolchain) ---
  # Uses SoapySource.hpp which calls only C functions from soapy_c_bridge.
  # Does NOT include any SoapySDR C++ headers.
  add_executable(lora_rx_soapy lora_rx_soapy.cpp)
  target_link_libraries(lora_rx_soapy PRIVATE gnuradio-blocklib-core gr-lora soapy_c_bridge)

  # --- lora_tx_soapy: compiled with LLVM 19 (GR4 toolchain) ---
  # Uses algorithm headers directly (no GR4 graph needed for batch TX).
  # Calls soapy_c_bridge for hardware access.
  add_executable(lora_tx_soapy lora_tx_soapy.cpp)
  target_link_libraries(lora_tx_soapy PRIVATE gr-lora soapy_c_bridge)

  # --- lora_loopback_soapy: TX+RX hardware loopback test ---
  # Opens both TX and RX on same B210, transmits a LoRa frame, captures IQ,
  # then decodes offline to verify the round-trip.
  add_executable(lora_loopback_soapy lora_loopback_soapy.cpp)
  target_link_libraries(lora_loopback_soapy PRIVATE gnuradio-blocklib-core gr-lora gr-testing soapy_c_bridge)
else()
  message(STATUS "SoapySDR not found -- skipping lora hardware apps")
endif()
