# SPDX-License-Identifier: ISC
add_library(gr-lora INTERFACE)
target_link_libraries(gr-lora INTERFACE gnuradio-core gnuradio-algorithm)
target_include_directories(gr-lora INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)
