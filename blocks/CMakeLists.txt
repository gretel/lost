# SPDX-License-Identifier: ISC
add_library(gr-lora INTERFACE)
target_link_libraries(gr-lora INTERFACE gnuradio-core gnuradio-algorithm)
target_include_directories(gr-lora INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

gr_add_block_library(
    GrLoraBlocks
    MAKE_SHARED_LIBRARY
    HEADERS
        include/gnuradio-4.0/lora/FrameSync.hpp
        include/gnuradio-4.0/lora/DemodDecoder.hpp
        include/gnuradio-4.0/lora/FreqXlatingDecimator.hpp
        include/gnuradio-4.0/lora/DiversityCombiner.hpp
        include/gnuradio-4.0/lora/Splitter.hpp
    LINK_LIBRARIES
        gr-lora)
