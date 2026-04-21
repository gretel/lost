# SPDX-License-Identifier: ISC
add_library(gr-lora INTERFACE)
target_link_libraries(gr-lora INTERFACE gnuradio-core gnuradio-algorithm)
target_include_directories(gr-lora INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

# Upstream gr_add_block_library (post fair-acc/gnuradio4 aac24cc "Add
# gnuradio4 namespace for gr_add_block_library libraries") installs the
# generated shared target as part of EXPORT gnuradio4Targets; the shared
# target transitively depends on gr-lora via LINK_LIBRARIES, so gr-lora
# itself must live in the same export set or CMake's generate step fails
# with "target ... requires target 'gr-lora' that is not in any export set".
install(TARGETS gr-lora EXPORT gnuradio4Targets)

gr_add_block_library(
    GrLoraBlocks
    MAKE_SHARED_LIBRARY
    HEADERS
        include/gnuradio-4.0/lora/Splitter.hpp
        include/gnuradio-4.0/lora/ChannelActivityDetector.hpp
        include/gnuradio-4.0/lora/FrameSink.hpp
        include/gnuradio-4.0/lora/TxQueueSource.hpp
        include/gnuradio-4.0/lora/MultiSfDecoder.hpp
    LINK_LIBRARIES
        gr-lora)
