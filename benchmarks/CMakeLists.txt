# SPDX-License-Identifier: ISC

function(add_lora_benchmark BM_NAME)
    add_executable(${BM_NAME} ${BM_NAME}.cpp)
    target_link_libraries(${BM_NAME} PRIVATE
        gr-lora gnuradio-options gnuradio-core gnuradio-algorithm ut ut-benchmark)
    target_compile_options(${BM_NAME} PRIVATE -O3 -march=native)
endfunction()

add_lora_benchmark(bm_lora_algorithms)
add_lora_benchmark(bm_lora_decode)
