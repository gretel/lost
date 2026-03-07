# SPDX-License-Identifier: ISC

# --- Local test helpers (independent of GR4 core test infrastructure) ---
function(setup_test TEST_NAME)
    target_link_libraries(${TEST_NAME} PRIVATE
        gr-lora gnuradio-options gnuradio-blocklib-core gr-testing ut)
    add_test(NAME ${TEST_NAME}
             COMMAND ${CMAKE_CROSSCOMPILING_EMULATOR} ${CMAKE_CURRENT_BINARY_DIR}/${TEST_NAME})
    set_tests_properties(${TEST_NAME} PROPERTIES TIMEOUT 30)
endfunction()

function(add_ut_test TEST_NAME)
    add_executable(${TEST_NAME} ${TEST_NAME}.cpp)
    setup_test(${TEST_NAME})
endfunction()

# --- Test targets ---
add_ut_test(qa_lora_algorithms)
target_compile_definitions(qa_lora_algorithms PRIVATE
    TEST_VECTORS_DIR="${PROJECT_SOURCE_DIR}/test_vectors")

add_ut_test(qa_lora_tx)
target_compile_definitions(qa_lora_tx PRIVATE
    TEST_VECTORS_DIR="${PROJECT_SOURCE_DIR}/test_vectors")

add_ut_test(qa_lora_phase3)
target_compile_definitions(qa_lora_phase3 PRIVATE
    TEST_VECTORS_DIR="${PROJECT_SOURCE_DIR}/test_vectors")

add_ut_test(qa_lora_rx)
target_compile_definitions(qa_lora_rx PRIVATE
    TEST_VECTORS_DIR="${PROJECT_SOURCE_DIR}/test_vectors")

add_ut_test(qa_lora_loopback)
target_compile_definitions(qa_lora_loopback PRIVATE
    TEST_VECTORS_DIR="${PROJECT_SOURCE_DIR}/test_vectors")

add_ut_test(qa_lora_burst)
target_compile_definitions(qa_lora_burst PRIVATE
    TEST_VECTORS_DIR="${PROJECT_SOURCE_DIR}/test_vectors")

add_ut_test(qa_lora_channelizer)
target_compile_definitions(qa_lora_channelizer PRIVATE
    TEST_VECTORS_DIR="${PROJECT_SOURCE_DIR}/test_vectors")

add_ut_test(qa_lora_diversity)
target_compile_definitions(qa_lora_diversity PRIVATE
    TEST_VECTORS_DIR="${PROJECT_SOURCE_DIR}/test_vectors")

add_ut_test(qa_lora_splitter)
target_compile_definitions(qa_lora_splitter PRIVATE
    TEST_VECTORS_DIR="${PROJECT_SOURCE_DIR}/test_vectors")

add_ut_test(qa_lora_cad)
target_compile_definitions(qa_lora_cad PRIVATE
    TEST_VECTORS_DIR="${PROJECT_SOURCE_DIR}/test_vectors")
set_tests_properties(qa_lora_cad PROPERTIES TIMEOUT 60)
