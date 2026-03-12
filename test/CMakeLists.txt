# SPDX-License-Identifier: ISC

# --- Local test helpers (independent of GR4 core test infrastructure) ---
function(setup_test TEST_NAME)
    target_link_libraries(${TEST_NAME} PRIVATE
        gr-lora gnuradio-options gnuradio-core-group gr-testing ut)
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

add_ut_test(qa_lora_splitter)
target_compile_definitions(qa_lora_splitter PRIVATE
    TEST_VECTORS_DIR="${PROJECT_SOURCE_DIR}/test_vectors")

add_ut_test(qa_lora_cad)
target_compile_definitions(qa_lora_cad PRIVATE
    TEST_VECTORS_DIR="${PROJECT_SOURCE_DIR}/test_vectors")
set_tests_properties(qa_lora_cad PROPERTIES TIMEOUT 60)

# Test vector generator (not a test — run manually to regenerate vectors)
add_executable(generate_vectors generate_vectors.cpp)
target_link_libraries(generate_vectors PRIVATE gr-lora)

# --- Hardware tests (require SoapySDR + B210/B220) ---
if(TARGET gr-soapy)
    add_executable(qa_SoapySink qa_SoapySink.cpp)
    target_link_libraries(qa_SoapySink PRIVATE
        gnuradio-options gnuradio-core-group gr-lora gr-soapy gr-testing ut)
    add_test(NAME qa_SoapySink
             COMMAND ${CMAKE_CROSSCOMPILING_EMULATOR} ${CMAKE_CURRENT_BINARY_DIR}/qa_SoapySink)
    set_tests_properties(qa_SoapySink PROPERTIES
        ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0"
        TIMEOUT 30)
endif()
