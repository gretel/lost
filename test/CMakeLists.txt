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

add_ut_test(qa_lora_multisf)
target_compile_definitions(qa_lora_multisf PRIVATE
    TEST_VECTORS_DIR="${PROJECT_SOURCE_DIR}/test_vectors")
set_tests_properties(qa_lora_multisf PROPERTIES TIMEOUT 120)

add_ut_test(qa_lora_udp)

# Config test: needs config.cpp (from apps/) + toml++ (not part of gr-lora)
add_executable(qa_lora_config qa_lora_config.cpp ${PROJECT_SOURCE_DIR}/apps/config.cpp)
setup_test(qa_lora_config)
target_link_libraries(qa_lora_config PRIVATE tomlplusplus::tomlplusplus)
target_compile_definitions(qa_lora_config PRIVATE
    CONFIG_TOML_PATH="${PROJECT_SOURCE_DIR}/apps/config.toml"
    GIT_REV="test")

add_ut_test(qa_lora_framesink)
target_compile_definitions(qa_lora_framesink PRIVATE
    TEST_VECTORS_DIR="${PROJECT_SOURCE_DIR}/test_vectors")

add_ut_test(qa_lora_scan)
target_compile_definitions(qa_lora_scan PRIVATE
    TEST_VECTORS_DIR="${PROJECT_SOURCE_DIR}/test_vectors")
set_tests_properties(qa_lora_scan PROPERTIES TIMEOUT 60)

add_ut_test(qa_lora_wideband)
target_compile_definitions(qa_lora_wideband PRIVATE
    TEST_VECTORS_DIR="${PROJECT_SOURCE_DIR}/test_vectors")
set_tests_properties(qa_lora_wideband PROPERTIES TIMEOUT 120)

# --- SoapyLoopback: hardware-free SoapySDR loopback testing ---
# Override with -DFETCHCONTENT_SOURCE_DIR_SOAPYLOOPBACK=<path> for local development
FetchContent_Declare(
    SoapyLoopback
    GIT_REPOSITORY https://github.com/gretel/SoapyLoopback.git
    GIT_TAG        bffb12274ac657802d5f14e46996a77f023a4077  # fix/tx-loopback-qa
)
FetchContent_MakeAvailable(SoapyLoopback)

add_executable(qa_soapy_loopback qa_soapy_loopback.cpp)
target_link_libraries(qa_soapy_loopback PRIVATE
    gnuradio-options gnuradio-core-group gr-soapy gr-testing ut)
target_compile_definitions(qa_soapy_loopback PRIVATE
    SOAPY_LOOPBACK_MODULE_PATH="$<TARGET_FILE:soapyloopback>")
add_test(NAME qa_soapy_loopback
         COMMAND ${CMAKE_CROSSCOMPILING_EMULATOR} ${CMAKE_CURRENT_BINARY_DIR}/qa_soapy_loopback)
set_tests_properties(qa_soapy_loopback PROPERTIES
    TIMEOUT 60
    ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")
add_dependencies(qa_soapy_loopback soapyloopback)

# Test vector generator (not a test — run manually to regenerate vectors)
add_executable(generate_vectors generate_vectors.cpp)
target_link_libraries(generate_vectors PRIVATE gr-lora)

