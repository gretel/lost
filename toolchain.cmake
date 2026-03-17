set(CMAKE_C_COMPILER /opt/homebrew/opt/llvm/bin/clang)
set(CMAKE_CXX_COMPILER /opt/homebrew/opt/llvm/bin/clang++)
set(CMAKE_EXE_LINKER_FLAGS "-L/opt/homebrew/opt/llvm/lib/c++ -Wl,-rpath,/opt/homebrew/opt/llvm/lib/c++")
set(CMAKE_SHARED_LINKER_FLAGS "-L/opt/homebrew/opt/llvm/lib/c++ -Wl,-rpath,/opt/homebrew/opt/llvm/lib/c++")

# macOS SDK sysroot — required for Homebrew LLVM to find system headers
execute_process(
    COMMAND xcrun --show-sdk-path
    OUTPUT_VARIABLE _sdk_path
    OUTPUT_STRIP_TRAILING_WHITESPACE)
set(CMAKE_OSX_SYSROOT "${_sdk_path}")
