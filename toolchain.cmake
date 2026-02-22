set(CMAKE_C_COMPILER /opt/homebrew/opt/llvm@20/bin/clang)
set(CMAKE_CXX_COMPILER /opt/homebrew/opt/llvm@20/bin/clang++)
set(CMAKE_EXE_LINKER_FLAGS "-L/opt/homebrew/opt/llvm@20/lib/c++ -Wl,-rpath,/opt/homebrew/opt/llvm@20/lib/c++")
set(CMAKE_SHARED_LINKER_FLAGS "-L/opt/homebrew/opt/llvm@20/lib/c++ -Wl,-rpath,/opt/homebrew/opt/llvm@20/lib/c++")
