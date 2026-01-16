cmake_minimum_required(VERSION 4.2.1)
project (GNESProject CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the build type (Release, Debug, RelWithDebInfo, MinSizeRel)." FORCE)
endif()

add_executable(GNES)

target_sources(GNES
    PRIVATE
        src/main.cpp
        src/GNES.cpp
        src/APU.cpp
        src/CPU.cpp
        src/PPU.cpp
        src/Logger.cpp
        src/MemoryMapper.cpp
        src/OpCodeDecoder.cpp
        src/RomLoader.cpp
        src/Timer.cpp)

target_compile_features(GNES PRIVATE cxx_std_20)

if(NOT WIN32)
    find_package(Threads REQUIRED)
    target_link_libraries(GNES Threads::Threads m)
endif()
