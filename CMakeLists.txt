cmake_minimum_required(VERSION 4.2.1)
project (GNESProject)
set(CMAKE_CXX_STANDARD 20)

add_executable(GNES)

target_sources(GNES
    PRIVATE
        src/main.cpp
        src/GNES.cpp
        src/CPU.cpp
        src/MemoryMapper.cpp
        src/OpCodeDecoder.cpp
	src/RomLoader.cpp)
