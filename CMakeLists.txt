cmake_minimum_required(VERSION 4.2.1)
project (GNESProject C CXX)

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
        src/Timer.cpp
        src/Renderer.cpp)

target_include_directories(GNES PRIVATE include)
target_include_directories(GNES PRIVATE thirdparty/imgui)

target_compile_features(GNES PRIVATE cxx_std_20)

if(NOT WIN32)
    find_package(Threads REQUIRED)
    target_link_libraries(GNES PRIVATE Threads::Threads m)
endif()

find_package(OpenGL REQUIRED)
target_link_libraries(GNES PUBLIC OpenGL::OpenGL)

add_library(glad STATIC src/glad.c)
target_include_directories(glad PUBLIC include)
target_link_libraries(GNES PUBLIC glad)

add_subdirectory(thirdparty/glfw)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

target_link_libraries(GNES PRIVATE glfw)

set(IMGUI_PATH  "thirdparty/imgui")

file(GLOB IMGUI_SOURCES
    ${IMGUI_PATH}/*.cpp
    ${IMGUI_PATH}/backends/imgui_impl_glfw.cpp
    ${IMGUI_PATH}/backends/imgui_impl_opengl3.cpp
    ${IMGUI_PATH}/backends/imgui_impl_opengl3_loader.cpp)

add_library(ImGui STATIC ${IMGUI_SOURCES} ${IMGUI_PATH}/)
target_include_directories(ImGui PRIVATE ${IMGUI_PATH})
target_include_directories(ImGui PRIVATE ${IMGUI_PATH}/backends)
target_include_directories(ImGui PRIVATE thirdparty/glfw/include)

target_link_libraries(GNES PRIVATE ImGui)
