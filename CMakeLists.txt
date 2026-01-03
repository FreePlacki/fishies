cmake_minimum_required(VERSION 3.20)
project(fishies LANGUAGES C CXX CUDA)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Disable everything we don't need
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

add_subdirectory(external/glfw)

add_executable(fishies
    src/main.cu
)

target_compile_options(fishies PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:-Xptxas=-v>
    $<$<COMPILE_LANGUAGE:CUDA>:-lineinfo>
    $<$<COMPILE_LANGUAGE:CUDA>:-O3>
)

set_target_properties(fishies PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
)

target_link_libraries(fishies PRIVATE glfw)

add_library(glad external/glad/src/glad.c)
target_include_directories(glad PUBLIC external/glad/include)

target_link_libraries(fishies PRIVATE glad)

find_package(OpenGL REQUIRED)
target_link_libraries(fishies PRIVATE OpenGL::GL)

