cmake_minimum_required(VERSION 3.20)
project(fishies LANGUAGES CXX CUDA)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

add_executable(fishies
    src/main.cu
)

target_compile_options(fishies PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:-Xptxas=-v>
    $<$<COMPILE_LANGUAGE:CUDA>:-lineinfo>
    $<$<COMPILE_LANGUAGE:CUDA>:-O3>
    $<$<COMPILE_LANGUAGE:CUDA>:-DFORMATTING> # remove to disable console formatting
    $<$<COMPILE_LANGUAGE:CUDA>:-DINTRIN> # remove to disable intrinsics usage (if it doesn't compile on old machines)
)

set_target_properties(fishies PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
)
