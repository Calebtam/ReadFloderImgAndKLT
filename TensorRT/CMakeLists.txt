# cmake_minimum_required(VERSION 3.5.1)

# # Project name
# project(mins)

# # We need C++17 for ROS2
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)

# Include TensorRT and CUDA paths
set(tensorrt_dir "/home/tam/wk/vpr/TensorRT-8.6.1.6")
set(CUDA_DIR  "/home/tam/wk/vpr/cuda/cuda-11.7_cudnn-8.9.7")

# Include TensorRT and CUDA headers
include_directories(${tensorrt_dir}/include)
include_directories(${CUDA_DIR}/include)

# Link TensorRT and CUDA libraries
link_directories(${tensorrt_dir}/lib)
link_directories(${CUDA_DIR}/lib64)

# Find CUDA
find_package(CUDA REQUIRED)

# Set up executable and link libraries
add_executable(tensorrt test_tensorrt.cpp)
# target_link_libraries(tensorrt ${LIBS})

# Link required libraries
target_link_libraries(tensorrt nvinfer cudart)

# For TensorRT and CUDA Debugging messages
message(STATUS "TensorRT include: ${tensorrt_dir}/include")
message(STATUS "CUDA include: ${CUDA_DIR}/include")
message(STATUS "TensorRT lib: ${tensorrt_dir}/lib")
message(STATUS "CUDA lib: ${CUDA_DIR}/lib64")