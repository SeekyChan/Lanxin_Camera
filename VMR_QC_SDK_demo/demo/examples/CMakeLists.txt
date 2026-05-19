cmake_minimum_required(VERSION 3.10)
project(QCSDK_Demo)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

# 根据目标 CPU 选择子目录
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64|arm64")
    set(ARCH_DIR arm64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i[3-6]86|x86_64|AMD64")
    set(ARCH_DIR x86)
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# 加库搜索路径
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib/${ARCH_DIR}/)

# 原有的 demo
add_executable(demo demo.cpp)
target_link_libraries(demo VMR_QCSDK)

add_executable(demo2 demo_2.cpp)
target_link_libraries(demo2 VMR_QCSDK)
# 新增的 snapshot_demo
add_executable(snapshot_demo snapshot_demo.cpp)
target_link_libraries(snapshot_demo VMR_QCSDK)
