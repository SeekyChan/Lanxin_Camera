cmake_minimum_required(VERSION 3.0.2)
project(qr_code_identify_lanxin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  common_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib std_msgs common_msgs
)

set(SDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/VMR_QC_SDK_demo/demo)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64|arm64")
  set(SDK_ARCH_DIR arm64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i[3-6]86|x86_64|AMD64|amd64")
  set(SDK_ARCH_DIR x86)
else()
  message(FATAL_ERROR "Unsupported CPU architecture for VMR_QCSDK: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(SDK_LIB_DIR ${SDK_ROOT}/lib/${SDK_ARCH_DIR})
set(SDK_LIBRARY ${SDK_LIB_DIR}/libVMR_QCSDK.so)

if(NOT EXISTS ${SDK_LIBRARY})
  message(FATAL_ERROR "VMR_QCSDK library not found: ${SDK_LIBRARY}")
endif()

include_directories(
  include
  ${SDK_ROOT}/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(qr_code_identify_node
  src/main.cpp
  src/qr_camera_config.cpp
  src/qr_code_identify_node.cpp
  src/qr_code_parser.cpp
  src/qr_result_buffer.cpp
  src/lanxin_sdk_client.cpp
)

add_dependencies(qr_code_identify_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(qr_code_identify_node
  ${catkin_LIBRARIES}
  ${SDK_LIBRARY}
)

set_target_properties(qr_code_identify_node PROPERTIES
  BUILD_RPATH "${SDK_LIB_DIR}"
  INSTALL_RPATH "\$ORIGIN/../share/${PROJECT_NAME}/VMR_QC_SDK_demo/demo/lib/${SDK_ARCH_DIR}"
)

install(TARGETS qr_code_identify_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY VMR_QC_SDK_demo
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
