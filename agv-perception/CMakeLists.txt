# CMakeLists.txt 顶部 
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
cmake_minimum_required(VERSION 3.22)
project(agv-perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(TensorRT_DIR /usr/local/TensorRT/)

# include(cmake/FindTensorRT.cmake)

include_directories(include)
# include_directories("${TENSORRT_DIR}/include")

add_library(YoloV8_TRT SHARED IMPORTED)
set_target_properties(YoloV8_TRT PROPERTIES
    IMPORTED_LOCATION "/home/git/ws_agv_perception/agv-perception/agv-perception/libs/libYoloV8_TRT.so"
    # INTERFACE_INCLUDE_DIRECTORIES "include"
)

add_library(tensorrt_cpp_api SHARED IMPORTED)
set_target_properties(tensorrt_cpp_api PROPERTIES
    IMPORTED_LOCATION "/home/git/ws_agv_perception/agv-perception/agv-perception/libs/libtensorrt_cpp_api.so"
    # INTERFACE_INCLUDE_DIRECTORIES "include"
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(TensorRT REQUIRED)
find_package(CUDA REQUIRED)

add_executable(main src/main.cpp src/ImageProcessor.cpp src/sensing/MsgGroup.cpp src/utils.cpp  src/sensing/MsgManager.cpp src/pc-clustering/ece.cpp src/sensing/inference.cpp)

ament_target_dependencies(main
rclcpp
sensor_msgs
std_msgs
cv_bridge
image_transport
OpenCV
PCL
pcl_conversions
pcl_ros
TensorRT
CUDA
)

target_link_libraries(main YoloV8_TRT tensorrt_cpp_api)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
main
DESTINATION lib/${PROJECT_NAME}
)

# install(FILES
#   "/home/git/ws_agv_perception/agv-perception/agv-perception/libs/libYoloV8_TRT.so"
#   "/home/git/ws_agv_perception/agv-perception/agv-perception/libs/libtensorrt_cpp_api.so"
#   DESTINATION lib/${PROJECT_NAME}
# )


ament_package()
