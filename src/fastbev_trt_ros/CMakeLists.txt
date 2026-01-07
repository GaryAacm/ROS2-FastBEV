cmake_minimum_required(VERSION 3.5)
project(my_fastbev)

# --- 设置 C++ 标准 ---
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(message_filters REQUIRED)
# --- 1. 查找 ROS 2 基础包 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# --- 2. 设置路径变量 (TensorRT 和 CUDA) ---
# 请确认你的 TensorRT 路径是否确实为这个
set(TENSORRT_ROOT /root/TensorRT-8.6.1.6)
set(CUDA_ROOT /usr/local/cuda)

# --- 3. 包含头文件路径 ---
include_directories(
    include
    ${TENSORRT_ROOT}/include
    ${CUDA_ROOT}/include
    ${OpenCV_INCLUDE_DIRS}
)

# --- 4. 设置库搜索路径 ---
link_directories(
    ${TENSORRT_ROOT}/lib
    ${CUDA_ROOT}/lib64
)

# --- 5. 添加可执行文件 ---
# 对应你 src 目录下的 fastbev_node.cpp
add_executable(fastbev_node src/fastbev_node.cpp)

# --- 6. 链接库文件 ---
target_link_libraries(fastbev_node
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${cv_bridge_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${message_filters_LIBRARIES}  # <-- 必须手动添加这一行
    nvinfer
    cudart
    cuda
)

# --- 7. 安装设置 (ros2 run 命令依赖此设置) ---
install(TARGETS fastbev_node
    DESTINATION lib/${PROJECT_NAME})

# 复制 launch 或 config (如果有的话，可选)
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()