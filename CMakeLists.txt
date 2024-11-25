cmake_minimum_required(VERSION 3.5)
project(ir_camera)  # 使用小寫包名

# 設置編譯選項
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_COMPILER g++)

# 添加包含目錄
include_directories(
  include
  /usr/local/include/opencv4
)

# 添加依賴
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_library(UDEV_LIB udev REQUIRED)
find_library(M_LIB m REQUIRED)

# 設定庫和鏈接
link_directories(${PROJECT_SOURCE_DIR}/lib/x64)

# 設定執行檔名稱
add_executable(ir_camera_node src/IR_camera.cpp)
add_executable(ir_image_node src/IR_rqt.cpp)

ament_target_dependencies(ir_camera_node rclcpp sensor_msgs cv_bridge OpenCV)
ament_target_dependencies(ir_image_node rclcpp sensor_msgs cv_bridge OpenCV)

# 將靜態庫加入目標
target_link_libraries(ir_camera_node
  GuideUSB2LiveStream
  pthread
  m
  udev
  opencv_core
  opencv_highgui
  opencv_imgproc
  memcached
)

target_link_libraries(ir_image_node
  GuideUSB2LiveStream
  pthread
  m
  udev
  opencv_core
  opencv_highgui
  opencv_imgproc
  memcached
)

# 安裝
install(TARGETS ir_camera_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS ir_image_node DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY launch include lib msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
