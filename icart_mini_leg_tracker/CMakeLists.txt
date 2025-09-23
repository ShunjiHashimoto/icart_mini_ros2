cmake_minimum_required(VERSION 3.8)
project(icart_mini_leg_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ヘッダーファイルのディレクトリを追加
include_directories(
  include 
  include/icart_mini_leg_tracker
  include/icart_mini_leg_tracker/utils)
  
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ClusterInfo.msg"
  "msg/ClusterInfoArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# ライブラリの設定
add_library(marker_helper src/utils/marker_helper.cpp)
add_library(csv_logger src/utils/csv_logger.cpp)
# 実行ファイルの設定
add_executable(leg_cluster_tracking_node src/leg_cluster_tracking.cpp)
rosidl_target_interfaces(leg_cluster_tracking_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(csv_logger ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ライブラリをリンク
target_link_libraries(leg_cluster_tracking_node
  marker_helper
  csv_logger
)

# 依存パッケージの設定
ament_target_dependencies(leg_cluster_tracking_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  visualization_msgs 
  pcl_ros 
  pcl_conversions
)
ament_target_dependencies(marker_helper 
  rclcpp 
  geometry_msgs
  visualization_msgs
)
ament_target_dependencies(csv_logger 
  rclcpp 
  std_msgs
  geometry_msgs
)

# インストール設定
install(TARGETS leg_cluster_tracking_node 
    DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/led_status.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
