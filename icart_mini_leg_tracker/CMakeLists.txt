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

# ヘッダーファイルのディレクトリを追加
include_directories(include)

# 実行ファイルの設定
add_executable(leg_cluster_tracking_node src/leg_cluster_tracking.cpp)

# 必要なライブラリをリンク
ament_target_dependencies(leg_cluster_tracking_node 
    rclcpp 
    sensor_msgs 
    geometry_msgs  # 追加
    visualization_msgs 
    pcl_ros 
    pcl_conversions
)

# インストール設定
install(TARGETS leg_cluster_tracking_node 
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
