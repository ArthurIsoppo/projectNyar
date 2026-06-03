cmake_minimum_required(VERSION 3.8)
project(accel_logic_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies [9]
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies 
  rclcpp 
  sensor_interfaces
  geometry_msgs
)

# Compile the node [9]
add_executable(vector_logic_client src/vector_logic_client.cpp)
ament_target_dependencies(vector_logic_client ${dependencies})

# Install rules to make the executable accessible to ROS 2 [10]
install(TARGETS
  vector_logic_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# --- ADICIONE ESTE BLOCO AQUI ---
install(DIRECTORY 
  launch 
  config
  DESTINATION share/${PROJECT_NAME}
)
# -------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
