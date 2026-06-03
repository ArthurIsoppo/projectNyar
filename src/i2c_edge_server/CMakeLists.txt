cmake_minimum_required(VERSION 3.8)
project(i2c_edge_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_interfaces REQUIRED)
find_package(std_msgs REQUIRED) # <--- ADICIONADO: Pacote de mensagens padrão (para o UInt8MultiArray)

set(dependencies 
  rclcpp 
  sensor_interfaces
  std_msgs                      # <--- ADICIONADO: Incluído na lista de dependências do nó
)

# Compile the generic i2c server node
add_executable(i2c_server src/i2c_server.cpp)
ament_target_dependencies(i2c_server ${dependencies})

# Install rules so ROS 2 can find the executable
install(TARGETS
  i2c_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# <--- ADICIONADO: Regra para instalar o YAML de parâmetros na Pi --->
install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)
# <----------------------------------------------------------------->

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()