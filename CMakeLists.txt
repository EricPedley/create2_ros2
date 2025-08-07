cmake_minimum_required(VERSION 3.16)
project(create2_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Create the hardware interface library
add_library(create2_hardware SHARED
  src/create2_hardware_interface.cpp
)

target_compile_features(create2_hardware PUBLIC cxx_std_17)
target_include_directories(create2_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/create2_hardware>
)

ament_target_dependencies(create2_hardware PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface create2_hardware.xml)

# Install
install(DIRECTORY include/
  DESTINATION include/create2_hardware
)

install(TARGETS create2_hardware
  EXPORT export_create2_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES create2_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_targets(export_create2_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()