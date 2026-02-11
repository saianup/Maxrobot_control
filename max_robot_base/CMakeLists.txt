cmake_minimum_required(VERSION 3.16)
project(max_robot_base LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux:
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  #serial
)

find_package(Boost REQUIRED COMPONENTS system)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  max_robot_base
  SHARED
  src/max_robot_system.cpp
  src/max_robot_base.cpp
)
target_compile_features(max_robot_base PUBLIC cxx_std_17)
target_include_directories(max_robot_base PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/>
$<INSTALL_INTERFACE:include/>
)
ament_target_dependencies(
  max_robot_base PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
#target_link_libraries(max_robot_base Boost::system)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface max_robot_base.xml)

# INSTALL
install(
  DIRECTORY include/max_robot_base
  DESTINATION include/max_robot_base
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS max_robot_base
  EXPORT export_max_robot_base
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  #ament_add_pytest_test(example_11_urdf_xacro test/test_urdf_xacro.py)
  #ament_add_pytest_test(view_example_11_launch test/test_view_robot_launch.py)
  #ament_add_pytest_test(run_example_11_launch test/test_max_robot_launch.py)
  #ament_add_pytest_test(run_example_11_launch_remapped test/test_max_robot_launch_remapped.py)
endif()

## EXPORTS
ament_export_targets(export_max_robot_base HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
