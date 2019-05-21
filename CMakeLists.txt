cmake_minimum_required(VERSION 3.5)

project(ros_sec_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(geometry_msgs)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcpputils REQUIRED)

# Enable strict compiler flags if possible.
include(CheckCXXCompilerFlag)
set(FLAGS -pedantic -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings -Wconversion -Wmissing-declarations)
foreach(FLAG ${FLAGS})
  check_cxx_compiler_flag(${FLAG} R${FLAG})
  if(${R${FLAG}})
    set(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} ${FLAG}")
  endif()
endforeach()

if(NOT DEFINED CXX_DISABLE_WERROR)
  set(WARNING_CXX_FLAGS "-Werror ${WARNING_CXX_FLAGS}")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARNING_CXX_FLAGS}")

include_directories(
    include
    src   # For private headers
    ${lifecycle_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_lifecycle_INCLUDE_DIRS}
    ${rcpputils_INCLUDE_DIRS}
)

# This package contains multiple targets:
#
# ros_sec_utility <- ros_sec_attacks <- runner
# (all attacks)   <-/

# Define utility library containing all the shared code.
add_library(
    ros_sec_test_utilities
    SHARED
    src/utilities/lifecycle_service_client.cpp
    src/utilities/service_utils.cpp
)
ament_target_dependencies(ros_sec_test_utilities "rclcpp" "rclcpp_lifecycle")

# Define all attacks.
function(custom_component target file_path component)
    add_library(${target} SHARED ${file_path})
    target_compile_definitions(${target}
        PRIVATE "COMPOSITION_BUILDING_DLL")
    ament_target_dependencies(${target} "class_loader")
    rclcpp_components_register_nodes(${target} ${component})
endfunction()

custom_component(
    noop_component src/attacks/noop/component.cpp
    ros_sec_test::attacks::noop::Component)
custom_component(
    cpu_component src/attacks/resources/cpu/component.cpp
    ros_sec_test::attacks::resources::cpu::Component)
custom_component(
    disk_component src/attacks/resources/disk/component.cpp
    ros_sec_test::attacks::resources::disk::Component)

# Define the attack library containing all attack nodes and the factory.
add_library(
    ros_sec_test_attacks
    SHARED
    src/attacks/factory_utils.cpp
)
target_link_libraries(
    ros_sec_test_attacks
    noop_component
    cpu_component
    disk_component
)
ament_target_dependencies(ros_sec_test_attacks "class_loader" "rclcpp" "rclcpp_lifecycle")

install(TARGETS
  ros_sec_test_attacks
  ros_sec_test_utilities
  noop_component
  cpu_component
  disk_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Define the runner node.
add_executable(
    runner
    src/runner/main.cpp
    src/runner/runner.cpp
)
target_link_libraries(
    runner
    ros_sec_test_attacks
    ros_sec_test_utilities
)
ament_target_dependencies(runner "rclcpp" "rclcpp_lifecycle")
install(TARGETS runner DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_service_utils test/utilities/test_service_utils.cpp)
  target_link_libraries(test_service_utils ros_sec_test_utilities)

  ament_add_gtest(test_client_utils test/utilities/test_client_utils.cpp)
  target_link_libraries(test_client_utils ros_sec_test_utilities)
endif()

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()


ament_package()
