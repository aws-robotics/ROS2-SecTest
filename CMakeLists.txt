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
    ${geometry_msgs_INCLUDE_DIRS}
    ${lifecycle_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_lifecycle_INCLUDE_DIRS}
)


function(custom_component target file_path component)
    add_library(${target} SHARED ${file_path})
    target_compile_definitions(${target}
        PRIVATE "COMPOSITION_BUILDING_DLL")
    ament_target_dependencies(${target}
        "class_loader"
        "${rclcpp_LIBRARIES}"
        "${rclcpp_lifecycle_LIBRARIES}"
        "geometry_msgs")
    rclcpp_components_register_nodes(${target} ${component})
endfunction()

custom_component(
    noop_component src/attacks/noop/component.cpp
    ros_sec_test::attacks::noop::Component)
custom_component(
    teleop_component src/attacks/coms/teleop/component.cpp
    ros_sec_test::attacks::coms::teleop::Component)
custom_component(
    disk_component src/attacks/resources/disk/component.cpp
    ros_sec_test::attacks::resources::disk::Component)

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_library(
    utilities
    SHARED
    src/utilities/lifecycle_service_client.cpp
    src/utilities/service_utils.cpp
)

add_executable(
    runner
    src/runner/main.cpp
    src/runner/runner.cpp
)
target_link_libraries(
    runner
    noop_component
    disk_component
    teleop_component
    utilities
)
ament_target_dependencies(runner "rclcpp" "rclcpp_lifecycle")

install(TARGETS
  noop_component
  teleop_component
  disk_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
    runner
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
