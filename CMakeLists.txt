cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom service interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetCount.srv"
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
    include
)

# publisher library
add_library(pub_lib src/pub.cpp)

ament_target_dependencies(
  pub_lib
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# For internal use of custom services or message
# in the same package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(pub_lib
  "${cpp_typesupport_target}"
)

# subscriber library
add_library(sub_lib src/sub.cpp)

ament_target_dependencies(
  sub_lib
  rclcpp
  std_msgs
)

# Add publisher executable
add_executable(talker src/talker.cpp)
target_link_libraries(talker
  pub_lib
)

# Add subscriber executable
add_executable(listener src/listener.cpp)
target_link_libraries(listener
  sub_lib
)

# Install executable
install(
  TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/beginner_tutorials
  DESTINATION include
)

# Install shared library
install(
  TARGETS pub_lib
  EXPORT pub_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS sub_lib
  EXPORT sub_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install launch file
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
    test/main.cpp
    test/test_pub.cpp
  )

  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  target_link_libraries(${PROJECT_NAME}_test
    pub_lib
  )

  ament_target_dependencies(
    ${PROJECT_NAME}_test
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  install(
    TARGETS ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME}
  )

endif()

ament_package()
