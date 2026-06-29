cmake_minimum_required(VERSION 3.8)
project(alice_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_executable(alice_control_node 
#   src/dk.cpp
# )

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)

add_executable(test_dk 
  src/test_dk.cpp
  src/dk.cpp
  src/ik.cpp
)

target_include_directories(test_dk PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  include
)

ament_target_dependencies(test_dk rclcpp std_msgs urdf)

install(TARGETS test_dk
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(test_ik 
  src/test_ik.cpp
  src/dk.cpp
  src/ik.cpp
)

target_include_directories(test_ik PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  include
)

ament_target_dependencies(test_ik rclcpp std_msgs urdf)

install(TARGETS test_ik
  DESTINATION lib/${PROJECT_NAME}
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
endif()


ament_package()
