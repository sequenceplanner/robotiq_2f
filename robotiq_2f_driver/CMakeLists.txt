cmake_minimum_required(VERSION 3.5)
project(robotiq_2f_driver)

find_package(ament_cmake REQUIRED)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

include(r2r_cargo.cmake)

# put ros package dependencies here.
r2r_cargo(std_msgs               # just to test that it works
          std_srvs
          rcl                    # we need the c ros2 api
          rcl_action
          robotiq_2f_msgs
         )

# install binaries
install(PROGRAMS
  ${CMAKE_SOURCE_DIR}/target/release/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# we need this for ros/colcon
ament_package()
