cmake_minimum_required(VERSION 3.13)
project(inrof_swerm CXX)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
  include
  include/${PROJECT_NAME}
)

add_library(Boid
    src/Boid.cpp
)

target_link_libraries(Boid
    Eigen3::Eigen
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(leader_follower
    src/leader_follower.cpp
)

target_link_libraries(leader_follower
    Boid
    Eigen3::Eigen
    ${OpenCV_LIBS}
)
