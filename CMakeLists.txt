cmake_minimum_required(VERSION 3.13)
project(inrof_swerm CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP)

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

add_library(follower
    src/follower.cpp
)

target_link_libraries(follower
    Boid
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
    follower
)
