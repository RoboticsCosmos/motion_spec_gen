cmake_minimum_required(VERSION 3.0.0)
project(motion_spec VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(kdl_utils REQUIRED)
find_package(controllers REQUIRED)

link_directories(${CMAKE_INSTALL_PREFIX}/lib)

# export library
add_executable(${PROJECT_NAME}
  out.cpp
)

# link libraries
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  kdl_utils::mutils
  kdl_utils::math_utils
  kdl_utils::solver_utils
  controllers::pid_controller
)
