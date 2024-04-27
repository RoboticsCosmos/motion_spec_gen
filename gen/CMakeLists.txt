cmake_minimum_required(VERSION 3.0.0)
project(motion_spec VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(motion_spec_utils REQUIRED)

link_directories(${CMAKE_INSTALL_PREFIX}/lib)

# add all cpp files in the folder
file(GLOB SOURCES "*.cpp")

# make executables
foreach(source ${SOURCES})
  get_filename_component(name ${source} NAME_WE)
  add_executable(${name} ${source})
  target_link_libraries(${name} Eigen3::Eigen motion_spec_utils::mutils motion_spec_utils::math_utils motion_spec_utils::solver_utils)

  # set runtime path
  set_target_properties(${name} PROPERTIES
   BUILD_WITH_INSTALL_RPATH TRUE
   INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
   INSTALL_RPATH_USE_LINK_PATH TRUE)
  install(TARGETS ${name} RUNTIME DESTINATION bin)
endforeach(source ${SOURCES})

