cmake_minimum_required(VERSION 3.0.0)
project(motion_spec VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
add_compile_definitions(_OS_UNIX)

LINK_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/lib/)

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(controllers REQUIRED)
find_package(motion_spec_utils REQUIRED)

list(APPEND local_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include/)

# include directories
include_directories(
  ${motion_spec_utils_INCLUDE_DIRS}
  ${local_INCLUDE_DIRS}

  ${CMAKE_INSTALL_PREFIX}/include/kinova_api/
  ${CMAKE_INSTALL_PREFIX}/include/kinova_api/client
  ${CMAKE_INSTALL_PREFIX}/include/kinova_api/common
  ${CMAKE_INSTALL_PREFIX}/include/kinova_api/messages
  ${CMAKE_INSTALL_PREFIX}/include/kinova_api/client_stubs
  ${CMAKE_INSTALL_PREFIX}/include/kinova_api/google
)

link_directories(${CMAKE_INSTALL_PREFIX}/lib)

# add all cpp files in the folder
file(GLOB SOURCES "*.cpp")

# make executables
foreach(source ${SOURCES})
  get_filename_component(name ${source} NAME_WE)
  add_executable(${name} ${source})
  target_link_libraries(${name} 
    Eigen3::Eigen 
    motion_spec_utils::mutils 
    motion_spec_utils::math_utils 
    motion_spec_utils::solver_utils
    controllers::pid_controller
  )

  # set runtime path
  set_target_properties(${name} PROPERTIES
   BUILD_WITH_INSTALL_RPATH TRUE
   INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
   INSTALL_RPATH_USE_LINK_PATH TRUE)
  install(TARGETS ${name} RUNTIME DESTINATION bin)
endforeach(source ${SOURCES})

