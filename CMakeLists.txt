cmake_minimum_required(VERSION 3.5)
project(sweep_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

find_package(Sweep REQUIRED)
find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#add_definitions(${PCL_DEFINITIONS})

ament_auto_add_executable(sweep_node src/node.cpp)

target_link_libraries(sweep_node
  ${PCL_LIBRARIES}
  ${LIBSWEEP_LIBRARY}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()