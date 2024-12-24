cmake_minimum_required(VERSION 3.8)
project(witmotion_ros2_driver C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)  # boost::asio
ament_auto_find_build_dependencies()

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
if (CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
  add_compile_options(-DNDEBUG)
elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  add_compile_options(-O3 -g)
elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-O0 -g)
endif()

file(GLOB_RECURSE SOURCES "src/*.cc" "src/*.c")
ament_auto_add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} Boost::system)
target_include_directories(${PROJECT_NAME} PRIVATE include ${Boost_INCLUDE_DIRS})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()