cmake_minimum_required(VERSION 3.5)
project(mmg)

set(CMAKE_C_STANDARD 99)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost 1.45.0 COMPONENTS math) 

add_executable(${PROJECT_NAME} src/main.cpp src/mmg_sim.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE include/)

file(COPY config/mmg_config.yaml config/sim_config.yaml DESTINATION ./ )

include_directories(${Boost_INCLUDE_DIRS}) 
include_directories(${CMAKE_SOURCE_DIR}/yaml-cpp/include)

target_link_libraries(${PROJECT_NAME} yaml-cpp)
target_link_libraries (${PROJECT_NAME} Eigen3::Eigen)

#add_subdirectory(test)
