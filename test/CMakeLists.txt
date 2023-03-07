Include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.0.1 # or a later release
)

FetchContent_MakeAvailable(Catch2)

file(COPY test_mmg_config.yaml test_sim_config.yaml DESTINATION ./ )

add_executable(tests test.cpp ../src/mmg_sim.cpp)

target_include_directories(tests PRIVATE ../include/)

target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)

include_directories(${Boost_INCLUDE_DIRS}) 
include_directories(${CMAKE_SOURCE_DIR}/yaml-cpp/include)

target_link_libraries(tests PRIVATE yaml-cpp)
target_link_libraries (tests PRIVATE Eigen3::Eigen)
