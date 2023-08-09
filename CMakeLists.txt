# CMake 프로그램의 최소 버전
cmake_minimum_required(VERSION 3.11)

# 프로젝트 정보
project(
  ModooCode
  VERSION 0.1
  DESCRIPTION "예제 프로젝트"
  LANGUAGES CXX)


find_package(ompl)
find_package(Boost COMPONENTS system REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(algorithm/include)
include_directories(include)

add_executable (ompl_app
  src/Point2DPlanning.cpp
  algorithm/RRTstar.cpp
  algorithm/mRRT.cpp
  algorithm/Planner.cpp
)



target_link_libraries(ompl_app
  ${OMPL_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
)

add_executable( test_app
src/test.cpp
)

target_link_libraries(test_app
  ${OMPL_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
)