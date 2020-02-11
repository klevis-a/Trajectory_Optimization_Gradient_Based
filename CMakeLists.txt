cmake_minimum_required(VERSION 2.8.3)
project(m20ia_traj_opt)

add_compile_options(-std=c++11)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(Boost REQUIRED COMPONENTS filesystem system regex)
find_package(Pagmo REQUIRED)

find_package(catkin REQUIRED cmake_modules kdl_parser)
find_package(TinyXML2 REQUIRED)

catkin_package(
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)

set(PAGMO_LIBS Pagmo::pagmo  ${Boost_FILESYSTEM_LIBRARIES}  ${Boost_SYSTEM_LIBRARIES}  ${CMAKE_DL_LIBS})

add_library(TrajOpt_Solver src/TrajOpt_Solver.cpp src/XMLParser.cpp src/ConfigParser.cpp src/InputParser.cpp src/Robot_mocap_opt_problem.cpp src/DataReader.cpp src/GeometryUtils.cpp src/RobotKDL.cpp src/CsvWriter.cpp)

add_executable(single_solver src/single_solver.cpp)
target_link_libraries(single_solver TrajOpt_Solver ${PAGMO_LIBS} ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})

add_executable(batch_solver_gradient src/batch_solver.cpp)
target_link_libraries(batch_solver_gradient TrajOpt_Solver ${PAGMO_LIBS} ${Boost_REGEX_LIBRARIES} ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})

add_executable(seed_solver src/seed_solver.cpp)
target_link_libraries(seed_solver TrajOpt_Solver ${PAGMO_LIBS} ${Boost_REGEX_LIBRARIES} ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})
