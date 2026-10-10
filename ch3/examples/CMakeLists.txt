include_directories("/usr/include/eigen3")
add_executable(coordinateTransform coordinateTransform.cpp)
add_executable(my_coordinateTransform my_coordinateTransform.cpp)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
add_executable(plotTrajectory plotTrajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
