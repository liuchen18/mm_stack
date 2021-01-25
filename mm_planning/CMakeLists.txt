cmake_minimum_required(VERSION 3.17)
project(mm_planning)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED core highgui imgproc imgcodecs)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
        include/
)

add_library(${PROJECT_NAME}
        include/pose2d.h
        include/vec2d.h
        include/dynamic_voronoi.h
        include/bucket_queue.h
        include/kinematics.h
        include/point3d.h
        include/mm_planner.h
        include/iiwa_kinematics.h

        src/vec2d.cpp
        src/dynamic_voronoi.cpp
        src/bucket_queue.cpp

        src/mm_planner.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(base_planner src/plan_base_path.cpp)
target_link_libraries(base_planner ${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(manipulator_planner src/plan_manipulator_path.cpp)
target_link_libraries(manipulator_planner ${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(map_generator src/map_generator.cpp)
target_link_libraries(map_generator ${PROJECT_NAME} ${OpenCV_LIBS})

