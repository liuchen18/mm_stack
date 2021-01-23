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

        src/vec2d.cpp
        src/dynamic_voronoi.cpp
        src/bucket_queue.cpp

        include/point3d.h include/mm_planner.h src/mm_planner.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(planner main.cpp)
target_link_libraries(planner ${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(map_generator src/map_generator.cpp)
target_link_libraries(map_generator ${PROJECT_NAME} ${OpenCV_LIBS})

