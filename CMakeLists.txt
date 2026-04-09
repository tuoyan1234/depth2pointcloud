cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(depth2cloud)

# find_package(PCL 1.10 REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io visualization)
find_package(OpenCV REQUIRED)

# target_include_directories(depth2cloud ./include)
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

add_executable (depth2cloud main.cpp src/depth2cloud.cpp)
target_include_directories(depth2cloud PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries (depth2cloud ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(depth2cloud yaml-cpp)
