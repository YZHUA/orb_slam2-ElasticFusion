cmake_minimum_required(VERSION 2.8)
project(FormatConverter)
find_package(OpenCV 2.4.3 REQUIRED)
find_package(OpenNI2 REQUIRED)
include_directories(${OpenNI2_INCLUDE_PATH})
add_executable(FormatConverter
oni2klg.cpp)
target_link_libraries(FormatConverter
${OpenCV_LIBS}
${OpenNI2_LIBRARY})
