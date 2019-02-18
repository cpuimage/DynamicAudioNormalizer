cmake_minimum_required(VERSION 2.8)
project(DynamicAudioNormalizer)

SET(CMAKE_BUILD_TYPE "Release")


file(GLOB SOURCES src/*.cpp)

include_directories(include)

add_executable(DynamicAudioNormalizer src/DynamicAudioNormalizer.cpp)
target_link_libraries(DynamicAudioNormalizer m)
