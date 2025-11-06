cmake_minimum_required(VERSION 3.20)

project(locomotion_calibration LANGUAGES CXX)

option(LOCOMOTION_BUILD_TESTS "Build calibration module tests" OFF)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED aruco)
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(locomotion_calibration STATIC
    src/CalibrationPipeline.cpp
    src/CharucoDetector.cpp
    src/CalibrationSession.cpp
    src/PlaymatLayout.cpp
    src/FloorPlaneEstimator.cpp
)

target_include_directories(locomotion_calibration
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(locomotion_calibration
    PUBLIC
        realsense2::realsense2
        spdlog::spdlog
        ${OpenCV_LIBS}
        nlohmann_json::nlohmann_json
)

target_compile_features(locomotion_calibration PUBLIC cxx_std_20)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR AND LOCOMOTION_BUILD_TESTS)
    enable_testing()
endif()

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    add_executable(capture_calibration tools/capture_calibration.cpp)
    target_link_libraries(capture_calibration PRIVATE locomotion_calibration)
    target_compile_features(capture_calibration PRIVATE cxx_std_20)
endif()
