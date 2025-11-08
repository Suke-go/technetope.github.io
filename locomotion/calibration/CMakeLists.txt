cmake_minimum_required(VERSION 3.20)

project(locomotion_calibration LANGUAGES CXX)

option(LOCOMOTION_BUILD_TESTS "Build calibration module tests" OFF)
option(LOCOMOTION_ENABLE_HARDWARE_TESTS "Enable hardware-dependent tests (requires RealSense device)" OFF)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED aruco highgui imgcodecs)
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(locomotion_calibration STATIC
    src/CalibrationPipeline.cpp
    src/CharucoDetector.cpp
    src/CalibrationSession.cpp
    src/PlaymatLayout.cpp
    src/FloorPlaneEstimator.cpp
    src/ToioCoordinateTransform.cpp
    src/HumanDetector.cpp
    src/HumanDetectionConverter.cpp
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

    add_executable(capture_calibration_interactive tools/capture_calibration_interactive.cpp)
    target_link_libraries(capture_calibration_interactive
        PRIVATE
            locomotion_calibration
            ${OpenCV_LIBS}
            spdlog::spdlog
            nlohmann_json::nlohmann_json)
    target_compile_features(capture_calibration_interactive PRIVATE cxx_std_20)

    add_executable(calibrate_intrinsics tools/calibrate_intrinsics.cpp)
    target_link_libraries(calibrate_intrinsics PRIVATE locomotion_calibration)
    target_compile_features(calibrate_intrinsics PRIVATE cxx_std_20)

    add_executable(run_calibration_qc tools/run_calibration_qc.cpp)
    target_link_libraries(run_calibration_qc PRIVATE locomotion_calibration)
    target_compile_features(run_calibration_qc PRIVATE cxx_std_20)

    add_executable(monitor_human_detection tools/monitor_human_detection.cpp)
    target_link_libraries(monitor_human_detection
        PRIVATE
            locomotion_calibration
            ${OpenCV_LIBS}
            spdlog::spdlog
            nlohmann_json::nlohmann_json)
    target_compile_features(monitor_human_detection PRIVATE cxx_std_20)

    # QC test integration (requires real RealSense device)
    # Only enabled when LOCOMOTION_ENABLE_HARDWARE_TESTS is ON
    if(LOCOMOTION_BUILD_TESTS AND LOCOMOTION_ENABLE_HARDWARE_TESTS)
        add_test(NAME calibration_qc
                 COMMAND run_calibration_qc
                 WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
        set_tests_properties(calibration_qc PROPERTIES
            LABELS "qc;hardware"
            TIMEOUT 120)
    endif()

    # Unit tests (no hardware required)
    if(LOCOMOTION_BUILD_TESTS)
        add_executable(test_floor_plane_estimator test/test_floor_plane_estimator.cpp)
        target_link_libraries(test_floor_plane_estimator PRIVATE locomotion_calibration)
        target_compile_features(test_floor_plane_estimator PRIVATE cxx_std_20)
        target_include_directories(test_floor_plane_estimator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)

        add_test(NAME floor_plane_estimator_unit
                 COMMAND test_floor_plane_estimator
                 WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
        set_tests_properties(floor_plane_estimator_unit PROPERTIES
            LABELS "unit;mock"
            TIMEOUT 30)

        # Integration tests (no hardware required)
        add_executable(test_calibration_pipeline_integration test/test_calibration_pipeline_integration.cpp)
        target_link_libraries(test_calibration_pipeline_integration PRIVATE locomotion_calibration)
        target_compile_features(test_calibration_pipeline_integration PRIVATE cxx_std_20)

        add_test(NAME calibration_pipeline_integration
                 COMMAND test_calibration_pipeline_integration
                 WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
        set_tests_properties(calibration_pipeline_integration PROPERTIES
            LABELS "integration;mock"
            TIMEOUT 10)
    endif()
endif()
