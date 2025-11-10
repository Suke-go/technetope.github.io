cmake_minimum_required(VERSION 3.20)
project(toio_cpp_client LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(Boost_NO_BOOST_CMAKE ON)
set(Boost_NO_WARN_NEW_VERSIONS ON)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()
find_package(Boost 1.78 REQUIRED)

find_package(nlohmann_json 3.11 QUIET)
if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    message(STATUS "Fetching nlohmann_json via FetchContent")
    FetchContent_Declare(
        nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.3
    )
    FetchContent_MakeAvailable(nlohmann_json)
endif()

find_package(yaml-cpp 0.7 REQUIRED)

add_library(toio_lib STATIC
    src/transport/toio_client.cpp
    src/middleware/server_session.cpp
    src/middleware/fleet_manager.cpp
    src/control/goal_controller.cpp
    src/api/fleet_control.cpp
    src/cli/config_loader.cpp
)
target_include_directories(toio_lib
    PUBLIC
        include
        ${Boost_INCLUDE_DIRS}
)
target_link_libraries(toio_lib
    PUBLIC
        nlohmann_json::nlohmann_json
        yaml-cpp::yaml-cpp
)

# Boost.System をヘッダオンリーで利用
target_compile_definitions(toio_lib
    PRIVATE
        BOOST_ERROR_CODE_HEADER_ONLY
        BOOST_SYSTEM_NO_LIB
)

add_executable(toio_cli
    src/main.cpp
)
target_link_libraries(toio_cli PRIVATE toio_lib)

add_executable(fleet_control_sample
    samples/fleet_control_sample.cpp
)
target_link_libraries(fleet_control_sample PRIVATE toio_lib)

add_executable(circle_motion_sample
    samples/circle_motion_sample.cpp
    samples/motion_planner.cpp
)
target_link_libraries(circle_motion_sample PRIVATE toio_lib)
