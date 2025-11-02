cmake_minimum_required(VERSION 3.16)
project(acoustics_pc_tools LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ACOUSTICS_ENABLE_ASAN "Enable AddressSanitizer for local builds" OFF)

if(ACOUSTICS_ENABLE_ASAN)
    if (MSVC)
        message(WARNING "ASan option is currently not supported on MSVC in this project.")
    else()
        add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
        add_link_options(-fsanitize=address -fno-omit-frame-pointer)
    endif()
endif()

include(FetchContent)

set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG 10.2.1
)

FetchContent_Declare(spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.12.0
)

FetchContent_Declare(asio
    GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
    GIT_TAG asio-1-28-0
)

set(SPDLOG_FMT_EXTERNAL ON CACHE BOOL "" FORCE)
set(ASIO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(ASIO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ASIO_BUILD_DOC OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(fmt spdlog asio)

if (NOT TARGET asio::asio)
    add_library(asio INTERFACE)
    target_include_directories(asio INTERFACE ${asio_SOURCE_DIR}/asio/include)
    add_library(asio::asio ALIAS asio)
endif()

add_subdirectory(libs)
add_subdirectory(scheduler)
add_subdirectory(monitor)
