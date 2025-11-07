set(LIBS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenSSL REQUIRED)

add_library(acoustics_osc STATIC
    src/OscPacket.cpp
    src/OscTransport.cpp
    src/DeviceRegistry.cpp
    src/OscEncryptor.cpp
)

target_include_directories(acoustics_osc
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party
)

target_link_libraries(acoustics_osc
    PUBLIC
        asio::asio
        spdlog::spdlog
        fmt::fmt
        OpenSSL::Crypto
)

target_compile_definitions(acoustics_osc
    PUBLIC
        ASIO_STANDALONE
)

target_compile_features(acoustics_osc PUBLIC cxx_std_17)
