add_executable(agent_a_monitor
    src/main.cpp
)

target_include_directories(agent_a_monitor
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party
)

target_link_libraries(agent_a_monitor
    PRIVATE
        acoustics_osc
)

if (WIN32)
    target_link_libraries(agent_a_monitor PRIVATE ws2_32)
endif()
