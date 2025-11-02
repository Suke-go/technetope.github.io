add_executable(agent_a_scheduler
    src/main.cpp
    src/SoundTimeline.cpp
)

target_include_directories(agent_a_scheduler
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party
)

target_link_libraries(agent_a_scheduler
    PRIVATE
        acoustics_osc
)

if (WIN32)
    target_link_libraries(agent_a_scheduler PRIVATE ws2_32)
endif()
