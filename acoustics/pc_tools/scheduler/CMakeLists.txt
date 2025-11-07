add_library(acoustics_scheduler STATIC
    src/SchedulerController.cpp
    src/SoundTimeline.cpp
    src/TargetResolver.cpp
)

target_include_directories(acoustics_scheduler
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party
)

target_link_libraries(acoustics_scheduler
    PUBLIC
        acoustics_osc
)

add_executable(agent_a_scheduler
    src/main.cpp
)

target_include_directories(agent_a_scheduler
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party
)

target_link_libraries(agent_a_scheduler
    PRIVATE
        acoustics_scheduler
)

if (WIN32)
    target_link_libraries(agent_a_scheduler PRIVATE ws2_32)
endif()

if (BUILD_TESTING)
    add_executable(acoustics_scheduler_tests
        tests/SoundTimelineTests.cpp
        tests/OscEncryptorTests.cpp
    )

    target_link_libraries(acoustics_scheduler_tests
        PRIVATE
            acoustics_scheduler
            Catch2::Catch2WithMain
    )

    add_test(NAME acoustics_scheduler_tests COMMAND acoustics_scheduler_tests)
endif()
