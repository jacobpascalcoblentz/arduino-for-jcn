cmake_minimum_required(VERSION 3.14)
project(HydroponicsController VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Source files (excluding Arduino-specific main)
set(LIB_SOURCES
    src/sensors.cpp
    src/controller.cpp
    src/yaml_config.cpp
)

set(LIB_HEADERS
    src/config.h
    src/sensors.h
    src/controller.h
    src/yaml_config.h
)

# Create library for testing
add_library(hydro_lib STATIC ${LIB_SOURCES})
target_include_directories(hydro_lib PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_compile_definitions(hydro_lib PRIVATE)

# ============================================================================
# Google Test Setup
# ============================================================================

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

# Test executable
add_executable(test_runner
    test/test_main.cpp
    test/test_sensors.cpp
    test/test_controller.cpp
    test/test_yaml_config.cpp
)

target_link_libraries(test_runner
    PRIVATE
    hydro_lib
    GTest::gtest
    GTest::gtest_main
)

target_include_directories(test_runner PRIVATE ${CMAKE_SOURCE_DIR}/src)

include(GoogleTest)
gtest_discover_tests(test_runner)

# ============================================================================
# Code Coverage (optional)
# ============================================================================

option(ENABLE_COVERAGE "Enable code coverage" OFF)

if(ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(hydro_lib PRIVATE --coverage -O0 -g)
        target_link_options(hydro_lib PRIVATE --coverage)
        target_compile_options(test_runner PRIVATE --coverage -O0 -g)
        target_link_options(test_runner PRIVATE --coverage)
    endif()
endif()

# ============================================================================
# Custom Targets
# ============================================================================

# Run tests
add_custom_target(run_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS test_runner
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Verbose test output
add_custom_target(run_tests_verbose
    COMMAND test_runner --gtest_color=yes
    DEPENDS test_runner
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# ============================================================================
# Installation (for library use)
# ============================================================================

install(TARGETS hydro_lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES ${LIB_HEADERS}
    DESTINATION include/hydro
)

# ============================================================================
# Print Configuration Summary
# ============================================================================

message(STATUS "")
message(STATUS "Hydroponics Controller Build Configuration:")
message(STATUS "  Version: ${PROJECT_VERSION}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Coverage: ${ENABLE_COVERAGE}")
message(STATUS "")
message(STATUS "Build commands:")
message(STATUS "  cmake -B build -S .")
message(STATUS "  cmake --build build")
message(STATUS "  cmake --build build --target run_tests")
message(STATUS "")
