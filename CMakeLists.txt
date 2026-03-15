cmake_minimum_required(VERSION 3.16)
project(motor_model_f1404 VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# === Bibliotheken ===
add_library(motor_model STATIC
    src/motor.cpp
)

target_include_directories(motor_model PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/includes
)

# === UNIT TEST ===
add_executable(test_motor
    src/test_motor.cpp
)

target_link_libraries(test_motor PRIVATE motor_model)

enable_testing()
add_test(NAME motor_tests COMMAND test_motor)

# === AUSGABE ===
message(STATUS "Motor Model F1404 KV4600 - Build konfiguriert")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")    