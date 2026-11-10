# Host tests for gate_core. They build without ESP-IDF.
cmake_minimum_required(VERSION 3.16)
project(gate_core_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Werror -fsanitize=address,undefined -fno-sanitize-recover=all)
add_link_options(-fsanitize=address,undefined)

enable_testing()

set(GATE_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../components/gate_core)
set(NICE_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../emulator/components/nice_core)
set(EMU_MAIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../emulator/main)
set(SHARED_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/include)

function(add_host_test name)
    add_executable(${name} ${name}.c ${ARGN})
    target_include_directories(${name} PRIVATE ${GATE_CORE_DIR}/include ${NICE_CORE_DIR}/include ${SHARED_INCLUDE_DIR} ${EMU_MAIN_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(${name} PRIVATE m)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_host_test(test_flash_decoder ${GATE_CORE_DIR}/flash_decoder.c)
add_host_test(test_gate_controller ${GATE_CORE_DIR}/gate_controller.c)
add_host_test(test_status_policy ${GATE_CORE_DIR}/status_policy.c)
add_host_test(test_button_press ${GATE_CORE_DIR}/button_press.c)
add_host_test(test_gate_view ${GATE_CORE_DIR}/gate_view.c)
add_host_test(test_gate_status ${GATE_CORE_DIR}/gate_status.c ${GATE_CORE_DIR}/gate_controller.c)
add_host_test(test_nice_sim ${NICE_CORE_DIR}/nice_sim.c)
add_host_test(test_pulse_spacing ${GATE_CORE_DIR}/pulse_spacing.c)
add_host_test(test_bench_round_trip ${GATE_CORE_DIR}/flash_decoder.c ${GATE_CORE_DIR}/gate_controller.c ${GATE_CORE_DIR}/pulse_spacing.c ${NICE_CORE_DIR}/nice_sim.c)
add_host_test(test_emu_logic ${EMU_MAIN_DIR}/emu_logic.c)
