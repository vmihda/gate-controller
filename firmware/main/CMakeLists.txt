set(srcs
    "app_main.cpp"
    "boot_button.cpp"
    "gate_task.cpp"
    "status_led.cpp"
)

set(requires console esp_driver_gpio esp_driver_rmt esp_hal_wdt esp_hw_support esp_timer nvs_flash gate_core)

# Kconfig values are known here in the normal pass, so sources may depend on them.
# GATE_DEMO builds demo_gate.cpp in place of flash_in.cpp and relay_out.cpp (see demo_gate.h).
if(CONFIG_GATE_DEMO)
    list(APPEND srcs "demo_gate.cpp")
else()
    list(APPEND srcs "flash_in.cpp" "relay_out.cpp")
endif()
if(CONFIG_GATE_SIM)
    list(APPEND srcs "sim_console.cpp")
else()
    list(APPEND srcs "matter_app.cpp" "setup_code.cpp")
endif()

# REQUIRES are expanded before sdkconfig is loaded, so they follow the build property set by the
# project CMakeLists.txt instead of a Kconfig option.
idf_build_get_property(gate_with_matter GATE_WITH_MATTER)
if(gate_with_matter)
    list(APPEND requires esp_matter nice_core)
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES ${requires}
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
