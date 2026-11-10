cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "0.1.0")
set(PROJECT_VER_NUMBER 1)

# Whether this build links Matter. ESP-IDF resolves component REQUIRES before it loads sdkconfig
# (build-system guide, "REQUIRES ... should not depend on any configuration options"), so CONFIG_GATE_SIM
# cannot switch esp_matter on and off. The choice is made here instead: a build whose SDKCONFIG_DEFAULTS
# include sdkconfig.sim has no Matter. It is cached per build directory, and after project() the generated
# sdkconfig is checked to agree with it.
if(NOT DEFINED CACHE{GATE_WITH_MATTER})
    set(gate_defaults "$ENV{SDKCONFIG_DEFAULTS}")
    if(SDKCONFIG_DEFAULTS)
        set(gate_defaults "${SDKCONFIG_DEFAULTS}")
    endif()
    if(gate_defaults MATCHES "sdkconfig\\.sim")
        set(gate_with_matter OFF)
    else()
        set(gate_with_matter ON)
    endif()
    set(GATE_WITH_MATTER ${gate_with_matter} CACHE BOOL "Link esp-matter into the firmware (off for GATE_SIM)")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# nice_core: the Nice controller model, used by the GATE_DEMO build.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../emulator/components")

if(GATE_WITH_MATTER)
    if(NOT DEFINED ENV{ESP_MATTER_PATH})
        message(FATAL_ERROR "Please set ESP_MATTER_PATH to the path of esp-matter repo")
    endif()
    set(ESP_MATTER_PATH $ENV{ESP_MATTER_PATH})
    set(MATTER_SDK_PATH ${ESP_MATTER_PATH}/connectedhomeip/connectedhomeip)
    list(APPEND EXTRA_COMPONENT_DIRS
        "${MATTER_SDK_PATH}/config/esp32/components"
        "${ESP_MATTER_PATH}/components")
endif()

# Read by main/CMakeLists.txt, also during the early requirements expansion.
idf_build_set_property(GATE_WITH_MATTER ${GATE_WITH_MATTER})

project(gate_controller)

if(CONFIG_GATE_SIM AND GATE_WITH_MATTER)
    message(FATAL_ERROR "sdkconfig has GATE_SIM but this build directory links Matter. "
                        "Use a separate build directory with SDKCONFIG_DEFAULTS including sdkconfig.sim.")
endif()
if(NOT CONFIG_GATE_SIM AND NOT GATE_WITH_MATTER)
    message(FATAL_ERROR "This build directory was configured without Matter (sdkconfig.sim), "
                        "but sdkconfig does not have GATE_SIM. Use a separate build directory.")
endif()

# C++17 like the Matter SDK needs, so the same sources build with and without Matter.
idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++17" APPEND)
if(GATE_WITH_MATTER)
    idf_build_set_property(CXX_COMPILE_OPTIONS "-DCHIP_HAVE_CONFIG_H;-Wno-overloaded-virtual" APPEND)
    idf_build_set_property(COMPILE_OPTIONS "-Wno-format-nonliteral;-Wno-format-security" APPEND)
endif()
