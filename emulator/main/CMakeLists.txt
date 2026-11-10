idf_component_register(
    SRCS "app_main.c" "emu_console.c" "emu_logic.c" "flash_out.c" "pp_input.c" "stepper.c"
    INCLUDE_DIRS "."
    REQUIRES nice_core console esp_timer esp_driver_gpio esp_driver_gptimer freertos)
