# Pure-C model of the Nice SNA2 control unit, shared by the ESP32 emulator and the Wokwi custom chip.
idf_component_register(
    SRCS "nice_sim.c"
    INCLUDE_DIRS "include" "../../../shared/include")
