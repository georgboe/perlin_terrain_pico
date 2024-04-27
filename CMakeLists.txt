cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/georg/Documents/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(perlin_terrain C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Ofast")
set(PICO_DEFAULT_BINARY_TYPE copy_to_ram)

add_subdirectory(common)
add_subdirectory(drivers)
add_subdirectory(libraries)

# Add executable. Default name is the project name, version 0.1
add_executable(perlin_terrain perlin_terrain.cpp )

pico_generate_pio_header(perlin_terrain ${CMAKE_CURRENT_LIST_DIR}/st7789_lcd.pio)

pico_set_program_name(perlin_terrain "perlin_terrain")
pico_set_program_version(perlin_terrain "0.2")

pico_enable_stdio_uart(perlin_terrain 0)
pico_enable_stdio_usb(perlin_terrain 1)

# Add the standard library to the build
target_link_libraries(perlin_terrain  pico_stdlib pico_multicore hardware_spi hardware_dma hardware_pio hardware_pwm)

# Add any user requested libraries
target_link_libraries(perlin_terrain
         rgbled pico_display pico_graphics st7789 button
)

pico_add_extra_outputs(perlin_terrain)

#aasdf
add_executable(perlin_terrain_dp2 perlin_terrain.cpp )

pico_set_program_name(perlin_terrain_dp2 "perlin_terrain")
pico_set_program_version(perlin_terrain_dp2 "0.2")

pico_enable_stdio_uart(perlin_terrain_dp2 0)
pico_enable_stdio_usb(perlin_terrain_dp2 1)

# Add the standard library to the build
target_link_libraries(perlin_terrain_dp2 PRIVATE pico_stdlib pico_multicore hardware_spi hardware_dma hardware_pio hardware_pwm)

# Add any user requested libraries
target_link_libraries(perlin_terrain_dp2
        PRIVATE rgbled pico_display_2 pico_graphics st7789 button
)

target_compile_definitions(perlin_terrain_dp2 PRIVATE   
       PICO_DISPLAY_2  
    )
pico_add_extra_outputs(perlin_terrain_dp2)

# Slow down flash during boot for heavy overclocking.
pico_define_boot_stage2(slower_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
target_compile_definitions(slower_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
pico_set_boot_stage2(perlin_terrain slower_boot2)
pico_set_boot_stage2(perlin_terrain_dp2 slower_boot2)