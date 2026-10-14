cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(bearamin_virtual_2040 C CXX ASM)
pico_sdk_init()
add_executable(bearamin_virtual_2040
  main.cpp usb_descriptors.c
  vendor/vl53l7cx/vl53l7cx_api.cpp vendor/vl53l7cx/platform.cpp)
target_include_directories(bearamin_virtual_2040 PRIVATE
  ${CMAKE_CURRENT_LIST_DIR} vendor/vl53l7cx
  ${CMAKE_CURRENT_LIST_DIR}/../../third_party/ArduinoJson/src
  ${CMAKE_CURRENT_LIST_DIR}/../../third_party/bearfield-core/include)
target_link_libraries(bearamin_virtual_2040 PRIVATE
  pico_stdlib pico_rand hardware_i2c hardware_flash hardware_sync tinyusb_device tinyusb_board)
pico_enable_stdio_usb(bearamin_virtual_2040 0)
pico_enable_stdio_uart(bearamin_virtual_2040 0)
pico_add_extra_outputs(bearamin_virtual_2040)

set(BEARAMIN_VERSION "0.3.0.2")
string(REPLACE "." ";" BEARAMIN_VERSION_PARTS "${BEARAMIN_VERSION}")
foreach(i RANGE 0 3)
  list(GET BEARAMIN_VERSION_PARTS ${i} component)
  target_compile_definitions(bearamin_virtual_2040 PRIVATE BEARAMIN_ID_V${i}=${component})
endforeach()
pico_set_program_version(bearamin_virtual_2040 "${BEARAMIN_VERSION}")

target_compile_definitions(bearamin_virtual_2040 PRIVATE BEARAMIN_EDITOR_VERSION="${BEARAMIN_VERSION}")

# Reserve an immutable16KiB loader and a separate1MiB staging area.
file(READ "${PICO_SDK_PATH}/src/rp2_common/pico_crt0/rp2040/memmap_default.ld" APP_LINKER)
string(REPLACE "INCLUDE \"pico_flash_region.ld\"" "FLASH(rx) : ORIGIN = 0x10004000, LENGTH = 1008k" APP_LINKER "${APP_LINKER}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/application.ld" "${APP_LINKER}")
pico_set_linker_script(bearamin_virtual_2040 "${CMAKE_CURRENT_BINARY_DIR}/application.ld")
target_link_libraries(bearamin_virtual_2040 PRIVATE pico_mbedtls hardware_watchdog)
add_executable(bearamin_virtual_loader loader/main.cpp)
target_include_directories(bearamin_virtual_loader PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(bearamin_virtual_loader PRIVATE pico_stdlib hardware_flash hardware_sync)
pico_enable_stdio_usb(bearamin_virtual_loader 0)
pico_enable_stdio_uart(bearamin_virtual_loader 0)
pico_add_extra_outputs(bearamin_virtual_loader)

target_compile_definitions(bearamin_virtual_2040 PRIVATE MBEDTLS_CONFIG_FILE="mbedtls_config.h")
