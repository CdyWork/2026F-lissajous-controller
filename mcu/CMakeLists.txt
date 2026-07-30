cmake_minimum_required(VERSION 3.20)

project(f2026_controller C ASM)
enable_language(C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(PROJECT_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(STM32CUBE_F4 "E:/MCU-help/STM32HAL/CubePackages/STM32CubeF4" CACHE PATH "STM32CubeF4 package root")
set(HAL_DRIVER ${STM32CUBE_F4}/Drivers/STM32F4xx_HAL_Driver)
set(CMSIS_ROOT ${STM32CUBE_F4}/Drivers/CMSIS)
set(FREERTOS_ROOT "E:/RTOS/FreeRTOS" CACHE PATH "FreeRTOS workspace root")
set(FREERTOS_KERNEL ${FREERTOS_ROOT}/FreeRTOS-Kernel)

foreach(required_dir IN ITEMS ${STM32CUBE_F4} ${HAL_DRIVER} ${CMSIS_ROOT} ${FREERTOS_KERNEL})
    if(NOT EXISTS "${required_dir}")
        message(FATAL_ERROR "Required MCU package directory not found: ${required_dir}")
    endif()
endforeach()

add_executable(${PROJECT_NAME})

target_sources(${PROJECT_NAME} PRIVATE
    ${PROJECT_ROOT}/Src/main.c
    ${PROJECT_ROOT}/Src/f2026_app.c
    ${PROJECT_ROOT}/Src/f2026_fpga.c
    ${PROJECT_ROOT}/Src/f2026_pi.c
    ${PROJECT_ROOT}/Src/bsp_board.c
    ${PROJECT_ROOT}/Src/bsp_clock.c
    ${PROJECT_ROOT}/Src/bsp_dma.c
    ${PROJECT_ROOT}/Src/bsp_error.c
    ${PROJECT_ROOT}/Src/bsp_lcd.c
    ${PROJECT_ROOT}/Src/bsp_lcd_font.c
    ${PROJECT_ROOT}/Src/bsp_rs485.c
    ${PROJECT_ROOT}/Src/bsp_spi.c
    ${PROJECT_ROOT}/Src/bsp_uart.c
    ${PROJECT_ROOT}/Src/stm32f4xx_hal_msp.c
    ${PROJECT_ROOT}/Src/stm32f4xx_it.c
    ${PROJECT_ROOT}/Src/syscall.c
    ${PROJECT_ROOT}/Src/sysmem.c
    ${PROJECT_ROOT}/Src/startup_stm32f407xx.S
    ${HAL_DRIVER}/Src/stm32f4xx_hal.c
    ${HAL_DRIVER}/Src/stm32f4xx_hal_cortex.c
    ${HAL_DRIVER}/Src/stm32f4xx_hal_dma.c
    ${HAL_DRIVER}/Src/stm32f4xx_hal_flash.c
    ${HAL_DRIVER}/Src/stm32f4xx_hal_gpio.c
    ${HAL_DRIVER}/Src/stm32f4xx_hal_rcc.c
    ${HAL_DRIVER}/Src/stm32f4xx_hal_spi.c
    ${HAL_DRIVER}/Src/stm32f4xx_hal_uart.c
    ${CMSIS_ROOT}/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
    ${FREERTOS_KERNEL}/tasks.c
    ${FREERTOS_KERNEL}/queue.c
    ${FREERTOS_KERNEL}/list.c
    ${FREERTOS_KERNEL}/timers.c
    ${FREERTOS_KERNEL}/event_groups.c
    ${FREERTOS_KERNEL}/stream_buffer.c
    ${FREERTOS_KERNEL}/portable/GCC/ARM_CM4F/port.c
    ${FREERTOS_KERNEL}/portable/MemMang/heap_4.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${PROJECT_ROOT}/Inc
    ${HAL_DRIVER}/Inc
    ${HAL_DRIVER}/Inc/Legacy
    ${CMSIS_ROOT}/Include
    ${CMSIS_ROOT}/Device/ST/STM32F4xx/Include
    ${FREERTOS_KERNEL}/include
    ${FREERTOS_KERNEL}/portable/GCC/ARM_CM4F
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32F407xx
    HSE_VALUE=25000000U
)

target_compile_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -mthumb
    -fdata-sections
    -ffunction-sections
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    -Wno-missing-field-initializers
    $<$<CONFIG:Debug>:-O0 -g3 -ggdb>
    $<$<CONFIG:Release>:-O2 -g0>
)

target_link_options(${PROJECT_NAME} PRIVATE
    -T${PROJECT_ROOT}/stm32f407xg_flash.ld
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -mthumb
    -Wl,-Map=${PROJECT_NAME}.map
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    LINK_DEPENDS ${PROJECT_ROOT}/stm32f407xg_flash.ld
)
