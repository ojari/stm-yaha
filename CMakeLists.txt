cmake_minimum_required(VERSION 3.22)

#
# User is free to modify the file as much as necessary
#

#list(APPEND CMAKE_MODULE_PATH "C:/Users/user1/Downloads/CmakeTestProject/cmake")
list(APPEND CMAKE_MODULE_PATH "{{sr:cmake_path}}")
message("Build CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})
include("cmake/gcc-arm-none-eabi.cmake")
message("Build CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

# Core project settings
project(stmyaha)
enable_language(C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Core MCU flags, CPU type, instruction set and FPU setup
set(cpu_PARAMS 
    # Other parameters
    # -mthumb
    # -mcpu, -mfloat, -mfloat-abi, ...
)

# Sources
set(sources_SRCS)

# Include directories for all compilers
set(include_DIRS)

# Include directories for each compiler
set(include_c_DIRS)
set(include_cxx_DIRS)
set(include_asm_DIRS)

# Symbols definition for all compilers
set(symbols_SYMB)

# Symbols definition for each compiler
set(symbols_c_SYMB)
set(symbols_cxx_SYMB)
set(symbols_asm_SYMB)

# Link directories and names of libraries
set(link_DIRS)
set(link_LIBS)

# Linker script
set(linker_script_SRC)

# Compiler options
set(compiler_OPTS)

# Linker options
set(linker_OPTS)

# Now call generated cmake
# This will add script generated
# information to the project
include("cmake/vscode_generated.cmake")

# Link directories setup
# Must be before executable is added
link_directories(${CMAKE_PROJECT_NAME} ${link_DIRS})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PUBLIC 
    ${sources_SRCS}
    src/terminal.c
    #src/lwip.c
    ../common/FreeRTOS/croutine.c
    ../common/FreeRTOS/event_groups.c
    ../common/FreeRTOS/list.c
    ../common/FreeRTOS/tasks.c
    ../common/FreeRTOS/queue.c
    ../common/FreeRTOS/timers.c
    ../common/FreeRTOS/portable/GCC/ARM_CM7/r0p1/port.c
    ../common/FreeRTOS/portable/MemMang/heap_4.c
    ../common/FreeRTOS/CMSIS_RTOS_V2/cmsis_os2.c
    ../common/cmsis/system_stm32f7xx.c
    #../common/HAL_F7/Src/stm32f7xx_hal_eth.c
    ../common/HAL_F7/Src/stm32f7xx_hal_gpio.c
    ../common/HAL_F7/Src/stm32f7xx_hal_rcc.c
    ../common/HAL_F7/Src/stm32f7xx_hal_rcc_ex.c
    ../common/HAL_F7/Src/stm32f7xx_hal.c
    ../common/HAL_F7/Src/stm32f7xx_hal_cortex.c
    ../common/HAL_F7/Src/stm32f7xx_hal_tim.c
    ../common/HAL_F7/Src/stm32f7xx_hal_uart.c
    ../common/HAL_F7/Src/stm32f7xx_hal_pwr.c
    ../common/HAL_F7/Src/stm32f7xx_hal_pwr_ex.c
    #../common/HAL_F7/Src/stm32f7xx_hal_dma.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${include_DIRS}
    ../common/cmsis
    ../common/FreeRTOS/include
    ../common/FreeRTOS/portable/GCC/ARM_CM7/r0p1
    ../common/FreeRTOS/CMSIS_RTOS_V2
    ../common/HAL_F7/Inc
    #../common/LwIP/src/include
    #../common/LwIP/system
    $<$<COMPILE_LANGUAGE:C>: ${include_c_DIRS}>
    $<$<COMPILE_LANGUAGE:CXX>: ${include_cxx_DIRS}>
    $<$<COMPILE_LANGUAGE:ASM>: ${include_asm_DIRS}>
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ${symbols_SYMB}
    $<$<COMPILE_LANGUAGE:C>: ${symbols_c_SYMB}>
    $<$<COMPILE_LANGUAGE:CXX>: ${symbols_cxx_SYMB}>
    $<$<COMPILE_LANGUAGE:ASM>: ${symbols_asm_SYMB}>

    STM32F767xx
    # Configuration specific
    $<$<CONFIG:Debug>:DEBUG>
    $<$<CONFIG:Release>: >
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME} ${link_LIBS})

# Compiler options
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    ${cpu_PARAMS}
    ${compiler_OPTS}
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    $<$<COMPILE_LANGUAGE:C>: >
    $<$<COMPILE_LANGUAGE:CXX>:

    # -Wno-volatile
    # -Wold-style-cast
    # -Wuseless-cast
    # -Wsuggest-override
    >
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp -MMD -MP>
    $<$<CONFIG:Debug>:-Og -g3 -ggdb>
    $<$<CONFIG:Release>:-Og -g0>
)

# Linker options
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T${linker_script_SRC}
    ${cpu_PARAMS}
    ${linker_OPTS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -u _printf_float # STDIO float formatting support (remove if not used)
    --specs=nosys.specs
    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -lsupc++
    -Wl,--end-group
    -Wl,-z,max-page-size=8 # Allow good software remapping across address space (with proper GCC section making)
    -Wl,--print-memory-usage
)

# Execute post-build to print size, generate hex and bin
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
)
