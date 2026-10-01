# ============================================================================
#  XXU-RoboMaster-Sentry   新乡学院 RoboMaster 哨兵底盘框架
# ============================================================================
#  注意：CubeMX 每次 GENERATE CODE 都会用模板覆盖本文件。
#        重新生成后请执行： powershell -File tools/cubemx_postgen.ps1
#        基准副本： cmake/CMakeLists.app.txt
# ============================================================================

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.20)

# 交叉编译工具链
set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR           arm-none-eabi-ar)
set(CMAKE_OBJCOPY      arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP      arm-none-eabi-objdump)
set(SIZE               arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(XXU-RoboMaster-Sentry C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

# 不用硬件浮点，省掉开 FPU 的初始化，避免 NoCP 异常
add_compile_options(-mcpu=cortex-m4 -mthumb)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-Ofast)
elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    add_compile_options(-Ofast -g)
elseif (CMAKE_BUILD_TYPE STREQUAL "MinSizeRel")
    add_compile_options(-Os)
else ()
    add_compile_options(-Og -g)
endif ()

include_directories(
        Core/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Drivers/CMSIS/Include
        Drivers/CMSIS/DSP/Include
        Drivers/CMSIS/DSP/PrivateInclude
)

add_definitions(-DDEBUG -DUSE_HAL_DRIVER -DSTM32F427xx -DARM_MATH_CM4 -D__FPU_PRESENT=1U)

file(GLOB_RECURSE SOURCES
        "User_Drives/*.*"
        "User_Algorithm/*.*"
        "User_Architect/*.*"
        "User_Application/*.*"
        "User_Lib/*.*"
        "SEGGER_RTT/*.*"
        "Core/*.*"
        "Drivers/*.*")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F427IIHX_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

# 数学库：姿态解算用 sqrtf / atan2f
target_link_libraries(${PROJECT_NAME}.elf m)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Generating ${HEX_FILE} and ${BIN_FILE}")
