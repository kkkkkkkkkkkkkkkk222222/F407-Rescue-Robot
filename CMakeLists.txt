cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME WWW)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(F405_PROFILE "" CACHE STRING "F405 firmware: MotionDebug or NormalRun")
if(F405_PROFILE STREQUAL "MotionDebug")
    set(APP_NORMALDEBUG_MOTION_TEST ON)
elseif(F405_PROFILE STREQUAL "NormalRun")
    set(APP_NORMALDEBUG_MOTION_TEST OFF)
else()
    message(FATAL_ERROR "Configure with cmake --preset MotionDebug or cmake --preset NormalRun")
endif()
if(DEFINED APP_BOARD AND NOT APP_BOARD STREQUAL "F405_ROBO")
    message(FATAL_ERROR "Legacy board cache. Select a preset with a fresh F405 build directory.")
endif()
set(APP_BOARD F405_ROBO)
set(APP_ACTIVE_MODE_OVERRIDE APP_MODE_NORMAL_DEBUG)
set(APP_NORMALDEBUG_UPPER_BAUD 230400)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${F405_PROFILE})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${F405_PROFILE} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${F405_PROFILE} PRIVATE
    Main/Src/CenteringTask.c
    Main/Src/Debug.c
    Main/Src/DebugClock.c
    Main/Src/DebugTelemetry.c
    Main/Src/MotionDiag.c
    Main/Src/Gamepad.c
    Main/Src/DebugConsole.c
    Main/Src/Robot.c
    Main/Src/imu.c
    Main/Src/imu_uart.c
    Main/Src/Location.c
    Main/Src/mechanism.c
    Main/Src/motor.c
    Main/Src/pid.c
    Main/Src/Rescue.c
    Main/Src/route_demo.c
    Main/Src/Task.c
    Main/Src/TaskDiag.c
    Main/Src/Uart.c
    Main/Src/vision.c
    Main/Src/WheelCheck.c
)
target_sources(${F405_PROFILE} PRIVATE
    Boards/F405/Src/encoder_f405.c
    Boards/F405/Src/diagnostics_f405.c
    Boards/F405/Src/motor_f405.c
    Boards/F405/Src/servo_f405.c
    Boards/F405/Src/lcd_none.c
)

# Add include paths
target_include_directories(${F405_PROFILE} PRIVATE
    Main/Inc
)

target_compile_definitions(${F405_PROFILE} PRIVATE APP_ACTIVE_MODE=${APP_ACTIVE_MODE_OVERRIDE})
target_compile_definitions(${F405_PROFILE} PRIVATE
    APP_NORMALDEBUG_MOTION_TEST=$<BOOL:${APP_NORMALDEBUG_MOTION_TEST}>)
target_compile_definitions(${F405_PROFILE} PRIVATE APP_NORMALDEBUG_UPPER_BAUD=${APP_NORMALDEBUG_UPPER_BAUD})
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/generated")
add_custom_target(debug_build_identity
    COMMAND ${CMAKE_COMMAND} "-DSOURCE_DIR=${CMAKE_SOURCE_DIR}"
        "-DOUTPUT_FILE=${CMAKE_BINARY_DIR}/generated/DebugBuildIdentity.h"
        "-DIDENTITY_CONFIG=${CMAKE_BUILD_TYPE}|${APP_BOARD}|${APP_ACTIVE_MODE_OVERRIDE}|${APP_NORMALDEBUG_MOTION_TEST}|${APP_NORMALDEBUG_UPPER_BAUD}|${CMAKE_C_COMPILER}|${CMAKE_C_FLAGS}|${CMAKE_C_FLAGS_DEBUG}"
        -P "${CMAKE_SOURCE_DIR}/cmake/DebugBuildIdentity.cmake"
    BYPRODUCTS "${CMAKE_BINARY_DIR}/generated/DebugBuildIdentity.h")
add_dependencies(${F405_PROFILE} debug_build_identity)
target_include_directories(${F405_PROFILE} PRIVATE "${CMAKE_BINARY_DIR}/generated")

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# A successful firmware link is the user's explicit backup point. The script
# creates a dated local commit, pushes it, and verifies that the upstream branch
# matches HEAD. Any backup failure stops the build so a launch task cannot flash
# firmware that has not reached GitHub.
find_program(POWERSHELL_EXECUTABLE NAMES pwsh powershell)

# Add linked libraries
target_link_libraries(${F405_PROFILE}
    stm32cubemx

    # Add user defined libraries
)

option(APP_AUTO_BACKUP "Backup after firmware link" ON)
add_custom_command(TARGET ${F405_PROFILE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${F405_PROFILE}> ${F405_PROFILE}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${F405_PROFILE}> ${F405_PROFILE}.bin
    COMMENT "Generating firmware images"
)

# Custom targets without outputs run on every default build, even when the ELF
# is already up to date. This makes every pre-flash build verify GitHub rather
# than limiting the check to builds that happened to relink the firmware.
if(APP_AUTO_BACKUP)
add_custom_target(github_backup ALL
    COMMAND ${POWERSHELL_EXECUTABLE} -NoProfile -ExecutionPolicy Bypass
            -File ${CMAKE_SOURCE_DIR}/tools/AutoBackup.ps1
            -Repository ${CMAKE_SOURCE_DIR}
    DEPENDS ${F405_PROFILE}
    COMMENT "Committing, pushing, and verifying GitHub backup"
    VERBATIM
)
endif()
