cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(robot)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/services/Mpu_9250_service)

set(BOARD_FLASH_RUNNER stm32cubeprogrammer)

# Master build
if(CONFIG_MASTER_FILE)

    target_sources(app PRIVATE applications/stm_robot_arm.c)

    target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/services/Mpu_9250_service)
endif()

# Slave build
if(CONFIG_SLAVE_FILE)
    target_sources(app PRIVATE applications/aero_imu.c)
endif()
