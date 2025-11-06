cmake_minimum_required(VERSION 3.13.1)


list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/services/Pca_service ${CMAKE_CURRENT_SOURCE_DIR}/modules/lib/zscilib)

# Find Zephyr after setting extra modules
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(stm_arm)

set(BOARD_FLASH_RUNNER stm32cubeprogrammer)

# Include local headers
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/services/Pca_service)

# Add source files based on build config
if(CONFIG_MASTER_FILE)
    target_sources(app PRIVATE applications/stm_robot_arm.c)
endif()

if(CONFIG_SLAVE_FILE)
    target_sources(app PRIVATE applications/stm_robot_arm.c)
endif()
