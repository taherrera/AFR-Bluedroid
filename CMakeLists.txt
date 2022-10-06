cmake_minimum_required(VERSION 3.13)

project(instacrop_app)


# Tell IDF build to link against this target.
#set(IDF_PROJECT_EXECUTABLE instacrop_app)
set(IDF_EXECUTABLE_SRCS "/home/tom/Documents/SLT/tecnocal/instacrop/fw/instacrop_test/main.c")
set(IDF_PROJECT_EXECUTABLE instacrop_app)


# Add some extra components. IDF_EXTRA_COMPONENT_DIRS is an variable used by ESP-IDF
# to collect extra components.
#get_filename_component(
#    EXTRA_COMPONENT_DIRS
#    "amazon-freertos/vendors/espressif/esp-idf/components/fatfs" ABSOLUTE
#)
#list(APPEND IDF_EXTRA_COMPONENT_DIRS ${EXTRA_COMPONENT_DIRS})

#get_filename_component(
#    EXTRA_COMPONENT_DIRS
#    "amazon-freertos/vendors/espressif/esp-idf/components/fatfs/vfs" ABSOLUTE
#)
#list(APPEND IDF_EXTRA_COMPONENT_DIRS ${EXTRA_COMPONENT_DIRS})
#list(APPEND IDF_COMPONENT_REQUIRES_COMMON fatfs)
# As of now there's no offical way to redefine config files outside of FreeRTOS source tree.
# This is a temporary approach to inject an include path so that this takes precedence over the
# config file directory inside FreeRTOS.
include_directories(BEFORE freertos-configs)

# Add freertos as an subdirectory. AFR_BOARD tells which board to target.
set(AFR_BOARD espressif.esp32_devkitc CACHE INTERNAL "")

add_subdirectory(amazon-freertos)

# Add liventus sources
add_subdirectory(/home/tom/Documents/SLT/tecnocal/instacrop/fw/instacrop_test/src)

#add_subdirectory(amazon-freertos/vendors/espressif/esp-idf/components/fatfs)

# Link against the mqtt demo so that we can use it. Dependencies of this demo are transitively
# linked.
target_link_libraries(
    ${PROJECT_NAME} PRIVATE
    AFR::freertos
    AFR::demo_core_mqtt
    AFR::common_io
)

