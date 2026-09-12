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
set(CMAKE_PROJECT_NAME FCH743_V1.0)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# ============================================================================
#  Code ung dung - thu muc App/
#
#  CubeMX chi ghi de: Core/ Drivers/ Middlewares/ FATFS/ USB_DEVICE/
#  cmake/stm32cubemx/ . Moi thu trong App/ deu an toan khi Generate Code.
#
#  CONFIGURE_DEPENDS: CMake tu quet lai khi ban them file .c moi, khong can
#  sua file nay. Chi can tao thu muc con moi trong App/ va them vao
#  target_include_directories ben duoi.
# ============================================================================
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/App/*.c
)

#  Lop USB MSC (o dia di dong) - chep tay tu goi STM32Cube_FW_H7_V1.12.1
#
#  Khai bao O DAY chu khong phai trong cmake/stm32cubemx/CMakeLists.txt, vi
#  file do do CubeMX sinh ra va bi ghi de moi lan Generate Code. De o day thi
#  no song sot.
#
#  CubeMX khong biet toi lop MSC vi trong .ioc USB_DEVICE.CLASS_NAME_FS van la
#  CDC - va PHAI de nguyen nhu vay. Doi sang MSC thi CubeMX se bo CDC ra khoi
#  build (dut duong telemetry USB) va sinh usbd_storage_if.c dung ten voi
#  App/Storage/usb_msc.c. Bo dieu khien nay chon lop luc chay, khong phai luc
#  cau hinh - xem App/Storage/usb_msc.h.
set(USB_MSC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/MSC)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${APP_SOURCES}
    ${USB_MSC_DIR}/Src/usbd_msc.c
    ${USB_MSC_DIR}/Src/usbd_msc_bot.c
    ${USB_MSC_DIR}/Src/usbd_msc_data.c
    ${USB_MSC_DIR}/Src/usbd_msc_scsi.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Config
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Common
    ${CMAKE_CURRENT_SOURCE_DIR}/App/State
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Estimator
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Control
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Telemetry
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Storage
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Mavlink
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Mavlink/mavlink_lib
    ${USB_MSC_DIR}/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
