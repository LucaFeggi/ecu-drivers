cmake_minimum_required(VERSION 3.25)
project(ecu-drivers-installed-consumer LANGUAGES CXX)

find_package(ecu-drivers 0.1 REQUIRED CONFIG)

add_executable(installed_consumer consumer.cpp)
target_link_libraries(installed_consumer PRIVATE ${ECU_CONSUMER_TARGET})
target_compile_features(installed_consumer PRIVATE cxx_std_20)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(installed_consumer PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror
        -fno-exceptions -fno-rtti)
endif()

if(ECU_CONSUMER_TARGET STREQUAL "hal::linux")
    target_compile_definitions(installed_consumer PRIVATE ECU_CONSUMER_LINUX=1)
elseif(ECU_CONSUMER_TARGET STREQUAL "hal::stm32f446vet6")
    target_compile_definitions(installed_consumer PRIVATE ECU_CONSUMER_STM32F4=1)
elseif(ECU_CONSUMER_TARGET STREQUAL "hal::stm32g484ce")
    target_compile_definitions(installed_consumer PRIVATE ECU_CONSUMER_STM32G4=1)
elseif(ECU_CONSUMER_TARGET STREQUAL "hal::stm32h563vit6")
    target_compile_definitions(installed_consumer PRIVATE ECU_CONSUMER_STM32H5=1)
elseif(ECU_CONSUMER_TARGET STREQUAL "hal::stm32h723vgt6")
    target_compile_definitions(installed_consumer PRIVATE ECU_CONSUMER_STM32H7=1)
elseif(ECU_CONSUMER_TARGET STREQUAL "hal::esp32s3-wroom-1-n16r8")
    target_compile_definitions(installed_consumer PRIVATE ECU_CONSUMER_ESP32S3=1)
else()
    message(FATAL_ERROR "Unsupported installed consumer target '${ECU_CONSUMER_TARGET}'")
endif()
