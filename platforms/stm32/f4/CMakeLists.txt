add_library(hal-target-stm32f446vet6 INTERFACE)
add_library(hal::stm32f446vet6 ALIAS hal-target-stm32f446vet6)
set_target_properties(hal-target-stm32f446vet6
                      PROPERTIES EXPORT_NAME stm32f446vet6)
target_link_libraries(hal-target-stm32f446vet6 INTERFACE hal::devices)
target_include_directories(hal-target-stm32f446vet6 INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../mcu/stm32f446vet6/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/targets/stm32f4>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/profiles/stm32f446vet6>)
target_compile_definitions(hal-target-stm32f446vet6 INTERFACE STM32F446xx=1)
target_include_directories(hal-target-stm32f446vet6 SYSTEM INTERFACE
    $<BUILD_INTERFACE:${ECU_DRIVERS_THIRD_PARTY}/cmsis-device-f4/Include>
    $<BUILD_INTERFACE:${ECU_DRIVERS_THIRD_PARTY}/cmsis-core/CMSIS/Core/Include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/third_party/cmsis-device-f4>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/third_party/cmsis-core>)

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/targets/stm32f4)
install(DIRECTORY ../../../mcu/stm32f446vet6/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/profiles/stm32f446vet6)
install(DIRECTORY ${ECU_DRIVERS_THIRD_PARTY}/cmsis-device-f4/Include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/third_party/cmsis-device-f4)
install(DIRECTORY ${ECU_DRIVERS_THIRD_PARTY}/cmsis-core/CMSIS/Core/Include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/third_party/cmsis-core)

if(ECU_DRIVERS_BUILD_TESTS AND BUILD_TESTING)
    add_executable(hal_stm32f4_device_profile_tests tests/device_profile.cpp)
    target_link_libraries(hal_stm32f4_device_profile_tests
                          PRIVATE hal::stm32f446vet6)
    ecu_drivers_enable_strict_warnings(hal_stm32f4_device_profile_tests)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(hal_stm32f4_device_profile_tests PRIVATE
                               -Wno-error=conversion -Wno-error=sign-conversion)
    endif()
    add_test(NAME hal_stm32f4_device_profile
             COMMAND hal_stm32f4_device_profile_tests)
    set_tests_properties(hal_stm32f4_device_profile
                         PROPERTIES LABELS "compile_validated;register_model_validated")
endif()
