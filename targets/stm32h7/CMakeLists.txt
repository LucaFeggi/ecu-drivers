add_library(hal-target-stm32h7 INTERFACE)
add_library(hal::stm32h7 ALIAS hal-target-stm32h7)
set_target_properties(hal-target-stm32h7 PROPERTIES EXPORT_NAME stm32h7)
target_compile_features(hal-target-stm32h7 INTERFACE cxx_std_20)
target_include_directories(hal-target-stm32h7 INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(hal-target-stm32h7 INTERFACE hal::core)

add_library(hal-target-stm32h723vgt6 INTERFACE)
add_library(hal::stm32h723vgt6 ALIAS hal-target-stm32h723vgt6)
set_target_properties(hal-target-stm32h723vgt6
                      PROPERTIES EXPORT_NAME stm32h723vgt6)
target_link_libraries(hal-target-stm32h723vgt6 INTERFACE hal::stm32h7)
target_compile_definitions(hal-target-stm32h723vgt6 INTERFACE STM32H723xx=1)
target_include_directories(hal-target-stm32h723vgt6 SYSTEM INTERFACE
    $<BUILD_INTERFACE:${ECU_DRIVERS_THIRD_PARTY}/cmsis-device-h7/Include>
    $<BUILD_INTERFACE:${ECU_DRIVERS_THIRD_PARTY}/cmsis-core/CMSIS/Core/Include>)

if(ECU_DRIVERS_BUILD_TESTS AND BUILD_TESTING)
    add_executable(hal_stm32h7_device_profile_tests tests/device_profile.cpp)
    target_link_libraries(hal_stm32h7_device_profile_tests
                          PRIVATE hal::stm32h723vgt6)
    ecu_drivers_enable_strict_warnings(hal_stm32h7_device_profile_tests)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(hal_stm32h7_device_profile_tests PRIVATE
                               -Wno-error=conversion -Wno-error=sign-conversion)
    endif()
    add_test(NAME hal_stm32h7_device_profile
             COMMAND hal_stm32h7_device_profile_tests)

    # The exact template-instantiation test touches CMSIS cache helpers that
    # intentionally use 32-bit pointer casts. Build it with the target
    # toolchain, where those casts are valid, rather than weakening a native
    # host build with -fpermissive.
    if(CMAKE_CROSSCOMPILING)
        add_executable(hal_stm32h7_device_profile_exact_tests
                       tests/device_profile_exact.cpp)
        target_link_libraries(hal_stm32h7_device_profile_exact_tests
                              PRIVATE hal::stm32h723vgt6)
        ecu_drivers_enable_strict_warnings(hal_stm32h7_device_profile_exact_tests)
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(hal_stm32h7_device_profile_exact_tests PRIVATE
                                   -Wno-error=conversion -Wno-error=sign-conversion)
        endif()
        add_test(NAME hal_stm32h7_device_profile_exact
                 COMMAND hal_stm32h7_device_profile_exact_tests)
    endif()
endif()
