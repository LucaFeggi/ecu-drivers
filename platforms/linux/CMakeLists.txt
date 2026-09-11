add_library(hal-target-linux INTERFACE)
add_library(hal::linux ALIAS hal-target-linux)
set_target_properties(hal-target-linux PROPERTIES EXPORT_NAME linux)
target_compile_features(hal-target-linux INTERFACE cxx_std_20)
target_compile_options(hal-target-linux INTERFACE $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Ulinux>)
target_link_libraries(hal-target-linux INTERFACE hal::devices)
target_include_directories(hal-target-linux INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/targets/linux>)

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ecu-drivers/targets/linux)

if(ECU_DRIVERS_BUILD_TESTS AND BUILD_TESTING)
    add_executable(hal_linux_contract_tests tests/contracts.cpp)
    target_link_libraries(hal_linux_contract_tests PRIVATE hal::linux)
    ecu_drivers_enable_strict_warnings(hal_linux_contract_tests)
    add_test(NAME hal_linux_contract COMMAND hal_linux_contract_tests)
    set_tests_properties(hal_linux_contract PROPERTIES
                         LABELS "compile_validated;host_validated")

    add_executable(hal_linux_file_tests tests/file_devices.cpp)
    target_link_libraries(hal_linux_file_tests PRIVATE hal::linux)
    ecu_drivers_enable_strict_warnings(hal_linux_file_tests)
    add_test(NAME hal_linux_file COMMAND hal_linux_file_tests)
    set_tests_properties(hal_linux_file PROPERTIES LABELS "host_validated")

    add_executable(hal_linux_serial_tests tests/serial_pty.cpp)
    target_link_libraries(hal_linux_serial_tests PRIVATE hal::linux)
    if(UNIX AND NOT APPLE)
        target_link_libraries(hal_linux_serial_tests PRIVATE util)
    endif()
    ecu_drivers_enable_strict_warnings(hal_linux_serial_tests)
    add_test(NAME hal_linux_serial_pty COMMAND hal_linux_serial_tests)
    set_tests_properties(hal_linux_serial_pty PROPERTIES LABELS "host_validated")
endif()
