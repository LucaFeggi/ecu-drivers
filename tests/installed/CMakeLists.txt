set(_installed_prefix "${CMAKE_CURRENT_BINARY_DIR}/prefix")
add_test(
    NAME hal_installed_package_install
    COMMAND ${CMAKE_COMMAND} --install ${CMAKE_BINARY_DIR}
            --prefix ${_installed_prefix})
set_tests_properties(hal_installed_package_install PROPERTIES
    FIXTURES_SETUP ecu_drivers_installed_package
    LABELS "installed_package")

foreach(_target IN LISTS ECU_DRIVERS_TARGETS)
    string(REPLACE "-" "_" _test_suffix "${_target}")
    add_test(
        NAME hal_installed_consumer_${_test_suffix}
        COMMAND ${CMAKE_COMMAND}
            -DECU_CONSUMER_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/consumer
            -DECU_CONSUMER_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/${_test_suffix}
            -DECU_CONSUMER_PREFIX=${_installed_prefix}
            -DECU_CONSUMER_TARGET=hal::${_target}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_consumer.cmake)
    set_tests_properties(hal_installed_consumer_${_test_suffix} PROPERTIES
        FIXTURES_REQUIRED ecu_drivers_installed_package
        LABELS "compile_validated;installed_package")
endforeach()
