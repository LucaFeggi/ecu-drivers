file(REMOVE_RECURSE "${ECU_CONSUMER_BINARY_DIR}")
execute_process(
    COMMAND "${CMAKE_COMMAND}"
        -S "${ECU_CONSUMER_SOURCE_DIR}"
        -B "${ECU_CONSUMER_BINARY_DIR}"
        "-DCMAKE_PREFIX_PATH=${ECU_CONSUMER_PREFIX}"
        "-DECU_CONSUMER_TARGET=${ECU_CONSUMER_TARGET}"
    RESULT_VARIABLE _configure_result)
if(NOT _configure_result EQUAL 0)
    message(FATAL_ERROR
        "Installed consumer configure failed for ${ECU_CONSUMER_TARGET}")
endif()

execute_process(
    COMMAND "${CMAKE_COMMAND}" --build "${ECU_CONSUMER_BINARY_DIR}"
    RESULT_VARIABLE _build_result)
if(NOT _build_result EQUAL 0)
    message(FATAL_ERROR
        "Installed consumer build failed for ${ECU_CONSUMER_TARGET}")
endif()
