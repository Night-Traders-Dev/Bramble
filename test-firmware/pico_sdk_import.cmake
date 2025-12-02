# Minimal wrapper to locate the Pico SDK via PICO_SDK_PATH environment variable
if(NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "PICO_SDK_PATH is not set. Please set it to your pico-sdk path.")
endif()

string(REGEX REPLACE "^~" "$ENV{HOME}" _PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
set(PICO_SDK_PATH ${_PICO_SDK_PATH})
if(NOT EXISTS "${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    message(FATAL_ERROR "Pico SDK import not found at ${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
endif()
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
