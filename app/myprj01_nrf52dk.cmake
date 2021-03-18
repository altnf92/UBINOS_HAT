set_cache(UBINOS__BSP__LINK_MEMMAP_RAM_ORIGIN 0x20003400 STRING)
set_cache(UBINOS__BSP__LINK_MEMMAP_RAM_LENGTH 0x0000CC00 STRING)

set_cache(NRF5SDK__SWI_DISABLE0 TRUE BOOL)

include(${PROJECT_UBINOS_DIR}/config/ubinos_nrf52dk_softdevice.cmake)

include(${PROJECT_LIBRARY_DIR}/nrf5sdk/config/nrf5sdk.cmake)

####

set(INCLUDE__APP TRUE)
set(APP__NAME "myprj01")

get_filename_component(_tmp_source_dir "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)

string(TOLOWER ${UBINOS__BSP__NRF52_SOFTDEVICE_NAME} _temp_softdevice_name)
string(TOLOWER ${NRF5SDK__BOARD_NAME} _temp_board_name)

include_directories(${_tmp_source_dir}/${_temp_board_name}/${_temp_softdevice_name}/config)
include_directories(${_tmp_source_dir})

file(GLOB_RECURSE _tmp_sources
    "${_tmp_source_dir}/*.c"
    "${_tmp_source_dir}/*.cpp"
    "${_tmp_source_dir}/*.S"
    "${_tmp_source_dir}/*.s")

set(PROJECT_APP_SOURCES ${PROJECT_APP_SOURCES} ${_tmp_sources})

