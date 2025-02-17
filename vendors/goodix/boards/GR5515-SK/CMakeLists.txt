# These Amazon FreeRTOS related global variables are available to use.
# AFR_ROOT_DIR                  Amazon FreeRTOS source root.
# AFR_KERNEL_DIR                FreeRTOS kernel root.
# AFR_MODULES_DIR               Amazon FreeRTOS modules root.
# AFR_MODULES_C_SDK_DIR         C-SDK libraries root.
# AFR_MODULES_FREERTOS_PLUS_DIR FreeRTOS-Plus libraries root.
# AFR_MODULES_ABSTRACTIONS_DIR  Abstractions layers root.
# AFR_DEMOS_DIR                 Amazon FreeRTOS demos root.
# AFR_TESTS_DIR                 Amazon FreeRTOS common tests and framework root.
# AFR_VENDORS_DIR               vendors content root.
# AFR_3RDPARTY_DIR              3rdparty libraries root.

# AFR_VENDOR_NAME           Folder name for vendor.
# AFR_BOARD_NAME            Folder name for this board.

# AFR_TOOLCHAIN             Compiler chosen by the user. Should be one of
#                           the file names under ${AFR_ROOT_DIR}/tools/cmake/toolchains
# AFR_IS_TESTING            1 if testing enabled, otherwise, 0.

# You may also use these 2 functions we defined to glob files when needed. However, we recommend
# to specify your source files explicitly to avoid unexpected behavior unless you're 100% sure.
# CMake reference link: https://cmake.org/cmake/help/latest/command/file.html#filesystem
# afr_glob_files(<out_var> [RECURSE] <DIRECTORY> <directory> [<GLOBS> <glob-expressions>...])
# afr_glob_src(<out_var> [RECURSE] <DIRECTORY> <directory> [<EXTENSIONS> <file-extensions>...])

# If you don't specify GLOBS or EXTENSIONS parameters,
# afr_glob_files: glob all files including hidden files in the specified directory.
# afr_glob_src:   glob all files ending with either .c, .h, .s or .asm

# Use RECURSE if you want to recursively search all subdirectories.

# Example usage,
# afr_glob_src(board_code DIRECTORY "${board_dir}/application_code/${vendor}_code")
# afr_glob_src(driver_code RECURSE DIRECTORY "${driver_path}")
# afr_glob_src(headers DIRECTORY "${some_path}" EXTENSIONS h)

# -------------------------------------------------------------------------------------------------
# Set board config
# -------------------------------------------------------------------------------------------------
afr_set_board_metadata(ID "GR5515-SK")
afr_set_board_metadata(DISPLAY_NAME "GR5515-SK")
afr_set_board_metadata(DESCRIPTION "Goodix BLE GR5515-SK Board for AmazonFreeRTOS")
afr_set_board_metadata(VENDOR_NAME "Goodix")
afr_set_board_metadata(FAMILY_NAME "Goodix BLE GR551x")
afr_set_board_metadata(DATA_RAM_MEMORY "256KB")
afr_set_board_metadata(PROGRAM_MEMORY "1MB")
afr_set_board_metadata(CODE_SIGNER "null")
afr_set_board_metadata(SUPPORTED_IDE "uVision5")
afr_set_board_metadata(RECOMMENDED_IDE "uVision5")
afr_set_board_metadata(IDE_uVision5_NAME "Keil uVision5")
afr_set_board_metadata(IDE_uVision5_COMPILER "ARMCC")
afr_set_board_metadata(IDE_uVision5_PROJECT_LOCATION "null")
afr_set_board_metadata(IS_ACTIVE "TRUE")

# -------------------------------------------------------------------------------------------------
# Disable AFR_MODULE_defender: NOT Support defender Module which depends socket module
# -------------------------------------------------------------------------------------------------
option(AFR_MODULE_defender "Disable defender module" OFF)

# -------------------------------------------------------------------------------------------------
# Global settings
# -------------------------------------------------------------------------------------------------

set(afr_ports_dir "${CMAKE_CURRENT_LIST_DIR}/ports")
set(board_demos_dir "${CMAKE_CURRENT_LIST_DIR}/aws_demos")
set(board_tests_dir "${CMAKE_CURRENT_LIST_DIR}/aws_tests")
set(cmake_project_board_dir "${CMAKE_BINARY_DIR}/vendors/goodix/boards/GR5515-SK")

if(AFR_IS_TESTING)
    set(board_dir "${board_tests_dir}")
else()
    set(board_dir "${board_demos_dir}")
endif()


# -------------------------------------------------------------------------------------------------
# Pre-Compiler settings
# -------------------------------------------------------------------------------------------------

set(compiler_defined_symbols
    # Needed by tinycbor to determine byte order
    __little_endian__=1
)

set(assembler_defined_symbols
    __ASSEMBLER__
)

set(compiler_flags
    --cpu Cortex-M4.fp --c99 -c --li -g -O0 --apcs=interwork --split_sections
)

set(assembler_flags
    --cpu Cortex-M4.fp --li -g --apcs=interwork --pd  "ARMCM4_FP SETA 1"
)

# Linker flags

set(link_flags
    "--cpu=Cortex-M4.fp"
    "--strict"
    "--scatter=${AFR_ROOT_DIR}/vendors/goodix/GR551x_SDK_V1_00/toolchain/gr551x/source/arm/flash_scatter_common.sct"
    "${AFR_ROOT_DIR}/vendors/goodix/GR551x_SDK_V1_00/components/patch/symbol_table/rom_symbol.txt"
    "--diag_suppress=6314,6329"
    "--summary_stderr"
    "--info=summarysizes,sizes,totals,unused,veneers"
    "--map"
    "--xref"
    "--callgraph"
    "--symbols"
)

set(ble_link_dependent_libs
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/sdk/linker/lib/ble_sdk.lib"
)

set(aws_common_includes
    # freertos_kernel
    "${AFR_KERNEL_DIR}"
    "${AFR_KERNEL_DIR}/include"
    "${AFR_KERNEL_DIR}/portable/RVDS/GR5515-SK"

    # tool ref
    "${AFR_ROOT_DIR}/tools/cbmc/windows"

    # libraries\3rdparty
    "${AFR_3RDPARTY_DIR}/jsmn"
    "${AFR_3RDPARTY_DIR}/mbedtls/include"
    "${AFR_3RDPARTY_DIR}/mbedtls/include/mbedtls"
    "${AFR_3RDPARTY_DIR}/pkcs11"
    "${AFR_3RDPARTY_DIR}/tinycbor"
    "${AFR_3RDPARTY_DIR}/unity/src"
    "${AFR_3RDPARTY_DIR}/unity/extras/fixture/src"
    "${AFR_3RDPARTY_DIR}/tinycrypt/lib/include"
    "${AFR_3RDPARTY_DIR}/tinycrypt/lib/include/tinycrypt"
    "${AFR_3RDPARTY_DIR}/tinycrypt/asn1"

    # libraries\abstractions
    "${AFR_MODULES_ABSTRACTIONS_DIR}/ble_hal/include"
    "${AFR_MODULES_ABSTRACTIONS_DIR}/pkcs11"
    "${AFR_MODULES_ABSTRACTIONS_DIR}/pkcs11/include"
    "${AFR_MODULES_ABSTRACTIONS_DIR}/pkcs11/mbedtls"
    "${AFR_MODULES_ABSTRACTIONS_DIR}/pkcs11/test"
    "${AFR_MODULES_ABSTRACTIONS_DIR}/platform/include"
    "${AFR_MODULES_ABSTRACTIONS_DIR}/platform/include/platform"
    "${AFR_MODULES_ABSTRACTIONS_DIR}/platform/include/types"
    "${AFR_MODULES_ABSTRACTIONS_DIR}/platform/freertos"
    "${AFR_MODULES_ABSTRACTIONS_DIR}/platform/freertos/include"
    "${AFR_MODULES_ABSTRACTIONS_DIR}/platform/freertos/include/platform"
    "${AFR_MODULES_ABSTRACTIONS_DIR}/posix/include"
    "${AFR_MODULES_ABSTRACTIONS_DIR}/posix/include/FreeRTOS_POSIX"
    "${AFR_MODULES_ABSTRACTIONS_DIR}/posix/include/FreeRTOS_POSIX/sys"
    "${AFR_MODULES_ABSTRACTIONS_DIR}/secure_sockets/include"
    # libraries\c_sdk
    "${AFR_MODULES_C_SDK_DIR}/aws/defender/include"
    "${AFR_MODULES_C_SDK_DIR}/aws/defender/src/private"
    "${AFR_MODULES_C_SDK_DIR}/aws/shadow/include"
    "${AFR_MODULES_C_SDK_DIR}/aws/shadow/include/types"
    "${AFR_MODULES_C_SDK_DIR}/aws/shadow/src"
    "${AFR_MODULES_C_SDK_DIR}/aws/shadow/src/private"
    "${AFR_MODULES_C_SDK_DIR}/standard/ble/src"
    "${AFR_MODULES_C_SDK_DIR}/standard/ble/include"
    "${AFR_MODULES_C_SDK_DIR}/standard/common/include"
    "${AFR_MODULES_C_SDK_DIR}/standard/common/include/private"
    "${AFR_MODULES_C_SDK_DIR}/standard/common/include/types"
    "${AFR_MODULES_C_SDK_DIR}/standard/mqtt"
    "${AFR_MODULES_C_SDK_DIR}/standard/mqtt/include"
    "${AFR_MODULES_C_SDK_DIR}/standard/mqtt/include/types"
    "${AFR_MODULES_C_SDK_DIR}/standard/mqtt/src"
    "${AFR_MODULES_C_SDK_DIR}/standard/mqtt/src/private"
    "${AFR_MODULES_C_SDK_DIR}/standard/mqtt/test/access"
    "${AFR_MODULES_C_SDK_DIR}/standard/serializer/include"

    # libraries\freertos_plus
    "${AFR_MODULES_FREERTOS_PLUS_DIR}/aws/ota"
    "${AFR_MODULES_FREERTOS_PLUS_DIR}/aws/ota/include"
    "${AFR_MODULES_FREERTOS_PLUS_DIR}/aws/ota/src"
    "${AFR_MODULES_FREERTOS_PLUS_DIR}/aws/ota/test"
    "${AFR_MODULES_FREERTOS_PLUS_DIR}/aws/ota/test/test_files"
    "${AFR_ROOT_DIR}/libraries/freertos_plus/standard/freertos_plus_posix/include"
    "${AFR_MODULES_FREERTOS_PLUS_DIR}/standard/freertos_plus_posix/include"
    "${AFR_MODULES_FREERTOS_PLUS_DIR}/standard/utils"
    "${AFR_MODULES_FREERTOS_PLUS_DIR}/standard/utils/include"
    "${AFR_MODULES_FREERTOS_PLUS_DIR}/standard/crypto/include"
    "${AFR_MODULES_FREERTOS_PLUS_DIR}/standard/tls/include"

    # Aws Ref
    "${AFR_DEMOS_DIR}/network_manager"
    "${AFR_DEMOS_DIR}/dev_mode_key_provisioning"
    "${AFR_DEMOS_DIR}/dev_mode_key_provisioning/include"
)

set(gr_ble_sdk_includes
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/build/config"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/inc"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/boards"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/drivers_ext/gr551x"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/sdk"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/app_assert"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/app_error"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/app_key"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/app_log"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/app_timer"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/at_cmd"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/bsp"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/gui"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/gui/gui_config"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/hal_flash"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/hal_flash/vflash"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/hci_uart"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/pmu_calibration"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/ring_buffer"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/sensorsim"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/utility"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/patch/ind"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/drivers/inc"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/external/segger_rtt"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/toolchain/gr551x/include"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/toolchain/gr551x/source/arm"
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/toolchain/cmsis/include"
    "${AFR_VENDORS_DIR}/goodix/boards/GR5515-SK/ports/ble/user_callback"
    "${AFR_VENDORS_DIR}/goodix/boards/GR5515-SK/ports/ble"
    "${AFR_VENDORS_DIR}/goodix/boards/GR5515-SK/ports/posix"
    "${AFR_VENDORS_DIR}/goodix/boards/GR5515-SK/ports/ota"
)

set(all_share_includes
    ${gr_ble_sdk_includes}
    ${aws_common_includes}
)

set(board_tests_sources
    "${AFR_VENDORS_DIR}/goodix/boards/GR5515-SK/aws_tests/application_code/main.c"
    "${AFR_VENDORS_DIR}/goodix/boards/GR5515-SK/aws_tests/application_code/goodix_code/user_periph_setup.c"
)

set(board_tests_includes
    "${AFR_TESTS_DIR}"
    "${AFR_TESTS_DIR}/include"
    "${AFR_MODULES_FREERTOS_PLUS_DIR}/standard/utils/include"
    "${AFR_MODULES_FREERTOS_PLUS_DIR}/aws/ota/test"
    "${AFR_MODULES_FREERTOS_PLUS_DIR}/aws/ota/test/test_files"
    "${AFR_VENDORS_DIR}/goodix/boards/GR5515-SK/aws_tests"
    "${AFR_VENDORS_DIR}/goodix/boards/GR5515-SK/aws_tests/application_code"
    "${AFR_VENDORS_DIR}/goodix/boards/GR5515-SK/aws_tests/application_code/goodix_code"
    "${AFR_VENDORS_DIR}/goodix/boards/GR5515-SK/aws_tests/config_files"
)

set(board_demos_sources
    "${AFR_VENDORS_DIR}/goodix/boards/GR5515-SK/aws_demos/application_code/main.c"
    "${AFR_VENDORS_DIR}/goodix/boards/GR5515-SK/aws_demos/application_code/goodix_code/user_periph_setup.c"
    "${AFR_DEMOS_DIR}/ble/iot_ble_numericComparison.c"
)

set(board_demos_includes
    "${AFR_DEMOS_DIR}"
    "${AFR_DEMOS_DIR}/include"
    "${AFR_DEMOS_DIR}/network_manager"
    "${AFR_MODULES_FREERTOS_PLUS_DIR}/standard/utils/include"
    "${AFR_VENDORS_DIR}/goodix/boards/GR5515-SK/aws_demos"
    "${AFR_VENDORS_DIR}/goodix/boards/GR5515-SK/aws_demos/application_code"
    "${AFR_VENDORS_DIR}/goodix/boards/GR5515-SK/aws_demos/application_code/goodix_code"
    "${AFR_VENDORS_DIR}/goodix/boards/GR5515-SK/aws_demos/config_files"
)


# -------------------------------------------------------------------------------------------------
# Compiler settings
# -------------------------------------------------------------------------------------------------
afr_mcu_port(compiler)
target_compile_definitions(
    AFR::compiler::mcu_port
    INTERFACE
        $<$<COMPILE_LANGUAGE:C>:${compiler_defined_symbols}>
        -DARMCM4_FP
        -DGR5515_SK
)

target_compile_definitions(
    AFR::compiler::mcu_port
    INTERFACE
        $<$<COMPILE_LANGUAGE:ASM>:${assembler_defined_symbols}>
)

target_compile_options(
    AFR::compiler::mcu_port
    INTERFACE
        $<$<COMPILE_LANGUAGE:C>:${compiler_flags}>
)
target_compile_options(
    AFR::compiler::mcu_port
    INTERFACE
        $<$<COMPILE_LANGUAGE:ASM>:${assembler_flags}>
)

target_link_options(
    AFR::compiler::mcu_port
    INTERFACE ${link_flags}
)

# comment for now
target_link_libraries(
    AFR::compiler::mcu_port
    INTERFACE ${link_dependent_libs}
)

# -------------------------------------------------------------------------------------------------
# Amazon FreeRTOS portable layers
# -------------------------------------------------------------------------------------------------
# Normally the portable layer for kernel should be vendor's driver code.
afr_mcu_port(kernel)
target_sources(
    AFR::kernel::mcu_port
    INTERFACE
        # Goodix FreeRTOS Porting Src Files
        "${AFR_KERNEL_DIR}/portable/RVDS/GR5515-SK/port.c"
        "${AFR_KERNEL_DIR}/portable/RVDS/GR5515-SK/port_pm.c"
        "${AFR_KERNEL_DIR}/portable/MemMang/heap_4.c"

        # Goodix's Libs & Drivers
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/toolchain/gr551x/source/system_gr55xx.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/toolchain/gr551x/source/interrupt_gr55xx.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/toolchain/gr551x/source/platform_gr55xx.c"

        # app
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_adc.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_aes.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_comp.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_dma.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_dual_tim.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_gpiote.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_hmac.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_i2c.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_i2s.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_io.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_pwr_mgmt.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_qspi.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_rng.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_spi.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_systick.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_tim.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/app_drivers/src/app_uart.c"
        # ext drivers
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/drivers_ext/gr551x/gr551x_spi_flash.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/drivers_ext/gr551x/gr551x_temp_api.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/drivers_ext/gr551x/gr551x_tim_delay.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/drivers_ext/gr551x/gr551x_vbat_api.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/drivers_ext/gr551x/gr551x_adc_voltage_api.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/app_timer/app_timer.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/app_log/app_log.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/app_error/app_error.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/app_assert/app_assert.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/pmu_calibration/pmu_calibration.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/ring_buffer/ring_buffer.c"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/components/libraries/utility/utility.c"

        # Segger RTT
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/external/segger_rtt/SEGGER_RTT.c"

        # other libs
        "${AFR_ROOT_DIR}/libraries/c_sdk/standard/common/logging/aws_logging_task_dynamic_buffers.c"
)

target_include_directories(
    AFR::kernel::mcu_port
    INTERFACE
        # Board dir
        ${all_share_includes}
        $<IF:${AFR_IS_TESTING}, ${board_tests_includes}, ${board_demos_includes}>
        "${AFR_3RDPARTY_DIR}/tracealyzer_recorder/Include"
)



# -------------------------------------------------------------------------------------------------
# BLE Hal
# -------------------------------------------------------------------------------------------------
set(BLE_SUPPORTED 1 CACHE INTERNAL "BLE is supported on this platform.")

afr_mcu_port(ble_hal)
target_sources(
    AFR::ble_hal::mcu_port
    INTERFACE
        "${afr_ports_dir}/ble/user_callback/gatts_profile_callback.c"
        "${afr_ports_dir}/ble/user_callback/user_gap_callback.c"
        "${afr_ports_dir}/ble/user_callback/user_gatt_common_callback.c"
        "${afr_ports_dir}/ble/user_callback/user_gattc_callback.c"
        "${afr_ports_dir}/ble/user_callback/user_l2cap_callback.c"
        "${afr_ports_dir}/ble/user_callback/user_sm_callback.c"
        "${afr_ports_dir}/ble/iot_ble_hal_gatt_server.c"
        "${afr_ports_dir}/ble/iot_ble_hal_gap.c"
        "${afr_ports_dir}/ble/iot_ble_hal_common_gap.c"
        "${afr_ports_dir}/ble/gr_utils.c"
        "${afr_ports_dir}/ble/gr_porting.c"
        "${afr_ports_dir}/ble/gr_message.c"
        "${afr_ports_dir}/ble/gr_debug.c"
)

target_link_libraries(
    AFR::ble_hal::mcu_port
    INTERFACE
        ${ble_link_dependent_libs}
)

target_include_directories(
    AFR::ble_hal::mcu_port
    INTERFACE
        ${all_share_includes}
)


# -------------------------------------------------------------------------------------------------
# PKCS11
# -------------------------------------------------------------------------------------------------

afr_mcu_port(pkcs11)
target_sources(
    AFR::pkcs11::mcu_port
    INTERFACE
        "${afr_ports_dir}/pkcs11/iot_pkcs11_pal.c"
        "${AFR_MODULES_ABSTRACTIONS_DIR}/pkcs11/mbedtls/aws_pkcs11_mbedtls.c"
        "${AFR_MODULES_FREERTOS_PLUS_DIR}/standard/crypto/src/aws_crypto.c"
)
target_link_libraries(
    AFR::pkcs11::mcu_port
    INTERFACE
        3rdparty::mbedtls
)

target_include_directories(
    AFR::pkcs11::mcu_port
    INTERFACE
        "${AFR_MODULES_FREERTOS_PLUS_DIR}/standard/crypto/include/"
        "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/drivers/inc"
)

# -------------------------------------------------------------------------------------------------
# POSIX
# -------------------------------------------------------------------------------------------------
afr_mcu_port(posix)

target_sources(
    AFR::posix::mcu_port
    INTERFACE
        "${afr_ports_dir}/posix/FreeRTOS_POSIX_portable.h"
)

target_include_directories(
    AFR::posix::mcu_port
    INTERFACE
        "${afr_ports_dir}/posix"
)

target_link_libraries(
    AFR::posix::mcu_port
    INTERFACE
        AFR::freertos_plus_posix
)

# -------------------------------------------------------------------------------------------------
# OTA
# -------------------------------------------------------------------------------------------------
afr_mcu_port(ota)

target_sources(
    AFR::ota::mcu_port
    INTERFACE
        "${afr_ports_dir}/ota/aws_ota_pal.c"
        "${afr_ports_dir}/ota/gr_ota_pal.c"
)

target_include_directories(
    AFR::ota::mcu_port
    INTERFACE
        "${afr_ports_dir}/ota"
)

target_link_libraries(
    AFR::ota::mcu_port
    INTERFACE
        AFR::crypto
        AFR::pkcs11
)

# -------------------------------------------------------------------------------------------------
# Amazon FreeRTOS demos and tests
# -------------------------------------------------------------------------------------------------
set(CMAKE_EXECUTABLE_SUFFIX ".axf")

if(AFR_IS_TESTING)
    set(exe_target aws_tests)
else()
    set(exe_target aws_demos)
endif()

# set exe_target compile options

set (target_compiler_flags
    "--cpu=Cortex-M4.fp" "--c99" "-c" "--li" "-g" "-O0" "--apcs=interwork"
)

set (target_assembler_flags
    "--cpu=Cortex-M4.fp" "--li" "-g" "--apcs=interwork"
)

add_compile_options(
    "--cpu=Cortex-M4.fp" "--li" "-g" "--apcs=interwork"
)

add_executable(
    ${exe_target}
    "${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/toolchain/gr551x/source/arm/startup_gr55xx.s"
    $<IF:${AFR_IS_TESTING}, ${board_tests_sources} , ${board_demos_sources}>
)

target_link_libraries(
    ${exe_target}
    PRIVATE
        AFR::compiler::mcu_port
        AFR::ble
        AFR::kernel
        AFR::posix
        AFR::ota
)

target_include_directories(
    ${exe_target}
    PUBLIC
        ${all_share_includes}
        $<TARGET_PROPERTY:AFR::kernel,INTERFACE_INCLUDE_DIRECTORIES>
)

find_program(fromelf fromelf.exe PATHS "${AFR_COMPILER_DIR}")
set(output_bin_file "$<TARGET_FILE_DIR:${exe_target}>/${exe_target}.bin")
set(output_axf_file "$<TARGET_FILE_DIR:${exe_target}>/${exe_target}.axf")

add_custom_command(
    TARGET ${exe_target} POST_BUILD
    COMMAND "echo" "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
    COMMAND "echo" "After Build, call .bat to re-map the .hex file:"
    COMMAND call ${AFR_VENDORS_DIR}/goodix/GR551x_SDK_V1_00/build/scripts/amazon_cmake_build.bat ${exe_target} ${AFR_ROOT_DIR} ${cmake_project_board_dir}
    COMMAND "echo" "build out dir: ${PROJECT_BINARY_DIR}/vendors/goodix/boards/GR5515-SK/build/"
)

