# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

# compile ASM with /usr/sbin/arm-none-eabi-gcc
# compile C with /usr/sbin/arm-none-eabi-gcc
# compile CXX with /usr/sbin/arm-none-eabi-g++
ASM_DEFINES = -DARCH=3 -DARDWIINO_BOARD=\"pico\" -DCFG_TUSB_DEBUG=1 -DCFG_TUSB_MCU=OPT_MCU_RP2040 -DCFG_TUSB_OS=OPT_OS_PICO -DFLASH_TARGET_OFFSET=0x80000 -DF_CPU=133000000 -DPICO_BIT_OPS_PICO=1 -DPICO_BOARD=\"pico\" -DPICO_BUILD=1 -DPICO_BUILD_BOOT_STAGE2_NAME=\"boot2_w25q080\" -DPICO_CMAKE_BUILD_TYPE=\"Debug\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_DIVIDER_HARDWARE=1 -DPICO_DOUBLE_PICO=1 -DPICO_EXTRAS=1 -DPICO_FLOAT_PICO=1 -DPICO_INT64_OPS_PICO=1 -DPICO_MEM_OPS_PICO=1 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_PRINTF_PICO=1 -DPICO_STDIO_UART=1 -DPICO_TARGET_NAME=\"ardwiino-pico-rp2040\" -DPICO_USE_BLOCKED_RAM=0 -DPROGMEM="" -DPSTR="" -DRF_TARGET_OFFSET=0x40000 -DRP2040_USB_DEVICE_MODE=1 -DTINYUSB_DEVICE_LINKED=1 -DUSE_INTERNAL_SERIAL=3 -DVERSION_MAJOR=8 -DVERSION_MINOR=3 -DVERSION_REVISION=3 -Dmemcpy_P=memcpy -Dstrcpy_P=strcpy -Duint_reg_t=uint8_t

ASM_INCLUDES = -I/home/sanjay/Code/Ardwiino/src/shared/output -I/home/sanjay/Code/Ardwiino/src/shared -I/home/sanjay/Code/Ardwiino/src/shared/lib -I/home/sanjay/Code/Ardwiino/lib -I/home/sanjay/Code/Ardwiino/src/pico -I/home/sanjay/Code/Ardwiino/src/pico/lib -I/home/sanjay/Code/Ardwiino/lib/lufa -I/home/sanjay/Code/Ardwiino/src/pico/main -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_stdlib/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_gpio/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_base/include -I/home/sanjay/Code/Ardwiino/buildDebug/generated/pico_base -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/boards/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_platform/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2040/hardware_regs/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_base/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2040/hardware_structs/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_claim/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_sync/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_uart/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_divider/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_time/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_timer/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_sync/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_util/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_runtime/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_clocks/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_resets/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_watchdog/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_xosc/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_pll/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_vreg/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_irq/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_printf/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_bootrom/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_bit_ops/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_divider/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_double/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_int64_ops/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_float/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_malloc/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/boot_stage2/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_binary_info/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_stdio/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_stdio_uart/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_i2c/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_spi/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_adc/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_flash/include -I/home/sanjay/Code/Ardwiino/submodules/pico-extras/src/rp2_common/pico_sleep/include -I/home/sanjay/Code/Ardwiino/submodules/pico-extras/src/rp2_common/hardware_rosc/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_rtc/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_unique_id/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/lib/tinyusb/src -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/lib/tinyusb/src/common -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/lib/tinyusb/hw -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include

ASM_FLAGS = -march=armv6-m -mcpu=cortex-m0plus -mthumb -Og -g -ffunction-sections -fdata-sections

C_DEFINES = -DARCH=3 -DARDWIINO_BOARD=\"pico\" -DCFG_TUSB_DEBUG=1 -DCFG_TUSB_MCU=OPT_MCU_RP2040 -DCFG_TUSB_OS=OPT_OS_PICO -DFLASH_TARGET_OFFSET=0x80000 -DF_CPU=133000000 -DPICO_BIT_OPS_PICO=1 -DPICO_BOARD=\"pico\" -DPICO_BUILD=1 -DPICO_BUILD_BOOT_STAGE2_NAME=\"boot2_w25q080\" -DPICO_CMAKE_BUILD_TYPE=\"Debug\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_DIVIDER_HARDWARE=1 -DPICO_DOUBLE_PICO=1 -DPICO_EXTRAS=1 -DPICO_FLOAT_PICO=1 -DPICO_INT64_OPS_PICO=1 -DPICO_MEM_OPS_PICO=1 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_PRINTF_PICO=1 -DPICO_STDIO_UART=1 -DPICO_TARGET_NAME=\"ardwiino-pico-rp2040\" -DPICO_USE_BLOCKED_RAM=0 -DPROGMEM="" -DPSTR="" -DRF_TARGET_OFFSET=0x40000 -DRP2040_USB_DEVICE_MODE=1 -DTINYUSB_DEVICE_LINKED=1 -DUSE_INTERNAL_SERIAL=3 -DVERSION_MAJOR=8 -DVERSION_MINOR=3 -DVERSION_REVISION=3 -Dmemcpy_P=memcpy -Dstrcpy_P=strcpy -Duint_reg_t=uint8_t

C_INCLUDES = -I/home/sanjay/Code/Ardwiino/src/shared/output -I/home/sanjay/Code/Ardwiino/src/shared -I/home/sanjay/Code/Ardwiino/src/shared/lib -I/home/sanjay/Code/Ardwiino/lib -I/home/sanjay/Code/Ardwiino/src/pico -I/home/sanjay/Code/Ardwiino/src/pico/lib -I/home/sanjay/Code/Ardwiino/lib/lufa -I/home/sanjay/Code/Ardwiino/src/pico/main -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_stdlib/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_gpio/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_base/include -I/home/sanjay/Code/Ardwiino/buildDebug/generated/pico_base -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/boards/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_platform/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2040/hardware_regs/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_base/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2040/hardware_structs/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_claim/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_sync/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_uart/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_divider/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_time/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_timer/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_sync/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_util/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_runtime/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_clocks/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_resets/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_watchdog/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_xosc/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_pll/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_vreg/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_irq/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_printf/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_bootrom/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_bit_ops/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_divider/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_double/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_int64_ops/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_float/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_malloc/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/boot_stage2/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_binary_info/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_stdio/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_stdio_uart/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_i2c/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_spi/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_adc/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_flash/include -I/home/sanjay/Code/Ardwiino/submodules/pico-extras/src/rp2_common/pico_sleep/include -I/home/sanjay/Code/Ardwiino/submodules/pico-extras/src/rp2_common/hardware_rosc/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_rtc/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_unique_id/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/lib/tinyusb/src -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/lib/tinyusb/src/common -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/lib/tinyusb/hw -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include

C_FLAGS = -march=armv6-m -mcpu=cortex-m0plus -mthumb -Og -g -ffunction-sections -fdata-sections -std=gnu11

CXX_DEFINES = -DARCH=3 -DARDWIINO_BOARD=\"pico\" -DCFG_TUSB_DEBUG=1 -DCFG_TUSB_MCU=OPT_MCU_RP2040 -DCFG_TUSB_OS=OPT_OS_PICO -DFLASH_TARGET_OFFSET=0x80000 -DF_CPU=133000000 -DPICO_BIT_OPS_PICO=1 -DPICO_BOARD=\"pico\" -DPICO_BUILD=1 -DPICO_BUILD_BOOT_STAGE2_NAME=\"boot2_w25q080\" -DPICO_CMAKE_BUILD_TYPE=\"Debug\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_DIVIDER_HARDWARE=1 -DPICO_DOUBLE_PICO=1 -DPICO_EXTRAS=1 -DPICO_FLOAT_PICO=1 -DPICO_INT64_OPS_PICO=1 -DPICO_MEM_OPS_PICO=1 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_PRINTF_PICO=1 -DPICO_STDIO_UART=1 -DPICO_TARGET_NAME=\"ardwiino-pico-rp2040\" -DPICO_USE_BLOCKED_RAM=0 -DPROGMEM="" -DPSTR="" -DRF_TARGET_OFFSET=0x40000 -DRP2040_USB_DEVICE_MODE=1 -DTINYUSB_DEVICE_LINKED=1 -DUSE_INTERNAL_SERIAL=3 -DVERSION_MAJOR=8 -DVERSION_MINOR=3 -DVERSION_REVISION=3 -Dmemcpy_P=memcpy -Dstrcpy_P=strcpy -Duint_reg_t=uint8_t

CXX_INCLUDES = -I/home/sanjay/Code/Ardwiino/src/shared/output -I/home/sanjay/Code/Ardwiino/src/shared -I/home/sanjay/Code/Ardwiino/src/shared/lib -I/home/sanjay/Code/Ardwiino/lib -I/home/sanjay/Code/Ardwiino/src/pico -I/home/sanjay/Code/Ardwiino/src/pico/lib -I/home/sanjay/Code/Ardwiino/lib/lufa -I/home/sanjay/Code/Ardwiino/src/pico/main -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_stdlib/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_gpio/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_base/include -I/home/sanjay/Code/Ardwiino/buildDebug/generated/pico_base -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/boards/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_platform/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2040/hardware_regs/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_base/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2040/hardware_structs/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_claim/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_sync/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_uart/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_divider/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_time/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_timer/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_sync/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_util/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_runtime/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_clocks/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_resets/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_watchdog/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_xosc/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_pll/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_vreg/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_irq/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_printf/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_bootrom/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_bit_ops/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_divider/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_double/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_int64_ops/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_float/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_malloc/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/boot_stage2/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/common/pico_binary_info/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_stdio/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_stdio_uart/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_i2c/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_spi/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_adc/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_flash/include -I/home/sanjay/Code/Ardwiino/submodules/pico-extras/src/rp2_common/pico_sleep/include -I/home/sanjay/Code/Ardwiino/submodules/pico-extras/src/rp2_common/hardware_rosc/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/hardware_rtc/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_unique_id/include -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/lib/tinyusb/src -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/lib/tinyusb/src/common -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/lib/tinyusb/hw -I/home/sanjay/Code/Ardwiino/submodules/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include

CXX_FLAGS = -march=armv6-m -mcpu=cortex-m0plus -mthumb -Og -g -ffunction-sections -fdata-sections -fno-exceptions -fno-unwind-tables -fno-rtti -fno-use-cxa-atexit -std=gnu++17

