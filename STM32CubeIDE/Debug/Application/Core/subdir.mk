################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/board_resources.c \
C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/dma.c \
C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/main.c \
C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/rtc.c \
C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/stm32_lpm_if.c \
C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/stm32wlxx_hal_msp.c \
C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/stm32wlxx_it.c \
C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/subghz.c \
C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/sys_app.c \
C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/sys_debug.c \
../Application/Core/syscalls.c \
../Application/Core/sysmem.c \
C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/timer_if.c \
C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/usart.c \
C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/usart_if.c 

OBJS += \
./Application/Core/board_resources.o \
./Application/Core/dma.o \
./Application/Core/main.o \
./Application/Core/rtc.o \
./Application/Core/stm32_lpm_if.o \
./Application/Core/stm32wlxx_hal_msp.o \
./Application/Core/stm32wlxx_it.o \
./Application/Core/subghz.o \
./Application/Core/sys_app.o \
./Application/Core/sys_debug.o \
./Application/Core/syscalls.o \
./Application/Core/sysmem.o \
./Application/Core/timer_if.o \
./Application/Core/usart.o \
./Application/Core/usart_if.o 

C_DEPS += \
./Application/Core/board_resources.d \
./Application/Core/dma.d \
./Application/Core/main.d \
./Application/Core/rtc.d \
./Application/Core/stm32_lpm_if.d \
./Application/Core/stm32wlxx_hal_msp.d \
./Application/Core/stm32wlxx_it.d \
./Application/Core/subghz.d \
./Application/Core/sys_app.d \
./Application/Core/sys_debug.d \
./Application/Core/syscalls.d \
./Application/Core/sysmem.d \
./Application/Core/timer_if.d \
./Application/Core/usart.d \
./Application/Core/usart_if.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Core/board_resources.o: C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/board_resources.c Application/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32WL55xx -DCORE_CM4 -DDEBUG -c -I../../Core/Inc -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../../../../../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../../../../../../Utilities/lpm/tiny_lpm -I../../../../../../../Utilities/misc -I../../../../../../../Utilities/sequencer -I../../../../../../../Utilities/timer -I../../../../../../../Utilities/trace/adv_trace -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/Core/board_resources.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/Core/dma.o: C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/dma.c Application/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32WL55xx -DCORE_CM4 -DDEBUG -c -I../../Core/Inc -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../../../../../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../../../../../../Utilities/lpm/tiny_lpm -I../../../../../../../Utilities/misc -I../../../../../../../Utilities/sequencer -I../../../../../../../Utilities/timer -I../../../../../../../Utilities/trace/adv_trace -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/Core/dma.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/Core/main.o: C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/main.c Application/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32WL55xx -DCORE_CM4 -DDEBUG -c -I../../Core/Inc -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../../../../../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../../../../../../Utilities/lpm/tiny_lpm -I../../../../../../../Utilities/misc -I../../../../../../../Utilities/sequencer -I../../../../../../../Utilities/timer -I../../../../../../../Utilities/trace/adv_trace -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/Core/main.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/Core/rtc.o: C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/rtc.c Application/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32WL55xx -DCORE_CM4 -DDEBUG -c -I../../Core/Inc -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../../../../../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../../../../../../Utilities/lpm/tiny_lpm -I../../../../../../../Utilities/misc -I../../../../../../../Utilities/sequencer -I../../../../../../../Utilities/timer -I../../../../../../../Utilities/trace/adv_trace -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/Core/rtc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/Core/stm32_lpm_if.o: C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/stm32_lpm_if.c Application/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32WL55xx -DCORE_CM4 -DDEBUG -c -I../../Core/Inc -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../../../../../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../../../../../../Utilities/lpm/tiny_lpm -I../../../../../../../Utilities/misc -I../../../../../../../Utilities/sequencer -I../../../../../../../Utilities/timer -I../../../../../../../Utilities/trace/adv_trace -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/Core/stm32_lpm_if.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/Core/stm32wlxx_hal_msp.o: C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/stm32wlxx_hal_msp.c Application/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32WL55xx -DCORE_CM4 -DDEBUG -c -I../../Core/Inc -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../../../../../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../../../../../../Utilities/lpm/tiny_lpm -I../../../../../../../Utilities/misc -I../../../../../../../Utilities/sequencer -I../../../../../../../Utilities/timer -I../../../../../../../Utilities/trace/adv_trace -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/Core/stm32wlxx_hal_msp.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/Core/stm32wlxx_it.o: C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/stm32wlxx_it.c Application/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32WL55xx -DCORE_CM4 -DDEBUG -c -I../../Core/Inc -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../../../../../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../../../../../../Utilities/lpm/tiny_lpm -I../../../../../../../Utilities/misc -I../../../../../../../Utilities/sequencer -I../../../../../../../Utilities/timer -I../../../../../../../Utilities/trace/adv_trace -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/Core/stm32wlxx_it.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/Core/subghz.o: C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/subghz.c Application/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32WL55xx -DCORE_CM4 -DDEBUG -c -I../../Core/Inc -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../../../../../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../../../../../../Utilities/lpm/tiny_lpm -I../../../../../../../Utilities/misc -I../../../../../../../Utilities/sequencer -I../../../../../../../Utilities/timer -I../../../../../../../Utilities/trace/adv_trace -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/Core/subghz.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/Core/sys_app.o: C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/sys_app.c Application/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32WL55xx -DCORE_CM4 -DDEBUG -c -I../../Core/Inc -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../../../../../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../../../../../../Utilities/lpm/tiny_lpm -I../../../../../../../Utilities/misc -I../../../../../../../Utilities/sequencer -I../../../../../../../Utilities/timer -I../../../../../../../Utilities/trace/adv_trace -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/Core/sys_app.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/Core/sys_debug.o: C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/sys_debug.c Application/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32WL55xx -DCORE_CM4 -DDEBUG -c -I../../Core/Inc -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../../../../../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../../../../../../Utilities/lpm/tiny_lpm -I../../../../../../../Utilities/misc -I../../../../../../../Utilities/sequencer -I../../../../../../../Utilities/timer -I../../../../../../../Utilities/trace/adv_trace -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/Core/sys_debug.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/Core/%.o: ../Application/Core/%.c Application/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32WL55xx -DCORE_CM4 -DDEBUG -c -I../../Core/Inc -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../../../../../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../../../../../../Utilities/lpm/tiny_lpm -I../../../../../../../Utilities/misc -I../../../../../../../Utilities/sequencer -I../../../../../../../Utilities/timer -I../../../../../../../Utilities/trace/adv_trace -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/Core/timer_if.o: C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/timer_if.c Application/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32WL55xx -DCORE_CM4 -DDEBUG -c -I../../Core/Inc -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../../../../../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../../../../../../Utilities/lpm/tiny_lpm -I../../../../../../../Utilities/misc -I../../../../../../../Utilities/sequencer -I../../../../../../../Utilities/timer -I../../../../../../../Utilities/trace/adv_trace -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/Core/timer_if.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/Core/usart.o: C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/usart.c Application/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32WL55xx -DCORE_CM4 -DDEBUG -c -I../../Core/Inc -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../../../../../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../../../../../../Utilities/lpm/tiny_lpm -I../../../../../../../Utilities/misc -I../../../../../../../Utilities/sequencer -I../../../../../../../Utilities/timer -I../../../../../../../Utilities/trace/adv_trace -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/Core/usart.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Application/Core/usart_if.o: C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/Core/Src/usart_if.c Application/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32WL55xx -DCORE_CM4 -DDEBUG -c -I../../Core/Inc -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../../../../../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../../../../../../Utilities/lpm/tiny_lpm -I../../../../../../../Utilities/misc -I../../../../../../../Utilities/sequencer -I../../../../../../../Utilities/timer -I../../../../../../../Utilities/trace/adv_trace -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/Core/usart_if.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
