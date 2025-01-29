################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT.c \
../ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT_printf.c \
../ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_SYSVIEW.c 

S_UPPER_SRCS += \
../ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT_ASM_ARMv7M.S 

OBJS += \
./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT.o \
./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT_ASM_ARMv7M.o \
./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT_printf.o \
./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_SYSVIEW.o 

S_UPPER_DEPS += \
./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT_ASM_ARMv7M.d 

C_DEPS += \
./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT.d \
./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT_printf.d \
./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_SYSVIEW.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/SystemView_Src_V354/SEGGER/%.o ThirdParty/SystemView_Src_V354/SEGGER/%.su ThirdParty/SystemView_Src_V354/SEGGER/%.cyclo: ../ThirdParty/SystemView_Src_V354/SEGGER/%.c ThirdParty/SystemView_Src_V354/SEGGER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"C:/Users/julio/Downloads/STM32F407_FreeRTOS/App_014_Example_5.2_Block_When_Send_Queue_and_Send_Struct_Queue/ThirdParty/FreeRTOS/Source/include" -I"C:/Users/julio/Downloads/STM32F407_FreeRTOS/App_014_Example_5.2_Block_When_Send_Queue_and_Send_Struct_Queue/ThirdParty/SystemView_Src_V354/Sample" -I"C:/Users/julio/Downloads/STM32F407_FreeRTOS/App_014_Example_5.2_Block_When_Send_Queue_and_Send_Struct_Queue/ThirdParty/SystemView_Src_V354/SEGGER" -I"C:/Users/julio/Downloads/STM32F407_FreeRTOS/App_014_Example_5.2_Block_When_Send_Queue_and_Send_Struct_Queue/ThirdParty/SystemView_Src_V354/Config" -I"C:/Users/julio/Downloads/STM32F407_FreeRTOS/App_014_Example_5.2_Block_When_Send_Queue_and_Send_Struct_Queue/ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/julio/Downloads/STM32F407_FreeRTOS/App_014_Example_5.2_Block_When_Send_Queue_and_Send_Struct_Queue/ThirdParty/FreeRTOS" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ThirdParty/SystemView_Src_V354/SEGGER/%.o: ../ThirdParty/SystemView_Src_V354/SEGGER/%.S ThirdParty/SystemView_Src_V354/SEGGER/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/julio/Downloads/STM32F407_FreeRTOS/App_014_Example_5.2_Block_When_Send_Queue_and_Send_Struct_Queue/ThirdParty/SystemView_Src_V354/Config" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-ThirdParty-2f-SystemView_Src_V354-2f-SEGGER

clean-ThirdParty-2f-SystemView_Src_V354-2f-SEGGER:
	-$(RM) ./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT.cyclo ./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT.d ./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT.o ./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT.su ./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT_ASM_ARMv7M.d ./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT_ASM_ARMv7M.o ./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT_printf.cyclo ./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT_printf.d ./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT_printf.o ./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_RTT_printf.su ./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_SYSVIEW.cyclo ./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_SYSVIEW.d ./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_SYSVIEW.o ./ThirdParty/SystemView_Src_V354/SEGGER/SEGGER_SYSVIEW.su

.PHONY: clean-ThirdParty-2f-SystemView_Src_V354-2f-SEGGER

