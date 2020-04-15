################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/commander.c \
../src/main.c \
../src/ourTest.c 

OBJS += \
./src/commander.o \
./src/main.o \
./src/ourTest.o 

C_DEPS += \
./src/commander.d \
./src/main.d \
./src/ourTest.d 


# Each subdirectory must supply rules for building sources it contributes
src/commander.o: ../src/commander.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -c -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/drivers/inc" -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"src/commander.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
src/main.o: ../src/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -c -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/drivers/inc" -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"src/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
src/ourTest.o: ../src/ourTest.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -c -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/drivers/inc" -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"src/ourTest.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

