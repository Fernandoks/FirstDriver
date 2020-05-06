################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/CircularBuffer.c \
../drivers/src/CircularUART.c \
../drivers/src/stm32f446xx_delay.c \
../drivers/src/stm32f446xx_gpio_driver.c \
../drivers/src/stm32f446xx_gpio_driver_course.c \
../drivers/src/stm32f446xx_rcc.c \
../drivers/src/stm32f446xx_spi_driver.c \
../drivers/src/stm32f446xx_usart.c 

OBJS += \
./drivers/src/CircularBuffer.o \
./drivers/src/CircularUART.o \
./drivers/src/stm32f446xx_delay.o \
./drivers/src/stm32f446xx_gpio_driver.o \
./drivers/src/stm32f446xx_gpio_driver_course.o \
./drivers/src/stm32f446xx_rcc.o \
./drivers/src/stm32f446xx_spi_driver.o \
./drivers/src/stm32f446xx_usart.o 

C_DEPS += \
./drivers/src/CircularBuffer.d \
./drivers/src/CircularUART.d \
./drivers/src/stm32f446xx_delay.d \
./drivers/src/stm32f446xx_gpio_driver.d \
./drivers/src/stm32f446xx_gpio_driver_course.d \
./drivers/src/stm32f446xx_rcc.d \
./drivers/src/stm32f446xx_spi_driver.d \
./drivers/src/stm32f446xx_usart.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/CircularBuffer.o: ../drivers/src/CircularBuffer.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -c -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/drivers/inc" -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/inc" -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/CircularBuffer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/CircularUART.o: ../drivers/src/CircularUART.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -c -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/drivers/inc" -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/inc" -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/CircularUART.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/stm32f446xx_delay.o: ../drivers/src/stm32f446xx_delay.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -c -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/drivers/inc" -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/inc" -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f446xx_delay.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/stm32f446xx_gpio_driver.o: ../drivers/src/stm32f446xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -c -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/drivers/inc" -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/inc" -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f446xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/stm32f446xx_gpio_driver_course.o: ../drivers/src/stm32f446xx_gpio_driver_course.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -c -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/drivers/inc" -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/inc" -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f446xx_gpio_driver_course.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/stm32f446xx_rcc.o: ../drivers/src/stm32f446xx_rcc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -c -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/drivers/inc" -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/inc" -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f446xx_rcc.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/stm32f446xx_spi_driver.o: ../drivers/src/stm32f446xx_spi_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -c -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/drivers/inc" -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/inc" -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f446xx_spi_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/stm32f446xx_usart.o: ../drivers/src/stm32f446xx_usart.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -c -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/drivers/inc" -I"/home/fernandoks/Desktop/CubeWorkbench/FirstDriver-1/inc" -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f446xx_usart.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

