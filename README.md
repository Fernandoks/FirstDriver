# C Embedded Training Course

This Repository is used for teaching purpose, this repository is incomplete and is filled during the Course. 

C Embedded Training Course - First bare metal drivers for STM32

  - APIs and Drivers for STM32F446RE
  - Uses the NUCLEO Board STM32F446RE
  - Available Drivers: GPIO, SPI, I2C, and UART
  
# Requirements
1. arm-none-aebi-gcc complier [ARM GCC](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
   For a complete installation in Linux use the [xPack](https://xpack.github.io/arm-none-eabi-gcc/)
2. Eclipse or Eclise based IDE
  
## Target
Cortex-M4 STM32F446RE, but the drivers can be easily converted to any other STM32 F4 family just correcting the memory mapping.
Compile usign arm-none-eabi-gcc 


## IDE
This project uses a STM32CubeIDE, which is a Eclipse based IDE. So any other Eclipse version is capable of convert the .project file. 

# Instructions

1. Clone the project repository: `git clone https://github.com/Fernandoks/FirstDriver.gitt`
2. Using STM32CubeIDE [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) and import the project
3. Compile and write to your target.


# MIT License

Copyright (c) 2020 Ferando Kaba Surjus

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

