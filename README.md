# Instru-Martin-Nick

This is the STM32 code built using the STM32Cube IDE for the STM32F446 Nucleo-64. It performs to function of an LCR meter with the use of the fourier analysis of a signal provided by a pre-built ABB based circuit. Our code successfully analysised the DUT in terms of resistance and capacitance.

## Structure

```bash

.
├── Core
│   ├── Inc                       # Include folder with ARM Matrix Math Library
│   ├── Src                       # Source File
│   └── Startup
├── Debug
│   ├── Core
│   ├── Drivers
├── Drivers                       # CMSIS Library
├── STM32F446RETX_FLASH.ld
├── STM32F446RETX_RAM.ld
├── Testing_ Debug.launch
├── Testing_.ioc                  # IOC configuration file
├── libs                          # Math library
└── past_versions                 # Past versions
```

## Requirements

- [STM32F446 Nucleo-64](https://www.st.com/en/microcontrollers-microprocessors/stm32f446.html) 
- LCR Pre-Build PCB
- +/- 10V
- 5V
- 3.3V
- Gnd
- ARM Library
  - [CMSIS DSP Library](https://www.keil.com/pack/doc/cmsis/DSP/html/index.html)
  - [CMSIS ATM Matrix Accelerated Library](https://www.keil.com/pack/doc/cmsis/DSP/html/group__groupMatrix.html)

## Setup

- Download and install [STM32Cube IDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- Clone all contents to known location
- Create new STM32 project with existing Testing_.IOC file based upon the STM32F446
- Copy mainFinal1.c into \Src
- Copy arm_math.h and main.h into \Inc
- Copy \libs folder into project and link it via the MCU G++ Linker
- Connect STM32 to PC using usb port
- Upload onto STM32

## Picture

![FinalPic]()
