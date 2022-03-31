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
