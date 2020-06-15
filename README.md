# STM32_ADC

## Brief
This repository contains a basic example of the ADC module for STM32L476 NUCLEO board by using registers (without HAL). The example will be explained in detail later.

## Analog-To-Digital Conversion
It consists of transcribing analog signals to digital signals, which facilitates their processing, making the digital signal more immune to noise and other interferences to which analog signals are more sensitive.
STM32 mcus provide at least one Analog-to-Digital converter(ADC), a peripheral able to acquire several input voltages through dedicated I/O ports.

The input voltage is compared against a known fixed voltage or reference voltage (Vref) that could be internal or external, each MCU has a VREF+ pin.

The majority  of STM32 MCUs provide 12-bit ADC. In this case STM32L476RG mcu allows
8, 10 and 12-bit resolution.

## Configuration
Following registers must be set to configure a basic ADC.
### Clock configuration
The ADC peripheral is enabled by modifying the register associated to the AHB2 bus, as well as PORTA (LD2) and PORTC (button B1).

![image](https://user-images.githubusercontent.com/28329247/80532430-c2b9a200-8961-11ea-9048-1db6ebbaab6b.png)

![image](https://user-images.githubusercontent.com/28329247/80536300-c6e8be00-8967-11ea-899d-5e2939e3ad14.png)
### Clock ADC
ADC clock could be enabled by SYSCL, PLLSAI1 or PLLSAI2. In this case, PLLSAI1 was selected, so following following registers are responsible for configuring it.

![clock_ADC](https://user-images.githubusercontent.com/28329247/80518522-2a64f280-894c-11ea-88b0-7678e2a53a8b.png)

#### RCC_CR register
Yellow boxes means that bits must be configured, red boxes means bits are configured by default like we need.
  1. MSIRGSEL = 0 = MSI Range is provided by MSISRANGE[3:0] in RCC_CSR register (Must be set).
  2. PLLSAI1ON = 0 = SAI1 PLL disabled (Must be set AFTER all configurations).

![imagen](https://user-images.githubusercontent.com/28329247/84611045-41bb5900-ae82-11ea-92d0-20b76eedff29.png)
#### RCC_PLLCFGR register
Yellow boxes means that bits must be configured, red boxes means bits are configured by default like we need.
  1. PLLSRC = 00 = No Main PLL, PLLSAI1 nor PLLSAI2 entry clock source selected (MSI clock must be selected setting 01)

![imagen](https://user-images.githubusercontent.com/28329247/84611627-0a4dac00-ae84-11ea-8739-8e47bdaad9e7.png)

#### RCC_PLLSAI1CFGR register
Yellow boxes means that bits must be configured, red boxes means bits are configured by default like we need.
  1. PLLSAIR1EN = 0 = PLLSAI1 PLLADC1CLK output disabled (Must be set).

![imagen](https://user-images.githubusercontent.com/28329247/84614656-6288ac00-ae8c-11ea-8130-4efca6295c1c.png)


#### RCC_CCIPR register
Yellow boxes means that bits must be configured, red boxes means bits are configured by default like we need.
  1. ADCSEL[1:0] = 00 = No clock selected (Must be set as 01 for selection of PLLSAI1"R" as ADCs clk)

![imagen](https://user-images.githubusercontent.com/28329247/84614342-6f58d000-ae8b-11ea-8a5a-78bd2c1d6f74.png)

### GPIO configuration

#### GPIOx_MODER register
Yellow boxes means that bits must be configured, red boxes means bits are configured by default like we need.
  1. MODE0 = 11 = Analog mode (Pontentiometer connected to PC0)
  2. MODE5 = 11 = Analog mode (For LD2 this must be set 01 as General Purpose Output)

![imagen](https://user-images.githubusercontent.com/28329247/84615337-67e6f600-ae8e-11ea-9f44-45ed00e6f561.png)

#### GPIOx_ASCR register
Yellow boxes means that bits must be configured, red boxes means bits are configured by default like we need.
  1. ASC0 = 0 = Disconnect analog switch to the ADC input (Must be set to enable ADC input).
  
![imagen](https://user-images.githubusercontent.com/28329247/84615933-f445e880-ae8f-11ea-8991-70965914e39f.png)

### ADC module configuration

#### ADC_CR register
Yellow boxes means that bits must be configured, red boxes means bits are configured by default like we need.
  1. DEEPPWD = 1 = in Deep-power-down (Must be cleared).
  2. ADVREGEN = 0 = Voltage regulator disabled (Must be set after DEEPPWD).
  3. ADCALDIF = 0 = Single-ended inputs mode.
  4. ADCAL = 0 = Calibration complete (Must be set to start it, then is cleared by HW).
  5. ADSTART = 0 = No ADC regular conversion is ongoing (Must be set to start a conversion).
  6. ADEN = 0 = ADC is disabled (OFF state) (Must be set after all configurations).
  
![imagen](https://user-images.githubusercontent.com/28329247/84617297-19d4f100-ae94-11ea-822a-058f1c82030e.png)
  
#### ADC1_SQR1 register
Yellow boxes means that bits must be configured, red boxes means bits are configured by default like we need.
  1. L = 0000 = 1 conversion.
  2. SQ1 = 0000 = 1st conversion in regular sequence (Must be written with #channel (CN1 = 0001)).

![imagen](https://user-images.githubusercontent.com/28329247/84617759-93211380-ae95-11ea-9ac8-2e07765d59fb.png)

## ADC data acquisition (polling)

### Check EOC flag
#### ADC_ISR register

In polling mode for data acquisition, EOC flag must be checked continuously. 

![imagen](https://user-images.githubusercontent.com/28329247/84618529-180d2c80-ae98-11ea-8157-48d691589174.png)

### Measurement
#### ADC_DR register

When EOC flag is set, measure must be taken.

![imagen](https://user-images.githubusercontent.com/28329247/84619064-a6ce7900-ae99-11ea-9e52-25e88aa100b7.png)

