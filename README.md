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
1.  fff

2.  ddf

## Clock ADC
The ADC peripheral is enabled by modifying the register associated to the AHB2 bus, as well as PORTA (LD2) and PORTC (button B1).

![image](https://user-images.githubusercontent.com/28329247/80532430-c2b9a200-8961-11ea-9048-1db6ebbaab6b.png)
![image](https://user-images.githubusercontent.com/28329247/80536300-c6e8be00-8967-11ea-899d-5e2939e3ad14.png)


![clock_ADC](https://user-images.githubusercontent.com/28329247/80518522-2a64f280-894c-11ea-88b0-7678e2a53a8b.png)



## Conversion modes

## Channel selection

## ADC resolution and Conversion speed

## ADC (polling)
