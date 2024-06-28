# STM32 Smart Energy Switcher
## Description
This project is being developed for the course "Topics in Electronic Engineering 4" as part of the curriculum for the Electronic Engineering program at the Federal Rural University of Pernambuco (UFRPE), Academic Unit of Cabo de Santo Agostinho (UACSA).

The project consists of analyzing different energy sources to protect the connected load and aims to reduce the cost of energy utilization. The project should be capable of analyzing three different energy sources and define which will be used:

* AC energy source (220V AC).
* Energy stored in stationary batteries.
* Energy from solar panels.

For this project, an STM32F302 microcontroller was used, a mixed-signal MCU with an Arm® Cortex®-M4 core (with FPU and DSP instructions) running at 72 MHz (used 48 MHz, I will explain more ahead why I choose this frequency). The choice of this MCU was based on the peripherals it offers, especially the analog peripherals.

## Development

Firstly, for the development, I chose to start by designing the entire circuit for analyzing the 220V AC grid. I believe this energy source will require the most hardware to condition the signal so that the MCU's ADC can read it accurately. The path followed by the electrical grid signal is illustrated below:

![diagram](https://github.com/otaviocmaciel/STM32SmartEnergySwitcher/assets/93693421/00c0f741-25c3-4d56-82b5-7dd8ba0937c4)

I used some diodes in the tests I conducted; however, I considered using Schottky diodes due to their junction, which results in a lower forward voltage drop. This allows me not to worry about the dead time between the voltage levels of +0.7V and -0.7V caused by the PN junction of silicon diodes, as in the case of the 1N60P, we have a Metal-N junction. The stage following to retifier has maximum current arround to 300uA, because the Peak Voltage in the output of retifier is arround to 12*sqrt(2) and the voltage divisor have 57kΩ.

1N60P:

![chrome_b9aTRNprm0](https://github.com/otaviocmaciel/STM32SmartEnergySwitcher/assets/93693421/6bddd397-fda7-41da-96fb-95e096b144c5)

## Development Roadmap

1. DMA for acquiring data from the ADC
  * Calculations to analyze if the electrical grid is in good health
  * Development of the Analysis for Other Energy Sources (DC)

2. Analysis of voltage levels from both the solar panel and the batteries
  * The analysis includes monitoring the discharge of the stationary battery to limit the depth of discharge
  * Selection of the Energy Source Based on a User-Configured Usage Profile

3. Initially, the profile will be fixed; however, it is within the scope to implement a dynamic profile definition
  * Profile configuration through the native USB port of the STM32 with a PC
