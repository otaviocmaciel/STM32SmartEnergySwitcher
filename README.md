# STM32 Smart Energy Switcher
## Description
This project is being developed for the course "Topics in Electronic Engineering 4" as part of the curriculum for the Electronic Engineering program at the Federal Rural University of Pernambuco (UFRPE), Academic Unit of Cabo de Santo Agostinho (UACSA).

The project consists of analyzing different energy sources to protect the connected load and aims to reduce the cost of energy utilization. The project should be capable of analyzing three different energy sources and define which will be used:

* AC energy source (220V AC).
* Energy stored in stationary batteries.
* Energy from solar panels.

For this project, an STM32F302 microcontroller was used, a mixed-signal MCU with an Arm® Cortex®-M4 core (with FPU and DSP instructions) running at 72 MHz (used 48 MHz, I will explain more ahead why I choose this frequency). The choice of this MCU was based on the peripherals it offers, especially the analog peripherals.

## Development

Firstly, for the development, I chose to start by designing the entire circuit for analyzing the 220V AC grid. I believe this energy source will require the most hardware to condition the signal so that the MCU's ADC can read it accurately. The path followed by the electrical grid signal is illustrated below::

![diagram](https://github.com/otaviocmaciel/STM32SmartEnergySwitcher/assets/93693421/00c0f741-25c3-4d56-82b5-7dd8ba0937c4)

