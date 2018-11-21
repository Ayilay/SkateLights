# SkateLights

My skateboard, despite being a plain old-fashioned non-electric skateboard with cruiser wheels, features lots of fancy circuitry, including color-changing LED's, a spedometer, and a 2 digit 7-segment display readout on the front tail.

This repository features the code that makes my skateboard operate.

## Microcontroller and Logic

The heart of my board is an STM32 microcontroller, specifically the STM32F103. It is an entry-level 32-bit microcontroller that features several GPIOs, 72 MHz of processing power, and enough Flash Memory and RAM to run an RTOS with little concern of code memory footprint.

The STM32 runs a [FreeRTOS](http://freertos.org) kernel, which allows the functional software components of the code to be separated into independent tasks.

## Features

My skateboard features the following:
* A magnet and reed sensor on one wheel to measure the speed of the board
* A 2-digit 7-segment display carved into the front tail to display that speed
* Individually adressable RGB LED's across the entire length of the board
