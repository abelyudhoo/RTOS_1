# RTOS STM32 Task2
# Embedded RTOS Project

This project leverages a Real-Time Operating System (RTOS) to manage and synchronize multiple tasks on an embedded platform. The primary functions include reading and processing analog sensor data, controlling an LED based on button input, and communicating status information over UART.

## Demontration Video

https://github.com/user-attachments/assets/29c1acab-92fb-40b4-b253-c92adde6389f

## Overview

This project is designed to run on an RTOS platform, with four main tasks:

    StartDefaultTask: Reads ADC values from a sensor and stores them in a global variable.
    Startledtask: Uses the ADC values to calculate a voltage, then sends the voltage data via UART.
    Startbutontask: Monitors a button and toggles an LED status based on button presses.
    StartTask04: Controls the physical LED and sends the LED state via UART.

The tasks communicate with each other using shared global variables, ensuring coordinated and responsive operation.
System Components

    ADC (Analog-to-Digital Converter): Reads sensor data which is then processed.
    UART (Universal Asynchronous Receiver-Transmitter): Sends data to an external system for monitoring.
    GPIO (General Purpose Input/Output): Used for reading button input and controlling LED output.
    RTOS: Manages task scheduling and execution.

## Tasks
### StartDefaultTask

Purpose: This task reads an analog value from an ADC channel and updates a shared global variable adc_value with the most recent reading.

    Operation Details:
        Starts ADC conversion and waits for it to complete.
        On successful conversion, retrieves the ADC result and stores it in adc_value.
        Loops with a 500 ms delay to continuously provide updated sensor data.

    Dependencies:
        Provides data used by Startledtask for real-time voltage calculation.

    RTOS Priority: Runs at a normal priority, ensuring it doesn't interrupt more time-sensitive tasks.

### Startledtask

Purpose: This task reads the adc_value set by StartDefaultTask, calculates the corresponding voltage, and sends this information via UART.

    Operation Details:
        Converts adc_value to a voltage using a scaling factor (e.g., 3.3V reference divided by the ADC resolution).
        Formats the voltage and ADC value into a string message.
        Sends the message over UART.
        Repeats every 1000 ms, providing continuous data updates.

    Dependencies:
        Requires adc_value from StartDefaultTask to ensure the voltage is current and accurate.

    RTOS Priority: Runs at a normal priority, spaced apart from sensor reading to avoid potential conflicts.

### Startbutontask

Purpose: This task monitors a button connected to a GPIO pin, debounces the input, and toggles an led_status variable based on button presses.

    Operation Details:
        Reads the GPIO state associated with the button and uses a simple debounce logic with a delay of 200 ms.
        Detects state transitions from high to low (indicating a button press).
        Toggles led_status, which is used by StartTask04 to control the LED.
        Runs every 500 ms to ensure quick responsiveness without excessive polling.

    Dependencies:
        Sets led_status which is read by StartTask04 to manage the LED state.

    RTOS Priority: Also runs at a normal priority, as it is a responsive but not critical task in the system.

### StartTask04

Purpose: This task reads the led_status variable and sets the actual LED GPIO accordingly. Additionally, it sends the LED state over UART.

    Operation Details:
        Checks the value of led_status to determine the LED state.
        Controls the LED using GPIO based on led_status.
        Sends a UART message indicating whether the LED is on or off.
        Runs every 500 ms, frequently checking and updating the LED state.

    Dependencies:
        Depends on led_status which is set by Startbutontask based on user interaction.

    RTOS Priority: Runs at a normal priority, coordinated with other tasks to avoid resource contention.

## Inter-Task Communication

The tasks interact through shared global variables:

    adc_value: Updated by StartDefaultTask, accessed by Startledtask.
    led_status: Updated by Startbutontask, accessed by StartTask04.

This shared variable setup allows tasks to be loosely coupled, simplifying communication without needing complex inter-task messaging mechanisms like queues or semaphores.
## Summary

This project implements a multi-tasking system where sensor data is continuously collected, processed, and transmitted while an LED is controlled in response to user input. The RTOS provides task scheduling, ensuring efficient use of CPU time, while shared global variables facilitate straightforward data sharing between tasks. The modular task-based design allows for easy expansion or modification to accommodate additional features or peripherals.
