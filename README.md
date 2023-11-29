# Embedded Systems Project

## Overview

This repository contains the code and documentation for an embedded systems project developed as part of the "Electronics for Embedded Systems" course. The project involves interfacing an STM32 microcontroller with an FPGA via UART communication. The main functionalities include temperature sensing, PWM control of a DC motor, and real-time data transmission.

## Components

- **STM32 Microcontroller Code**: The firmware for the STM32 microcontroller is responsible for temperature sensing through an ADC, PWM signal generation for motor control, and UART communication with the FPGA.

- **FPGA VHDL Code**: The VHDL code is implemented on a Basys3 FPGA, receiving temperature data from the STM32 microcontroller and controlling the motor speed based on PWM signals.

- **UART Communication**: The project utilizes UART for communication between the microcontroller and FPGA, enabling real-time data exchange.

- **ADC, PWM, and Motor Control**: Detailed implementations of analog-to-digital conversion, PWM signal generation, and DC motor control.

## Project Structure

- `STM32_Code/`: STM32 microcontroller firmware.
- `FPGA_VHDL_Code/`: VHDL code for the Basys3 FPGA.
- `Documentation/`: Additional project documentation.
- `Datasheets/`: Datasheets for the components used.

## How to Use

1. **STM32 Microcontroller Setup**: Follow instructions in the `STM32_Code` directory to set up and flash the microcontroller.
2. **FPGA Setup**: Implement the VHDL code in the `FPGA_VHDL_Code` directory onto the Basys3 FPGA.
3. **Connectivity**: Ensure proper wiring and connectivity between the microcontroller and FPGA.
4. **Power On**: Power on the system and monitor the motor response based on temperature variations.

## Contributing

Feel free to contribute to the project by opening issues, suggesting improvements, or submitting pull requests.

## License

This project is licensed under the [MIT License](LICENSE).
