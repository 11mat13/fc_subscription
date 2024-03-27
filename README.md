# Modified DJI Payload SDK - `fc_subscription` Module

This repository contains the modified `fc_subscription` module from DJI's Payload SDK, tailored for my Engineering Thesis. The modifications are based on the original DJI Payload SDK code, which is available at: https://github.com/dji-sdk/Payload-SDK. I do not claim any rights to the original DJI code.

## Purpose

The purpose of this repository is to provide a way to apply necessary modifications to the DJI Payload SDK without publishing the entire codebase. This is achieved by using a companion bash script, located in a separate repository: https://github.com/11mat13/Engineering_thesis_bash_scripts, which automatically clones the official DJI Payload SDK and copies the modified files from this repository into the correct directory structure.

## Usage

To replicate my project setup, please use the provided bash script in the aforementioned repository. The script handles the cloning and file operations required to integrate the modifications with the official DJI Payload SDK.

## Disclaimer

Please note that this repository is not affiliated with DJI, and the modifications are provided "as is" for academic purposes only. Be sure to comply with DJI's licensing terms when using their SDK.

## Acknowledgements and credits

This project utilizes code from several sources. The parts of my project that use modified code from the following sources are acknowledged here:

- **soft_uart** by **Adriano Marto Reis**: A library for software UART implementation. (https://github.com/adrianomarto/soft_uart)
- **raspberry-pi-bme280** by **Andrei Vainik**: Code for interfacing BME280 sensor with Raspberry Pi. (https://github.com/andreiva/raspberry-pi-bme280/tree/master)
- **MAX31856 thermocouple** by **scootergarrett** - RaspberryPi Forum user: Source library for MAX31856 thermocouple. (https://forums.raspberrypi.com/viewtopic.php?t=261105)
- **LED control Raspberry Pi** by **Jeff Tranter**: Demonstrates controlling GPIO for LEDs on a Raspberry Pi. (https://www.ics.com/blog/how-control-gpio-hardware-c-or-c)

## License:

DJI Publishes their Payload SDK under MIT license.
