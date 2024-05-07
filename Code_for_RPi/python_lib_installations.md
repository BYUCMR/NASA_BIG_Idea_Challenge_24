# Python Library Installations
## Purpose
This document outlines the necessary python libraries for the NASA Big Ideas project. These libraries are required to run the code for the nRF24 network of nodes, as well as the TCP client for communication with MATLAB. This page assumes users are familiar with basic [Linux command line](https://ubuntu.com/tutorials/command-line-for-beginners#1-overview) operations and have a Raspberry Pi set up with SSH or VNC access.

For most of the libaries listed below, there is no consequence for attempting to install a library that already exists.

## Table of Contents
- [PiGPIO](#pigpio)
- [Socket](#socket)
- [NRF24](#nrf24)

## PiGPIO
PiGPIO is a standard python library that enables the user to interface with the General Purpose Input/Output (GPIO) pins on a RPi. It may or may not be included by default on the base Raspian installation. In a terminal, run:
```bash
sudo apt install pigpio
```
This library is one of those used by the NRF24 library and users will generally not need it directly in their scripts.

## Socket
This is another standard python library that may or may not be included by default on the RPi. It enables the RPi to communicate over a server/client interface known as Transmission Control Protocol (TCP). This enables the RPi to communicate with MATLAB over Ethernet or LAN. To install, run:
```bash
sudo apt install socket
```
The specific commands to use the socket library are outlined in the [TCP Client](/Code_for_RPi/TCP_Client.md) page.

## NRF24
This is a custom library written and maintained by TMRh20 on Github. It requires some additional setup to install and use because the library itself is written in C++. This makes it directly compatible with **Arduino**, but also necessitates a [python wrapper](https://medium.com/@brandon.a.alvarez1/python-wrappers-and-c-supercharge-your-projects-c18ffe8476c6) to use it on the RPi.

### Prerequisites
The RF24 Library requires that SPI be enabled on the RPi. This can be done by running
```bash
sudo raspi-config
```
then navigating to **Interfacing Options -> SPI** and selecting **Yes**. The RPi may need to be rebooted afterward.

### Installation Options
Multiple installation methods are available for this library. The links below offer some guidance, and are presented in order of preference:
1. [Automatic Install](https://github.com/nRF24/RF24/blob/master/docs/linux_install.md)
2. **High Level Manual Install**
- Open the Github [RF24](https://github.com/nRF24/RF24) Repository
- Click the green **Code** button and copy the **HTTPS** URL
- In a terminal, navigate to the **Documents** directory and clone the repository:
```bash
git clone "URL_HERE" (no quotes)
```
- Navigate to the **RF24** directory and run the following commands:
```bash
make
sudo make install
```
- The library should now be installed and ready to use in python scripts. Additional information on this process can be found [here](https://github.com/nRF24/RF24/blob/master/docs/rpi_general.md)
 3. **Low Level Manual Install**
 - After cloning the repository as in 2, see commentary from Fritzi16 on a previous [issue](https://github.com/nRF24/RF24/issues/615)
    - **Note**: Making and using the additional `rf24libs` directory is good practice but not strictly necessary.