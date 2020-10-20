# Basic Delta Things OS Application Template

## Overview

This is a basic DTOS application template that will enable the following features

1) Mqtt connectivity to Deltathings Thingsboard cloud
2) OTA (Over the Air) update using Deltathings Thingsboard cloud
3) RPC 
4) Wifi
5) i2C
6) SPI
7) Application skeleton for Mqtt & GPIO 


## 1.DTOS application development requirements

- Access to Deltathings Droppy (https://dtiot-dev.ddns.net:1443) firmware upload website
    - For access send email to deltathings@outlook.com 
      - Include subject - Access to DTOS Droppy 
      - Email , name 
      - Admin will provide the access with default password "password"
- Access to Deltathings Dev Cloud  (https://dtiot-dev.ddns.net:8080) 
    - For access send email to deltathings@outlook.com 
      - Include subject - Access to Deltathingns Dev Cloud  
      - Email , name
- Access to Deltathings github  (https://dtiot-dev.ddns.net:8080) 
    - For access send email to deltathings@outlook.com 
      - Include subject - Access to Github Deltathings  
      - Email , name

## 2. Install base installation packages using below script
- Go to Projects folder
- open terminal and enter below commands
- git clone https://github.com/delta-things/Installation-Scripts
- cd Installation-Scripts
- sudo chmod +x install-base-sw-packages-with-prompts.sh
### Install base packages
- ./install-base-sw-packages-with-prompts.sh -b 
### Install docker
- ./install-base-sw-packages-with-prompts.sh -d
- After Installing docker, logout PC and log in to get SUDO privileages for docker
### Install visual code
- ./install-base-sw-packages-with-prompts.sh -v
### Install DTOS PC tool
- ./install-base-sw-packages-with-prompts.sh -m

## 3.Clone DTOS embedded application git repo
- Go to Installation-Scripts
- open terminal and run ./git-cache-credentials.sh
- Connect ESP32 or ESP8266 board to pc (NodeMCU or ESP32 Wroom)
- open terminal,navigate to location where you wanted clone the project
- git clone https://github.com/delta-things/DTOS
- cd DTOS
- dtos
- This command will open dtos PC tool in browser , you should see the serial port traffic from ESP

## 4.Procedure to setup Visual code studio in linux

- Open vcode from start menu in linux
- Install these VSCode extensions: C/C++, Native Debug, Task Explorer
  Follow this (https://code.visualstudio.com/docs/introvideos/extend) incase if you are not familiar with vcode extensions
- After installing extensions , go to file--> open folder --> navigate to step.2 downloaded DTOS location --> open folder
- Go to Takexplorer -->vscode --> Buld ESP32 or ESP8266 applicaion based on your project needs , followed by click flash to program the controller

Note : After flash operation from vscode , DTOS PC tool browser will stop working (vscode will hijack the serial port ) , to resume DTOS tool browser serial port just refresh browser !!!
 

## 5.After Flashing 

- GO to DTOS PC tool browser terminal
- configure ESP device wifi by enterring below commnd in browser 
- mos wifi WIFISSID WIFIPASSWORD followed by enter
- mos config-set mqtt.user="DEVICE TOKEN from DT cloud"
- mos call Config.Save '{"reboot": true}'
- GPIO2 of ESP need to be connected to LED .. 
    -- LED is flashing slowly --> Not connected DT cloud
    -- LED constant --> Successful connection to DT cloud
    -- LED flashing at 0.5 hz --> OTA software update is ongoing




