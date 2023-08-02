# Introduction

V2V communication is one of the recent technologies in modern day cars. It's a technology that makes roads safer and makes cars more reliable. It allows cars to communicate with each other, and with the infrastructure around them. This project aims at implementing a V2V communication system, and using it to implement some use-cases.

# Project Description

This project aims at implementing two use-cases for V2V communication. The first use-case is called Emergency Electronic Brake Lights. The second use-case is Criminal Car Control.

## Emergency Electronic Brake Lights

Consider the following scenario:
<img src="https://github.com/ZyadZ2000/v2v_project/assets/85132955/c37a9f30-c037-4251-b53c-5117460b19e2">
If Car3 hard brakes, Car2 has a chance to see it and avoid a collision, but there is a very big risk that Car1 will respond when it's too late and an crash happens. V2V communication would be useful in this case, since Car3 will automatically send a warning message/indicator to both Car2 and Car1 when the speed drops dramatically.

## Criminal Car Control

This use-case aims at using V2V communication to allow authenticated and authorized police cars to stop a car they are chasing. The police car will send a message to the car they are chasing, and the car will stop automatically.

# Hardware Used

Main MCU: stm32f446re<br>
Wireless Module: esp8266<br>
GPS: Neo m8n<br>
Speed Sensro: LM393<br>
Display: LCD Display<br>
Control: Through an android app and bluetooth<br>
Bluetooth: HC-05<br>
<br>

Showcased on an RC car.

# Tools and Technologies Used

MQTT was used as the main protocol. A custom MQTT broker was implemented to authenticate and authorize cars.

# Additional Documentation and Showcase

Slides:
https://docs.google.com/presentation/d/1AYvZMOPNye9VQ_AN6Ss3iqvlZ1svswHA/edit#slide=id.p1

Video:
https://drive.google.com/file/d/1Iq1DgmGgCEPR7nZtgdZwj2YPcRdrk-Ru/view
