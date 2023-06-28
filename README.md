# V2V_COMMUNICATION_PROJECT

This project implements two use-cases for V2V communication: Electronic Emergency Brake Lights, and Criminal Car Control.

## Electronic Emergency Brake Lights

When a car hard brakes, and there are two cars behind it, all on the same line, the car directly behind the hard braking car would notice the car in front and avoid it, but the third car is most likely to notice when it's too late. This use-case aims at making the hard braking car send a warning message whenever the speed begins to drop dramatically, so all the cars can be aware of the situation and avoid a collision.

## Criminal Car Control

This use-case aims at allowing police cars to stop a car they are chasing, by sending a message to the car to stop. The car would then stop, and the police car would be able to catch up to it and arrest the driver.

## Implementation Details

This project uses MQTT for delivering messages between cars. The MCU used is stm32f446re, and the IDE used is STM32CubeIDE. Other hardware includes ESP8266, Neo m8n GPS module, HC-05 Bluetooth module, IR speed sensor, LCD.

The project is implemented on an RC car.
