# RoboMat-Project

## Introduction

<img src="images/RobotGeneral.png" width="50%"/>

## Components

## Chassis

### Battery

4 x alkaline battery : 1.5V & 1700-2800mA

### Single Board Computer

NodeMCU board version 1.0 base on ESP8266

<img src="images/Board.png" width="50%"/>

Characteristic  | Value
------------ | -------------
Processor |  Tensilica L106 32-bit
Processor Frecuency | 80MHz
Memory SRAM user |  50kB
Memory Flash | 16Mbytes
Wifi | 802.11 b/g/n/e/i
GPIO | 17 GPIO
Operating Voltage | 3.0V ~ 3.6V
Operating Current | Average Value: 80mA
Output Voltage pins | 3.3V
Current nominal pins | 12mA

### Motor

<img src="./images/yellow-gear-motor.jpg" width="25%"/>

Characteristic  | Value
------------ | -------------
Operating voltage | 3V ~ 6V DC ( recommended  5V )
No Load current | 190mA ( maximum 250mA)
Stall Current | ~1A
Maximum torque | 800g.cm
Speed without load | 90±10rpm
Reduction ratio | 1:48

### Wheel

### Caster

### Encoder

### Encoder Filter

### Servo

ES 3154

<img src="./images/Servo" width="25%"/>

Characteristic  | Value
------------ | -------------
Operating voltage | 4.8V ~ 6V DC

### Ultrasonic

HC-SR04

<img src="./images/Ultrasonic" width="25%"/>

Characteristic  | Value
------------ | -------------
Operating voltage | 5V
Operating current | 15ma
Max Range | 4m
Min Range | 2cm
Measuring Angle | 15 degree

## Pinout

ESP8266 | Arduino | Description
------------ | ------------- | ------------
D0 | 16 | Ultrasonic Trigger
D1 | 5  | Power Motor Right
D2 | 4  | Power Motor Left
D3 | 0  | Direction Motor Right
D4 | 2  | Direction Motor Left
D5 | 14 | Encoder Motor Right
D6 | 12 | Encoder Motor Left
D7 | 13 | Servo
D8 | 15 | Untrasonic Echo
