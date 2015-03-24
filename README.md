# Pump Controller
ENEL517-15A Assignment 2

## Introduction
This pump controller demonstrates control theory through the use of a sump pump and a tank.
The water in the tank can be drained by a tap at the bottom and pumped in at the top.
If the water level gets too high the water will drain out the overflow pipe.
The aim is to set the water level and hold it steady using a control loop based on the water level and the pump output.

## Method
This project makes use of a NARM_V1 board and libnarm to prepare the STM32F051K6 for use.
The ARM sends an ultrasonic pulse and measures the reflection to get the current water level through the use of interrupts and DMA.
The control loop runs a PID algorithm based on the measured water level (to calculate the error) to generate a PWM value with which to drive the motor.
