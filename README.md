# FRC(RCC) Team 5849 2021

This is Team 5849's 2021 robot code for RCC, which is FRC held by Chinese committee in Hangzhou. The code is written in Java and is based off of WPILib's Java control system.

## Code structure

---

The code used the Command Based Programming Framework to structure this project. So the codebase can be divided into three main parts:

- commands
- subsystems
- binding commands and subsystems to joystick buttons(for tele-operation)

## Setup Instructions

- Follow the instructions on WPILib to setup the environment (https://docs.wpilib.org/en/stable/index.html)
- Clone this repo
- Run `./gradlew` to download gradle and required FRC libraries
- Run `./gradlew tasks` to see available build options

## Robot Wiring Instructions

To lookup the port mapping of motors and sensors on the robot, checkout the Constants.java file in the directory.
