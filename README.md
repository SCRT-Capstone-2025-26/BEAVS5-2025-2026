# BEAVS5 Firmware and Simulation

## Overview

This is the repo for SCRT's beavs module. Which is designed to extend servo
actuated blades to control rocket drag and hit a target height of 10,000 ft.
This contains the arduino firmware (in Arduino/BEAVS5_Main) for the system as
well as a C++ simulation to run the firmware in a variety of test conditions.
This simulation has python bindings through pybind11 to allow for usage a python
rocket simulation [here](https://github.com/SCRT-Capstone-2025-26/SCRT_Rocket_SIM).

This board is undergoing hardware redesign so this code is in flux.

## How Run

To build the firmware see Arduino/BEAVS5_Main

Currently the project is setup to go into a larger python project and function
a module that can be imported normally. This means it will auto build on import
which is likely to change in the future.

If you want to build it yourself this project uses make and running make in the
Arduino directory will cause the simulation to be built. Running ``make exp``
will export the built python to different parts of the file structure where it is
expected.

To run the tests simply run ``pytest`` in the root.

## Team

This was created for an OSU capstone and more info can be found at the main [repo](https://github.com/SCRT-Capstone-2025-26/SCRT_Rocket_SIM).
