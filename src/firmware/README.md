# BEAVS5 Firmware

## NOTE: This is currently under development for the new board

## Overview

This contains the actual firmware the will run on the BEAVS5 system. It is an
arduino project and has the following dependencies on a generic rp2040 board.

-

See <https://github.com/osu-asdt/beavs-6> for more details about the hardware.

## Notes

- The rp2040 has two cores. All logging and stuff is on core 1 and the rest is on core 0.
