yolo-octo-avenger
=================

A re-structuring of Terry Fritz public domain SW-COMPUTER Factory Control Program for ATTINY85-20PU micro-controller.
In the state I received this it was quite a mess and did not fit into the flash of the ATTINY85.

This will be an adventure.

The target
----------

To get this program to fit in on a ATTINY85 we need to:

- get the .text+.data sections enough below 8192 bytes to fit a bootloader
- keep the .data+.bss sections enough below 512 bytes to still fit a decent stack

*The original code is Public Domain, my changes will be under the MIT License.*
