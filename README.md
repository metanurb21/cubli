# Cubli style ESP32 Self balancing cube.

This is based on design and code by [RemRC.](https://github.com/remrc/Self-Balancing-Cube/blob/main/README.md)
I made a lot of modifications and refactoring of the code to suite my needs, PlatformIO, espressif32@5.0.0 and to use BLE to work with my iPhone along with a PCB schematic and gerber files which have been updated due to an error. A new laser cut design from 1/16" Stainless Steel and a custom made PCP.
This code also has the ability to make the cube spin in both directions by sending "l" and "r" commands via ESP32 BLE terminal app.
There is experimental code commented out to switch to Dabble API to get gyro and accell data from your phone to use phone orientation to drive the spin.

[YouTube Video](https://youtu.be/8PgxoO3G0Uw?si=-hIFl6_N7yUKP_Ou)

![ESP32 Cube balancing on vertex.](https://github.com/metanurb21/cubli/blob/main/images/Cubli.jpg)

![ESP32 cube controller schematic.](https://github.com/metanurb21/cubli/blob/main/images/bb-schematic.png)
