# robot-follow-path-uwb

Use of Ultra-Wide-Band radio module to detect the movement of a person in order to follow him in case of acceptable range distance.

Features:
The robot is capable to calculate the distance (max 50m) between the point (person) and itself.
Detects the polar coordinates using particle filter and statistics correlation matrix.
Control the system in order that the robot always follows the human.

The sensors UWB and ST board are programmed in C (controlled by SPI) mixed with C++ follow-path algorithm.
