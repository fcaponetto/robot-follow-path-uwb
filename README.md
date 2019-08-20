[img01]: ./images/My_Thesis_1.jpg
[img02]: ./images/My_Thesis_2.jpg


# Robotic system following real-time human-path

Use of a pair of Ultra-Wide-Band (UWB) radio modules to detect the location of a person and follow him in case of acceptable range distance.

![img01]
![img02]

### Features
The robot is capable to calculate the distance (max 50m) between the point (person) and itself.
Detects the polar coordinates, together with particle filter and statistics correlation matrix.
Control the robot's wheel for generating the algorithm capable to follow the taget person.

The UWB sensors, ST and Arduino board are programmed in C (controlled by SPI) mixed with C++ follow-path algorithm.
