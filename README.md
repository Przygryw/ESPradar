Useless quasi-radar made with ESP32.
ESP32 reads position with GY-GPS6MV2 connected to it, then SR04 gets distance of detected object.
SR04 should be mounted on Servo (I used SG90). Depends on current SR04 location, math puts detected object on proper coords. 
Results are then printed in terminal.
Project is useless, because SR04 has only 4 meters range, so it's not possible to make result noticeable in coords (unless using many decimal places).
