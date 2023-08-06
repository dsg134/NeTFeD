# Sensor_Fusion_Optimization
This project fuses RGB and Thermal Imaging to detect hot/burning objects.
- RGB Imaging is done with the ESP32-CAM. A CNN was trained with 300+ images of burning objects using Edge Impulse to get ESP32-CAM detection to work.
- Thermal Imaging is done with Adafruit's AMG8833 module. The 8x8 pixel array was interpolated into a size of 13x13 for better resolution.
- To optimize the FOV and detection area of the dual sensing system, you can use the Optimizer.py code to conduct all the calculations.
