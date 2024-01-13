# Self-Balancing-Robot-Using-PID-Controller
Self Balancing Robot using a 6-axis IMU to get the tilt angle. The tilt angle is calculated using the accelerometer and using the gyroscope. These readings are then fused together to get a more accurate measurement. The error between the actual tilt angle and the desired tilt angle is then calculated, and a PID controller is used to finely adjust the speed of the motors.

# Components
- Arduino Nano as the main controller
- MPU6050: 3-axis Accelerometer + 3-axis Gyroscope
- L298N Motor driver
- JGA25-370 DC motor
# Schematic
![image](https://github.com/mazenelgabalawy/Self-Balancing-Robot-Using-PID-Controller/assets/72276135/e47e7323-81e1-4ff8-85cc-c4f51d2b9f83)
