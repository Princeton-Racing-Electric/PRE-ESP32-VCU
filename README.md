# ESP32 VCU code

Welcome to the VCU repo! This is all written in native C, with RTOS as the underlying operating system. This avoids a significant amount of Arduino bloat that comes with using it.

## RTOS Tasks
1.  Send Torque Commands
    1.  Rate: 50Hz
    2.  Priority: 9 (should be high)
2. Read Throttle
   1. Rate: 50Hz
   2. Priority: 8
3. Read Acceleration
   1. Rate: 800Hz (based off of interupts of when its ready, but the low pass filter can be set programatically)
   2. Priority: 7
4. Read Motor Speeds Over CAN
   1. Rate: 50Hz
   2. Priority: 6
5. Read Real Motor Torque Over CAN
   1. Rate: 50Hz
   2. Priority: 5
6. Read Gyroscope
   1. Rate: 1000Hz (based off of interupts of when its ready, but the low pass filter can be set programatically)
   2. Priority: 4
7. Read Motor Temperatures
   1. Rate: 1Hz
   2. Priority: 3
8. Read Motor Controller Temperatures over CAN
   1. Rate: 1Hz
   2. Priority: 2
9. Read Steering Angle Sensor
   1. Rate: 10Hz
   2. Priority: 1