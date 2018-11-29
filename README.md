# HR-monitor-server

This is a server that runs on Linux and detects heartrate and falls by connecting via I2C to a max30100 pulse oximeter and mpu-6050 gyro/accelerometer. The data is then sent via bluetooth using btgatt-server based on bluez. Works on a Raspberry Pi 3 with the integrated bluetooth. It can connect on to a client application on a mobile device to send alerts when the pulse is abnormal.

## Fall detection

A state machine is used to detect falls. 

The accelerometer code is based on the following implementation :

https://github.com/richardghirst/PiBits/tree/master/MPU6050-Pi-Demo


## Pulse oximeter

The pulse oximeter code is based on the following implementation for Arduino:

https://morf.lv/implementing-pulse-oximeter-using-max30100

It was ported to work without Arduino librairies on plain Raspbian.
