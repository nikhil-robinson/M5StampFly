# StampFly

This is a port of the official stampfly to support BLE and WIFI control using any smartphone The whole project is taken from the M5 stack repo i just modified the RC edited out the esp-now and added BLE support. And i also have a UDP example which uses wifi and UDP which is fasterand have more range on the udp branch.

The following are the Bytes to be sent.

Byte    Data                Description
0       MAC Addr[3]         MAC address byte 3 (used for filtering).
1       MAC Addr[4]         MAC address byte 4.
2       MAC Addr[5]         MAC address byte 5.
3–6     RUDDER              Noramlized Rudder value ad 4-byte float. -1 to 1.
7–10    Throttle            Normalized Throttle value as 4-byte float -1  to 1.
11–14   AILERON             Normalized Roll value as 4-byte float -1 to 1.
15–18   ELEVATOR            Normalized ELEVATOR value as 4-byte float -1 to 1. 
19      ARM                 Boolean (0 or 1) for arming the drone.
20      Flip Button Status  Boolean (e.g., for flip commands).
21      CONTROLMODE         Drone mode value (e.g., 0 = Stable mode, 1 = Sports mode.).
22      ALTCONTROLMODE      Alternate mode flag to switch between manula and auto (5 for manual, 4 for auto).
23      Proactive Flag      Additional proactive control flag.
24      Checksum            Sum of bytes 0–23 to validate data.



Step 1 : Once connected the app needs to send the buffer continuously every 10ms to the drone with default values.
Step 2 : The ALTCONTROLMODE should be set to 5 for manual control and to 4 for  auto mode.
Step 3 : Toggle the Proactive Flag bit make the bit high for 20ms and make if low which will reset the AHRS.
Step 4 : For arming the drone you need to send 1 on the BUTTON_ARM index for exactly 20MS and Off . same for disarming.
Step 5 : Fly the Drone the joystick values should be normalized between -1 to 1.

#TODO

add telemetry for sensor feedback

## Framework

Platformio

## Base on project

[M5Fly-kanazawa/StampFly2024June (github.com)](https://github.com/M5Fly-kanazawa/StampFly2024June)

## Product introduction

[M5Stampfly](https://docs.m5stack.com/en/app/Stamp%20Fly)

## Third-party libraries

fastled/FastLED

tinyu-zhao/INA3221

mathertel/OneButton @ ^2.5.0