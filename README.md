# Teensy_actuators_PPZ
This repository provides the capability to establish a connection between the Paparazzi UAV autopilot and a Teensy 4.0, enabling the control of motors and servomotors using the Teensy.

For the control of ESCs, the Teensyshot library from https://github.com/jacqu/teensyshot is utilized.
Servos can be controlled in both PWM mode and Serial mode. The testing was conducted with the Feetech STS3032 servos (https://nl.aliexpress.com/item/1005001678105997.html), 
but any serial servos employing the dinamixel half-duplex serial communication can be employed. 
A more detailed explanation of the serial servos protocol can be found in the Documentation/Serial_servo_protocol.pdf file.

In the current setup, the refresh rate for the control and feedback of the motors is approximately 500 Hz, while the refresh rate for the control and feedback of the servos is approximately 350 Hz.

The user has the flexibility to determine the refresh time of the UART communication packets with the Paparazzi UAV autopilot by modifying the "COMM_REFRESH_TIME" define in the 
"servo_esc_control_w_feedback_T4_PPZ.ino" file. This define specifies the delay in microseconds for each subsequent packet sent back to the Autopilot.

It is important to note that for the proper utilization of this repository, the Teensy_actuators_PPZ module should be loaded and executed on Paparazzi UAV.
 
# Wiring:
Currently, the setup implements 4 KISS 32A ESCs, 8 Feetech STS3032 servos and 2 PWM servos connected to a Teensy 4.0 wired in this configuration: 

**KISS 32 ESCs connection:**
|  ESCs | Signal PIN | Telemetry PIN |
| ----- | ---------- | ------------- |
| ESC 1 | 4 | 0 |
| ESC 2 | 8 | 7 |
| ESC 3 | 24 | 15 |
| ESC 4 | 9 | 25 |

**Feetech STS3032 servos connection:**
|  Servo number | Servo ID | PIN |
| ----- | ---------- | ------------- |
| Servo 1 | 1 | 28\*29 | 
| Servo 2 | 2 | 28\*29 | 
| Servo 3 | 3 | 28\*29 | 
| Servo 4 | 4 | 28\*29 | 
| Servo 5 | 5 | 20\*21 | 
| Servo 6 | 6 | 20\*21 | 
| Servo 7 | 7 | 20\*21 | 
| Servo 8 | 8 | 20\*21 | 

The asterisk (\*) symbol indicates that the pins need to be connected together. This connection is required because the protocol utilizes a half-duplex serial protocol, 
where transmission and reception occur on the same line.
The Servo ID refers to the ID of the Serial servo that is connected to the BUS. The ID of the servo can be programmed using the board provided in this link: https://nl.aliexpress.com/item/1005004007925311.html.

**Generic PWM servos connection:**
|  Servo number | PIN |
| ----- | -------- |
| Servo 9 | 2| 
| Servo 10 | 3| 

**Connection to the Autopilot:**
|  AP PIN | Teensy PIN |
| ----- | -------- |
| AP UART TX | 16| 
| AP UART RX | 17| 
