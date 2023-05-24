# Teensy_actuators_PPZ
This repository provides the capability to establish a connection between the Paparazzi UAV autopilot and a Teensy 4.0, enabling the control of motors and servomotors using the Teensy.

For the control of ESCs, the Teensyshot library from https://github.com/jacqu/teensyshot is utilized.
Servos can be controlled in both PWM mode and Serial mode. The testing was conducted with the Feetech STS3032 servos (https://nl.aliexpress.com/item/1005001678105997.html?gatewayAdapt=glo2nld), 
but any serial servos employing the dinamixel half-duplex serial communication can be employed. 
A more detailed explanation of the serial servos protocol can be found in the Documentation/Serial_servo_protocol.pdf file.

In the current setup, the refresh rate for the control and feedback of the motors is approximately 500 Hz, while the refresh rate for the control and feedback of the servos is approximately 350 Hz.

The user has the flexibility to determine the refresh time of the UART communication packets with the Paparazzi UAV autopilot by modifying the "COMM_REFRESH_TIME" define in the 
"servo_esc_control_w_feedback_T4_PPZ.ino" file. This define specifies the delay in microseconds for each subsequent packet sent back to the Autopilot.

# Wiring:
Currently, the setup implements 4 KISS 32A ESCs, 8 Feetech STS3032 servos and 2 PWM servos wired in this configuration: 
