# ESP32 WROOM Code for Motor Control
## MOTOR CONTROL
This section contains code to be uploaded to the ESP32 WROOM microcontroller. It defines the GPIO pin configuration for motor control and encoder feedback, enabling precise control and monitoring.


<div align="center">

## GPIO Pinouts for Control


| Function | GPIO Pin | Motor cable colour |
| :-------- | :--------: | :--------: |
| Encoder Channel A |	GPIO 39(VP) | yellow |
| Encoder Channel B	| GPIO 36(VN) | dark-green |
| Motor Control - IN1 | GPIO 5 | N/A |
| Motor Control - IN2 | GPIO 4 | N/A |
| Motor Control - Enable | GPIO 15 | N/A |

![ESP32 WROOM IMAGE](/encoded_dc_motor_kit_arduino/documentation/images/ESP32%20WROOM.png)
</div>

## OPTICAL ENCODER PINOUT

<div align="center">

## GPIO Pinouts for velocity reading and position reading.


| Function | GPIO Pin | Encoder cable colour | Jumper colour | Wire function|
| :-------- | :--------: | :--------: | :--------: | :--------:|
| Encoder Channel A |	 | green | orange | A phase |
| Encoder Channel B	|  | white | yellow | B phase |
| POWER |  | red | red | VDD |
| GROUND |  | black | brown | GND|

</div>
