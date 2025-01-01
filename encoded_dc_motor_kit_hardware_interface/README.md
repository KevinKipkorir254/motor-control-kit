# Introduction
This follows the stop_and_wait (protocol) it works by sending data and waiting for an ack. We also send the numbers as two 8bit ints then put them back together on the ohter side using bit shifting:

```cpp
serial_port->ReadByte(received_data[0], 1000);
serial_port->ReadByte(received_data[1], 1000);
final_received_data = received_data[0];
final_received_data = final_received_data << 8;
final_received_data |= received_data[1];
*read_output = final_received_data;
```

formatting the data before sending:

```cpp
unsigned char outgoing_data
serial_port->WriteByte(outgoing_data[0]);
serial_port->WriteByte(outgoing_data[1]);
```
These two images give a grapghical explanation:
![READ DATA GRAPH](/encoded_dc_motor_kit_hardware_interface/documentation_images/read_data_from_esp.png)

![WRITE DATA GRAPH](/encoded_dc_motor_kit_hardware_interface/documentation_images/write_data_to_esp.png)