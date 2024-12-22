#include "encoded_dc_motor_kit_hardware_interface/stop_and_wait.hpp"


// Function to delay by a specific duration
//This is great software
void delay(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

int read_data_data(uint16_t* read_output, LibSerial::SerialPort* serial_port)
{

        // Define the bytes to send
    char start_byte = 0x54;
    uint8_t received_data[2] = { 0x00, 0x00};
    uint16_t final_received_data = 0x0000;
    char read_data = 0x00;
    uint8_t read_data_test = 0x00;
    char read_command = 0x55;
    char write_command = 0x56;
    char outgoing_data[2] = { 0x67, 0x32};


    int spinner_index = 0;
    const char spinner[] = {'|', '/', '-', '\\'};
    //READ SEQUENCE
            // Write the start byte (if needed for your protocol)
            
            //std::cout << "In while loop, writing: "<< std::hex << static_cast<int>(start_byte) << std::endl;
            serial_port->FlushIOBuffers();
                
 
            try
            {
               serial_port->WriteByte(start_byte); 
            }
            catch(const LibSerial::SerialPort& e)
            {
                
                RCLCPP_INFO(rclcpp::get_logger("PORT READ"), "Data not sent");
                exit(1);

            }

            //std::cout << "Data sent "<< std::endl;

            //receive acknowledge
            // Wait until data is available

            {
              int i = 0;
             while (!serial_port->IsDataAvailable()) { 

                          
                 // Show loading spinner
                 //std::cout << "\r" << spinner[spinner_index] << std::flush;
                 //RCLCPP_INFO(rclcpp::get_logger("PORT READ"), "|");
                 delay(1); // Sleep for 100 milliseconds
                 // Move to the next spinner character
                 i++;
                 if(i > 100)
                 {
                   RCLCPP_INFO(rclcpp::get_logger("PORT READ"), "FAILED WAIT");
                   return -1;
                 }
             }

            }

             //std::cout << " "<< std::endl;

            //std::cout << "Reading data " << std::endl;
                //RCLCPP_INFO(rclcpp::get_logger("PORT READ"), "Reading data");
            serial_port->ReadByte( read_data, 100);
            //std::cout << "Receiving acknowledge: " << std::hex << static_cast<int>(read_data) << std::endl;
               // RCLCPP_INFO(rclcpp::get_logger("PORT READ"), "Receiving acknowledge");

            if(!(read_data == 0x55))
            {
             //std::cerr << "Wrong acknowledge data" << std::endl;Receiving acknowledge:
            RCLCPP_INFO(rclcpp::get_logger("PORT READ"), "Wrong acknowledge data");
             return -1;
            }


            // Write the start byte (if needed for your protocol)
             try
            {
               //std::cout << "Writing read command: " << std::hex << static_cast<int>(read_command) << std::endl;
            //RCLCPP_INFO(rclcpp::get_logger("PORT READ"), "Writing read command:");
               serial_port->WriteByte(read_command); 
            }
            catch(const LibSerial::SerialPort& e)
            {
                //std::cerr << "Data not sent" << std::endl;
                RCLCPP_INFO(rclcpp::get_logger("PORT READ"), "Data not sent");
                exit(1);

            }

            //Read the two bytes
            //receive acknowledge
            // Wait until data is available
            {

              int i = 0;
             while (!serial_port->IsDataAvailable()) {        
                 // Show loading spinner
                 //std::cout << "\r" << spinner[spinner_index] << std::flush;
                 //RCLCPP_INFO(rclcpp::get_logger("PORT READ"), "|");
                 delay(1); // Sleep for 100 milliseconds
                 // Move to the next spinner character
                 i++;
                 if(i > 100)
                 {
                   RCLCPP_INFO(rclcpp::get_logger("PORT READ"), "FAILED WAIT");
                   return -1;
                }
             }

            }
             //std::cout << " "<< std::endl;

            serial_port->ReadByte(received_data[0], 1000);
            serial_port->ReadByte(received_data[1], 1000);
            final_received_data = received_data[0];
            final_received_data = final_received_data << 8;
            final_received_data |= received_data[1];
            *read_output = final_received_data;
            //std::cout << "Received hex value from Arduino: " << final_received_data << std::endl;
             //RCLCPP_INFO(rclcpp::get_logger("PORT READ"), "Received hex value from Arduino: %d", final_received_data);
            return 1;

}

int write_data_data(unsigned char *outgoing_data, LibSerial::SerialPort* serial_port)
{

        // Define the bytes to send
    char start_byte = 0x54;
    uint8_t received_data[2] = { 0x00, 0x00};
    uint16_t final_received_data = 0x0000;
    char read_data = 0x00;
    uint8_t read_data_test = 0x00;
    char read_command = 0x55;
    char write_command = 0x56;
    //char outgoing_data[2] = { 0x67, 0x32};


    int spinner_index = 0;
    const char spinner[] = {'|', '/', '-', '\\'};

 //WRITE SEQUENCE
            // Write the start byte (if needed for your protocol)
            //std::cout << "In while loop, writing: "<< std::hex << static_cast<int>(start_byte) << std::endl;
            serial_port->FlushIOBuffers();      
 
            try
            {
               serial_port->WriteByte(start_byte); 
            }
            catch(const LibSerial::SerialPort& e)
            {
                //std::cerr << "Data not sent" << std::endl;
                RCLCPP_INFO(rclcpp::get_logger("PORT WRITE"), "Data not sent");
                exit(1);

            }

            //std::cout << "Data sent "<< std::endl;

            //receive acknowledge
            // Wait until data is available

            {
              int i = 0;
             while (!serial_port->IsDataAvailable()) { 

                          
                 // Show loading spinner
                 //std::cout << "\r" << spinner[spinner_index] << std::flush;
                 //RCLCPP_INFO(rclcpp::get_logger("PORT READ"), "|");
                 delay(1); // Sleep for 100 milliseconds
                 // Move to the next spinner character
                 i++;
                 if(i > 100)
                 {
                   RCLCPP_INFO(rclcpp::get_logger("PORT READ"), "FAILED WAIT");
                   return -1;
                 }
             }

            }
             //std::cout << " "<< std::endl;

            //std::cout << "Reading data " << std::endl;
            serial_port->ReadByte( read_data, 100);
            //std::cout << "Receiving acknowledge: " << std::hex << static_cast<int>(read_data) << std::endl;

            if(!(read_data == 0x55))
            {
             //std::cerr << "Wrong acknowledge data" << std::endl;
             RCLCPP_INFO(rclcpp::get_logger("PORT WRITE"), "Wrong acknowledge data");
             return -1;
            }


            // Write the start byte (if needed for your protocol)
             try
            {
               //std::cout << "Writing write command: " << std::hex << static_cast<int>(write_command) << std::endl;
               serial_port->WriteByte(write_command); 
            }
            catch(const LibSerial::SerialPort& e)
            {
                //std::cerr << "Data not sent" << std::endl;
                RCLCPP_INFO(rclcpp::get_logger("PORT WRITE"), "Data not sent");
                exit(1);

            }


            //receive acknowledge
            // Wait until data is available

            {
              int i = 0;
             while (!serial_port->IsDataAvailable()) { 

                          
                 // Show loading spinner
                 //std::cout << "\r" << spinner[spinner_index] << std::flush;
                 //RCLCPP_INFO(rclcpp::get_logger("PORT READ"), "|");
                 delay(1); // Sleep for 100 milliseconds
                 // Move to the next spinner character
                 i++;
                 if(i > 100)
                 {
                   RCLCPP_INFO(rclcpp::get_logger("PORT READ"), "FAILED WAIT");
                   return -1;
                 }
             }

            }
             //std::cout << " "<< std::endl;

            //std::cout << "Reading data " << std::endl;
            serial_port->ReadByte( read_data, 100);
            //std::cout << "Receiving acknowledge: " << std::hex << static_cast<int>(read_data) << std::endl;

            if(!(read_data == 0x55))
            {
             //std::cerr << "Wrong acknowledge data" << std::endl;
            RCLCPP_INFO(rclcpp::get_logger("PORT WRITE"), "Wrong acknowledge data");
             return -1;
            }

            serial_port->WriteByte(outgoing_data[0]);
            serial_port->WriteByte(outgoing_data[1]);

/*-------------------------------TEST CODE---------------------------------*/
 /*WRITING FOR TESTING, UNHIGHLIGHT THIS*/
 

 /*          
 //Read the two bytes
            //receive acknowledge
            // Wait until data is available
            uint8_t test_data[2];
             while (!serial_port->IsDataAvailable()) {           
                 // Show loading spinner
                 //std::cout << "\r" << spinner[spinner_index] << std::flush;
                 RCLCPP_INFO(rclcpp::get_logger("PORT READ"), "|");
                 delay(1); // Sleep for 100 milliseconds
                 // Move to the next spinner character
                 //spinner_index = (spinner_index + 1) % 4;
             }
             //std::cout << " "<< std::endl;

            serial_port->ReadByte(test_data[0], 1000);
            serial_port->ReadByte(test_data[1], 1000);

            if((test_data[0] == outgoing_data[0]) && (test_data[1] = outgoing_data[1]))
            {

            }
            else
            {
                std::cerr << "This shit is trash Dowg"<< std::hex << static_cast<int>(test_data[0]) << std::endl;
                std::cerr << "This shit is trash Dowg"<< std::hex << static_cast<int>(outgoing_data[0]) << std::endl;
                std::cerr << "This shit is trash Dowg"<< std::hex << static_cast<int>(test_data[1]) << std::endl;
                std::cerr << "This shit is trash Dowg"<< std::hex << static_cast<int>(outgoing_data[1]) << std::endl;
            }
*/
/*-------------------------------TEST CODE---------------------------------*/
       return 1;  

}
