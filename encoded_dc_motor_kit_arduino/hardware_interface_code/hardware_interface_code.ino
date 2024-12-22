#include <Arduino.h>

// Define GPIO pins for encoder A and B channels
#define ENC_A_PIN 39
#define ENC_B_PIN 36

//motor control pins
int IN1 = 5;
int IN2 = 4;
int enablePin = 15;

// Define PWM parameters
const int freq = 5000; // PWM frequency in Hz
const int resolution = 8; // PWM resolution in bits (8, 10, 12, 15 bits are supported)

// Define variables to store encoder counts and direction
volatile int encoderCount = 0;

//speed calculations
double radians_ = 0.0;
double current_position = 0.0;
double velocity = 0.0;
volatile int past_time = 0.0;
volatile int past_position = 0.0;
volatile int print_value = 0;

//| 0000 0000 | | 0000 0000 | | 0000 0000 |
//|start byte | |  1st byte | | 2nd byte  |
//1st 0000  -> gives the start bit 0200
//2nd 0000 -> gives the number of bytes to be received typically 2.
uint8_t receive_data;
uint8_t error_ack = 0x22;
int8_t data_feedback[2] = {0x00, 0x00};
uint16_t data_received = 0;
uint8_t acknowledge = 0x55;


// Interrupt Service Routine (ISR) for encoder A pin
void IRAM_ATTR encoderISR() {
  // Read the state of the A and B channels
  int bState = digitalRead(ENC_B_PIN);

    // If A is high, check B to determine direction
    if (bState == HIGH) {
      encoderCount--; // Counterclockwise
    } else {
      encoderCount++; // Clockwise
    }
  }


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Initialize PWM channel
  ledcSetup(0, freq, resolution);
  ledcAttachPin(enablePin, 0);

  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode( IN1, OUTPUT);
  pinMode( IN2, OUTPUT);
  digitalWrite( IN1, HIGH); 
  digitalWrite( IN2, HIGH); 

  // Set encoder pins as inputs
  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);

  // Attach interrupt to the encoder A pin
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, RISING);


}

void loop() {
  // put your main code here, to run repeatedly:

 while(1){
  // put your main code here, to run repeatedly:

//WAIT FOR THE START COMMAND
while(Serial.available() == 0){
}

receive_data = Serial.read();

if(!receive_data == 0x54)
{
  Serial.write(error_ack); //FAILED FEEDBACK
  continue;
}
else
{
  Serial.write(acknowledge); //SUCCESS FEEDBACK
}

//WAIT FOR DATA READ OR WRITE QUESTION
while(Serial.available() == 0){
}

receive_data = Serial.read();


if(receive_data == 0x55)//READ COMMAND FROM MASTER
{
  //IF IT IS A READ 
  data_feedback[0] = (encoderCount >> 8) & 0xFF;
  data_feedback[1] = (encoderCount) & 0xFF;

  Serial.write( data_feedback[0]); 
  Serial.write( data_feedback[1]); 
  
}
else if(receive_data == 0x56)//WRITE COMMAND FROM MASTER
{
  Serial.write(acknowledge); 

  uint8_t data_input[2] = { 0, 0};



  for(int i = 0; i < 2; i++)
  {
      //WAIT FOR DATA READ OR WRITE QUESTION
       while(Serial.available() == 0){
       }   
      data_input[i] = Serial.read();
  }

   data_received = data_input[0];
   data_received = data_received << 8;
   data_received |= data_input[1];

    // Split data_output into two bytes
    uint8_t high_byte = (data_received >> 8) & 0xFF;
    uint8_t low_byte = data_received & 0xFF;

    int16_t signed_data;
    
    // Combine them into a signed 16-bit integer
    signed_data = (int16_t)((high_byte << 8) | low_byte);

        if(signed_data > 0)
        {
           digitalWrite(IN1, HIGH);
           digitalWrite(IN2, LOW);
        }
        else if(signed_data < 0)
        {
           digitalWrite(IN2, HIGH);
           digitalWrite(IN1, LOW);
        }
    ledcWrite(0, abs((int)signed_data));

/*-------------------------------TEST CODE---------------------------------*/
  //Serial.write(data_input[0]); 
  //Serial.write(data_input[1]); 
/*-------------------------------TEST CODE---------------------------------*/
   
}
else
{
  Serial.write(error_ack); 
  receive_data = 0;
  continue;
}

}


}
