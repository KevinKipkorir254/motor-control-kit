#include <Arduino.h>

// Define GPIO pins for encoder A and B channels
#define ENC_A_PIN 39
#define ENC_B_PIN 36

//Define GPIO PINS for optical encoder
#define ENC_PHASE_A 34
#define ENC_PHASE_B 35

// Motor control pins
const int IN1 = 5;
const int IN2 = 4;
const int enablePin = 15;

// Define PWM parameters
const int pwmRange = 255;   // PWM range (0-255 for analogWrite)

// Define variables to store encoder counts and direction
volatile int encoderCount = 0;
volatile int OpticalEncoderCount = 0;

// Speed calculations
double radians_ = 0.0;
double current_position = 0.0;
double velocity = 0.0;
volatile int past_time = 0;
volatile int past_position = 0;
volatile int print_value = 0;

// Communication protocol variables
// First byte is start byte (0x54), followed by command bytes
uint8_t receive_data;
uint8_t error_ack = 0x22;   // Error acknowledgment
int8_t data_feedback[4] = {0x00, 0x00, 0x00, 0x00};
uint16_t data_received = 0;
uint8_t acknowledge = 0x55; // Success acknowledgment

// Interrupt Service Routine (ISR) for encoder A pin
void ARDUINO_ISR_ATTR encoderISR() {
  // Read the state of the B channel when A has a rising edge
  int bState = digitalRead(ENC_B_PIN);
  
  // Determine direction based on B state
  if (bState == HIGH) {
    encoderCount--; // Counterclockwise
  } else {
    encoderCount++; // Clockwise
  }
}

// Interrupt Service Routine (ISR) for encoder A pin
void ARDUINO_ISR_ATTR encoderISR() {
  // Read the state of the B channel when A has a rising edge
  int bpState = digitalRead(ENC_PHASE_B);
  
  // Determine direction based on B state
  if (bpState == HIGH) {
    OpticalEncoderCount--; // Counterclockwise
  } else {
    OpticalEncoderCount++; // Clockwise
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(230400);
  
  // Initialize PWM channel
  analogWrite(enablePin, 0); // Initialize with 0 duty cycle
  
  // Initialize motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, HIGH); 
  
  // Set encoder pins as inputs
  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);
  pinMode(ENC_PHASE_A, INPUT);
  pinMode(ENC_PHASE_B, INPUT);
  
  // Attach interrupt to the encoder A pin
  attachInterrupt(ENC_A_PIN, encoderISR, RISING);
  attachInterrupt(ENC_PHASE_A, encoderISR, RISING);
  
  //Serial.println("ESP32 Motor Control System Initialized");
}

void loop()
{
    while(1)
    {
    // Wait for the start command (0x54)
    while(Serial.available() == 0) {
      // Wait for data
    }

    receive_data = Serial.read();
    if(!receive_data == 0x54) {
      Serial.write(error_ack); // Failed feedback
      continue;
    } else {
      Serial.write(acknowledge); // Success feedback
    }
    
    // Wait for data read or write command
    while(Serial.available() == 0) {
      // Wait for data
    }
    
    receive_data = Serial.read();
    if(receive_data == 0x55) {  // Read command from master
      // Prepare encoder count data to send back
      data_feedback[0] = (encoderCount >> 8) & 0xFF;        // High byte
      data_feedback[1] = encoderCount & 0xFF;               // Low byte
      data_feedback[2] = (OpticalEncoderCount >> 8) & 0xFF; // High byte
      data_feedback[3] = OpticalEncoderCount & 0xFF;        // Low byte
      
      // Send encoder count back to master
      Serial.write(data_feedback[0]); 
      Serial.write(data_feedback[1]); 
      Serial.write(data_feedback[2]); 
      Serial.write(data_feedback[3]); 

      //send link 2 optical encoder count back to master

    }
    else if(receive_data == 0x56) {  // Write command from master
      Serial.write(acknowledge);
      
      // Read two bytes of motor control data
      uint8_t data_input[2] = {0, 0};
      for(int i = 0; i < 2; i++) {
        while(Serial.available() == 0) {
          // Wait for data
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
      
      // Set motor speed (PWM duty cycle)
      analogWrite(enablePin, abs((int)signed_data));
    }
    else {
      Serial.write(error_ack);
      receive_data = 0;
      continue;
    }
  }

}
