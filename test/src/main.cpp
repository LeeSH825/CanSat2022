// Library
#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

// constexpr int c_strcmp( char const* lhs, char const* rhs )
// {
//     return (('\0' == lhs[0]) && ('\0' == rhs[0])) ? 0
//         :  (lhs[0] != rhs[0]) ? (lhs[0] - rhs[0])
//         : c_strcmp( lhs+1, rhs+1 );
// }

// #define STATION "station"

// #define BOARD_TYPE STATION

//#if 0 == c_strcmp( BOARD_TYPE , STATION )
// #define 

// #endif

//Flags
#define STATION
#define CANSAT

#define CE_PIN 8
#define CSN_PIN 10

#define POWER_MODE RF24_PA_MIN


// Data Structure
struct ctrl_status {
  int gyro_x;
  int gyro_y;
  int gyro_z;

  int accel_x;
  int accel_y;
  int accel_z;

  int temp;
};

struct ctrl_sig {
  int pos_servo1;
  int pos_servo2;
};

// Pin Configuration
// RF24 radio(8, 10); // CE: Digital(8), CSN: Digital(10)
RF24 radio(CE_PIN, CSN_PIN);

// Common Variable
const byte address[6] = "00001";

// Function

void setRF_Mode(char mode)
{
  if (mode == 't') {
    radio.stopListening();
    radio.openWritingPipe(address);
    Serial.println("Tx Mode");
  }
  else if (mode == 'r') {
    radio.startListening();
    radio.openReadingPipe(0, address);
    Serial.println("Rx Mode");
  }
  else
    Serial.println("Wrong Command!");
}

void sendData()
{

}


void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(POWER_MODE);
  #ifdef CANSAT
  // 처음 시작은 Tx모드로 
  setRF_Mode('t');
  #endif

  #ifdef STATION
  // 처음 시작은 Rx 모드로
  setRF_Mode('r');
  #endif
  // setRF_Mode()
}
void loop() {
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    Serial.println(text);
  }

  //
}