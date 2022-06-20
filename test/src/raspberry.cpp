#include <ctime>
#include <iostream>
#include <string>
#include <time.h>
#include <RF24.h>

#define POWER_MODE RF24_PA_MIN

using namespace std;

RF24 radio(22,0);
byte rf_mode;

void setRF_Mode(char mode) {
  if (mode == 'r') {
    radio.startListening();
    radio.openReadingPipe(0, address);
    rf_mode = 0;
    // Serial.println("Rx Mode");
  }
  else if (mode == 't') {
    radio.stopListening();
    radio.openWritingPipe(address);
    rf_mode = 1;
    // Serial.println("Tx Mode");
  }
  else
    ;// Serial.println("Wrong Command!");
}

int main() {
	if(!radio.begin()){
		cout << "radio not responding!" << endl;
		return 0;
	}
	uint8_t address = "00001";
	radio.setPALevel(POWER_MODE);
	setRF_Mode('t');

	while() {
		// 입력 없으면 ctrl_data에 암것도 안넣고 고
		// 입력모드로 바꿈
		// 센서 데이터 받아서 화면에 출력함
	}
}