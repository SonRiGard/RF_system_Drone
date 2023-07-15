/*
#include <SPI.h>
#include "printf.h"
#include "RF24.h"

void Send_NRF_value (void);
void Init_NRF24(void);
RF24 radio(7,8);  // using pin 7 for the CE pin, and pin 8 for the CSN pin
uint8_t address[][6] = { "1Node", "2Node" };
bool radioNumber = 0;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit
bool role = true;  // true = TX role, false = RX role
float payload[5];


const int analog_pin1 = A1;   //Naming analog input pin//toi-800    - lui 0
const int analog_pin2 = A0;   //Naming analog input pin//trais =0 ;  phai=800
const int analog_pin3 = A3;   //Naming analog input pin// goc yall
const int analog_pin4 = A2;   //Naming analog input pin//do cao

int PIN_button_power_on=5;

float inputVal1  = 0;          //Variable to store analog input values
float inputVal2  = 0;          //Variable to store analog input values
float inputVal3  = 0;          //Variable to store analog input values
float inputVal4  = 0;          //Variable to store analog input values
float inputVal5  = 0;          //Variable to power ON/OFF 1: ON ; 0:OFF Drone continue transmission

void setup() {
  Serial.begin(115200);
  pinMode(PIN_button_power_on, INPUT);
  Init_NRF24();
}  // setup

void loop() {
  inputVal1 = analogRead (analog_pin1);
  inputVal2 = analogRead (analog_pin2);
  inputVal3 = analogRead (analog_pin3);
  inputVal4 = analogRead (analog_pin4);
  if (digitalRead(PIN_button_power_on) == HIGH ){
      inputVal5 = 1; 
  }else inputVal5 = 0;
  Send_NRF_value();
  Serial.print(inputVal1);
  Serial.print(" ");
  Serial.print(inputVal2);
  Serial.print(" ");
  Serial.print(inputVal3);
  Serial.print(" ");
  Serial.print(inputVal4);
  Serial.print("");
  Serial.print(inputVal5);
  Serial.println("");
}  // loop

void Init_NRF24(void){
    if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
  radioNumber  = 0 ;
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  radio.setPayloadSize(20);  // float datatype occupies 4 bytes
  radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1
  radio.stopListening();  // put radio in TX mod
}

void Send_NRF_value (void){//
      // This device is a TX node
      payload[0] = inputVal1;          // increment float payload
      payload[1] = inputVal2;
      payload[2] = inputVal3;
      payload[3] = inputVal4;
      payload[4] = inputVal5;
    unsigned long start_timer = micros();                // start the timer
    bool report = radio.write(&payload, sizeof(payload));  // transmit & save the report
    unsigned long end_timer = micros();                  // end the timer

    if (report) {
      Serial.print(F("Transmission successful! "));  // payload was delivered
      Serial.print(F("Time to transmit = "));
      Serial.print(end_timer - start_timer);  // print the timer result
      Serial.print(F(" us. Sent: "));
      Serial.println(payload[0]);  // print payload sent
    } else {
      Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
    }
    //delay(10);  // slow transmissions down by 1 second
}
*/

/*
 * See documentation at https://nRF24.github.io/RF24
 * See License information at root directory of this library
 * Author: Brendan Doherty (2bndy5)
 */

/**
 * A simple example of sending data from 1 nRF24L01 transceiver to another.
 *
 * This example was written to be used on 2 devices acting as "nodes".
 * Use the Serial Monitor to change each node's behavior.
 */
#include <SPI.h>
#include "printf.h"
#include "RF24.h"

void Send_NRF_value (void);
void Init_NRF24(void);
RF24 radio(7,8);  // using pin 7 for the CE pin, and pin 8 for the CSN pin
uint8_t address[][6] = { "1Node", "2Node" };
bool radioNumber = 0;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit
bool role = true;  // true = TX role, false = RX role
float payload[5];

const int analog_pin1 = A1;   //Naming analog input pin//toi-800    - lui 0
const int analog_pin2 = A0;   //Naming analog input pin//trais =0 ;  phai=800
const int analog_pin3 = A3;   //Naming analog input pin// goc yall
const int analog_pin4 = A2;   //Naming analog input pin//do cao

int PIN_button_power_on=5;

float inputVal1  = 0;          //Variable to store analog input values
float inputVal2  = 0;          //Variable to store analog input values
float inputVal3  = 0;          //Variable to store analog input values
float inputVal4  = 0;          //Variable to store analog input values
float inputVal5  = 0;          //Variable to power ON/OFF 1: ON ; 0:OFF Drone continue transmission

void setup() {
  Serial.begin(115200);
  pinMode(PIN_button_power_on, INPUT);
  //Init_NRF24();
}  // setup

void loop() {
  inputVal1 = analogRead (analog_pin1);Serial.print(inputVal1);Serial.print(",");
  inputVal2 = analogRead (analog_pin2);Serial.print(inputVal2);Serial.print(",");
  inputVal3 = analogRead (analog_pin3);Serial.print(inputVal3);Serial.print(",");
  inputVal4 = analogRead (analog_pin4);Serial.println(inputVal4);

  if (digitalRead(PIN_button_power_on) == HIGH ){
      inputVal5 = 1; 
  }else inputVal5 = 0;
  Send_NRF_value();
  delay(1000);
}  // loop

void Init_NRF24(void){
    if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  radio.setPayloadSize(20);  // float datatype occupies 4 bytes
  radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1
  radio.stopListening();  // put radio in TX mod
}

void Send_NRF_value (void){//
      // This device is a TX node
      payload[0] = map( inputVal1,74,825,1000,2000);          // increment float payload
      payload[1] = map( inputVal2,55,798,-90,90);
      payload[2] = map( inputVal3,40,778,-90,90);
      payload[3] = map( inputVal4,41,802,-90,90);
      payload[4] = inputVal5;
    unsigned long start_timer = micros();                // start the timer
    bool report = radio.write(&payload, sizeof(payload));  // transmit & save the report
    unsigned long end_timer = micros();                  // end the timer

    if (report) {
      Serial.print(F("Transmission successful! "));  // payload was delivered
      Serial.print(F("Time to transmit = "));
      Serial.print(end_timer - start_timer);  // print the timer result
      Serial.print(F(" us. Sent: "));
      Serial.println(payload[0]);  // print payload sent
    } else {
      Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
    }
}


