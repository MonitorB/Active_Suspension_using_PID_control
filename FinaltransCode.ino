#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CSN, CE
const byte address[6] = "00001";

int x_key = A0;                                               
//int y_key = A0;                                               
int x_pos;
//int y_pos;

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();

  pinMode (x_key, INPUT) ;                     
//  pinMode (y_key, INPUT) ;    
}

void loop() {
  x_pos = analogRead (x_key) ;  
//  y_pos = analogRead (y_key) ;   
  radio.write(&x_pos, sizeof(x_pos));
  Serial.println(x_pos);
  delay(100);
}