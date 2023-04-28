#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
Servo servo;
RF24 radio(7, 8); // CSN, CE
const byte address[6] = "00001";
int esc = 3;

void setup() {
  Serial.begin(115200);
  radio.begin();
  servo.attach(esc,  1000, 2000);//RX
  servo.write(0);//RX
  delay(2000); 
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    int x_pos ;
    radio.read(&x_pos, sizeof(x_pos));
    Serial.println(x_pos);
    x_pos = constrain(x_pos,550,1023);
    int motorSpeed = map(x_pos, 550, 1023, 0,50 );
    servo.write (motorSpeed) ;
    
  }
}
