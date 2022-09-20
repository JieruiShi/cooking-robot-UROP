#include <Servo.h>
#include <HCSR04.h>
#include "SerialTransfer.h"

//specifies pin numbers
const byte servoPin = 6;
const byte usPowerPin = 8; //virtual 5V pin
//const byte usGroundPin = 7; //virtual 0V pin
const byte saltPin = 7;
const byte triggerPin = 11;
const byte echoPin = 12;


//define variables
unsigned long startTime;
unsigned long endTime;
unsigned long saltStartTime;
unsigned long waterStartTime;
float saltTime; // in seconds
float waterTime; // in seconds
long saltTimeMs; //in milliseconds
long waterTimeMs; //in milliseconds
float distanceInCm;
bool saltOn = false;
bool waterOn = false;
int sendCounter; // only make an US sensor measurement and transfer data through serial per sendGap steps, too frequent sends will hinder main program performance
int sendGap = 20;

//create objects
Servo myservo;  // creates servo object to control water valve
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin); // creates HCSR04 sensor object
SerialTransfer myTransfer; // creates SerialTransfer object

void setup() {
  Serial.begin(115200);
  myTransfer.begin(Serial);
  myservo.attach(servoPin);  // attaches the servo pin to the servo object  
  pinMode(saltPin, INPUT);
  pinMode(usPowerPin, OUTPUT);
  digitalWrite(usPowerPin, HIGH); //Virtual 5V pin

}

void loop() {
//Valve control
//    valveTest();

//Salt Dispenser - relay connection
//    saltDispenserTest();

//US sensor
//distanceSensor.measureDistanceCm()
//    usSensorTest();

//startTime = millis();
//distanceSensor.measureDistanceCm();
//endTime = millis();
//Serial.println(endTime - startTime);
//delay(1000);

step();

}
void turnOnSalt(){
    pinMode(saltPin, OUTPUT);
    analogWrite(saltPin, 0);
    saltStartTime = millis();
    saltOn = true;
  }

void turnOffSalt(){
    pinMode(saltPin, INPUT);
//    digitalWrite(saltPin, LOW);
    saltOn = false;
}

void turnOnWater(int percentage = 100){
    float range = 0.8;
    myservo.write(120 - range * percentage);
//    myservo.write(150);
    waterStartTime = millis();
    waterOn = true;
}

void turnOffWater(){
    myservo.write(120);
    waterOn = false;
}

void valveTest(){
    myservo.write(120);
    delay(1000);
    myservo.write(120);
    delay(1000);
}

void saltDispenserTest(){
    digitalWrite(saltPin, HIGH);
    delay(1000);
    digitalWrite(saltPin, LOW);
    delay(1000);
}

void usSensorTest(){
    Serial.println(distanceSensor.measureDistanceCm());
    delay(1000);
}

void step(){
  // advances a step, takes roughly equal to 10ms, includes all essential actions in this function:
  // Receives salt, water info
  if(myTransfer.available()){
    ((uint8_t*)&saltTime)[0] = myTransfer.packet.rxBuff[0];
    ((uint8_t*)&saltTime)[1] = myTransfer.packet.rxBuff[1];
    ((uint8_t*)&saltTime)[2] = myTransfer.packet.rxBuff[2];
    ((uint8_t*)&saltTime)[3] = myTransfer.packet.rxBuff[3];
    ((uint8_t*)&waterTime)[0] = myTransfer.packet.rxBuff[4];
    ((uint8_t*)&waterTime)[1] = myTransfer.packet.rxBuff[5];
    ((uint8_t*)&waterTime)[2] = myTransfer.packet.rxBuff[6];
    ((uint8_t*)&waterTime)[3] = myTransfer.packet.rxBuff[7];

    if (saltTime > 0){turnOnSalt();saltTimeMs = 1000 * saltTime;}
    else if (saltTime < -10){turnOffSalt();}
    if (waterTime > 0){turnOnWater();waterTimeMs = 1000 * waterTime;}
    else if (waterTime < -10){turnOffWater();}

//    turnOnSalt();
//    turnOnWater();
  }
    
  if (saltOn){if ((millis() - saltStartTime) > saltTimeMs) {turnOffSalt();} } // Turn off when water/salt time is up
  if (waterOn){if ((millis() - waterStartTime) > waterTimeMs) {turnOffWater();} } //Turn off when water/salt time is up

  if (sendCounter >= sendGap){
    sendCounter = 0;
    distanceInCm = distanceSensor.measureDistanceCm();
    myTransfer.packet.txBuff[0] = ((uint8_t*)&distanceInCm)[0];
    myTransfer.packet.txBuff[1] = ((uint8_t*)&distanceInCm)[1];
    myTransfer.packet.txBuff[2] = ((uint8_t*)&distanceInCm)[2];
    myTransfer.packet.txBuff[3] = ((uint8_t*)&distanceInCm)[3];
    myTransfer.packet.txBuff[4] = saltOn;
    myTransfer.packet.txBuff[5] = waterOn;
    myTransfer.sendData(6);
  } else {
    sendCounter ++;
  }

  delay(10);
  }
