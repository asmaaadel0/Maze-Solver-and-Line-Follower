int enA = 9;
int in1 = 8;
int in2 = 7;
int enB = 3;
int in3 = 5;
int in4 = 4;
uint8_t black = 1;
#include <avr/io.h>
#include <util/delay.h>
#define front_left PC0
#define front_right PC1
#define back_left PC2
#define back_right PC3
void setup() {
  // put your setup code here, to run once:
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);

   Serial.begin(9600); // Init Serial at 115200 Baud Rate.
  Serial.println("Serial Working"); // Test to check if serial is working or not
  DDRC &= ~(1<<front_left); //PC0 left front 
  DDRC &= ~(1<<front_right); //PC1 right front 
  DDRC &= ~(1<<back_left); //PC2 left back 
  DDRC &= ~(1<<back_right); //PC3 right back 
}

void moveBackword(int speed){
  // speed range from 0~255
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, speed);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, speed);
}
void moveForword(int speed){
  // speed range from 0~255
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);   
}
void StopMoving(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
void moveRight(int speed){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, speed);
  analogWrite(enB, 0);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
void moveLeft(int speed){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
 analogWrite(enB, speed);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 
}
void loop() {

  //until we get small kit, we have connect the vcc for left sensors on 13
  //and H-bridge on 12 
  digitalWrite(13, HIGH);
  digitalWrite(12, HIGH);

  // moveForword(200);
  // delay(5000);
  // StopMoving();
  // delay(5000);
  // moveBackword(200);
  // delay(5000);
  // moveLeft(100);
  // delay(5000);
  // moveRight(100);
  // delay(500);
   // put your main code here, to run repeatedly:
//  int sensorStatus = ; // Set the GPIO as Input
// bool print = (PINC & (0x1 << front_right) );
// Serial.println( print); 
// if ((PINC & (0x1 << front_left)) && (PINC & (0x1 << front_right))) // Check if the pin high or not 
//   {
//     //continue forword
//     moveForword(50);
//     Serial.println("continue Forword"); // print Motion Detected! on the serial monitor window
//   }
//   else if(PINC & (0x1 << front_left))  {
//     // turn left
//     moveLeft(40);
//     Serial.println("turn left");
//   }
//   else if(PINC & (0x1 << front_right))  {
//     // turn right
//     moveRight(40); 
//     Serial.println("turn right"); 
//   }
//   else {
//     //go forword
//     moveForword(50);
//     Serial.println("Forword"); 
//   }

  moveForword(60);
  delay(1000);
 moveRight(50); 
 delay(1000);
moveLeft(50);
delay(1000);

}