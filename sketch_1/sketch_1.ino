// int enA = 9;
// int in1 = 8;
// int in2 = 7;
// int enB = 3;
// int in3 = 5;
// int in4 = 4;
#include <avr/io.h>
#include <util/delay.h>
#define front_left PC0
#define front_right PC1
#define back_left PC2
#define back_right PC3
#define enA PB
#define enB PB
#define in1 PB
#define in2 PB
#define in3 PB
#define in4 PB
#define temp_VCC_1 13
#define temp_VCC_2 12
void setup()
{
  // put your setup code here, to run once:

  ///////////////
  //////////////////to be removed when we get the small kit/////////////////////
  pinMode(temp_VCC_1, OUTPUT);
  pinMode(temp_VCC_2, OUTPUT);
  /////////////////to be removed when we get the small kit/////////////////////
  /////////////////

  DDRB |= (1 << enA);               // PB0
  DDRB |= (1 << enB);               // PB1
  DDRB |= (1 << in1);               // PB2
  DDRB |= (1 << in2);               // PB3
  DDRB |= (1 << in3);               // PB4
  DDRB |= (1 << in4);               // PB5
  Serial.begin(9600);               // Init Serial at 115200 Baud Rate.
  Serial.println("Serial Working"); // Test to check if serial is working or not
  DDRC &= ~(1 << front_left);       // PC0 left front
  DDRC &= ~(1 << front_right);      // PC1 right front
  DDRC &= ~(1 << back_left);        // PC2 left back
  DDRC &= ~(1 << back_right);       // PC3 right back
}

void moveBackword(int speed)
{
  // speed range from 0~255
  PORTB &= ~(1 << in2);
  PORTB &= ~(1 << in4);
  PORTB |= (1 << in1);
  PORTB |= (1 << in3);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
}
void moveForword(int speed)
{
  // speed range from 0~255
  PORTB &= ~(1 << in1);
  PORTB &= ~(1 << in3);
  PORTB |= (1 << in2);
  PORTB |= (1 << in4);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
}
void StopMoving()
{
  PORTB &= ~(1 << in2);
  PORTB &= ~(1 << in4);
  PORTB &= ~(1 << in1);
  PORTB &= ~(1 << in3);
}
void moveRight(int speed)
{
  PORTB &= ~(1 << in4);
  PORTB &= ~(1 << in1);
  PORTB &= ~(1 << in3);
  PORTB |= (1 << in2);
  analogWrite(enA, speed);
  analogWrite(enB, 0);
}
void moveLeft(int speed)
{
  PORTB &= ~(1 << in2);
  PORTB &= ~(1 << in1);
  analogWrite(enA, 0);
  analogWrite(enB, speed);
  PORTB &= ~(1 << in3);
  PORTB |= (1 << in4);
}
void loop()
{

  // until we get small kit, we have connect the vcc for left sensors on 13
  // and H-bridge on 12
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
