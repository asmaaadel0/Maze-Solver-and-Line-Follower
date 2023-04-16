#include <avr/io.h>
#include <util/delay.h>


// #define MOVING_SPEED 60

int MOVING_SPEED = 90;

// 0 : left_most
// 1 : left
// 2 : front
// 3 : back
// 4 : right
// 5 : right_most
#define left_far  PC2
#define left_near PC3
#define right_far PC1
#define right_near PC4
#define center PC5

#define threshold 500
uint8_t ADC_end = 0;
uint16_t adc_value = 0;
// sensors
// int counterLeft=0;
// int counterRight=0;
int enA = 10;
int in1 = 9;
int in2 = 8;
int enB = 3;
int in4 = 4;
int in3 = 5;
uint8_t black = 1;

void adc_init()
{
// AREF = AVcc
ADMUX = (1<<REFS0);
// ADC Enable and prescaler of 128
ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

uint16_t adc_read(uint8_t ch)
{
// select the corresponding channel 0~5
// ANDing with ’7′ will always keep the value
// of ‘ch’ between 0 and 5
ch &= 0b00000111; // AND operation with 7
ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
// start single conversion
// write ’1′ to ADSC
ADCSRA |= (1<<ADSC);
// wait for conversion to complete
// ADSC becomes ’0′ again
// till then, run loop continuously
while(ADCSRA & (1<<ADSC));
return (ADC);
}

int detect_black(uint8_t pin) {
  adc_value = adc_read(pin);
  if (adc_value != -1) {
    if (adc_value < threshold)
      // black
      return 1;
    else
      // white
      return 0;
  } else
    return 2;
}

void work() {
  if(!detect_black(left_near) && !detect_black(right_near)) {
    Serial.println("Moving Forward.");
      // counterRight=0;
      // counterLeft=0;
    moveForward();
    // delay(100);
  }

  if(!detect_black(left_near) && detect_black(right_near)){
    Serial.println("Moving Left.");
    // counterLeft++;
    // if(counterLeft>=500){
    //     counterLeft=0;
    // delay(10);
        moveRightError();  
        // while(detect_black(left_near) || detect_black(right_near)){
        //   // readSensors();
        // }
    // }
    // moveForward();
  }

  if(detect_black(left_near) && !detect_black(right_near)){
    Serial.println("Moving Right.");
    // counterRight++;
    // if(counterRight>=500){
    //     counterRight=0;
    // delay(10);

        moveLeftError(); 
        // while(detect_black(left_near) || detect_black(right_near)){
        //   // readSensors();
        // }
    // }
    // moveForward();

  }

  // cross case
  if(detect_black(left_near) && detect_black(right_near)){
    Serial.println("Cross.");
      //   counterRight=0;
      // counterLeft=0;
    moveForward();
  }

}

void moveForward() {
   analogWrite(enA, MOVING_SPEED+15);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enB, MOVING_SPEED+15);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void moveRight() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, MOVING_SPEED);
  analogWrite(enB, 0);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
}
void moveRightError() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, MOVING_SPEED-40);
  analogWrite(enB, 0);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);  


}
void moveLeftError() {
  analogWrite(enA, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enB, MOVING_SPEED-40);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  

}
void moveLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 0);
  analogWrite(enB, MOVING_SPEED);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void setup() {

  DDRC &= ~(1<<center); //PC5 center

  // put your setup code here, to run once:
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(12, OUTPUT);
  // pinMode(2, OUTPUT);   // GND for testing
  pinMode(7, OUTPUT);   // VCC for testing
  pinMode(13, OUTPUT);  // VCC for testing

  digitalWrite(7, HIGH);
  digitalWrite(13, HIGH);
  digitalWrite(2, LOW);

  // motors
  DDRB |= (1 << enA);  // PB0 out
  DDRB |= (1 << enB);  // PB1
  DDRB |= (1 << in1);  // PB2
  DDRB |= (1 << in2);  // PB3
  DDRB |= (1 << in3);  // PB4
  DDRB |= (1 << in4);  // PB5

  Serial.begin(9600);
  
  // center sensor
  pinMode(center, INPUT);

  // sensors
  adc_init();
}


void loop() {
  // moveLeft();
  // Serial.println('left : ');
  Serial.println(detect_black(right_far));
  // Serial.println(detect_black(left_near));
  // Serial.println(detect_black(right_near));
  // delay(1000);
  
  // Serial.println('right : ');
  // delay(1000);
// moveForward();
// delay(1000);
// moveRight();
// delay(1000);
// moveLeft();
// delay(1000);

  // readSensors();
work();
  // Serial.println(detect_black(left_far));
  // Serial.println(detect_black(right_near));
// 
  // Serial.print(detect_black(left_near));
   
  // Serial.print('left sensor : ', detect_black(left_near));
  // Serial.print('right sensor : ', detect_black(right_near));
  // delay(1000);

}