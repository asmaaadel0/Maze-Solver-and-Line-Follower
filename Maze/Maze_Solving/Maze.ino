#include <avr/io.h>
#include <util/delay.h>

// #define front_left PC0
// #define front_right PC1
// #define back_left PC2
// #define back_right PC3

#define MOVING_SPEED 60


// possible cases
#define STRAIGHT "00100"
#define LEFT_TURN "11000"
#define RIGHT_TURN "00011"
#define NORMAL_T "11011"
#define LEFT_T "11100"
#define RIGHT_T "00111"
#define CROSS "11111"
#define U_TURN "00000"


// 0 : left_most
// 1 : left
// 2 : front
// 3 : back
// 4 : right
// 5 : right_most
String sensor_reading = "00000";  // initial value
String path = "";
#define left_far PC0
#define left_near PC1
#define right_far PC4
#define right_near PC3
#define center PC2

#define threshold 500
uint8_t ADC_end = 0;
uint16_t adc_value = 0;
// sensors

int enA = 9;
int in1 = 8;
int in2 = 7;
int enB = 3;
int in4 = 4;
int in3 = 5;
uint8_t black = 1;

// void adc_init(void) {
//   ADMUX |= (1 << REFS0);                                 // Set ADC reference to AVCC
//   ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Set ADC prescaler to 128 - 125KHz sample rate @ 16MHz
//   ADCSRA |= (1 << ADEN);                                 // Enable ADC
//   ADCSRA |= (1 << ADIE);                                 // Enable ADC Interrupt
//   //ADCSRA |= (1 << ADSC); 
//   sei();                                // Start A2D Conversions
// }


// ISR(ADC_vect) {
//   ADC_end = 1;
// }


// uint16_t read_adc(uint8_t ch) {
//   // select the corresponding channel 0~7
//   // ANDing with ’7′ will always keep the value
//   // of ‘ch’ between 0 and 7
//   ch &= 0b00000111;             // AND operation with 7
//   ADMUX = (ADMUX & 0xF8) | ch;  // clears the bottom 3 bits before ORing
//                                 // start single convertion
//   // write ’1′ to ADSC
//   ADCSRA |= (1 << ADSC);
//   if (ADC_end == 1) {
//     ADC_end = 0;
//     //Serial.println(ADC);
//     return ADC;
//   } else
//     return -1;
// }

void adc_init()
{
// AREF = AVcc
ADMUX = (1<<REFS0);
// ADC Enable and prescaler of 128
// 16000000/128 = 125000
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




char detect_black(uint8_t pin) {
  adc_value = adc_read(pin);
  if (adc_value != -1) {
    if (adc_value < threshold)
      // black
      return '1';
    else
      // white
      return '0';
  } else
    return '2';
}


void readSensors() {
  // put your main code here, to run repeatedly:
  uint16_t adc_result0, adc_result1;
  // DDRB = 0x20; // to connect led to PB5

  // initialize adc
  adc_init();
  sei();

  sensor_reading[0] = detect_black(left_far);
  sensor_reading[1] = detect_black(left_near);
  sensor_reading[2] = detect_black(center);
  sensor_reading[3] = detect_black(right_near);
  sensor_reading[4] = detect_black(right_far);
  // sensor_reading[5] = digitalRead();
  //}
}


void work() {

  // normal case
  if (sensor_reading.equals(STRAIGHT)) {
    Serial.println("Moving Forward.");
    moveForward();
    path += "S";
  }
  // left turn
  else if (sensor_reading.equals(LEFT_TURN)) {
    Serial.println("Moving Left. (LEFT_TURN case)");
    moveLeft();
    path += "L";
  }
  // left-T
  else if (sensor_reading.equals(LEFT_T)) {
    Serial.println("Moving Left. (LEFT_T case)");
    moveLeft();
    path += "L";
  }
  // right turn
  else if (sensor_reading.equals(RIGHT_TURN)) {
    Serial.println("Moving Right. (RIGHT_TURN case)");
    moveRight();
    path += "R";
  }
  // right-T
  else if (sensor_reading.equals(RIGHT_T)) {
    Serial.println("Moving Right. (RIGHT_T case)");
    moveRight();
    path += "R";
  }
  // T case
  else if (sensor_reading.equals(NORMAL_T)) {
    Serial.println("Moving Left. (NORMAL_T case)");
    moveLeft();
    path += "L";
  }
  // cross case
  else if (sensor_reading.equals(CROSS)) {
    // TODO: End case might be here, need to be handled
    Serial.println("Moving Left. (CROSS case)");
    moveLeft();
    path += "L";
  }
  // U turn case
  else if (sensor_reading.equals(U_TURN)) {
    Serial.println("Moving Left. (U_TURN case)");
    moveBackward();
    path += "B";
  }
}


void moveBackward() {
  // speed range from 0~255
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, MOVING_SPEED);
  digitalWrite(in4, LOW);
  digitalWrite(in3, HIGH);
  analogWrite(enB, MOVING_SPEED);
}


void moveForward() {
  // speed range from 0~255
  analogWrite(enA, MOVING_SPEED);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enB, MOVING_SPEED);
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
  digitalWrite(in4, LOW);
}


void moveLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, MOVING_SPEED);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}


String optimizeThePath(String path) {
  Serial.println("Optimizing the path.");

  path.replace("LBL", "S");
  path.replace("LBS", "R");
  path.replace("RBL", "B");
  path.replace("SBS", "B");
  path.replace("SBL", "R");
  path.replace("LBR", "B");

  // Serial.println("The optimized path is : ", path);
  return path;
}



void setup() {
  // put your setup code here, to run once:
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(12, OUTPUT);

  //adc_init();
  //sei();  // Enable global interrupts

  pinMode(2, OUTPUT);   // GND for testing
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
  // pinMode(left_near, INPUT);
  // pinMode(left_far, INPUT);
  // pinMode(right_near, INPUT);
  // pinMode(right_far, INPUT);
  // sensors
}


void loop() {
  readSensors();
  Serial.println(sensor_reading);
  work();
  //moveForward();
  //moveBackward();
  //moveRight();
  //moveLeft();
  //stop();
}
