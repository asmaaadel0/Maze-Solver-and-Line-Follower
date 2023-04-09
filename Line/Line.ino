#include <avr/io.h>
#include <util/delay.h>



// Sensors pins
// #define left_far  PC2
#define left_near PC3
// #define right_far PC1
#define right_near PC4
#define center PC5

#define threshold 500
uint8_t ADC_end = 0;
uint16_t adc_value = 0;

// motors
int enA = 10;
int in1 = 9;
int in2 = 8;
int enB = 3;
int in4 = 4;
int in3 = 5;

uint8_t black = 1;
int sensor_reads[3];
int last_error = 0;
float Kp = 0, Ki = 0, Kd = 0;
int forward_speed = 110;
const MAX_SPEED = 200;
const MIN_SPEED = 80;


void adc_init()
{
// AREF = AVcc
ADMUX = (1<<REFS0);
// ADC Enable and prescaler of 128
ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

uint16_t adc_read(uint8_t ch){
    ch &= 0b00000111;
    ADMUX = (ADMUX & 0xF8)|ch;
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    return (ADC);
}

int detect_black(uint8_t pin){
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

void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void move_car(int left_speed, int right_speed){
    

    if (left_speed < 0)
    {
        // left motor backward
        left_speed = -1 * left_speed;
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }else{
        // left motor forward
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }

    if (right_speed < 0)
    {
        // right motor backward
        right_speed = -1 * right_speed;
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    }else{
        // right motor forward
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    }

    // Just for debugging
    if (right_speed < 0 && left_speed < 0)
    {
        Serial.println("Error in calculations!!!!!!!!!!!!");
    }


    analogWrite(enA, left_speed);
    analogWrite(enB, right_speed);
}

int detect_line() {

  sensor_reads[0] = detect_black(left_near);
  sensor_reads[1] = detect_black(center);
  sensor_reads[2] = detect_black(right_near);

  reads_weight = 1000 * sensor_reads[0] + 2000 * sensor_reads[1] + 3000 * sensor_reads[2];
  reads_sum    = sensor_reads[0] + sensor_reads[1] + sensor_reads[2];

  return reads_weight / reads_sum;
}

void change_motors_speed(int change_value){

    int motor_a_speed = forward_speed + change_value;        // A : left motor
    int motor_b_speed = forward_speed - change_value;       //  B : right motor

    if (motor_a_speed > MAX_SPEED) motor_a_speed = MAX_SPEED;
    if (motor_b_speed > maxspeedb) motor_b_speed = MAX_SPEED;
    if (motor_a_speed < (-1 * MIN_SPEED)) motor_a_speed = -1 * MIN_SPEED;
    if (motor_b_speed < (-1 * MIN_SPEED)) motor_b_speed = -1 * MIN_SPEED;
    
    move_car(motor_a_speed, motor_b_speed);
}

void PID(){
    const BEST_CASE = 2000;

    int current_position = detect_line();
    int error = (BEST_CASE - (int)current_position);
    /*
    * if the error = 0 --> correct moving 
    * if the error > 0 --> the car is moving left, so we need to turn right
    * if the error < 0 --> the car is moving right, so we need to turn left
    */

   // PID parameters
   int  P = error,
        I = I + error,
        D = last_error - error;

    last_error = error;

    int PID_value  = P * Kp + I * Ki + D * Kd;
    change_motors_speed(PID_value);
    
}

void setup() {

  DDRC &= ~(1<<center); //PC5 center

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(7, OUTPUT);   // VCC for testing
  pinMode(13, OUTPUT);  // VCC for testing
  pinMode(center, INPUT); // center sensor

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

  // sensors
  adc_init();
}

void loop() {
    PID();
}



// int detect_black(uint8_t pin) {
//   adc_value = adc_read(pin);
//     if (adc_value != -1) {
//     if (adc_value < threshold)
//       // black
//       return 1;
//     else
//       // white
//       return 0;
//   } else
//     return 2;
// }




// void moveForward(int speed) {
//   analogWrite(enA, speed);
//   digitalWrite(in1, LOW);
//   digitalWrite(in2, HIGH);
//   analogWrite(enB, speed);
//   digitalWrite(in3, LOW);
//   digitalWrite(in4, HIGH);
// }

// void moveRight(int left_speed, int right_speed) {
//   // left motor forward
//   digitalWrite(in1, LOW);
//   digitalWrite(in2, HIGH);
//   analogWrite(enA, left_speed);

//   // right motor backward
//   analogWrite(enB, right_speed);
//   digitalWrite(in3, HIGH);
//   digitalWrite(in4, LOW);
  
// }

// void moveLeft(int left_speed, int right_speed) {
//   // left motor backward
//   digitalWrite(in1, HIGH);
//   digitalWrite(in2, LOW);
//   analogWrite(enA, left_speed);

//   // right motor forward
//   analogWrite(enB, right_speed);
//   digitalWrite(in3, LOW);
//   digitalWrite(in4, HIGH);
// }


// uint16_t adc_read(uint8_t ch){
//     // select the corresponding channel 0~5 ANDing with ’7′ will always keep the value of ‘ch’ between 0 and 5
//     ch &= 0b00000111; // AND operation with 7
//     ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing start single conversion write ’1′ to ADSC
//     ADCSRA |= (1<<ADSC); // wait for conversion to complete // ADSC becomes ’0′ again // till then, run loop continuously
//     while(ADCSRA & (1<<ADSC));
//     return (ADC);
// }
