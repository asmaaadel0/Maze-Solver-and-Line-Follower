#include <avr/io.h>
#include <util/delay.h>
float Kp = 25;
float Ki = 0;
float Kd = 15;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;

int flag = 0;
// #define MOVING_SPEED 60

int MOVING_SPEED = 60;

//Initial Speed of Motor
int initial_motor_speed = 140;

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

int enA = 10;
int in1 = 9; // left
int in2 = 8;
int enB = 3;
int in4 = 4;  //right
int in3 = 5;
uint8_t black = 1;

//0 --> left far
//1 --> left near
//2 --> right near
//3 --> right far
//1 --> black
//0 --> white

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

void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_I = I;
  previous_error = error;
}

void calc_error()
{

	if(detect_black(left_far) && !detect_black(left_near) && !detect_black(right_near) && !detect_black(right_far))
		error = 3;
	else if(detect_black(left_far) && detect_black(left_near) && !detect_black(right_near) && !detect_black(right_far))
		error = 2;
	else if(!detect_black(left_far) && detect_black(left_near) && !detect_black(right_near) && !detect_black(right_far))
		error = 1;
	else if(!detect_black(left_far) && detect_black(left_near) && detect_black(right_near) && !detect_black(right_far))
		error = 0;
	else if(!detect_black(left_far) && !detect_black(left_near) && detect_black(right_near) && !detect_black(right_far))
		error = -1;
	else if(!detect_black(left_far) && !detect_black(left_near) && detect_black(right_near) && detect_black(right_far))
		error = -2;
	else if(!detect_black(left_far) && !detect_black(left_near) && !detect_black(right_near) && detect_black(right_far))
		error = -3;
	else if(detect_black(left_far) && detect_black(left_near) && detect_black(right_near) && !detect_black(right_far)) // Turn robot left side
		error = 100;
	else if(!detect_black(left_far) && detect_black(left_near) && detect_black(right_near) && detect_black(right_far)) // Turn robot right side
		error = 101;
	else if(!detect_black(left_far) && !detect_black(left_near) && !detect_black(right_near) && !detect_black(right_far)) // Make U turn
		error = 102;
	else if(detect_black(left_far) && detect_black(left_near) && detect_black(right_near) && detect_black(right_far)) // Turn left side or stop
		error = 103;


 
}

void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

  /*Serial.print(PID_value);
    Serial.print("\t");
    Serial.print(left_motor_speed);
    Serial.print("\t");
    Serial.println(right_motor_speed);*/

  analogWrite(enA, left_motor_speed); //Left Motor Speed
  analogWrite(enB, right_motor_speed - 30); //Right Motor Speed

  //following lines of code are to make the bot move forward
  moveForward();
}

void work() {

  if(!detect_black(left_far) && !detect_black(right_far)) {
    Serial.println("Moving Forward.");
    moveForward();
    delay(100);
  }

  if(!detect_black(left_far) && detect_black(right_far)){
    Serial.println("Moving Left.");
    moveLeft();  
    // moveForward();
  }

  if(detect_black(left_far) && !detect_black(right_far)){
    Serial.println("Moving Right.");
    moveRight(); 
    // moveForward();

  }

  // cross case
  if(detect_black(left_far) && detect_black(right_far)){
    Serial.println("Cross.");
   moveForward();
  }

}

void moveForward() {
   //analogWrite(enA, MOVING_SPEED);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  //analogWrite(enB, MOVING_SPEED);
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
  //analogWrite(enA, MOVING_SPEED);
  //analogWrite(enB, 0);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
}

void moveLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  //analogWrite(enA, 0);
  //analogWrite(enB, MOVING_SPEED);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void sharpRightTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
void sharpLeftTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
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

void loop()
{
    // Serial.println(detect_black(left_near));
  calc_error();
  Serial.println(error);
  if (error == 100) {               // Make left turn untill it detects straight path
    //Serial.print("\t");
    //Serial.println("Left");
    do {
      calc_error();
      analogWrite(enA, 80); //Left Motor Speed
      analogWrite(enB, 60); //Right Motor Speed
      moveLeft();
    } while (error != 0);

  } else if (error == 101) {          // Make right turn in case of it detects only right path (it will go into forward direction in case of staright and right "|--")
                                      // untill it detects straight path.
    //Serial.print("\t");
    //Serial.println("Right");
    analogWrite(enA, 80); //Left Motor Speed
    analogWrite(enB, 60); //Right Motor Speed
    moveForward();
    delay(200);
    stop();
    calc_error();
    if (error == 102) {
      do {
        analogWrite(enA, 80); //Left Motor Speed
        analogWrite(enB, 60); //Right Motor Speed
        moveRight();
        calc_error();
      } while (error != 0);
    }
  } 
  else if (error == 102) {        // Make left turn untill it detects straight path
    //Serial.print("\t");
    //Serial.println("Sharp Left Turn");
    do {
      analogWrite(enA, 80); //Left Motor Speed
      analogWrite(enB, 60); //Right Motor Speed
      moveLeft();
      calc_error();
      if (error == 0) {
        stop();
        delay(200);
      }
    } while (error != 0);
  }
  //  else if (error == 103) {        // Make left turn untill it detects straight path or stop if dead end reached.
  //   if (flag == 0) {
  //     analogWrite(enA, 80); //Left Motor Speed
  //     analogWrite(enB, 60); //Right Motor Speed
  //     moveForward();
  //     delay(200);
  //     stop();
  //     calc_error();
  //     if (error == 103) {     /**** Dead End Reached, Stop! ****/
  //       stop();
  //       flag = 1;
  //     } else {        /**** Move Left ****/
  //       analogWrite(enA, 80); //Left Motor Speed
  //       analogWrite(enB, 60); //Right Motor Speed
  //       sharpLeftTurn();
  //       delay(200);
  //       do {
  //         //Serial.print("\t");
  //         //Serial.println("Left Here");
  //         calc_error();
  //         analogWrite(enA, 80); //Left Motor Speed
  //         analogWrite(enB, 60); //Right Motor Speed
  //         sharpLeftTurn();
  //       } while (error != 0);
  //     }
  //   }
  // } 
  else {
    calculate_pid();
    motor_control();
  }
}
