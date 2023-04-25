# include "drivers.h"
/*!
 * @brief IR Sensors
 */
#define IR_L A3
#define IR_C A2
#define IR_R A4
#define IR_LL A5
#define IR_RR A1

#define MOTOR_L_B 2      // in1
#define MOTOR_L_F 8      // in2
#define MOTOR_R_F 4      // in4
#define MOTOR_R_B 5      // in3
#define MOTOR_R_SPEED 3  // enB
#define MOTOR_L_SPEED 9  // 10 // enA


/*!
 * @brief Thresholds
 */
#define THRESHOLD_W 500  // above it is white and below it all black
#define THRESHOLD_B 150  // above it is white and below it all black
#define ThresholdDiff 200
#define ThresholdDiff_W 150
// #define SPEEDSTEERING 70  // steering speed
#define NUM_OF_ERROR 70   // thershold for accepted errors
#define SPEEDFORWARD 110  //95   // forward speed

/*!
 * @brief PID Constants
 */
#define kp 0.05
#define ki 0.0000004
#define kd .02  //1.5

String path = "";

// #define kp 1.5
// #define ki .5
// #define kd .3  //1.5

/*!
 * @brief Global variables for the PID
 */
float eCurrent = 0;
float ePrev = 0;
long prevT = 0;
float eIntegral = 0;

/*!
 * @brief Gllobal variables to handle the direction errors
 */
int lastDir = 0;
int error = 0;
boolean dir = false;

/*!
 * @brief it is a function  
 */
void moveMotor2(int speedR, int speedL, int rDir, int lDir) {
  // rDir *= -1;
  // lDir *= -1;
  if (millis() % 200 > 120) {
    avr_analogWrite(MOTOR_R_SPEED, 0);
    avr_analogWrite(MOTOR_L_SPEED, 0);
    return;
  }
  avr_analogWrite(MOTOR_R_SPEED, speedR);
  avr_analogWrite(MOTOR_L_SPEED, speedL);
  if (rDir == 1) {
    avr_digitalWrite(MOTOR_R_F, HIGH);
    avr_digitalWrite(MOTOR_R_B, LOW);
  } else {
    avr_digitalWrite(MOTOR_R_F, LOW);
    avr_digitalWrite(MOTOR_R_B, HIGH);
  }
  if (lDir == 1) {
    avr_digitalWrite(MOTOR_L_F, HIGH);
    avr_digitalWrite(MOTOR_L_B, LOW);
  } else {
    avr_digitalWrite(MOTOR_L_F, LOW);
    avr_digitalWrite(MOTOR_L_B, HIGH);
  }
  if (rDir == 1 && lDir == 1) {
    if (strcmp(path[path.length() - 1], "S"))
      path += "S";
  }
}

/*!
 * @brief the setup code
 * @return void
*/
void setup() {
  // set motor pins as output
  Serial.begin(9600);
  avr_pinMode(MOTOR_R_F, OUTPUT);
  avr_pinMode(MOTOR_R_B, OUTPUT);
  avr_pinMode(MOTOR_L_F, OUTPUT);
  avr_pinMode(MOTOR_L_B, OUTPUT);

  // set motor direction
  avr_digitalWrite(MOTOR_R_F, HIGH);
  avr_digitalWrite(MOTOR_L_F, HIGH);
  avr_digitalWrite(MOTOR_R_B, LOW);
  avr_digitalWrite(MOTOR_L_B, LOW);
}
// uint16_t adc_read(uint8_t ch)
// {
// // select the corresponding channel 0~5
// // ANDing with ’7′ will always keep the value
// // of ‘ch’ between 0 and 5
// ch &= 0b00000111; // AND operation with 7
// ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
// // start single conversion
// // write ’1′ to ADSC
// ADCSRA |= (1<<ADSC);
// // wait for conversion to complete
// // ADSC becomes ’0′ again
// // till then, run loop continuously
// while(ADCSRA & (1<<ADSC));
// return (ADC);
// }


char detect_black(uint8_t pin) {
  float adc_value = avr_analogRead(pin);
  if (adc_value != -1) {
    if (adc_value < THRESHOLD_W)
      // black
      return '1';
    else
      // white
      return '0';
  } else
    return '2';
}
//  String sensor_reading='00000';
void readSensors() {
  // put your main code here, to run repeatedly:
  uint16_t adc_result0, adc_result1;
  // DDRB = 0x20; // to connect led to PB5

  // initialize adc
  // adc_init();
  // sei();

  // sensor_reading[0] = detect_black(IR_LL);
  // sensor_reading[1] = detect_black(IR_L);
  // sensor_reading[2] = detect_black(IR_C);
  // // sensor_reading[2] = digitalRead(2);    // for digital sensor
  // sensor_reading[3] = detect_black(IR_R);
  // sensor_reading[4] = detect_black(IR_RR);
  //}
}
int r_rr, l_ll, c_cc;
void loop() {

  // try moving forward

  // avr_digitalWrite(MOTOR_R_B, LOW);
  // avr_digitalWrite(MOTOR_L_B, LOW);
  // avr_digitalWrite(MOTOR_L_F, HIGH);
  // avr_digitalWrite(MOTOR_R_F, HIGH);
  // avr_analogWrite(MOTOR_R_SPEED, 100);
  // avr_analogWrite(MOTOR_L_SPEED, 100);


  r_rr = (avr_analogRead(IR_R));  // > THRESHOLD_W)? 1 : 0;
  l_ll = (avr_analogRead(IR_L));  // > THRESHOLD_W)? 1 : 0;
  c_cc = (avr_analogRead(IR_C));  // > THRESHOLD_W)? 1 : 0;

  //   Serial.println("left");
  //   Serial.println(l_ll);

  //   delay(1000);
  //   Serial.println("right");
  //   Serial.println(r_rr);
  //  delay(1000);
  //   Serial.println("center");
  //   Serial.println(c_cc);
  //    delay(1000);
  /////////////////////////////////////////////////////
  movecar2();
  // Serial.println(path);
  //moveMotor(SPEEDFORWARD, SPEEDFORWARD);
  //moveMotor2(SPEEDFORWARD,SPEEDFORWARD,1,-1);
  //  avr_analogRead(IR_R) > THRESHOLD_W
  //   && avr_analogRead(IR_C) < THRESHOLD_W;
  //   && avr_analogRead(IR_L) > THRESHOLD_W);
  // Serial.print('Right');
  //   Serial.println(avr_analogRead(IR_R) > THRESHOLD_W);
  //   delay(1000);
  // Serial.print('center');

  //   Serial.println(avr_analogRead(IR_C) < THRESHOLD_W);
  //   delay(1000);

  // Serial.print('left');

  //   Serial.println(avr_analogRead(IR_L) > THRESHOLD_W);
  //   delay(1000);
  //  Serial.println(sensor_reading);
  /////////////////////////////////////////////////////
}
int lIsBlack = 0;
int rIsBlack = 0;
int rrIsBlack = 0;
int llIsBlack = 0;

int lIsWhite = 0;
int rIsWhite = 0;
int cIsWhite = 0;
int llIsWhite = 0;
int rrIsWhite = 0;

unsigned long lastRight = 0;

int onLine() {
  return (
    avr_analogRead(IR_R) > THRESHOLD_W
    && avr_analogRead(IR_C) < THRESHOLD_W
    && avr_analogRead(IR_L) > THRESHOLD_W);
  //    return (
  //      avr_analogRead(IR_R) < THRESHOLD_W
  //      || avr_analogRead(IR_C)< THRESHOLD_W
  //      || avr_analogRead(IR_L) < THRESHOLD_W
  //      );
}
int kam2 = 9000;


void rotateRight() {
  // moveMotor2(0, 0, 1, 1);
  // moveMotor2(SPEEDFORWARD, 0, -1, -1);
  // moveMotor2(0, 0, 1, 1);
  // int mincnt = kam2;
  // while (mincnt > 0 || !onLine()) {
  //   moveMotor2(SPEEDFORWARD, SPEEDFORWARD, -1, 1);
  //   mincnt--;  
  int mincnt = kam2;
  while (mincnt > 0 || !onLine()) {
    moveMotor2(SPEEDFORWARD, SPEEDFORWARD, -1, 1);
    mincnt--;
  }
  //   moveMotor2(0, 0, 1, 1);
  // moveMotor2(SPEEDFORWARD, SPEEDFORWARD, -1, -1);
  // moveMotor2(0, 0, 1, 1);
  // int mincnt = kam2;
  // while (mincnt > 0 || !onLine()) {
  //   moveMotor2(SPEEDFORWARD, 0, -1, 1);
  //   mincnt--;
  // }
}


void rotateLeft() {
  moveMotor2(0, 0, 1, 1);
  moveMotor2(SPEEDFORWARD, SPEEDFORWARD, -1, -1);
  moveMotor2(0, 0, 1, 1);
  int mincnt = kam2;
  while (mincnt > 0 || !onLine()) {
    moveMotor2(SPEEDFORWARD, 0, 1, -1);
    mincnt--;
  }
}


void rotate180() {
  int mincnt = kam2;
  while (mincnt > 0 || !onLine()) {
    moveMotor2(SPEEDFORWARD, SPEEDFORWARD, 1, -1);
    mincnt--;
  }
}

void movecar2() {
  float LL_Reading = avr_analogRead(IR_LL);
  float RR_Reading = avr_analogRead(IR_RR);
  float C_Reading = avr_analogRead(IR_C);
  float L_Reading = avr_analogRead(IR_L);
  float R_Reading = avr_analogRead(IR_R);

  int C = C_Reading > THRESHOLD_W;
  int L = L_Reading > THRESHOLD_W;
  int R = R_Reading > THRESHOLD_W;
  int LL = LL_Reading > THRESHOLD_W;
  int RR = RR_Reading > THRESHOLD_W;

  if (!LL) llIsBlack++, llIsWhite = 0;
  else llIsBlack = 0, llIsWhite++;

  if (!RR) rrIsBlack++, rrIsWhite = 0;
  else rrIsBlack = 0, rrIsWhite++;

  if (!L) lIsBlack++, lIsWhite = 0;
  else lIsBlack = 0, lIsWhite++;

  if (!R) rIsBlack++, rIsWhite = 0;
  else rIsBlack = 0, rIsWhite++;

  if (!C) cIsWhite = 0;
  else cIsWhite++;

  int kam = 100;
  if (llIsBlack >= kam) {
    rotateLeft();
    if (strcmp(path[path.length() - 1], "L"))
      path += "L";
  }
  // else if(rrIsBlack >= kam)
  // {
  //   rotateRight();
  //   if (strcmp(path[path.length() - 1], "R")) {
  //     path += "R";
  // } }
  
  else if (cIsWhite >= kam && rIsWhite >= kam && lIsWhite >= kam) {
    if (millis() - lastRight < 1000) {
      rotateRight();
      if (strcmp(path[path.length() - 1], "R")) {
        path += "R";
      } 
    }
    else {
        rotate180();
        if (strcmp(path[path.length() - 1], "B"))
          path += "B";
      }
  }

  // if(cIsWhite > 0 && rIsWhite > 0 && lIsWhite > 0 && llIsWhite > 0 && rrIsWhite > 0)
  // {
  //   rotate180();
  //     if (strcmp(path[path.length() - 1], "B"))
  //       path += "B";
  // }  

  if (rrIsBlack >= kam)
    lastRight = millis();

  movecar();
}


float differential_steering(float left_align, float c, float right_align) {
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;

  eCurrent = 0 - (left_align - right_align);
  ePrev = (eCurrent - ePrev) / deltaT;
  eIntegral += eCurrent * deltaT;

  float delta_v = kp * eCurrent + ki * eIntegral + kd * ePrev;

  ePrev = eCurrent;
  prevT = currT;
  return delta_v;
}
// void read(){
//  sensor_reading[0] = detect_black(left_far);
//   sensor_reading[1] = detect_black(left_near);
//   sensor_reading[2] = avr_analogRead(IR_C);
//   // sensor_reading[2] = digitalRead(2);    // for digital sensor
//   sensor_reading[3] = detect_black(right_near);
//   sensor_reading[4] = detect_black(right_far);
// }
void movecar() {
  // read the sensors
  float C = avr_analogRead(IR_C);
  float L = avr_analogRead(IR_L);
  float R = avr_analogRead(IR_R);

  // calc the steering angle usign the PID controller
  float PIDError = differential_steering(L, C, R);

  // check the switch pin to switch between code : 1- if conditions  2- pid with if conditions
  // if(digitalRead(SWITCH) == HIGH)
  // {
  // PIDError = 0.0;
  // }
  // check if reached the end point
  //  if(C<THRESHOLD_B && L < THRESHOLD_B && R< THRESHOLD_B){
  //    moveMotor(0, 0);
  //    }

  // check if e have drifted out of the line
  // explore the surroundings to return back and count your fault moves
  if (C > THRESHOLD_W && abs(L - R) < ThresholdDiff_W) {
    if (error == NUM_OF_ERROR)
      dir = !dir;
    if (!dir) {
      moveMotor(0, SPEEDFORWARD);
      error++;
    } else {
      moveMotor(SPEEDFORWARD, 0);
      error--;
    }
  }
  // decide the direction based on the sensors readings and the velocity based on the PID controller
  else {
    if (L - R > ThresholdDiff)  //Left more in white than R
    {
      moveMotor(0, SPEEDFORWARD + abs(PIDError));
    } else if (ThresholdDiff < R - L)  //Right more in white than L
    {
      moveMotor(SPEEDFORWARD + abs(PIDError), 0);
    } else  // forward speed is faster than the steering speed
    {
      moveMotor(SPEEDFORWARD, SPEEDFORWARD);
    }
  }
}


void moveMotor(int speedR, int speedL) {
  moveMotor2(speedR, speedL, 1, 1);
  //  avr_analogWrite(MOTOR_R_SPEED, speedR);
  //  avr_analogWrite(MOTOR_L_SPEED, speedL);
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