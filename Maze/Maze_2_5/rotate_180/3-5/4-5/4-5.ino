//Abdelrahman
// white --> 1   black --> 0
#include <avr/io.h>
#include <util/delay.h>
#define sensor_right1 4  // 8 far
#define sensor_right2 8  // near
#define sensor_midel 13
#define sensor_left1 2  //11
#define sensor_left2 7

#define cC 12
#define threshold 500

long speed = 60;
/// final for 27/4/2023
/*!
 * @brief IR Sensors
 */
#define IR_L A3
#define IR_C A2
#define IR_R A4
#define IR_LL A5
#define IR_RR A1


#define c4 10  // in4
#define c3 6   // in3
#define c1 11  // in1
#define c2 5   // in2
#define cB 3   // enB
#define cA 9   // enA

uint8_t ADC_end = 0;
uint16_t adc_value = 0;
/*!
 * @brief Thresholds
 */
#define THRESHOLD_W 500  // above it is white and below it all black
#define THRESHOLD_B 150  // above it is white and below it all black
#define ThresholdDiff 200
#define ThresholdDiff_W 150
// #define SPEEDSTEERING 70  // steering speed
#define NUM_OF_ERROR 70           // thershold for accepted errors
#define SPEEDFORWARD 110          //95   // forward speed
String sensor_reading = "00000";  // initial value
// ///////////////////////////////////////////////////////////////////////////////

// /// doaa adds here ---------------------------begin--------------------------------------
int rot_speed = 70;

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

long pos;


long Error = 0;
long outlineCnt = 0;
int kam2 = 9000;


int onLine() {
  return (
    detect_black(IR_R) == 0  // white
    && detect_black(IR_C) == 1
    && detect_black(IR_L) == 0);
}


void rotate180() {
  int mincnt = kam2;
  while (mincnt > 0 || !onLine()) {
    SpeedLogic(rot_speed, -1 * rot_speed);
    mincnt--;
  }
}


void rotateRight() {
  int mincnt = kam2;
  while (mincnt > 0 || !onLine()) {
    SpeedLogic(-1 * rot_speed, rot_speed);
    // moveMotor2(speed, speed, -1, 1);
    mincnt--;
  }
}


void rotateLeft() {
  SpeedLogic(0, 0);
  SpeedLogic(-1 * rot_speed, -1 * rot_speed);
  SpeedLogic(0, 0);
  int mincnt = kam2;
  while (mincnt > 0 || !onLine()) {
    SpeedLogic(rot_speed, 0);
    mincnt--;
  }
}



void movecar2() {

  //TODO --> straight
  pos = sensTrace();
  sensLogic(pos);

  bool LL = detect_black(IR_LL);  // 1 --> black, 0 --> white
  bool RR = detect_black(IR_RR);
  bool C = detect_black(IR_C);
  bool L = detect_black(IR_L);
  bool R = detect_black(IR_R);


  if (LL) llIsBlack++, llIsWhite = 0;
  else llIsBlack = 0, llIsWhite++;

  if (RR) rrIsBlack++, rrIsWhite = 0;
  else rrIsBlack = 0, rrIsWhite++;

  if (L) lIsBlack++, lIsWhite = 0;
  else lIsBlack = 0, lIsWhite++;

  if (R) rIsBlack++, rIsWhite = 0;
  else rIsBlack = 0, rIsWhite++;

  if (C) cIsWhite = 0;
  else cIsWhite++;

  int kam = 100;
  if (llIsBlack >= kam) {
    rotateLeft();
    // if (strcmp(path[path.length() - 1], "L"))
    //   path += "L";
  }

  else if (cIsWhite >= kam && rIsWhite >= kam && lIsWhite >= kam) {
    if (millis() - lastRight < 1000) {
      rotateRight();
      // if (strcmp(path[path.length() - 1], "R")) {
      //   path += "R";
      // }
    } else {
      rotate180();
      // if (strcmp(path[path.length() - 1], "B"))
      //   path += "B";
    }
  }

  if (rrIsBlack >= kam)
    lastRight = millis();
}

// /// doaa adds here ---------------------------end--------------------------------------

char detect_black(uint8_t pin) {
  return analogRead(pin) < threshold ? 1 : 0;
}  // 1 --> black, 0 --> white




void readSensors() {
  // put your main code here, to run repeatedly:
  uint16_t adc_result0, adc_result1;


  sensor_reading[0] = detect_black(IR_LL);
  sensor_reading[1] = detect_black(IR_L);
  sensor_reading[2] = detect_black(IR_C);
  // sensor_reading[2] = digitalRead(2);    // for digital sensor
  sensor_reading[3] = detect_black(IR_R);
  sensor_reading[4] = detect_black(IR_RR);
  //}
}
// ////////////////////////////////////////////////////////////////////////

// SPEED CINTER
// ===============================================================================

void SpeedLogic(long spdL, long spdR) {
  spdR = -spdR;

  if (spdL < 0) {
    analogWrite(c2, 0);
    analogWrite(c1, -spdL);
  } else {
    analogWrite(c2, spdL);
    analogWrite(c1, 0);
  }

  if (spdR < 0) {
    analogWrite(c3, 0);
    analogWrite(c4, -spdR);
  } else {
    analogWrite(c3, spdR);
    analogWrite(c4, 0);
  }
}
//================================================================================================
// LOGIC CINTER

void sensLogic(long X) {
  switch (X) {
    case B00000:
      outlineCnt = 0;
      Error = Error;
      rotate180();
      // Serial.println(X,BIN);
      break;

    case B11111:
      outlineCnt = 0;
      Error = 0;
      // Serial.println(X,BIN);
      break;

    case B00010:
    case B00110:
      outlineCnt = 0;
      Error = .5;
      // Serial.println(X,BIN);
      break;

    case B00001:
    case B00011:
    case B00111:

      outlineCnt = 0;
      Error = 1;
      // Serial.println(X,BIN);
      break;

    case B00100:
    case B10100:  // mod
      outlineCnt = 0;
      Error = 0;
      // Serial.println(X,BIN);
      break;

    case B01000:
    case B01100:
      outlineCnt = 0;
      Error = -.5;
      // Serial.println(X,BIN);
      break;

    case B10000:
    case B11000:
    case B11100:
    case B10001:  // mod
    case B00101:  //mod
      outlineCnt = 0;
      Error = -1;
      // Serial.println(X,BIN);
      break;

    default:
      outlineCnt = 0;
      Error = Error;
      // Serial.println(X,BIN);
      break;
  }

  if (outlineCnt > 2) {
    SpeedLogic(0, 0);
  } else {

    double ctrl = calcPid(Error);
    double speedign = speed - ctrl;
    double speedign2 = speed + ctrl;

    speedign = constrain(speedign, -200, 250);
    speedign2 = constrain(speedign2, -200, 250);
    SpeedLogic(speedign, speedign2);
  }
}
//===============================================================================================


double Kp = 50;
double Ki = 0.1;
double Kd = 0.1;  // mod
// double kp =0.05;
// double ki =0.0000004;
// double kd =0.02  ;//1.5
double error = 0, errorLast = 0, erroInte = 0;
unsigned long last;
double calcPid(double input) {
  double errorDiff;
  double output;
  error = error * 0.7 + input * 0.3;  // filter
  //error = input;
  errorDiff = error - errorLast;
  erroInte = constrain(erroInte + error, -50, 50);
  output = Kp * error + Ki * erroInte + Kd * errorDiff;

  errorLast = error;

  return output;
}


// TRACE ===================================================================================
long sensTrace() {
  long ret = B00000;
  long a[5] = { detect_black(IR_RR),
                detect_black(IR_R),
                detect_black(IR_C),
                detect_black(IR_L),
                detect_black(IR_LL) };
  // Serial.println(a[4]+a[3]+a[2]+a[1]+a[0]);
  for (long i = 0; i < 5; i++) {
    Serial.print(a[i]);
    if (a[i] == HIGH) ret += (0x1 << i);
  }
  Serial.println(' ');


  return ret;
}
//========================================================================================

void setup() {

  // set motor pins as output
  Serial.begin(9600);

  pinMode(sensor_right1, INPUT);
  pinMode(sensor_right2, INPUT);
  pinMode(sensor_midel, INPUT);
  pinMode(sensor_left1, INPUT);
  pinMode(sensor_left2, INPUT);
  // set motor direction

  pinMode(c1, OUTPUT);
  pinMode(c2, OUTPUT);
  pinMode(c3, OUTPUT);
  pinMode(c4, OUTPUT);
  pinMode(cA, OUTPUT);
  pinMode(cB, OUTPUT);
  pinMode(cC, OUTPUT);
  digitalWrite(cA, HIGH);
  digitalWrite(cB, HIGH);
  digitalWrite(cC, HIGH);
  analogWrite(c1, 0);
  analogWrite(c2, 0);
  analogWrite(c3, 0);
  analogWrite(c4, 0);
  // adc_init();
}
//======================================================================================================

void loop() {


  // pos = sensTrace();
  // sensLogic(pos);
  movecar2();
}