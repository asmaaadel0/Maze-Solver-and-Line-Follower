// Abdelrahman

#include <avr/io.h>
#include <util/delay.h>
#define sensor_right1 4 // 8
#define sensor_right2 8
#define sensor_midel 13
#define sensor_left1 2 // 11
#define sensor_left2 7
// #define c1 9
// #define c2 6
// #define c3 5
// #define c4 3
// #define cA 11
// #define cB 10
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

// #define MOTOR_L_B 2      // in1
// #define MOTOR_L_F 8      // in2
// #define MOTOR_R_F 4      // in4
// #define MOTOR_R_B 5      // in3
// #define MOTOR_R_SPEED 3  // enB
// #define MOTOR_L_SPEED 9  // 10 // enA
#define c4 10 // in4
#define c3 6  // in3
#define c1 11 // in1
#define c2 5  // in2
#define cB 3  // enB
#define cA 9  // enA
//
// #define c1 9
// #define c2 6
// #define c3 5
// #define c4 3
// #define cA 11
// #define cB 10
// #define cC 12
uint8_t ADC_end = 0;
uint16_t adc_value = 0;
/*!
 * @brief Thresholds
 */
#define THRESHOLD_W 500 // above it is white and below it all black
#define THRESHOLD_B 150 // above it is white and below it all black
#define ThresholdDiff 200
#define ThresholdDiff_W 150
// #define SPEEDSTEERING 70  // steering speed
#define NUM_OF_ERROR 70          // thershold for accepted errors
#define SPEEDFORWARD 110         // 95   // forward speed
String sensor_reading = "00000"; // initial value

// booleans if rotating or not, if enabled, will be disabled when rotating is done
// i.e. after some time like 0.5 sec
bool rotating_left = false;
bool rotating_right = false;
bool rotating_180 = false;
// ///////////////////////////////////////////////////////////////////////////////

char detect_black(uint8_t pin)
{
  return analogRead(pin) < threshold ? 1 : 0;
}

void readSensors()
{
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

// SPEED CINTER
// ===============================================================================

void SpeedLogic(long spdL, long spdR)
{
  spdR = -spdR;

  if (spdL < 0)
  {
    analogWrite(c2, 0);
    analogWrite(c1, -spdL);
  }
  else
  {
    analogWrite(c2, spdL);
    analogWrite(c1, 0);
  }

  if (spdR < 0)
  {
    analogWrite(c3, 0);
    analogWrite(c4, -spdR);
  }
  else
  {
    analogWrite(c3, spdR);
    analogWrite(c4, 0);
  }
}
//================================================================================================
// LOGIC CINTER

long Error = 0;
long outlineCnt = 0;
int rot_speed = 70;

int onLine()
{
  return (
      digitalRead(IR_R) == 1 // white
      && digitalRead(IR_C) == 0 && digitalRead(IR_L) == 1);
}
int kam2 = 9000;

void rotateLeft()
{
  SpeedLogic(0, 0);
  SpeedLogic(-1 * rot_speed, -1 * rot_speed);
  SpeedLogic(0, 0);
  int mincnt = kam2;
  while (mincnt > 0 || !onLine())
  {
    SpeedLogic(rot_speed, 0);
    mincnt--;
  }
}
void rotateRight()
{
  int mincnt = kam2;
  while (mincnt > 0 || !onLine())
  {
    SpeedLogic(-1 * rot_speed, rot_speed);
    // moveMotor2(speed, speed, -1, 1);
    mincnt--;
  }
}

void rotate180()
{
  int mincnt = kam2;
  while (mincnt > 0 || !onLine())
  {
    SpeedLogic(rot_speed, -1 * rot_speed);
    mincnt--;
  }
}

void sensLogic(long X)
{
  // replace switch with if
  // 0 is white and 1 is black
  if (X == B00000)
  {
    // dead end
    rotating_180 = true;
    outlineCnt = 0;
    Error = Error;
    rotate180();
    // Serial.println(X,BIN);
  }
  else if (X == B11111)
  {
    // i think this means the end or the start
    outlineCnt = 0;
    Error = 0;
    // Serial.println(X,BIN);
  }
  
  else if (X & (1 << 0))
  {
    // turn left
    rotateLeft();
    // // if not rotating left or 180
    // if (!rotating_left && !rotating_180)
    // {
    //   // turn right
    //   rotating_right = true;
    // }
  }

  else if (X & (1 << 2))
  {
    // no turn
    // apply PID
    if (X == B00010 || X == B00110)
    {
      // on the line, but not centered
      outlineCnt = 0;
      Error = .5;
      // Serial.println(X,BIN);
    }
    // else if (X == B00001 || X == B00011 || X == B00111)
    // {
    //   outlineCnt = 0;
    //   Error = 1;
    //   // Serial.println(X,BIN);
    // }
    else if (X == B00100)
    { // mod
      outlineCnt = 0;
      Error = 0;
      // Serial.println(X,BIN);
    }
    else if (X == B01000 || X == B01100)
    {
      outlineCnt = 0;
      Error = -.5;
      // Serial.println(X,BIN);
    }
    // else if (X == B10000 || X == B11000 || X == B11100 || X == B10001 || X == B00101)
    // { // mod
    //   outlineCnt = 0;
    //   Error = -1;
    //   // Serial.println(X,BIN);
    // }
    else
    {
      outlineCnt = 0;
      Error = Error;
    }
  }
    else if (X & (1 << 4))
  {
    // turn right
    rotateRight();
    // // if not rotating right or 180
    // if (!rotating_right && !rotating_180)
    // {
    //   // turn left
    //   rotating_left = true;
    // }
  }
  if (outlineCnt > 2)
  {
    SpeedLogic(0, 0);
  }
  else
  {

    double ctrl = calcPid(Error);
    double speedign = speed - ctrl;
    double speedign2 = speed + ctrl;

    speedign = constrain(speedign, -200, 250);
    speedign2 = constrain(speedign2, -200, 250);
    SpeedLogic(speedign, speedign2);
  }
}
//===============================================================================================

// PID 85 80000 | 20 10000
// speed = 110
// Kp = 20;
// Ki = 0.2
// kd = 110
// double Kp =  50;
// double Ki = 0.1;
//  double Kd = 0;
// kp->go out of line increase
// kd->
// ki->
double Kp = 50;
double Ki = 0.1;
double Kd = 0.1; // mod
// double kp =0.05;
// double ki =0.0000004;
// double kd =0.02  ;//1.5
double error = 0, errorLast = 0, erroInte = 0;
unsigned long last;
double calcPid(double input)
{
  double errorDiff;
  double output;
  error = error * 0.7 + input * 0.3; // filter
  // error = input;
  errorDiff = error - errorLast;
  erroInte = constrain(erroInte + error, -50, 50);
  output = Kp * error + Ki * erroInte + Kd * errorDiff;

  errorLast = error;

  return output;
}
// TRACE ===================================================================================
// sensor_reading[0] = detect_black(IR_LL);
//   sensor_reading[1] = detect_black(IR_L);
//   sensor_reading[2] = detect_black(IR_C);
//   // sensor_reading[2] = digitalRead(2);    // for digital sensor
//   sensor_reading[3] = detect_black(IR_R);
//   sensor_reading[4] = detect_black(IR_RR);
long sensTrace()
{
  long ret = B00000;
  long a[5] = {detect_black(IR_RR),
               detect_black(IR_R),
               detect_black(IR_C),
               detect_black(IR_L),
               detect_black(IR_LL)};
  // Serial.println(a[4]+a[3]+a[2]+a[1]+a[0]);
  for (long i = 0; i < 5; i++)
  {
    Serial.print(a[i]);
    if (a[i] == HIGH)
      ret += (0x1 << i);
  }
  Serial.println(' ');

  return ret;
}
//========================================================================================

// void setup() {
//  Serial.begin(9600);

//       pinMode(c1,OUTPUT);
//       pinMode(c2,OUTPUT);
//       pinMode(c3,OUTPUT);
//       pinMode(c4,OUTPUT);
//       pinMode(cA,OUTPUT);
//       pinMode(cB,OUTPUT);
//       pinMode(cC,OUTPUT);
// digitalWrite(cA,HIGH);
// digitalWrite(cB,HIGH);
// digitalWrite(cC,HIGH);
//  analogWrite(c1,0);
//   analogWrite(c2,0);
//   analogWrite(c3,0);
//  analogWrite(c4,0);

//   last = millis();
// }
void setup()
{
  // set motor pins as output
  Serial.begin(9600);
  // pinMode(MOTOR_R_F, OUTPUT);
  // pinMode(MOTOR_R_B, OUTPUT);
  // pinMode(MOTOR_L_F, OUTPUT);
  // pinMode(MOTOR_L_B, OUTPUT);
  pinMode(sensor_right1, INPUT);
  pinMode(sensor_right2, INPUT);
  pinMode(sensor_midel, INPUT);
  pinMode(sensor_left1, INPUT);
  pinMode(sensor_left2, INPUT);
  // set motor direction
  // digitalWrite(MOTOR_R_F, HIGH);
  // digitalWrite(MOTOR_L_F, HIGH);
  // digitalWrite(MOTOR_R_B, LOW);
  // digitalWrite(MOTOR_L_B, LOW);
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

long pos;
void loop()
{
  //  analogWrite(c3, 0);
  // analogWrite(c4, 200);
  //   analogWrite(c2, 0);
  // analogWrite(c1,200);
  // Serial.println(digitalRead(sensor_midel));
  // delay(4);

  pos = sensTrace();
  // println detect_black(IR_LL);
  // println detect_black(IR_LL);
  // Serial.println(pos,BIN);

  // Serial.println((detect_black(IR_L)));
  // Serial.println((detect_black(IR_LL)));

  sensLogic(pos);

  // SpeedLogic(0,80);
}