// samaa:8-5-2023 : this is the worked whatsapp code
//  Yesterday's code
//  sensors array: 1 -> white, 0 -> black
#define sensor_right1 4 // 8 far
#define sensor_right2 8 // near
#define sensor_midel 13
#define sensor_left1 2 // 11  far
#define sensor_left2 7 // near
#define c1 9           // in1
#define c2 6
#define c3 5
#define c4 3
#define cA 11
#define cB 10
#define cC 12
#define IR_RR A1 // right 180
#define IR_L A3  // 0 -> white, 1 -> black
#define IR_R A2
#define IR_CR A0
#define IR_CL A4


long speed = 110; // 110;
int rot_speed = 100;

double Kp = 100; // 100;
double Ki = 0.1;
double Kd = 200; // 195;
double error = 0, errorLast = 0, erroInte = 0;
unsigned long last;

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

unsigned int rotation_delay_ms = 200;

long pos;
int kam2 = 9000; ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////9000;

int onLine()
{
  return (
      digitalRead(sensor_right2) == 0 || digitalRead(sensor_midel) == 0 || digitalRead(sensor_left1) == 0);
}
int onLine2()
{
  return (
      digitalRead(sensor_right2) == 0 || digitalRead(sensor_midel) == 0 || digitalRead(sensor_left1) == 0 || digitalRead(sensor_left2) == 0 || digitalRead(sensor_right1) == 0);
}

void rotateRight()
{
  int mincnt = kam2;
  while (mincnt > 0 || !onLine())
  { // to ask about the onLine
    // while (!onLine()) {
    SpeedLogic(-1 * rot_speed, rot_speed);
    // moveMotor2(speed, speed, -1, 1);
    mincnt--;
    // pos = sensTrace();
    // if (pos == B00000) {
    //   rotate180();
    //   break;
    // }
  }
}

void rotateLeft()
{
  SpeedLogic(0, 0);
  SpeedLogic(-1 * rot_speed, -1 * rot_speed);
  SpeedLogic(0, 0);
  int mincnt = kam2;
  while (mincnt > 0 || !onLine())
  {
    // while ( !onLine()) {
    SpeedLogic(rot_speed, 0);
    mincnt--;
    // pos = sensTrace();
    // if (pos == B00000) {
    //   rotate180();
    //   break;
    // }
  }
}

void rotate180()
{
  SpeedLogic(rot_speed, rot_speed);
  delay(100);
  int mincnt = kam2 * 2;
  while (mincnt > 0 || !onLine())
  {
    SpeedLogic(rot_speed, -1 * rot_speed);
    mincnt--;
  }
}
int counter = 3000;
int counter180 = 0;
void movecar2()
{

  // TODO --> straight
  pos = sensTrace();

  sensLogic(pos);

  bool LL = digitalRead(IR_L);
  bool RR = digitalRead(IR_R);
  bool C = digitalRead(sensor_midel) ^ 1;
  bool L = digitalRead(sensor_left1) ^ 1;
  bool R = digitalRead(sensor_right2) ^ 1;
  bool RRR = digitalRead(IR_RR);
  bool CR = digitalRead(IR_CR);
  bool CL = digitalRead(IR_CL);

  if (LL)
  {

    rotateLeft();
    SpeedLogic(0, 0);
    delay(rotation_delay_ms);
  }

  else if (RR && pos == B00000 && !CR && !CL) // right rear far sensor & the 5 sensors are white
  {
    // while(counter>=0){
    //    pos = sensTrace();
    //   sensLogic(pos);
    //   counter--;
    rotateRight();
    SpeedLogic(0, 0);
    delay(rotation_delay_ms);
  }

  if (pos == B00000 && !LL && !RRR && !CR && !CL)
  {
    counter180++;
  }
  else
  {
    counter180 = 0;
  }
  if (counter180 == 1000)
  {
    rotate180();
    // Serial.println(counter180);
    counter180 = 0;
  }
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
    analogWrite(c2, spdL); // rotate right
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
    analogWrite(c4, 0); // rotate left
  }
}
//================================================================================================
// LOGIC CINTER

long Error = 0;
long outlineCnt = 0;

void sensLogic(long X)
{
  switch (X)
  {
  case B00000:
    outlineCnt = 0;
    Error = Error;
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
  case B10100: // mod
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
  case B10001: // mod
  case B00101: // mod
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
    // if error is 1 or -1 then add delay like rotate functions
  }
}
//============

//===============================================================================================

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
long sensTrace()
{
  long ret = B00000;
  long a[5] = {!digitalRead(sensor_right1),
               !digitalRead(sensor_right2),
               !digitalRead(sensor_midel),
               !digitalRead(sensor_left1),
               !digitalRead(sensor_left2)};
  // Serial.println(a[4]+a[3]+a[2]+a[1]+a[0]);
  for (long i = 0; i < 5; i++)
  {
    // Serial.print(a[i], BIN);
    if (a[i] == HIGH)
      ret += (0x1 << i);
  }
  // Serial.println(' ');

  return ret;
}
//========================================================================================

void setup()
{
  Serial.begin(9600);
  pinMode(sensor_right1, INPUT);
  pinMode(sensor_right2, INPUT);
  pinMode(sensor_midel, INPUT);
  pinMode(sensor_left1, INPUT);
  pinMode(sensor_left2, INPUT);
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
  pinMode(A5, OUTPUT);
  // pinMode(A1, OUTPUT);
  //pinMode(A4, OUTPUT);

  digitalWrite(A5, HIGH);
  // digitalWrite(A1, HIGH);
  //digitalWrite(A4, LOW);

  analogWrite(c1, 0);
  analogWrite(c2, 0);
  analogWrite(c3, 0);
  analogWrite(c4, 0);

  last = millis();
}
//======================================================================================================

void loop()
{

  movecar2();
  //  Serial.print("left ir ");
  // Serial.println(digitalRead(IR_L));
  // delay(500);
  //   Serial.print("right ir ");
  // Serial.println(digitalRead(IR_R));
  // delay(500);

  //   Serial.print("right right ir ");
  // Serial.println(digitalRead(IR_RR));
  // delay(500);
}