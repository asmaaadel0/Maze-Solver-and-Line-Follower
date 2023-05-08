// Yesterday's code
// sensors array: 1 -> white, 0 -> black
#define sensor_right1 4  // 8 far
#define sensor_right2 8  // near
#define sensor_midel 13
#define sensor_left1 2  //11  far
#define sensor_left2 7  // near
#define c1 9            // in1
#define c2 6
#define c3 5
#define c4 3
#define cA 11
#define cB 10
#define cC 12

#define IR_L A3  // 0 -> white, 1 -> black
#define IR_R A2

long speed = 110;  //110;
int rot_speed = 70;

double Kp = 100;  //100;
double Ki = 0.1;
double Kd = 200;  //195;
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

long pos;
int kam2 = 9000;//9000;

int onLine() {
  return (
    digitalRead(sensor_right2) == 1
    && digitalRead(sensor_midel) == 0
    && digitalRead(sensor_left2) == 1);
}

void rotateRight() {
  int mincnt = kam2;
  while (mincnt > 0 || !onLine()) {
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


// void rotateLeft() {
//   SpeedLogic(0, 0);
//   SpeedLogic(-1 * rot_speed, -1 * rot_speed);
//   SpeedLogic(0, 0);
//   int mincnt = kam2;
//   while (mincnt > 0 || !onLine()) {
//     SpeedLogic(rot_speed, 0);

//     mincnt--;
//   }
// }

void rotateLeft() {
  SpeedLogic(0, 0);
  SpeedLogic(-1 * rot_speed, -1 * rot_speed);
  SpeedLogic(0, 0);
  int mincnt = kam2;
  while (mincnt > 0 || !onLine()) {
    SpeedLogic(rot_speed, 0);
    mincnt--;
    // pos = sensTrace();
    // if (pos == B00000) {
    //   rotate180();
    //   break;
    // }
  }
}


void rotate180() {
  int mincnt = kam2;
  while (mincnt > 0 || !onLine()) {
    SpeedLogic(rot_speed, -1 * rot_speed);
    mincnt--;
  }
}

void movecar2() {

  //TODO --> straight
  pos = sensTrace();

  sensLogic(pos);

  bool LL = digitalRead(IR_L);
  bool RR = digitalRead(IR_R);
  bool C = digitalRead(sensor_midel) ^ 1;
  bool L = digitalRead(sensor_left2) ^ 1;
  bool R = digitalRead(sensor_right2) ^ 1;


  if (LL) llIsBlack++, llIsWhite = 0;
  else llIsBlack = 0, llIsWhite++;

  if (RR) rrIsBlack++, rrIsWhite = 0;
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
      //rotate180();
      // if (strcmp(path[path.length() - 1], "B"))
      //   path += "B";
    }
  }

  if (rrIsBlack >= kam)
    lastRight = millis();
}


// SPEED CINTER
// ===============================================================================

void SpeedLogic(long spdL, long spdR) {
  spdR = -spdR;

  if (spdL < 0) {
    analogWrite(c2, 0);
    analogWrite(c1, -spdL);
  } else {
    analogWrite(c2, spdL);  // rotate right
    analogWrite(c1, 0);
  }

  if (spdR < 0) {
    analogWrite(c3, 0);
    analogWrite(c4, -spdR);
  } else {
    analogWrite(c3, spdR);
    analogWrite(c4, 0);  // rotate left
  }
}
//================================================================================================
// LOGIC CINTER

long Error = 0;
long outlineCnt = 0;

// void sensLogic(long X) {
//   switch (X) {
//     case B00000:
//       outlineCnt = 0;
//       Error = Error;
//       // Serial.println(X,BIN);
//       break;

//     case B11111:
//       outlineCnt = 0;
//       Error = 0;
//       //rotate180();
//       // Serial.println(X,BIN);
//       break;

//     case B00010:
//     case B00110:
//       outlineCnt = 0;
//       Error = 1;
//       // Serial.println(X,BIN);
//       break;

//     case B00001:
//     case B00011:
//     case B00111:
//       outlineCnt = 0;
//       Error = 2;
//       // Serial.println(X,BIN);
//       break;

//     case B00100:
//       outlineCnt = 0;
//       Error = 0;
//       // Serial.println(X,BIN);
//       break;

//     case B01000:
//     case B01100:
//       outlineCnt = 0;
//       Error = -1;
//       // Serial.println(X,BIN);
//       break;

//     case B10000:
//     case B11000:
//     case B11100:
//       outlineCnt = 0;
//       Error = -2;
//       // Serial.println(X,BIN);
//       break;

//     default:
//       outlineCnt = 0;
//       Error = Error;
//       // Serial.println(X,BIN);
//       break;
//   }

//   if (outlineCnt > 2) {
//     SpeedLogic(0, 0);
//   } else {

//     double ctrl = calcPid(Error);
//     double speedign = speed - ctrl;
//     double speedign2 = speed + ctrl;

//     speedign = constrain(speedign, -200, 250);
//     speedign2 = constrain(speedign2, -200, 250);
//     SpeedLogic(speedign, speedign2);
//   }
// }

void sensLogic(long X) {
  switch (X) {
    case B00000:
      outlineCnt=0;
      Error=Error;
      // Serial.println(X,BIN);
      break;
      
    case B11111:
      outlineCnt=0;
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
    case B00101: //mod
      outlineCnt = 0;
      Error = -1;
      // Serial.println(X,BIN);
      break;
      
    default:
      outlineCnt=0;
      Error=Error;
      // Serial.println(X,BIN);
      break;
  }

  if (outlineCnt > 2) {
    SpeedLogic(0,0);
  } else {
    
    double ctrl = calcPid(Error);
    double speedign = speed-ctrl;
    double speedign2 = speed+ctrl;

    speedign = constrain(speedign, -200, 250);
    speedign2 = constrain(speedign2, -200, 250);
    SpeedLogic(speedign, speedign2);
  }
}
//============

//===============================================================================================



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
  long a[5] = { !digitalRead(sensor_right1),
                !digitalRead(sensor_right2),
                !digitalRead(sensor_midel),
                !digitalRead(sensor_left1),
                !digitalRead(sensor_left2) };
  // Serial.println(a[4]+a[3]+a[2]+a[1]+a[0]);
  for (long i = 0; i < 5; i++) {
    // Serial.print(a[i], BIN);
    if (a[i] == HIGH) ret += (0x1 << i);
  }
  // Serial.println(' ');


  return ret;
}
//========================================================================================

void setup() {
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
  pinMode(A1, OUTPUT);
  pinMode(A4, OUTPUT);

  digitalWrite(A5, HIGH);
  digitalWrite(A1, HIGH);
  digitalWrite(A4, LOW);



  analogWrite(c1, 0);
  analogWrite(c2, 0);
  analogWrite(c3, 0);
  analogWrite(c4, 0);

  last = millis();
}
//======================================================================================================


void loop() {

  movecar2();
}