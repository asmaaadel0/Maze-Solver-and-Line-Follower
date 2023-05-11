#define sensor_right1 4   // 8 
#define sensor_right2 8
#define sensor_midel 13
#define sensor_left1 2  //11
#define sensor_left2 7
#define c1 9
#define c2 6
#define c3 5
#define c4 3
#define cA 11
#define cB 10
#define cC 12

long speed = 200;


// SPEED CINTER
// =============================================================================== 

void SpeedLogic(long spdL, long spdR) {
  spdR = -spdR;

  if (spdL < 0) {
    analogWrite(c2, 0);
    analogWrite(c1,-spdL);
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

long Error = 0;
long outlineCnt = 0;

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
      Error = 1;
      // Serial.println(X,BIN);
      break;
      
    case B00001:
    case B00011:
    case B00111:
      outlineCnt = 0;
      Error = 2;
      // Serial.println(X,BIN);
      break;
      
    case B00100:
      outlineCnt = 0;
      Error = 0;
      // Serial.println(X,BIN);
      break;
      
    case B01000:
    case B01100:
      outlineCnt = 0;
      Error = -1;
      // Serial.println(X,BIN);
      break;
      
    case B10000:
    case B11000:
    case B11100:
      outlineCnt = 0;
      Error = -2;
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
//===============================================================================================

// PID 85 80000 | 20 10000
// speed = 110
// Kp = 20;
// Ki = 0.2
// kd = 110
double Kp = 100;
double Ki = 0.1;
double Kd = 195;
double error=0, errorLast=0, erroInte=0;
unsigned long last;
double calcPid(double input) {
  double errorDiff;
  double output;
  error = error * 0.7 + input * 0.3; // filter
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
  long a[5]={!digitalRead(sensor_right1),
            !digitalRead(sensor_right2),
            !digitalRead(sensor_midel),
            !digitalRead(sensor_left1),
            !digitalRead(sensor_left2)};
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
  pinMode(sensor_right1,INPUT);
  pinMode(sensor_right2,INPUT);
  pinMode(sensor_midel,INPUT);
  pinMode(sensor_left1,INPUT);
  pinMode(sensor_left2,INPUT);
      pinMode(c1,OUTPUT);
      pinMode(c2,OUTPUT);
      pinMode(c3,OUTPUT);
      pinMode(c4,OUTPUT); 
      pinMode(cA,OUTPUT); 
      pinMode(cB,OUTPUT); 
      pinMode(cC,OUTPUT);
digitalWrite(cA,HIGH);       
digitalWrite(cB,HIGH);       
digitalWrite(cC,HIGH);       
 analogWrite(c1,0);
  analogWrite(c2,0);
  analogWrite(c3,0);
 analogWrite(c4,0);

  last = millis();
}
//======================================================================================================

long pos;
void loop() {
    //  analogWrite(c3, 0);
    // analogWrite(c4, 200);
    //   analogWrite(c2, 0);
    // analogWrite(c1,200);
  // Serial.println(digitalRead(sensor_midel));
  // delay(4);

  pos = sensTrace();
    sensLogic(pos);


  // SpeedLogic(0,80);
}