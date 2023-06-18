// samaa:8-5-2023 : this is the worked whatsapp code
//  Yesterday's code
//  sensors array: 1 -> white, 0 -> black
#include <EEPROM.h>
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
#define IR_RR A3 // A1 // right 180
#define IR_L A5  // 0 -> white, 1 -> black
#define IR_R A4

#define second_array_right_far
#define second_array_right_near
#define second_array_middle A0
#define second_array_left_near
#define second_array_left_far

long speed = 100; // 110;
int rot_speed = 100;

double Kp = 45; // 100;
double Ki = 0.4;
double Kd = 210; // 195;
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
String path = "";

unsigned long lastRight = 0;

unsigned int rotation_delay_ms = 200;

long pos;
int kam2 = 7000; ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////9000;

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

int onLine()
{
    return (
        digitalRead(sensor_right2) == 0 || digitalRead(sensor_midel) == 0 || digitalRead(sensor_left1) == 0 || digitalRead(second_array_middle) == 0);
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

bool rotate180(bool isRight = false)
{

    SpeedLogic(rot_speed, rot_speed);
    delay(100);
    int mincnt = kam2 * 2;
    while (mincnt > 0 || !onLine())
    {

        // doaa & rufaida change here -----------------
        // check if the right sensors read black before the car becomes online
        if ((digitalRead(IR_R) || digitalRead(IR_RR)) && isRight == false)
        {
            // return untill onlin
            while (!onLine())
            {
                SpeedLogic(-1 * rot_speed - 10, rot_speed - 10);
            }
            // then break
            return false;
        }
        // doaa & rufaida change here -----------------

        if (isRight)
            SpeedLogic(-1 * rot_speed, rot_speed);
        else
            SpeedLogic(rot_speed, -1 * rot_speed);
        mincnt--;
    }
    return true;
}
int counter = 3000;
int counter180 = 0;
int currentTime = 0;
int previousTime = 0;
int end_counter = 0;
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
    bool LL_first_array = digitalRead(sensor_left2) ^ 1;

    // End
    // if(){
    //   int strLength = path.length();
    //    for (int i = 0; i < strLength; i++) {
    //     EEPROM.write(i, path[i]);
    //   }
    //   EEPROM.write(strLength, '\0'); // Add null terminator to indicate end of string
    // // if you want to read it
    //  // Read the string back from EEPROM and print it
    //   // char readString[strLength+1];
    //   // for (int i = 0; i < strLength; i++) {
    //   //   readString[i] = EEPROM.read(i);
    //   // }
    //   // readString[strLength] = '\0'; // Add null terminator to indicate end of string
    //   // Serial.println(readString);
    //   // if you want to read it
    // }
    // End

    if(pos == B11111 && LL && RR && RRR)
    {
        end_counter++;
    }
    else
    {
        end_counter = 0;
            if (LL || LL_first_array)
    {

        SpeedLogic(0, 0);
        delay(500);
        rotateLeft();
        if (strcmp(path[path.length() - 1], "L"))
        {
            path += "L";
            path = optimizeThePath(path);
        }

        SpeedLogic(0, 0);
        delay(rotation_delay_ms);
    }

    else if (RR && pos == B00000) // right rear far sensor & the 5 sensors are white
    {
        // while(counter>=0){
        //    pos = sensTrace();
        //   sensLogic(pos);
        //   counter--;
        SpeedLogic(0, 0);
        delay(500);
        // doaa is trying something
        // let it scan whether it is in dead right or not by going left and right to make sure that no black line is there
        SpeedLogic(rot_speed - 20, 0);
        delay(400);
        SpeedLogic(0, 0);
        if (pos == B00000)
        {
            rotateRight();
            if (strcmp(path[path.length() - 1], "R"))
            {
                path += "R";
                path = optimizeThePath(path);
            }

            SpeedLogic(0, 0);
            delay(rotation_delay_ms);
        }
    }

    if (pos == B00000 && !LL && !RRR)
    {
        counter180++;
    }
    else
    {
        counter180 = 0;
    }
    if (counter180 == 65)
    {
        // try to rotate to left to check if it reads black line or not
        // SpeedLogic(60, 0);
        // delay(100);
        // pos = sensTrace();
        if (pos == B00000)
        {
            SpeedLogic(0, 0);
            delay(500);
            currentTime = millis();
            bool if_back = rotate180(); // if_back variable is made for abdelrahman for the optimization path  // doaa & rufaida change here -----------------
            SpeedLogic(0, 0);
            previousTime = millis() - currentTime;
            currentTime = millis();
            while (pos != B00000)
            {
                pos = sensTrace();
                // sensLogic(pos);
                SpeedLogic(0, rot_speed - 20);
            }
            rotate180(true);
            SpeedLogic(0, 0);
            currentTime = millis() - currentTime;
            if (strcmp(path[path.length() - 1], "B"))
            {
                path += "B";
                path = optimizeThePath(path);
            }

            if (currentTime < previousTime)
            {
                rotate180();
            }
            // Serial.println(counter180);
        }
        counter180 = 0;
    }

    }
}


void replay()
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
    bool LL_first_array = digitalRead(sensor_left2) ^ 1;

    if (path[i]=="L"&& (LL || LL_first_array))
    {
        SpeedLogic(0, 0);
        delay(500);
        rotateLeft();
        SpeedLogic(0, 0);
        delay(rotation_delay_ms);
        i++;
    }

    else if (path[i] == "R" && RR) // right rear far sensor & the 5 sensors are white
    {
        SpeedLogic(0, 0);
        delay(500);
        rotateRight();
        SpeedLogic(0, 0);
        delay(rotation_delay_ms);
        i++;

        // // doaa is trying something
        // // let it scan whether it is in dead right or not by going left and right to make sure that no black line is there
        // SpeedLogic(rot_speed - 20, 0);
        // delay(400);
        // SpeedLogic(0, 0);
        
        // if (path[i]=="R"&&pos == B00000)
        // {
        //     rotateRight();
        //     SpeedLogic(0, 0);
        //     delay(rotation_delay_ms);
        // }
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

double Error = 0;
long outlineCnt = 0;

void sensLogic(long X)
{
    switch (X)
    {
    case B00000:
        Error = Error;
        break;

    case B11111:
        Error = 0;
        break;

    case B00010:
    case B00110:
        Error = 0.75;
        break;

    case B00001:
    case B00011:
    case B00111:
        Error = 1;
        break;

    case B00100:
        Error = 0;
        break;

    case B01000:
    case B01100:

        Error = -0.75;
        break;

    case B10000:
    case B11000:
    case B11100:
        Error = -1;
        break;

    default:
        Error = Error;
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
    //   pinMode(A5, OUTPUT);
    // pinMode(A1, OUTPUT);
    //   pinMode(A4, OUTPUT);

    //   digitalWrite(A5, HIGH);
    // digitalWrite(A1, HIGH);
    //   digitalWrite(A4, LOW);

    analogWrite(c1, 0);
    analogWrite(c2, 0);
    analogWrite(c3, 0);
    analogWrite(c4, 0);

    last = millis();
}
//======================================================================================================

void loop()
{
    delay(5);
    movecar2();
}

