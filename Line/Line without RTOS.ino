#define right_far 4   // 8 
#define right_near 8
#define middle 13
#define left_near 2  //11
#define left_far 7
#define c1 9
#define c2 6
#define c3 5
#define c4 3
#define cA 11
#define cB 10
#define cC 12

int speed = 280;
int position_error = 0;
bool line = false;
double Kp = 100;
double Ki = 0.1;
double Kd = 195;
double error = 0,
       last_error = 0,
       erro_integral = 0;
bool default_case = true;    // bluetooth has not started yet

void move_car(int left_speed, int right_speed);
int calc_error(int car_position);
double PID_logic(double input);
int read_sensors();
void change_car_speed(int change_amount);
void car_logic();
void bluetooth();


void setup() {
  pinMode(right_far,INPUT);
  pinMode(right_near,INPUT);
  pinMode(middle,INPUT);
  pinMode(left_near,INPUT);
  pinMode(left_far,INPUT);
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
 
  pinMode(A0, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A5, OUTPUT);
  digitalWrite(A0,HIGH);

}

void loop() {
  car_logic();
}

void move_car(int left_speed, int right_speed) {
  right_speed = -right_speed;

  if (left_speed < 0) {
    analogWrite(c2, 0);
    analogWrite(c1,-left_speed);
  } else {
    analogWrite(c2, left_speed);
    analogWrite(c1, 0);
  }

  if (right_speed < 0) {
    analogWrite(c3, 0);
    analogWrite(c4, -right_speed);
  } else {
    analogWrite(c3, right_speed);
    analogWrite(c4, 0);
  }
} 

int calc_error(int car_position) {
  switch (car_position) {
    case B00000:
      position_error=position_error;
      break;
      
    case B11111:
      position_error = 0;
      break;
      
    case B00010:
    case B00110:
      position_error = 1;
      if (line) position_error = 0.5;
      break;
      
    case B00001:
    case B00011:
    case B00111:
      position_error = 2;
      if (line) position_error = 1;
      break;
      
    case B00100:
      position_error = 0;
      break;
      
    case B01000:
    case B01100:

      position_error = -1;
      if (line)position_error = -0.5;
      break;
      
    case B10000:
    case B11000:
    case B11100:
      position_error = -2;
      if (line)position_error = -1;
      break;
      
    default:
      position_error=position_error;
      break;
  }

  return position_error;


}

double PID_logic(double input) {
  double error_diff;
  double output;
  error = error * 0.7 + input * 0.3;
  error_diff = error - last_error;
  erro_integral = constrain(erro_integral + error, -50, 50);
  output = Kp * error + Ki * erro_integral + Kd * error_diff;
  last_error = error;
  return output;
}

int read_sensors() {
  int result = B00000;
  int a[5]={!digitalRead(right_far),
            !digitalRead(right_near),
            !digitalRead(middle),
            !digitalRead(left_near),
            !digitalRead(left_far)};
  
  for (int i = 0; i < 5; i++) {
    if (a[i] == HIGH) result += (0x1 << i);
  }

  
  return result;
}

void change_car_speed(int change_amount){

  double new_left_speed = speed - change_amount;
  double new_right_speed = speed + change_amount;

  new_left_speed = constrain(new_left_speed, -200, 250);
  new_right_speed = constrain(new_right_speed, -200, 250);
  move_car(new_left_speed, new_right_speed);
}

int current_position;
void car_logic(){
  current_position = read_sensors();
  double pid_error = calc_error(current_position);
  double speed_change = PID_logic(pid_error);
  change_car_speed(speed_change);
}
