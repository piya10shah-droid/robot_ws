#include <PID_v1.h>

// L298N H-Bridge Connection PINs
#define L298N_enA 6  // PWM
#define L298N_enB 5  // PWM
#define L298N_in4 37  // Dir Motor B
#define L298N_in3 39  // Dir Motor B
#define L298N_in2 50  // Dir Motor A
#define L298N_in1 53  // Dir Motor A

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 2  // Interrupt 
#define right_encoder_phaseB 31  
#define left_encoder_phaseA 3   // Interrupt
#define left_encoder_phaseB 41

//encoder speed
unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
//encoder sign
String right_wheel_sign = "p"; 
String left_wheel_sign = "p"; 


//speed received
double left_wheel_speed_desired=0.0;
double right_wheel_speed_desired=0.0;

//desired wheel direction
char right_wheel_direction_desired;
char left_wheel_direction_desired;

double right_wheel_cmd_vel = 0.0;     
double left_wheel_cmd_vel = 0.0;      
double right_wheel_meas_vel = 0.0;    
double left_wheel_meas_vel = 0.0;     
double right_wheel_cmd = 0.0;         
double left_wheel_cmd = 0.0;  

unsigned long last_millis = 0;
const unsigned long interval = 100;
unsigned long first_time = 0;
unsigned long second_time = 0;


String message_received = "";
String rpString;
String lpString;

//pid tuning variables
double Kp_r = 11.5, Ki_r = 7.5, Kd_r = 0.1;
double Kp_l = 12.8, Ki_l = 8.3, Kd_l = 0.1;

PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_speed_desired, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_speed_desired, Kp_l, Ki_l, Kd_l, DIRECT);

void setup() {
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);
  Serial.begin(115200);
  Serial2.begin(115200);
  //Serial23.begin(9600);
// Set the PID evaluation delay to 50 milliseconds
//rightMotor.SetSampleTime(50); 
//leftMotor.SetSampleTime(50); 
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
 if (Serial2.available()){
  unsigned long first_time = millis();//rp10.00,lp10.00,
  message_received = Serial2.readStringUntil('\n'); //rp10.00,lp10.00,\r
  Serial.println(message_received);
  message_received.trim();
  parseMessage(message_received);
  setMotorSpeedRight();
  setMotorSpeedLeft(); 
 }
 unsigned long current_millis = millis();
 if(current_millis - last_millis >= interval)
 {
  executeMotor();
  last_millis = current_millis;
  second_time = millis();
  //Serial23.println(second_time - first_time);
 }
}


void parseMessage(String packet){ 
  int firstComma = packet.indexOf(',');
  int secondComma = packet.indexOf(',', firstComma + 1);
  //Serial23.println(packet);
  if (firstComma == -1 || secondComma == -1){
    Serial2.println("packet has either only 1/no comma");
    return;
  }
  rpString = packet.substring(0, firstComma); //rp10.00
  lpString = packet.substring(firstComma + 1, secondComma); //lp10.00
  
  right_wheel_direction_desired = rpString[1];
  left_wheel_direction_desired = lpString[1];

  right_wheel_speed_desired = rpString.substring(2).toFloat();
  left_wheel_speed_desired  = lpString.substring(2).toFloat();
  packet = "";
 
}

void setMotorSpeedRight(){
  // Prepares the H-Bridge direction states for Right Motor (Motor A)
  if (right_wheel_direction_desired == 'p') {

    digitalWrite(L298N_in1, HIGH);
    digitalWrite(L298N_in2, LOW);
  } else if (right_wheel_direction_desired == 'n') {
    digitalWrite(L298N_in1, LOW);
    digitalWrite(L298N_in2, HIGH);
  }
 }

void setMotorSpeedLeft(){
  // Prepares the H-Bridge direction states for Left Motor (Motor B)
  if (left_wheel_direction_desired == 'p') {
    digitalWrite(L298N_in3, HIGH);
    digitalWrite(L298N_in4, LOW);
  } else if (left_wheel_direction_desired == 'n') {
    digitalWrite(L298N_in3, LOW);
    digitalWrite(L298N_in4, HIGH);
  }
}

void executeMotor(){
    right_wheel_meas_vel = (10 * right_encoder_counter * (60.0/351.0)) * 0.10472;
    left_wheel_meas_vel = (10 * left_encoder_counter * (60.0/351.0)) * 0.10472;
  
    
    rightMotor.Compute();
    leftMotor.Compute();

    if(right_wheel_speed_desired == 0.0) right_wheel_cmd = 0.0;
    if(left_wheel_speed_desired == 0.0) left_wheel_cmd = 0.0;

    String encoder_read = "r" + right_wheel_sign + String(right_wheel_meas_vel) + ",l" + left_wheel_sign + String(left_wheel_meas_vel) + ",";
    Serial2.println(encoder_read);
    
    right_encoder_counter = 0;
    left_encoder_counter = 0;

    analogWrite(L298N_enA, (int)right_wheel_cmd);
    analogWrite(L298N_enB, (int)left_wheel_cmd);
//    Serial2.println(right_wheel_cmd);
//    Serial2.println(left_wheel_cmd);
}


void rightEncoderCallback()
{
  if(digitalRead(right_encoder_phaseB) == HIGH)
  {
    right_wheel_sign = "p";
  }
  else
  {
    right_wheel_sign = "n";
  }
  right_encoder_counter++;
}

// New pulse from Left Wheel Encoder
void leftEncoderCallback()
{
  if(digitalRead(left_encoder_phaseB) == HIGH)
  {
    left_wheel_sign = "n";
  }
  else
  {
    left_wheel_sign = "p";
  }
  left_encoder_counter++;
}
