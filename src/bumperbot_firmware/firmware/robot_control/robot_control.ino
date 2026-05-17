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

// Encoders
unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
String right_wheel_sign = "p"; 
String left_wheel_sign = "p"; 
unsigned long last_millis = 0;
const unsigned long interval = 200;

// Interpret Serial Messages
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

// PID
double right_wheel_cmd_vel = 0.0;     
double left_wheel_cmd_vel = 0.0;      
double right_wheel_meas_vel = 0.0;    
double left_wheel_meas_vel = 0.0;     
double right_wheel_cmd = 0.0;         
double left_wheel_cmd = 0.0;          

// Tuning
double Kp_r = 11.5, Ki_r = 7.5, Kd_r = 0.1;
double Kp_l = 12.8, Ki_l = 8.3, Kd_l = 0.1;

// Controller
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

void setup() {
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  digitalWrite(LED_BUILTIN, LOW); // Start OFF

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);
  Serial.begin(115200);
  Serial3.begin(9600);

  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
  if (Serial.available())
  {
    char chr = Serial.read(); //rp10,lp10,
    // LED triggers the moment 'r' or 'l' is received
    if(chr == 'r' || chr == 'l')
    {
      if(chr == 'r') {
        is_right_wheel_cmd = true;
        is_left_wheel_cmd = false;
        value_idx = 0;
        is_cmd_complete = false;
      } else {
        is_right_wheel_cmd = false;
        is_left_wheel_cmd = true;
        value_idx = 0;
      }
    }
    else if(chr == 'p')
    {
      //digitalWrite(LED_BUILTIN, LOW);
      if(is_right_wheel_cmd && !is_right_wheel_forward)
      {
        digitalWrite(L298N_in1, !digitalRead(L298N_in1));
        digitalWrite(L298N_in2, !digitalRead(L298N_in2));
        is_right_wheel_forward = true;
      }
      else if(is_left_wheel_cmd && !is_left_wheel_forward)
      {
        digitalWrite(L298N_in3, !digitalRead(L298N_in3));
        digitalWrite(L298N_in4, !digitalRead(L298N_in4));
        is_left_wheel_forward = true;
      }
    }
    else if(chr == 'n')
    {
      //digitalWrite(LED_BUILTIN, HIGH);
      if(is_right_wheel_cmd && is_right_wheel_forward)
      {
        digitalWrite(L298N_in1, !digitalRead(L298N_in1));
        digitalWrite(L298N_in2, !digitalRead(L298N_in2));
        is_right_wheel_forward = false;
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward)
      {
        digitalWrite(L298N_in3, !digitalRead(L298N_in3));
        digitalWrite(L298N_in4, !digitalRead(L298N_in4));
        is_left_wheel_forward = false;
      }
    }
    else if(chr == ',')
    {
      
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }
      value_idx = 0;
      strcpy(value, "00.00");
    }
    else
    {
      if(value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
    Serial3.write(chr);
  }

  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval)
  {
     digitalWrite(LED_BUILTIN, HIGH);
    right_wheel_meas_vel = (10 * right_encoder_counter * (60.0/351.0)) * 0.10472;
    left_wheel_meas_vel = (10 * left_encoder_counter * (60.0/385.0)) * 0.10472;
    
    rightMotor.Compute();
    leftMotor.Compute();

    if(right_wheel_cmd_vel == 0.0) right_wheel_cmd = 0.0;
    if(left_wheel_cmd_vel == 0.0) left_wheel_cmd = 0.0;

    String encoder_read = "r" + right_wheel_sign + String(right_wheel_meas_vel) + ",l" + left_wheel_sign + String(left_wheel_meas_vel) + ",";
    Serial.println(encoder_read);
    
    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;

    analogWrite(L298N_enA, (int)right_wheel_cmd);
    analogWrite(L298N_enB, (int)left_wheel_cmd);
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void rightEncoderCallback()
{
  right_wheel_sign = (digitalRead(right_encoder_phaseB) == HIGH) ? "p" : "n";
  right_encoder_counter++;
}

void leftEncoderCallback()
{
  left_wheel_sign = (digitalRead(left_encoder_phaseB) == HIGH) ? "n" : "p";
  left_encoder_counter++;
}
