#define ROSSERIAL_ARDUINO_TCP

#include <Arduino.h>
#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <WiFi.h>


// Motor 1
const int motor1_pin_a = 16;
const int motor1_pin_b = 17;
const int motor1_pwm = 4;

// Motor 2
const int motor2_pin_a = 25;
const int motor2_pin_b = 26; 
const int motor2_pwm = 27;

// Motor 3
const int motor3_pin_a = 5;
const int motor3_pin_b = 18;
const int motor3_pwm = 19;

// Motor 4
const int motor4_pin_a = 32; 
const int motor4_pin_b = 33;
const int motor4_pwm = 23;
  
const int pwm_channel1 = 0;
const int pwm_channel2 = 1;
const int pwm_channel3 = 2;
const int pwm_channel4 = 3;

const int freq = 5000;
const int resolution = 8;

// Encoder Motor 1
const int encoder1_pin_a = 21;
const int encoder1_pin_b = 22;
volatile long encoder_value1 = 0;
bool last_encoder_a1 = LOW;
volatile long pulse1 = 0;

// Encoder Motor 2
const int encoder2_pin_a = 12;
const int encoder2_pin_b = 14;
volatile long encoder_value2 = 0;
bool last_encoder_a2 = LOW;
volatile long pulse2 = 0; 

// Encoder Motor 3
const int encoder3_pin_a = 15;
const int encoder3_pin_b = 0;
volatile long encoder_value3 = 0;
bool last_encoder_a3 = LOW;
volatile long pulse3 = 0;

// Encoder Motor 4
const int encoder4_pin_a = 13;
const int encoder4_pin_b = 2;
volatile long encoder_value4 = 0;
bool last_encoder_a4 = LOW;
volatile long pulse4 = 0; 

const int pulse_per_revolution = 374*2;
unsigned long previous_millis = 0;

// RPM
float rpm1 = 0.0;
float rpm2 = 0.0;
float rpm3 = 0.0;
float rpm4 = 0.0;

// Control PID
float pwm_motor1;
float pwm_motor1_1;
float motor1_error;
float motor1_error1;
float motor1_error2;

float pwm_motor2;
float pwm_motor2_1;
float motor2_error;
float motor2_error1;
float motor2_error2; 

float pwm_motor3;
float pwm_motor3_1;
float motor3_error;
float motor3_error1;
float motor3_error2;

float pwm_motor4;
float pwm_motor4_1;
float motor4_error;
float motor4_error1;
float motor4_error2; 

// Definición de las variables PID
double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
double Setpoint3, Input3, Output3;
double Setpoint4, Input4, Output4;

const double kp_m1 = 0.52998;//0.0079236; //0.18236; 
const double ki_m1 = 3.583;//0.75802; //1.5018; 
const double kd_m1 = 0.0058866;//0.000020707; //0; 

const double kp_m2 = 0.52998;//0.0079236; //0.37922; 
const double ki_m2 = 3.583;//0.75802; //2.4627; 
const double kd_m2 = 0.0058866;//0.000020707; //0;

const double kp_m3 = 0.52998;//0.0079236;  //0.18236; 
const double ki_m3 = 3.583;//0.75802; //1.5018; 
const double kd_m3 = 0.0058866;//0.000020707; //0; 

const double kp_m4 = 0.52998;//0.0079236; //0.18236; 
const double ki_m4 = 3.583;//0.75802; //1.5018; 
const double kd_m4 = 0.0058866;//0.000020707; //0; 

//Specify the links and initial tuning parameters
PID myPID1(&Input1, &Output1, &Setpoint1, kp_m1, ki_m1, kd_m1, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2, kp_m2, ki_m2, kd_m2, DIRECT);
PID myPID3(&Input3, &Output3, &Setpoint3, kp_m3, ki_m3, kd_m3, DIRECT);
PID myPID4(&Input4, &Output4, &Setpoint4, kp_m4, ki_m4, kd_m4, DIRECT);

const float R = 0.03; // Radio de las ruedas en metros (ajustar según tu robot)
const float L = 0.07; // Distancia desde el centro al eje X en metros (ajustar)
const float W = 0.093; // Distancia desde el centro al eje Y en metros (ajustar)


// Server settings
IPAddress server(192, 168, 100, 129);
uint16_t serverPort = 11411;

const char* ssid = "Flia Gonzales";
const char* password = "AVEG@12375358";

void cmd_velCallback( const geometry_msgs::Twist& msg);
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmd_velCallback);

void setup()
{

  // Inicializar todas las lecturas en 0
  pinMode(motor1_pin_a, OUTPUT);
  pinMode(motor1_pin_b, OUTPUT);
  pinMode(motor2_pin_a, OUTPUT);
  pinMode(motor2_pin_b, OUTPUT);
  pinMode(motor3_pin_a, OUTPUT);
  pinMode(motor3_pin_b, OUTPUT);
  pinMode(motor4_pin_a, OUTPUT);
  pinMode(motor4_pin_b, OUTPUT);

  //Confuiguracion de los PWM 
  ledcSetup(pwm_channel1, freq, resolution);
  ledcAttachPin(motor1_pwm, pwm_channel1);

  ledcSetup(pwm_channel2, freq, resolution);
  ledcAttachPin(motor2_pwm, pwm_channel2);

  ledcSetup(pwm_channel3, freq, resolution);
  ledcAttachPin(motor3_pwm, pwm_channel3);

  ledcSetup(pwm_channel4, freq, resolution);
  ledcAttachPin(motor4_pwm, pwm_channel4);

  // Configuracion de los pines del encoder
  pinMode(encoder1_pin_a, INPUT_PULLUP);
  pinMode(encoder1_pin_b, INPUT_PULLUP);
  pinMode(encoder2_pin_a, INPUT_PULLUP);
  pinMode(encoder2_pin_b, INPUT_PULLUP);
  pinMode(encoder3_pin_a, INPUT_PULLUP);
  pinMode(encoder3_pin_b, INPUT_PULLUP);
  pinMode(encoder4_pin_a, INPUT_PULLUP);
  pinMode(encoder4_pin_b, INPUT_PULLUP);

  //Adjuntando interrupciones para el encoder 
  attachInterrupt(digitalPinToInterrupt(encoder1_pin_a), handle_encoder_a1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1_pin_b), handle_encoder_b1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2_pin_a), handle_encoder_a2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2_pin_b), handle_encoder_b2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3_pin_a), handle_encoder_a3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3_pin_b), handle_encoder_b3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder4_pin_a), handle_encoder_a4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder4_pin_b), handle_encoder_b4, CHANGE);

  //Varibales PID
  Setpoint1 = 0;
  Input1 = rpm1;
  Output1 = 0;
    
  Setpoint2 = 0;
  Input2 = rpm2;
  Output2 = 0;

  Setpoint3 = 0;
  Input3 = rpm3;
  Output3 = 0;

  Setpoint4 = 0;
  Input4 = rpm4;
  Output4 = 0;
  
  //turn the PID on
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(30.0 * (255.0/500.0), 400.0 * (255.0/500.0));

  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(30.0 * (255.0/500.0), 400.0 * (255.0/500.0));

  myPID3.SetMode(AUTOMATIC);
  myPID3.SetOutputLimits(30.0 * (255.0/500.0), 400.0 * (255.0/500.0));

  myPID4.SetMode(AUTOMATIC);
  myPID4.SetOutputLimits(30.0 * (255.0/500.0), 400.0 * (255.0/500.0));
  
  Serial.begin(115200);
  setupWiFi();
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{

  unsigned long current_millis = millis();

  if(current_millis - previous_millis >= 100)
  {
    previous_millis = current_millis;

    rpm1 = 10 * pulse1 * (60.0 / 748);
    rpm2 = 10 * pulse2 * (60.0 / 748);
    rpm3 = 10 * pulse3 * (60.0 / 748);
    rpm4 = 10 * pulse4 * (60.0 / 748);
    
    Input1 = rpm1;
    myPID1.Compute();
    ledcWrite(pwm_channel1, Output1);
    Input2 = rpm2;
    myPID2.Compute();
    ledcWrite(pwm_channel2, Output2);
    Input3 = rpm3;
    myPID3.Compute();
    ledcWrite(pwm_channel3, Output3);
    Input4 = rpm4;
    myPID4.Compute();
    ledcWrite(pwm_channel4, Output4);

    //Serial.print("Velocidad Motor 1: ");
    //Serial.println(rpm1);
    //Serial.print("Velocidad Motor 2: ");
    //Serial.println(rpm2);
    //Serial.print("Velocidad Motor 3: "); 
    //Serial.println(rpm3);
    //Serial.print("Velocidad Motor 4: ");
    //Serial.println(rpm4);

    pulse1 = 0;
    pulse2 = 0;
    pulse3 = 0;
    pulse4 = 0;
  }
  nh.spinOnce();
  delay(10);
}

void setupWiFi()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500);Serial.print("."); }
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP:  ");
  Serial.println(WiFi.localIP());
}

void cmd_velCallback( const geometry_msgs::Twist& msg)
{
  Serial.println("Mensaje recibido");

  if (msg.linear.x == 0.0 && msg.linear.y == 0.0 && msg.angular.z == 0.0) {
    // Detener todos los motores
    Setpoint1 = 0;
    Setpoint2 = 0;
    Setpoint3 = 0;
    Setpoint4 = 0;
    
    // Asegurarse de que los PWM estén en 0
    ledcWrite(pwm_channel1, 0);
    ledcWrite(pwm_channel2, 0);
    ledcWrite(pwm_channel3, 0);
    ledcWrite(pwm_channel4, 0);

    digitalWrite(motor1_pin_a, LOW);
    digitalWrite(motor1_pin_b, LOW);

    digitalWrite(motor2_pin_a, LOW);
    digitalWrite(motor2_pin_b, LOW);

    digitalWrite(motor3_pin_a, LOW);
    digitalWrite(motor3_pin_b, LOW);

    digitalWrite(motor4_pin_a, LOW);
    digitalWrite(motor4_pin_b, LOW);
  
    return;  // Salir del callback
  }

  float vx = msg.linear.x;
  float vy = msg.linear.y;
  float wz = msg.angular.z;

  // Aplicando el modelo cinemático
  //float v1 = vx - vy - wz*(L + W);
  //float v2 = vx + vy + wz*(L + W);
  //float v4 = vx - vy + wz*(L + W);
  //float v3 = vx + vy - wz*(L + W);

  float v1 = vx + vy + wz*(L + W);
  float v2 = vx - vy + wz*(L + W);
  float v4 = vx + vy - wz*(L + W);
  float v3 = vx - vy - wz*(L + W);

  // Convertir las velocidades a RPM
  rpm1 = linearToRPM(v1, R);
  rpm2 = linearToRPM(v2, R);
  rpm3 = linearToRPM(v3, R);
  rpm4 = linearToRPM(v4, R);

  // Establecer la dirección de cada motor
  setMotorDirection(1, v1);
  setMotorDirection(2, v2);
  setMotorDirection(3, v3);
  setMotorDirection(4, v4);

  // Usar el valor absoluto de las RPM como setpoint para el PID
  Setpoint1 = abs(rpm1);
  Setpoint2 = abs(rpm2);
  Setpoint3 = abs(rpm3);
  Setpoint4 = abs(rpm4);

}

float linearToRPM(float v, float R)
{
  return v * 60 / (2 * PI * R);
}

void setMotorDirection(int motor, float speed)
{
  if (motor == 1) {
    if (speed > 0) {
      Serial.println(speed);
      digitalWrite(motor1_pin_a, HIGH);
      digitalWrite(motor1_pin_b, LOW);
    } else {
      digitalWrite(motor1_pin_a, LOW);
      digitalWrite(motor1_pin_b, HIGH);
    }
  }

  if (motor == 2) {
    if (speed > 0) {
      digitalWrite(motor2_pin_a, HIGH);
      digitalWrite(motor2_pin_b, LOW);
    } else {
      digitalWrite(motor2_pin_a, LOW);
      digitalWrite(motor2_pin_b, HIGH);
    }
  }

  if (motor == 3) {
    if (speed > 0) {
      digitalWrite(motor3_pin_a, HIGH);  // Invertido
      digitalWrite(motor3_pin_b, LOW); // Invertido
    } else {
      digitalWrite(motor3_pin_a, LOW); // Invertido
      digitalWrite(motor3_pin_b, HIGH);  // Invertido
    }
  }

  if (motor == 4) {
    if (speed > 0) {
      digitalWrite(motor4_pin_a, HIGH);
      digitalWrite(motor4_pin_b, LOW);
    } else {
      digitalWrite(motor4_pin_a, LOW);
      digitalWrite(motor4_pin_b, HIGH);
    }
  }

}

void handle_encoder_a1() 
{
  if (digitalRead(encoder1_pin_a) != last_encoder_a1) {
    if (digitalRead(encoder1_pin_a) != digitalRead(encoder1_pin_b)) {
      encoder_value1++;
    } else {
      encoder_value1--;
    }
    pulse1++;
  }
  last_encoder_a1 = digitalRead(encoder1_pin_a);
}

void handle_encoder_b1() {
  if (digitalRead(encoder1_pin_a) == digitalRead(encoder1_pin_b)) {
    encoder_value1++;
  } else {
    encoder_value1--;
  }
  pulse1++;
}

void handle_encoder_a2() 
{
  if (digitalRead(encoder2_pin_a) != last_encoder_a2) {
    if (digitalRead(encoder2_pin_a) != digitalRead(encoder2_pin_b)) {
      encoder_value2++;
    } else {
      encoder_value2--;
    }
    pulse2++;
  }
  last_encoder_a2 = digitalRead(encoder2_pin_a);
}

void handle_encoder_b2() {
  if (digitalRead(encoder2_pin_a) == digitalRead(encoder2_pin_b)) {
    encoder_value2++;
  } else {
    encoder_value2--;
  }
  pulse2++;
}

void handle_encoder_a3() 
{
  if (digitalRead(encoder3_pin_a) != last_encoder_a3) {
    if (digitalRead(encoder3_pin_a) != digitalRead(encoder3_pin_b)) {
      encoder_value3++;
    } else {
      encoder_value3--;
    }
    pulse3++;
  }
  last_encoder_a3 = digitalRead(encoder3_pin_a);
}

void handle_encoder_b3() {
  if (digitalRead(encoder3_pin_a) == digitalRead(encoder3_pin_b)) {
    encoder_value3++;
  } else {
    encoder_value3--;
  }
  pulse3++;
}

void handle_encoder_a4() 
{
  if (digitalRead(encoder4_pin_a) != last_encoder_a4) {
    if (digitalRead(encoder4_pin_a) != digitalRead(encoder4_pin_b)) {
      encoder_value4++;
    } else {
      encoder_value4--;
    }
    pulse4++;
  }
  last_encoder_a4 = digitalRead(encoder4_pin_a);
}

void handle_encoder_b4() {
  if (digitalRead(encoder4_pin_a) == digitalRead(encoder4_pin_b)) {
    encoder_value4++;
  } else {
    encoder_value4--;
  }
  pulse4++;
}
