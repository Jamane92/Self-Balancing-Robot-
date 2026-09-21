//Board : ESP32

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "FastAccelStepper.h"

// --- CONFIGURATION OLED ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- CONFIGURATION MOTEURS ---
#define DIR_PIN_A 25
#define STEP_PIN_A 26
#define DIR_PIN_B 27
#define STEP_PIN_B 14

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperL = NULL;
FastAccelStepper *stepperR = NULL;

// --- CONFIGURATION BATTERIE & LED ---
const int ledPin = 12;
const int batteryPin = 36;
const float divisionFactor = 5.5454; 
const float calibrationFactor = 1.098;
float adcValue = 0;
float voltageOnPin = 0;
float batteryVoltage = 0;

// --- VARIABLES GLOBALES MPU6050 ---
int16_t gyro_x, gyro_y, gyro_z; 
int16_t acc_x, acc_y, acc_z;    
long acc_total_vector;
int16_t temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
unsigned long loop_timer;       
float angle_pitch, angle_roll;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

// --- VARIABLES PID ---
float Kp = 325;  //525 //400
float Ki = 5;   //0.5 //3 //les deuxiemes parametres fonctionnent avec l'anti wind up d'activé
float Kd = 550; //50 //140
float integral, last_error, error, pid_output;

float setpoint = 0;

// --- NOUVEAUX PARAMÈTRES DE PERFORMANCE ---
float min_speed = 0; //100 // Vitesse minimale (Hz) pour vaincre l'inertie
float deadzone = 0.1;   //0,1
// Compteurs
int print_counter = 0;
int oled_counter = 0;
int led_counter = 0;
bool led_state = false;

void setup() {
  Wire.begin(21, 22);
  Wire.setClock(400000); 
  Serial.begin(115200);
  
  pinMode(2, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
  analogSetAttenuation(ADC_11db);
  /*
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 introuvable"));
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("MPU6050 + OLED");
  display.println("Calibrage...");
  display.display();
  */
  engine.init();
  stepperL = engine.stepperConnectToPin(STEP_PIN_A);
  if (stepperL) { 
    stepperL->setDirectionPin(DIR_PIN_A); 
    stepperL->setAcceleration(30000); 
  }
  stepperR = engine.stepperConnectToPin(STEP_PIN_B);
  if (stepperR) { 
    stepperR->setDirectionPin(DIR_PIN_B); 
    stepperR->setAcceleration(30000); 
  }
  delay(1000);
  setup_mpu_6050_registers();

  digitalWrite(2, HIGH); 
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){
    read_mpu_6050_data();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    delay(3);
  }
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
  digitalWrite(2, LOW); 
  /*
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("System Ready!");
  display.display();
  */
  delay(200);

  loop_timer = micros();
}

void loop(){
  // 1. Lecture et traitement IMU
  read_mpu_6050_data();
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
  
  angle_pitch += gyro_x * 0.0000611;
  angle_roll += gyro_y * 0.0000611;
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);
  
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;
  
  if(set_gyro_angles){
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
  } else {
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    set_gyro_angles = true;
  }
  
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;

  // --- CALCUL PID ---
  error = angle_roll_output - setpoint; 

  // Anti-windup lors du changement de signe
  
if ((error > 0 && last_error < 0) || (error < 0 && last_error > 0)) {
    integral = 0;
}
  
// Accumulation conditionnelle
if (abs(error) > 0.3) {
    integral = constrain(integral + error, -5000, 5000); //test avec 3000, marchait avant avec -+1000 
} else {
    integral *= 0.9;  // Décroissance
}
  float D = Kd * (error - last_error);
  pid_output = (Kp * error) + (Ki * integral) + D;
  last_error = error;

  // --- CONTRÔLE MOTEURS AVEC OFFSET ---
  float speed_hz = 0;
  float current_abs_error = abs(error);

  if (current_abs_error > 30) {
    // Sécurité chute : On coupe tout
    stepperL->stopMove();
    stepperR->stopMove();
    integral = 0;
  } 
  else if (current_abs_error < deadzone) {
    // Zone de stabilité : On arrête les moteurs pour éviter sifflements et jitter
    stepperL->stopMove();
    stepperR->stopMove();
    speed_hz = 0;
  } 
  else {
    // On calcule la vitesse en ajoutant l'offset min_speed
    speed_hz = abs(pid_output) + min_speed;
    speed_hz = constrain(speed_hz, 0, 20000);

    if (pid_output < 0) {
      stepperL->setSpeedInHz((uint32_t)speed_hz); 
      stepperL->runForward();
      stepperR->setSpeedInHz((uint32_t)speed_hz); 
      stepperR->runBackward();
    } else {
      stepperL->setSpeedInHz((uint32_t)speed_hz); 
      stepperL->runBackward();
      stepperR->setSpeedInHz((uint32_t)speed_hz); 
      stepperR->runForward();
    }
  }

  // --- GESTION LED & AFFICHAGE ---
  led_counter++;
  if (led_counter >= 250) {
    led_state = !led_state;
    digitalWrite(ledPin, led_state); 
    led_counter = 0;
  }
  /*
  oled_counter++;
  if (oled_counter >= 25) {
    adcValue = analogRead(batteryPin);
    voltageOnPin = (adcValue * 3.3) / 4095.0;
    batteryVoltage = (voltageOnPin * divisionFactor) * calibrationFactor;

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Roll: "); display.println(angle_roll_output, 1);
    display.setCursor(0, 15);
    display.print("Bat: "); display.print(batteryVoltage, 1); display.println("V");
    display.setCursor(0, 30);
    display.print("PID: "); display.print(pid_output, 0);
    display.setCursor(0, 45);
    display.print("Hz Total: "); display.println(speed_hz, 0);
    display.display();
    oled_counter = 0;
  }
  */
  while(micros() - loop_timer < 4000) yield();
  loop_timer = micros();
}

void read_mpu_6050_data(){
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  while(Wire.available() < 14);
  
  acc_x = (int16_t)(Wire.read()<<8|Wire.read());
  acc_y = (int16_t)(Wire.read()<<8|Wire.read());
  acc_z = (int16_t)(Wire.read()<<8|Wire.read());
  temperature = (int16_t)(Wire.read()<<8|Wire.read());
  gyro_x = (int16_t)(Wire.read()<<8|Wire.read());
  gyro_y = (int16_t)(Wire.read()<<8|Wire.read());
  gyro_z = (int16_t)(Wire.read()<<8|Wire.read());
}

void setup_mpu_6050_registers(){
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}
