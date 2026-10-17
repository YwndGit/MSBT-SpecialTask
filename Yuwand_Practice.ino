#include <Wire.h>
#include <ESP32Servo.h>

const int     PIN_SDA   = 21;      
const int     PIN_SCL   = 22;      
const int     PIN_SERVO = 13;      
const uint8_t MPU_ADDR  = 0x68;    

const unsigned long LOOP_US = 10000; 
const float SERVO_CENTER = 90.0;     
const float SERVO_LIMIT  = 40.0;     
const float SERVO_DIR    = +1.0;     
const float ALPHA        = 0.90;     
const float D_FILTER     = 0.05;     

float Kp = 2.0;    
float Ki = 0.0;     // i dinolkan untuk testing servo
float Kd = 0.05;   
float setpoint = 0.0;   // target pitch (derajat) 0 = datar

Servo servo;
float pitch     = 0.0;   
float pitchRate = 0.0;    
float integral  = 0.0;   
float dFilt     = 0.0;   
float output    = 0.0;   
unsigned long lastLoopUs  = 0;
unsigned long lastPrintMs = 0;
 
float clampf(float v, float lo, float hi) {     
  return v < lo ? lo : (v > hi ? hi : v);
}

void mpuWrite(uint8_t reg, uint8_t val) {       
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
 

bool readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;    
  int16_t r[7];
  for (int i = 0; i < 7; i++) r[i] = (Wire.read() << 8) | Wire.read();
  ax = r[0] / 16384.0;  ay = r[1] / 16384.0;  az = r[2] / 16384.0;
  gx = r[4] / 131.0;    gy = r[5] / 131.0;    gz = r[6] / 131.0;
  return true;
}

bool updatePitch(float dt) {
  float ax, ay, az, gx, gy, gz;
  if (!readMPU(ax, ay, az, gx, gy, gz)) return false;
  float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;  // sudut dari accel
  pitchRate = gy;                                                      // laju dari gyro
  pitch = ALPHA * (pitch + pitchRate * dt) + (1.0 - ALPHA) * pitchAcc;
  return true;
}

void computePID(float dt) {
  float error = setpoint - pitch;

  float P = Kp * error;

  dFilt += (dt / (D_FILTER + dt)) * (pitchRate - dFilt);   
  float D = -Kd * dFilt;

  float uUnsat = P + integral + D;                         
  float u      = clampf(uUnsat, -SERVO_LIMIT, SERVO_LIMIT);

  bool saturated    = (u != uUnsat);                                     
  bool pushesDeeper = saturated && ((error > 0) == (uUnsat > 0));         
  if (!pushesDeeper) integral += Ki * error * dt;                        

  output = u;
}

void driveServo() {
  float angle = SERVO_CENTER + SERVO_DIR * output;
  servo.writeMicroseconds(500 + (int)(angle / 180.0 * 1900.0));
}

void printLog() {
  if (millis() - lastPrintMs < 50) return;     
  lastPrintMs = millis();
  Serial.print(setpoint, 2); Serial.print(',');
  Serial.print(pitch, 2);    Serial.print(',');
  Serial.println(output, 2);
}

void handleSerial() {
  if (!Serial.available()) return;  
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() < 1) return;
  char  c = s[0];
  float v = s.substring(1).toFloat();
  if      (c == 's') setpoint = v;
  else if (c == 'p') Kp = v;
  else if (c == 'i') Ki = v;
  else if (c == 'd') Kd = v;
  else if (c == 'r') integral = 0;
}
 
void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  mpuWrite(0x6B, 0x00);                  

  ESP32PWM::allocateTimer(0);           
  servo.setPeriodHertz(50);              
  servo.attach(PIN_SERVO, 500, 2400);    
  servo.writeMicroseconds(500 + (int)(SERVO_CENTER / 180.0 * 1900.0));  

  float ax, ay, az, gx, gy, gz;         
  if (readMPU(ax, ay, az, gx, gy, gz)) pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  else Serial.println("PERINGATAN: MPU6050 tidak terbaca, cek kabel SDA/SCL.");

  lastLoopUs = micros();
  Serial.println("setpoint,pitch,servo_offset");
}
 
void loop() {
  handleSerial();
 
  unsigned long now = micros();
  if (now - lastLoopUs < LOOP_US) return;
  float dt = (now - lastLoopUs) * 1e-6;     
  lastLoopUs = now;
  if (dt > 0.05) dt = 0.05;                  

  if (!updatePitch(dt)) return;              
  computePID(dt);                           
  driveServo();                            
  printLog();                               
}
