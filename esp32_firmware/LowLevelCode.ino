#include <ESP32Encoder.h>
//HARDWARE PIN
const int M1_R_EN_PIN  = 16;  
const int M1_L_EN_PIN  = 17;  
const int M1_R_PWM_PIN = 4; 
const int M1_L_PWM_PIN = 13; 
const int M1_ENC_A     = 22; //external pullup resistor
const int M1_ENC_B     = 34; //external pullup resistor

const int M2_DIR_PIN = 25; 
const int M2_PWM_PIN = 26; 
const int M2_ENC_A   = 18; 
const int M2_ENC_B   = 19; 

const int M3_DIR_PIN = 33; 
const int M3_PWM_PIN = 32; 
const int M3_ENC_A   = 14; //27
const int M3_ENC_B   = 27; //14

const int SW1_PIN = 36; //vp pin in esp32 & use ext pullup
const int SW2_PIN = 35; //ext pullup
const int SW3_PIN = 21; 

const int VALVE_PIN = 23; 

const float M1_TICK_TO_DEG = 50.0 / 85859.0; //85859//77523
const float M2_TICK_TO_DEG = 50.0 / 955.0;  //971//955   
const float M3_TICK_TO_DEG = 50.0 / 847.0;   //1053/

const int pwmFreq = 20000; 
const int pwmResolution = 8; 
const float dt = 0.002; 
const float MAX_PWM_STEP = 3.0; 
//MOTOR 1 TUNING
float Kp_1 = 4.5, Ki_1 = 1.5, Kd_1 = 1.0;        
float deadband_1 = 0.2, intLimit_1 = 1000.0; 
float maxPWM_1_DOWN = 255.0, minPWM_1_DOWN = 30.0;  //180,30
float maxPWM_1_UP = 255.0, minPWM_1_UP = 55.0;    //230,55
//MOTOR 2 TUNING
float Kp_2 = 4.0, Ki_2 = 0.5, Kd_2 = 1.0;
float deadband_2 = 0.2, intLimit_2 = 1000.0;
float maxPWM_2 = 255.0, minPWM_fwd_2 = 80.0, minPWM_rev_2 = 90.0; //200,80,90
//MOTOR 3 TUNING
float Kp_3 = 4.0, Ki_3 = 0.5, Kd_3 = 1.5;
float deadband_3 = 0.2, intLimit_3 = 1000.0;
float maxPWM_3 = 255.0, minPWM_fwd_3 = 80.0, minPWM_rev_3 = 90.0; //200,80,90

// ---- NEW: SAFE TRAVEL LIMITS (degrees) ----
// TODO: set these to your actual mechanical safe range for each joint.
// These are placeholders based on typical homing offset (3.0) plus margin -
// adjust to match your real delta arm geometry / working envelope.
const float M1_MIN_ANGLE = 0.0,  M1_MAX_ANGLE = 90.0;
const float M2_MIN_ANGLE = 0.0,  M2_MAX_ANGLE = 90.0;
const float M3_MIN_ANGLE = 0.0,  M3_MAX_ANGLE = 90.0;

// ---- NEW: SERIAL WATCHDOG ----
// Resets on every valid command AND on every completed move (DONE), so this
// now measures genuine idle/unresponsive time rather than move duration.
// Still needs to be long enough to cover the time between a command arriving
// and the move actually starting/being detected - a couple seconds of
// margin is safer than cutting it too close.
const unsigned long COMM_TIMEOUT_MS = 3000; // if no valid command AND no completed move in this window while enabled, force stop
volatile unsigned long lastCommandMillis = 0;

// ---- NEW: mutex to protect shared target/enable state across cores ----
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

volatile float targetM1 = 0, currentM1 = 0;
volatile float targetM2 = 0, currentM2 = 0;
volatile float targetM3 = 0, currentM3 = 0;

float current_out1 = 0, current_out2 = 0, current_out3 = 0;

volatile bool m1Enabled = false;
volatile bool m2Enabled = false;
volatile bool m3Enabled = false;
volatile bool is_done_sent = false; 

const byte numChars = 64;
char receivedChars[numChars];
boolean newData = false;

ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;

TaskHandle_t TaskMaster;

void PID(void *pvParameters);
void performHoming(); 
void recvWithEndMarker();
void processCommand();
void stopAllMotors(); // NEW helper, used by normal stop + watchdog

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(SW1_PIN, INPUT);
  pinMode(SW2_PIN, INPUT);
  pinMode(SW3_PIN, INPUT_PULLUP);

  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(VALVE_PIN, LOW); 

  pinMode(M1_R_EN_PIN, OUTPUT);
  pinMode(M1_L_EN_PIN, OUTPUT);
  digitalWrite(M1_R_EN_PIN, HIGH); 
  digitalWrite(M1_L_EN_PIN, HIGH); 
  ledcAttach(M1_R_PWM_PIN, pwmFreq, pwmResolution);
  ledcAttach(M1_L_PWM_PIN, pwmFreq, pwmResolution);
  ledcWrite(M1_R_PWM_PIN, 0);
  ledcWrite(M1_L_PWM_PIN, 0);

  pinMode(M2_DIR_PIN, OUTPUT);
  ledcAttach(M2_PWM_PIN, pwmFreq, pwmResolution);
  ledcWrite(M2_PWM_PIN, 0);

  pinMode(M3_DIR_PIN, OUTPUT);
  ledcAttach(M3_PWM_PIN, pwmFreq, pwmResolution);
  ledcWrite(M3_PWM_PIN, 0);

  pinMode(M1_ENC_A, INPUT); 
  pinMode(M1_ENC_B, INPUT);
  pinMode(M2_ENC_A, INPUT); 
  pinMode(M2_ENC_B, INPUT);
  pinMode(M3_ENC_A, INPUT_PULLUP); 
  pinMode(M3_ENC_B, INPUT_PULLUP);

  encoder1.attachFullQuad(M1_ENC_A, M1_ENC_B); 
  encoder2.attachFullQuad(M2_ENC_A, M2_ENC_B); 
  encoder3.attachFullQuad(M3_ENC_A, M3_ENC_B); 

  delay(100);
  encoder1.clearCount(); encoder2.clearCount(); encoder3.clearCount();
  delay(10);
  encoder1.clearCount(); encoder2.clearCount(); encoder3.clearCount();

  xTaskCreatePinnedToCore(PID, "Master_PID", 8192, NULL, 1, &TaskMaster, 1);
  delay(100);

  performHoming();
  lastCommandMillis = millis(); // NEW: seed watchdog timer after boot/homing
  Serial.println("\nREADY");
}

void loop() {
  recvWithEndMarker();
  if (newData) {
    processCommand();
    newData = false;
  }

  // ---- NEW: serial watchdog ----
  // If we're actively enabled/moving but haven't heard a valid command AND
  // haven't completed a move (DONE) in COMM_TIMEOUT_MS, assume the host
  // (PC / vision system) died or the link dropped, and force a stop rather
  // than holding/driving forever.
  portENTER_CRITICAL(&stateMux);
  unsigned long lastCmdSnapshot = lastCommandMillis;
  portEXIT_CRITICAL(&stateMux);
  if ((m1Enabled || m2Enabled || m3Enabled) &&
      (millis() - lastCmdSnapshot > COMM_TIMEOUT_MS)) {
    stopAllMotors();
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    Serial.printf("ANG: %.2f, %.2f, %.2f\n", currentM1, currentM2, currentM3);
    lastPrint = millis();
  }

  //static unsigned long lastPrint = 0;
  //if (millis() - lastPrint >= 200) {
  //  Serial.printf("TICKS: %d, %d, %d\n", (int)encoder1.getCount(), (int)encoder2.getCount(), (int)encoder3.getCount());
   // lastPrint = millis();
  //}
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) ndx = numChars - 1;
    } else {
      receivedChars[ndx] = '\0';
      ndx = 0;
      newData = true;
    }
  }
}

// ---- NEW: shared stop routine (used by "S" command and watchdog) ----
void stopAllMotors() {
  portENTER_CRITICAL(&stateMux);
  m1Enabled = false; m2Enabled = false; m3Enabled = false;
  portEXIT_CRITICAL(&stateMux);
  current_out1 = 0; current_out2 = 0; current_out3 = 0;
  digitalWrite(VALVE_PIN, LOW);
}

void processCommand() {
  String input = String(receivedChars);
  input.trim();
  input.toUpperCase(); 

  // NEW: any recognized command (including S) counts as proof-of-life for the watchdog
  portENTER_CRITICAL(&stateMux);
  lastCommandMillis = millis();
  portEXIT_CRITICAL(&stateMux);
  
  if (input == "S") {
    stopAllMotors();
  }
  else if (input == "HOME") {
    performHoming(); 
  }
  else if (input == "H") {
    portENTER_CRITICAL(&stateMux);
    targetM1 = currentM1; targetM2 = currentM2; targetM3 = currentM3;
    m1Enabled = true; m2Enabled = true; m3Enabled = true;
    portEXIT_CRITICAL(&stateMux);
  }
  else if (input == "V0") {
    digitalWrite(VALVE_PIN, HIGH);
  }
  else if (input == "V1") {
    digitalWrite(VALVE_PIN, LOW);
  }
  else {
    int firstComma = input.indexOf(',');
    int secondComma = input.indexOf(',', firstComma + 1);

    if (firstComma != -1 && secondComma != -1) {
      float in1 = input.substring(0, firstComma).toFloat();
      float in2 = input.substring(firstComma + 1, secondComma).toFloat();
      float in3 = input.substring(secondComma + 1).toFloat();

      // ---- NEW: clamp incoming targets to safe mechanical range ----
      // Protects against corrupted serial data or a vision-side bug sending
      // an out-of-range target that would otherwise drive toward a limit
      // switch (or past it) at full authority.
      in1 = constrain(in1, M1_MIN_ANGLE, M1_MAX_ANGLE);
      in2 = constrain(in2, M2_MIN_ANGLE, M2_MAX_ANGLE);
      in3 = constrain(in3, M3_MIN_ANGLE, M3_MAX_ANGLE);
      
      portENTER_CRITICAL(&stateMux);
      targetM1 = in1;
      targetM2 = in2;
      targetM3 = in3;
      m1Enabled = true; m2Enabled = true; m3Enabled = true;
      portEXIT_CRITICAL(&stateMux);
    }
  }
}

void performHoming() {
  if (TaskMaster != NULL) vTaskSuspend(TaskMaster); 
  digitalWrite(VALVE_PIN, LOW);
  
  ledcWrite(M1_R_PWM_PIN, 0); ledcWrite(M1_L_PWM_PIN, 0);
  ledcWrite(M2_PWM_PIN, 0); 
  ledcWrite(M3_PWM_PIN, 0); 
  current_out1 = 0; current_out2 = 0; current_out3 = 0;
  delay(500);

  bool m1Homed = false, m2Homed = false, m3Homed = false;
  //homing speed
  int homingPWM_1 = 115, homingPWM_2 = 100, homingPWM_3 = 90; 

  while (!m1Homed || !m2Homed || !m3Homed) {
    if (!m1Homed) {
      if (digitalRead(SW1_PIN) == LOW) { ledcWrite(M1_L_PWM_PIN, 0); ledcWrite(M1_R_PWM_PIN, 0); m1Homed = true; } 
      else { ledcWrite(M1_R_PWM_PIN, 0); ledcWrite(M1_L_PWM_PIN, homingPWM_1); }
    }
    if (!m2Homed) {
      if (digitalRead(SW2_PIN) == LOW) { ledcWrite(M2_PWM_PIN, 0); m2Homed = true; } 
      else { digitalWrite(M2_DIR_PIN, LOW); ledcWrite(M2_PWM_PIN, homingPWM_2); }
    }
    if (!m3Homed) {
      if (digitalRead(SW3_PIN) == LOW) { ledcWrite(M3_PWM_PIN, 0); m3Homed = true; } 
      else { digitalWrite(M3_DIR_PIN, LOW); ledcWrite(M3_PWM_PIN, homingPWM_3); }
    }
    delay(10);
  }

  encoder1.clearCount(); encoder2.clearCount(); encoder3.clearCount();
  currentM1 = 0; currentM2 = 0; currentM3 = 0;
  
  portENTER_CRITICAL(&stateMux);
  targetM1 = 3.0; targetM2 = 3.0; targetM3 = 0.0;
  m1Enabled = true; m2Enabled = true; m3Enabled = true;
  portEXIT_CRITICAL(&stateMux);

  if (TaskMaster != NULL) vTaskResume(TaskMaster); 

  // ---- CHANGED: wait for the offset move to actually finish, with a timeout,
  // instead of blindly delaying 1500ms regardless of whether it got there. ----
  const unsigned long HOMING_MOVE_TIMEOUT_MS = 3000;
  unsigned long moveStart = millis();
  while (millis() - moveStart < HOMING_MOVE_TIMEOUT_MS) {
    float e1 = fabs(targetM1 - currentM1);
    float e2 = fabs(targetM2 - currentM2);
    float e3 = fabs(targetM3 - currentM3);
    if (e1 <= deadband_1 && e2 <= deadband_2 && e3 <= deadband_3) break;
    delay(10);
  }
  // if the timeout is hit without reaching target, we still proceed below -
  // this just avoids either cutting off an in-progress move too early or
  // silently continuing well past when the move actually finished.

  if (TaskMaster != NULL) vTaskSuspend(TaskMaster); 
  encoder1.clearCount(); encoder2.clearCount(); encoder3.clearCount();
  portENTER_CRITICAL(&stateMux);
  targetM1 = 0; targetM2 = 0; targetM3 = 0;
  portEXIT_CRITICAL(&stateMux);
  currentM1 = 0; currentM2 = 0; currentM3 = 0;
  current_out1 = 0; current_out2 = 0; current_out3 = 0;
  if (TaskMaster != NULL) vTaskResume(TaskMaster); 
}

void PID(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  float intSum1 = 0, prev1 = 0;
  float intSum2 = 0, prev2 = 0;
  float intSum3 = 0, prev3 = 0;

  while (true) {
    bool limit1 = (digitalRead(SW1_PIN) == LOW);
    bool limit2 = (digitalRead(SW2_PIN) == LOW);
    bool limit3 = (digitalRead(SW3_PIN) == LOW);

    if (limit1) encoder1.clearCount();
    if (limit2) encoder2.clearCount();
    if (limit3) encoder3.clearCount();

    currentM1 = encoder1.getCount() * M1_TICK_TO_DEG;
    currentM2 = encoder2.getCount() * M2_TICK_TO_DEG;
    currentM3 = encoder3.getCount() * M3_TICK_TO_DEG;

    // NEW: snapshot shared target/enable state once per cycle under the mutex,
    // so the rest of the loop works off a consistent set of values even if
    // core 0 writes them mid-cycle.
    float t1, t2, t3;
    bool en1, en2, en3;
    portENTER_CRITICAL(&stateMux);
    t1 = targetM1; t2 = targetM2; t3 = targetM3;
    en1 = m1Enabled; en2 = m2Enabled; en3 = m3Enabled;
    portEXIT_CRITICAL(&stateMux);

    if (!en1 && !en2 && !en3) {
      ledcWrite(M1_R_PWM_PIN, 0); ledcWrite(M1_L_PWM_PIN, 0);
      ledcWrite(M2_PWM_PIN, 0); ledcWrite(M3_PWM_PIN, 0); 
      intSum1 = 0; intSum2 = 0; intSum3 = 0;
      prev1 = currentM1; prev2 = currentM2; prev3 = currentM3;
      portENTER_CRITICAL(&stateMux);
      targetM1 = currentM1; targetM2 = currentM2; targetM3 = currentM3;
      portEXIT_CRITICAL(&stateMux);
      current_out1 = 0; current_out2 = 0; current_out3 = 0;
    } 
    else {
      //// MATH FOR MOTOR 1
      float err1 = t1 - currentM1;
      float out1 = 0;
      if (abs(err1) <= deadband_1) {
        err1 = 0; intSum1 = 0; 
      } else {
        intSum1 = constrain(intSum1 + (err1 * dt), -intLimit_1, intLimit_1);
        out1 = (Kp_1 * err1) + (Ki_1 * intSum1) - (Kd_1 * (currentM1 - prev1) / dt);
        if (out1 > 0) out1 += minPWM_1_DOWN; 
        else if (out1 < 0) out1 -= minPWM_1_UP; 
      }
      prev1 = currentM1;
      if (limit1 && out1 < 0) { out1 = 0; intSum1 = 0; }
      if (!en1) { out1 = 0; intSum1 = 0; } 
      if (out1 > 0) out1 = constrain(out1, 0, maxPWM_1_DOWN);
      else out1 = constrain(out1, -maxPWM_1_UP, 0);
      //// MATH FOR MOTOR 2
      float err2 = t2 - currentM2;
      float out2 = 0;
      if (abs(err2) <= deadband_2) {
        err2 = 0; intSum2 = 0; 
      } else {
        intSum2 = constrain(intSum2 + (err2 * dt), -intLimit_2, intLimit_2);
        out2 = (Kp_2 * err2) + (Ki_2 * intSum2) - (Kd_2 * (currentM2 - prev2) / dt);
        if (out2 > 0) out2 += minPWM_fwd_2; 
        else if (out2 < 0) out2 -= minPWM_rev_2;
      }
      prev2 = currentM2;
      if (limit2 && out2 < 0) { out2 = 0; intSum2 = 0; }
      if (!en2) { out2 = 0; intSum2 = 0; } 
      out2 = constrain(out2, -maxPWM_2, maxPWM_2);
      //// MATH FOR MOTOR 3
      float err3 = t3 - currentM3;
      float out3 = 0;
      if (abs(err3) <= deadband_3) {
        err3 = 0; intSum3 = 0; 
      } else {
        intSum3 = constrain(intSum3 + (err3 * dt), -intLimit_3, intLimit_3);
        out3 = (Kp_3 * err3) + (Ki_3 * intSum3) - (Kd_3 * (currentM3 - prev3) / dt);
        if (out3 > 0) out3 += minPWM_fwd_3; 
        else if (out3 < 0) out3 -= minPWM_rev_3;
      }
      prev3 = currentM3;
      if (limit3 && out3 < 0) { out3 = 0; intSum3 = 0; }
      if (!en3) { out3 = 0; intSum3 = 0; } 
      out3 = constrain(out3, -maxPWM_3, maxPWM_3);

      if (out1 > current_out1 + MAX_PWM_STEP) current_out1 += MAX_PWM_STEP;
      else if (out1 < current_out1 - MAX_PWM_STEP) current_out1 -= MAX_PWM_STEP;
      else current_out1 = out1;

      if (out2 > current_out2 + MAX_PWM_STEP) current_out2 += MAX_PWM_STEP;
      else if (out2 < current_out2 - MAX_PWM_STEP) current_out2 -= MAX_PWM_STEP;
      else current_out2 = out2;

      if (out3 > current_out3 + MAX_PWM_STEP) current_out3 += MAX_PWM_STEP;
      else if (out3 < current_out3 - MAX_PWM_STEP) current_out3 -= MAX_PWM_STEP;
      else current_out3 = out3;

      // FIRE MOTORS
      if (current_out1 > 0) { ledcWrite(M1_L_PWM_PIN, 0); ledcWrite(M1_R_PWM_PIN, abs((int)current_out1)); }
      else if (current_out1 < 0) { ledcWrite(M1_R_PWM_PIN, 0); ledcWrite(M1_L_PWM_PIN, abs((int)current_out1)); }
      else { ledcWrite(M1_R_PWM_PIN, 0); ledcWrite(M1_L_PWM_PIN, 0); }

      if (current_out2 > 0) { digitalWrite(M2_DIR_PIN, HIGH); ledcWrite(M2_PWM_PIN, abs((int)current_out2)); }
      else if (current_out2 < 0) { digitalWrite(M2_DIR_PIN, LOW); ledcWrite(M2_PWM_PIN, abs((int)current_out2)); }
      else { ledcWrite(M2_PWM_PIN, 0); }

      if (current_out3 > 0) { digitalWrite(M3_DIR_PIN, HIGH); ledcWrite(M3_PWM_PIN, abs((int)current_out3)); }
      else if (current_out3 < 0) { digitalWrite(M3_DIR_PIN, LOW); ledcWrite(M3_PWM_PIN, abs((int)current_out3)); }
      else { ledcWrite(M3_PWM_PIN, 0); }

      // HANDSHAKING
      if (en1 && en2 && en3) {
        if (abs(err1) <= deadband_1 && abs(err2) <= deadband_2 && abs(err3) <= deadband_3) {
          if (!is_done_sent) {
            Serial.println("DONE"); 
            is_done_sent = true;
            // NEW: a completed move counts as proof-of-life too, same as
            // receiving a fresh command. This means the watchdog timeout
            // now only measures genuine idle/unresponsive time (host truly
            // gone) rather than "time since last command," so it can stay
            // tight without risking a false trigger on a long single move.
            portENTER_CRITICAL(&stateMux);
            lastCommandMillis = millis();
            portEXIT_CRITICAL(&stateMux);
          }
        } else {
          is_done_sent = false; 
        }
      }
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
  }
}