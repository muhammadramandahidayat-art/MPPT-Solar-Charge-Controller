/*
 * PI CONTROL CHARGER - BUCK CONVERTER (INA219 + ADS1115)
 * SPESIAL UNTUK AKI 12V 5Ah
 * * Hardware Map (Sesuai Tes Manual Anda):
 * - Input V  : ADS1115 Channel 0 (A0)
 * - Output V : ADS1115 Channel 2 (A2)
 * - Arus     : INA219 (0x40)
 * - Driver   : Pin 25 (PWM), Pin 26 (SD)
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h> 
#include <Adafruit_INA219.h> 
#include <math.h>

// ==========================================
// 1. KONFIGURASI TARGET CHARGING (12V 5Ah)
// ==========================================
const float V_ABSORPTION = 14.4; // Batas Tegangan Penuh
const float V_FLOAT      = 13.6; // Batas Tegangan Standby
const float I_MAX_CHARGE = 1.0;  // Batas Arus (1A untuk aki 5Ah)

// ==========================================
// 2. PIN MAPPING & SENSOR
// ==========================================
#define PWM_PIN 25  
#define SD_PIN  26  

// ADS1115 Channel Map (Sesuai Tes Manual)
#define CH_VIN_BUCK  0  
#define CH_VOUT_BUCK 2  

Adafruit_ADS1115 ads;
Adafruit_INA219 ina219(0x40);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Kalibrasi (Sesuai Tes Manual)
float adsFactor = 0.125F; 
double V_DIV_IN  = 11.32; 
double V_DIV_OUT = 10.87;

// Tuning PI (Versi Smooth / Stabil)
float Kp_V = 10.0;  // Turunkan dikit
float Ki_V = 1.0;   // Integral diperlambat
float Kp_I = 4.0;   // Turun jauh biar tidak agresif
float Ki_I = 0.5;   // Biar koreksinya pelan tapi pasti

float integral_err = 0; // Penampung nilai integral
int pwm_out = 0;        // Output PWM (0-1023)

// ==========================================
// 4. STATE MANAGEMENT
// ==========================================
enum ChargerState { OFF, BULK_CC, ABSORPTION_CV, FLOAT_CV };
ChargerState state = OFF;
String stateStr = "OFF";

// Variabel Data
double v_in = 0, v_out = 0, i_out = 0;
unsigned long lastLoop = 0;
unsigned long lastLCD = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Init Driver
  pinMode(SD_PIN, OUTPUT); digitalWrite(SD_PIN, LOW);
  ledcSetup(0, 25000, 10); // 25kHz, 10-bit
  ledcAttachPin(PWM_PIN, 0);
  ledcWrite(0, 0);

  // Init LCD
  lcd.init(); lcd.backlight();
  lcd.setCursor(0,0); lcd.print("PI CHARGER INIT");

  // Init Sensor
  ads.begin(); ads.setGain(GAIN_ONE);
  if (!ina219.begin()) {
    lcd.setCursor(0,1); lcd.print("INA219 ERR"); while(1);
  }

  delay(1000); lcd.clear();
  digitalWrite(SD_PIN, HIGH); // Aktifkan Driver
  Serial.println("=== SYSTEM START ===");
}

void loop() {
  // Loop Control (100Hz / setiap 10ms)
  if (millis() - lastLoop > 10) {
    readSensors();
    runStateMachine();
    runPIControl();
    ledcWrite(0, pwm_out);
    lastLoop = millis();
  }

  // Display Update (2Hz / setiap 500ms)
  if (millis() - lastLCD > 500) {
    printData();
    lastLCD = millis();
  }
}

void readSensors() {
  // Baca Tegangan
  int16_t rVin = ads.readADC_SingleEnded(CH_VIN_BUCK);
  int16_t rVout = ads.readADC_SingleEnded(CH_VOUT_BUCK);
  
  v_in  = ((rVin * adsFactor) / 1000.0) * V_DIV_IN;
  v_out = ((rVout * adsFactor) / 1000.0) * V_DIV_OUT;
  
  // Baca Arus
  i_out = ina219.getCurrent_mA() / 1000.0;
  
  // Filter Zero
  if(v_in < 1.0) v_in = 0;
  if(v_out < 0.5) v_out = 0;
  if(i_out < 0) i_out = 0;
}

void runStateMachine() {
  // 1. SAFETY CHECK
  if (v_in < 12.0) { // Input kurang dari 12V
    state = OFF;
    return;
  }
  
  // 2. LOGIKA PINDAH MODE
  switch (state) {
    case OFF:
      // Start jika input ada dan baterai terdeteksi (>1V)
      // ATAU paksa start jika input tinggi (Soft Start baterai kosong)
      if (v_in > 13.0) {
        state = BULK_CC;
        integral_err = 0; // Reset Integral saat mulai
      }
      break;

    case BULK_CC:
      // Jika tegangan sudah capai 14.4V, pindah ke mode tegangan
      if (v_out >= V_ABSORPTION) {
        state = ABSORPTION_CV;
        integral_err = 0; // Reset agar transisi mulus
      }
      break;

    case ABSORPTION_CV:
      // Jika arus sudah turun di bawah 0.2A (Aki Penuh), pindah Float
      if (i_out < 0.2) {
        state = FLOAT_CV;
        integral_err = 0;
      }
      // Jika tegangan drop (kena beban), balik ke Bulk
      if (v_out < V_ABSORPTION - 0.5) state = BULK_CC;
      break;

    case FLOAT_CV:
      // Jaga tegangan di 13.6V. Jika drop jauh, charge ulang
      if (v_out < 12.8) state = BULK_CC;
      break;
  }
}

void runPIControl() {
  if (state == OFF) {
    pwm_out = 0;
    return;
  }

  float error = 0;
  float Kp = 0, Ki = 0;
  float setpoint = 0;
  float measurement = 0;

  // --- PILIH STRATEGI KONTROL BERDASARKAN STATE ---
  
  if (state == BULK_CC) {
    // FOKUS: Jaga Arus Konstan 1.0A
    setpoint = I_MAX_CHARGE; 
    measurement = i_out;
    Kp = Kp_I; Ki = Ki_I;
  } 
  else if (state == ABSORPTION_CV) {
    // FOKUS: Jaga Tegangan 14.4V
    setpoint = V_ABSORPTION;
    measurement = v_out;
    Kp = Kp_V; Ki = Ki_V;
  }
  else if (state == FLOAT_CV) {
    // FOKUS: Jaga Tegangan 13.6V
    setpoint = V_FLOAT;
    measurement = v_out;
    Kp = Kp_V; Ki = Ki_V;
  }

  // --- RUMUS PI (Proportional + Integral) ---
  error = setpoint - measurement;
  
  // Akumulasi Integral (dengan batasan/clamping agar tidak overflow)
  integral_err += error;
  integral_err = constrain(integral_err, -1000, 1000); // Anti Windup

  // Hitung Output PWM
  float output = (Kp * error) + (Ki * integral_err);
  
  // --- FEEDFORWARD (PENTING UNTUK STARTUP) ---
  // PWM dasar = Vout / Vin * 1023
  // Ini membantu PI supaya tidak kerja dari nol
  if (v_in > 1.0) {
    output += (v_out / v_in) * 1023.0; 
  }

  // Batasi PWM (0 - 950 agar driver bisa bootstrap)
  pwm_out = constrain((int)output, 0, 950);
}

void printData() {
  switch(state) {
    case OFF: stateStr = "OFF"; break;
    case BULK_CC: stateStr = "CC-1A"; break;
    case ABSORPTION_CV: stateStr = "CV-14.4"; break;
    case FLOAT_CV: stateStr = "FLT-13.6"; break;
  }

  // Serial Monitor
  Serial.print("State:"); Serial.print(stateStr);
  Serial.print(" | Vin:"); Serial.print(v_in);
  Serial.print(" | Vout:"); Serial.print(v_out);
  Serial.print(" | I:"); Serial.print(i_out);
  Serial.print(" | PWM:"); Serial.println(pwm_out);

  // LCD
  lcd.setCursor(0,0);
  lcd.print(stateStr); lcd.print("    ");
  lcd.setCursor(9,0); 
  lcd.print(v_in, 1); lcd.print("V");

  lcd.setCursor(0,1);
  lcd.print(v_out, 1); lcd.print("V ");
  lcd.print(i_out, 2); lcd.print("A  ");
}
