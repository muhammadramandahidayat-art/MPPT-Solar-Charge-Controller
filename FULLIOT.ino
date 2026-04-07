/*
 * FINAL ULTIMATE VERSION - SKRIPSI READY
 * Fitur: Hybrid MPPT, Adaptive Buck, Bus Protection
 * Update: Fast Response Voltage & Transition Delay
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h>

// =================================================
// PIN & HARDWARE
// =================================================
#define BST_PWM_PIN 19
#define BST_SD_PIN  18
#define BCK_PWM_PIN 25
#define BCK_SD_PIN  26

#define ACS_PV_PIN  35
#define ACS_BAT_PIN 34

// =================================================
// ADS1115 CHANNEL MAPPING
// =================================================
#define CH_SOLAR_V   0   
#define CH_DCBUS_V   1   
#define CH_BCK_OUT_V 2   // A2: Output Buck
#define CH_BATT_V    3   // A3: Aki Langsung

LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_ADS1X15 ads;

// =================================================
// KALIBRASI SENSOR (SESUAI REQUEST TERAKHIR)
// =================================================
float DIV_SOLAR   = 11.27;   
float DIV_BATT    = 5.417;  // Update terakhir
float DIV_DCBUS   = 10.899;  
float DIV_BCK_OUT = 8.8;

;  

// SENSITIVITAS
const float ACS_SENS_PV  = 185.0; 
float ACS_SENS_BAT       = 272.72; 

float ACS_PV_ZERO  = 2.3916;
float ACS_BAT_ZERO = 2.4112;

// =================================================
// PARAMETER BATERAI & CHARGER
// =================================================
const float V_ABSORPTION = 14.40;
const float V_FLOAT      = 13.60;
const float I_MAX_CHARGE = 0.50;  // Fixed typo 00.50 -> 0.50
const float I_DEADBAND   = 0.03;

// [BARU] TIMER JEDA TRANSISI
unsigned long cv_transition_timer = 0; 
const int DELAY_CV_MS = 10000; // Jeda 3 detik sebelum pindah CC->CV

// DATA GLOBAL
double pv_vin=0, dc_bus_vol=0, bck_vout=0, bat_vin=0;
double pv_iin=0, bat_iout=0, pv_power=0;
float pv_iin_f = 0, bat_iout_f = 0;
float dc_bus_avg = 0;

// PWM
int bst_pwm = 1023; 
int bck_pwm = 0;    

// MPPT
double v_ref_mppt = 17.0;
double prev_power = 0, prev_vin = 0;
unsigned long lastPnO = 0;

enum ChargerState { OFF, BULK_CC, ABSORP_CV, FLOAT_CV };
ChargerState chg_state = OFF;
String stateStr = "OFF";
float integral_err = 0;

unsigned long lastLoop = 0, lastLog = 0, lastControl = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Safety Init
  pinMode(BST_SD_PIN, OUTPUT); digitalWrite(BST_SD_PIN, LOW); 
  pinMode(BCK_SD_PIN, OUTPUT); digitalWrite(BCK_SD_PIN, LOW); 

  // Setup PWM
  ledcSetup(0, 20000, 10);
  ledcAttachPin(BST_PWM_PIN, 0);
  ledcWrite(0, 1023); 

  ledcSetup(1, 40000, 10);
  ledcAttachPin(BCK_PWM_PIN, 1);
  ledcWrite(1, 0);    

  ads.begin();
  ads.setGain(GAIN_ONE);

  lcd.init();
  lcd.backlight();
  lcd.print("SYSTEM START...");
  
  // Auto Calibration Zero
  lcd.setCursor(0,1); lcd.print("Calibrating...");
  long zeroPV = 0, zeroBat = 0;
  for(int i=0; i<100; i++) {
     zeroPV += analogRead(ACS_PV_PIN);
     zeroBat += analogRead(ACS_BAT_PIN);
     delay(2);
  }
  ACS_PV_ZERO = (zeroPV / 100.0 / 4095.0) * 3.3;
  ACS_BAT_ZERO = (zeroBat / 100.0 / 4095.0) * 3.3;

  delay(1000);
  lcd.clear();

  readAllSensors();
  if (pv_vin > 12.0) v_ref_mppt = pv_vin;
}

void loop() {
  if (millis() - lastLoop >= 50) {
    readAllSensors();
    runCascadedMPPT();
    lastLoop = millis();
  }

  if (millis() - lastControl >= 10) {
    runBuckPICharger();
    ledcWrite(0, bst_pwm);
    ledcWrite(1, bck_pwm);
    lastControl = millis();
  }

  if (millis() - lastLog >= 500) {
    printStatus();
    lastLog = millis();
  }
}

void runCascadedMPPT() {
  // Safety Overvoltage
  if (dc_bus_vol > 25.0) { 
    bst_pwm = 1023; 
    ledcWrite(0, 1023); 
    digitalWrite(BST_SD_PIN, LOW); 
    return; 
  }
  
  // Active Clamping > 24V
  if (dc_bus_vol > 24.0) {
      bst_pwm += 2; 
      bst_pwm = constrain(bst_pwm, 100, 1023);
      digitalWrite(BST_SD_PIN, HIGH);
      return; 
  }

  if (pv_vin < 10.0) { 
    bst_pwm = 1023; 
    digitalWrite(BST_SD_PIN, LOW); 
    return; 
  }

  digitalWrite(BST_SD_PIN, HIGH);

  // MPPT P&O
  unsigned long mppt_interval = (pv_power > 80) ? 80 : 200;
if (millis() - lastPnO > mppt_interval)
 {
    double dP = pv_power - prev_power;
    double dV = pv_vin - prev_vin;
    float step;
if (pv_power > 120)      step = 1.2;
else if (pv_power > 80)  step = 0.8;
else if (pv_power > 40)  step = 0.5;
else                     step = 0.25;

    if (abs(dP) > 0.05) {
      if (dP > 0) v_ref_mppt += (dV > 0 ? step : -step);
      else v_ref_mppt -= (dV > 0 ? step : -step);
    }
    v_ref_mppt = constrain(v_ref_mppt, 13.5, 19.2);
    prev_power = pv_power; prev_vin = pv_vin;
    lastPnO = millis();
  }

  // MPPT Inner Loop (SMC-like)
  double error = pv_vin - v_ref_mppt;
 int K;
if (pv_power > 100)      K = 8;
else if (pv_power > 60)  K = 5;
else                     K = 3;

if (error > 0.15) bst_pwm -= K; 
else if (error < -0.15) bst_pwm += K;
  bst_pwm = constrain(bst_pwm, 100, 1023);
}

void runBuckPICharger() {
  // Cut-off Safety
  if (dc_bus_vol < bat_vin + 0.5) {
    chg_state = OFF; 
    bck_pwm = 0; 
    integral_err = 0; 
    digitalWrite(BCK_SD_PIN, LOW); 
    ledcWrite(1, 0); 
    return;
  }

  switch (chg_state) {
    case OFF:
      stateStr = "OFF";
      digitalWrite(BCK_SD_PIN, LOW); 
      bck_pwm = 0;

      if (dc_bus_vol > 15.0 && bat_vin > 9.0) {
        if (dc_bus_vol > 0) {
           float feed_forward = (bat_vin / dc_bus_vol) * 1023.0;
           bck_pwm = (int)(feed_forward * 0.90); 
        } else {
           bck_pwm = 0;
        }
        bck_pwm = constrain(bck_pwm, 0, 800); 
        ledcWrite(1, bck_pwm);
        digitalWrite(BCK_SD_PIN, HIGH); 
        chg_state = BULK_CC;
      }
      break;

    case BULK_CC: 
      stateStr = "CC ";
      digitalWrite(BCK_SD_PIN, HIGH); 
      runCurrentControl(I_MAX_CHARGE, bat_iout_f);
      
      // [BARU] Logika Jeda Transisi 3 Detik
      if (bat_vin >= V_ABSORPTION) {
        if (cv_transition_timer == 0) cv_transition_timer = millis();
        
        if (millis() - cv_transition_timer > DELAY_CV_MS) {
           chg_state = ABSORP_CV;
           integral_err = 0; 
           cv_transition_timer = 0;
        }
      } else {
        cv_transition_timer = 0; // Reset jika tegangan turun lagi
      }
      break;

    case ABSORP_CV: 
      stateStr = "CV ";
      digitalWrite(BCK_SD_PIN, HIGH); 
      runVoltagePI(V_ABSORPTION, bat_vin);
      
      if (bat_iout_f < 0.15) chg_state = FLOAT_CV; 
      break;

    case FLOAT_CV: 
      stateStr = "FLT";
      digitalWrite(BCK_SD_PIN, HIGH); 
      runVoltagePI(V_FLOAT, bat_vin);
      
      if (bat_vin < 13.0) chg_state = BULK_CC; 
      break;
  }
}

void runVoltagePI(float setpoint, float input) {
  float error = setpoint - input;
  
  // Tuning PI: Sedikit lebih agresif agar cepat
  float Kp = 8.0;  // Naikkan dari 5.0
  float Ki = 1.2;  // Naikkan dari 0.5

  integral_err += error;
  integral_err = constrain(integral_err, -200, 200); // Perlebar integral window

  float output = (Kp * error) + (Ki * integral_err);
  
  // Feed Forward: Prediksi Duty Cycle ideal
  float input_voltage_now = (dc_bus_vol > 5.0) ? dc_bus_vol : 12.0; 
  float ff = (setpoint / input_voltage_now) * 1023.0;
  
  int pwm_target = (int)ff + (int)output;
  pwm_target = constrain(pwm_target, 0, 980); // Limit Max PWM

  // --- LOGIKA ADAPTIF (SPEED UP) ---
  int current_pwm = bck_pwm;
  int diff = pwm_target - current_pwm;

  // Jika selisih jauh (>10), loncat cepat. Jika dekat, pelan-pelan.
  if (abs(diff) > 20) {
    bck_pwm += (diff / 4); // Loncat 25% dari selisih (Cepat!)
  } else if (diff > 0) {
    bck_pwm += 1; // Fine tuning naik
  } else if (diff < 0) {
    bck_pwm -= 1; // Fine tuning turun
  }
  
  bck_pwm = constrain(bck_pwm, 0, 980);
}

void runCurrentControl(float target_amps, float current_now) {
  // Safety: Jika Bus drop parah, kurangi beban cepat
  if (dc_bus_vol < 14.5) { 
      bck_pwm -= 10; 
      bck_pwm = constrain(bck_pwm, 0, 1023);
      return; 
  }
  
  float error = target_amps - current_now; 
  
  // Deadband: Jika error sangat kecil, diam saja biar stabil
  if (abs(error) < 0.02) return; 

  if (error > 0) { 
    // ARUS KURANG: Naikkan PWM
    // Hapus slow_ramp, langsung naik!
    bck_pwm++; 
  } 
  else { 
    // ARUS LEBIH: Turunkan PWM
    // Jika kelebihan banyak (>0.1A), rem pakem (-5)
    // Jika kelebihan dikit, rem halus (-1)
    if (error < -0.1) bck_pwm -= 5;
    else bck_pwm--;
  }
  
  bck_pwm = constrain(bck_pwm, 0, 950); 
}

void readAllSensors() {
  float adsConst = 0.125F / 1000.0;
  
  float pv_r    = ads.readADC_SingleEnded(CH_SOLAR_V) * adsConst * DIV_SOLAR;
  float bus_r   = ads.readADC_SingleEnded(CH_DCBUS_V) * adsConst * DIV_DCBUS;
  float val_A3  = ads.readADC_SingleEnded(CH_BATT_V) * adsConst * DIV_BATT;    
  float val_A2  = ads.readADC_SingleEnded(CH_BCK_OUT_V) * adsConst * DIV_BCK_OUT; 

  pv_vin = (pv_r < 0.5) ? 0 : pv_r;
  dc_bus_vol = (bus_r < 0.5) ? 0 : bus_r;
  
  if (dc_bus_avg == 0) dc_bus_avg = dc_bus_vol; 
  dc_bus_avg = 0.9 * dc_bus_avg + 0.1 * dc_bus_vol;

  // --- BAGIAN YANG DIGANTI HANYA INI ---
  // Sebelumnya ada if(chg_state == OFF)... sekarang langsung pakai A3
  bat_vin = (val_A3 < 0.5) ? 0 : val_A3;
  // -------------------------------------
  
  bck_vout = val_A2;

  long rPV = 0, rBat = 0;
  for (int i=0; i<100; i++) { 
    rPV += analogRead(ACS_PV_PIN); 
    rBat += analogRead(ACS_BAT_PIN); 
  }

  float vPV  = (rPV / 100.0 / 4095.0) * 3.3;
  float vBat = (rBat / 100.0 / 4095.0) * 3.3;

  float curPV  = (vPV - ACS_PV_ZERO) * 1000.0 / ACS_SENS_PV;
  float curBat = (vBat - ACS_BAT_ZERO) * 1000.0 / ACS_SENS_BAT;

  pv_iin = (abs(curPV) < 0.05) ? 0 : abs(curPV);
  bat_iout = (abs(curBat) < 0.05) ? 0 : abs(curBat);

  float alpha = (pv_power > 80) ? 0.35 : 0.15;
  pv_iin_f = alpha * pv_iin + (1-alpha) * pv_iin_f;

  bat_iout_f = alpha * bat_iout + (1-alpha) * bat_iout_f;
  
  pv_power = pv_vin * pv_iin_f;
}

// =================================================
// TAMPILAN (LCD & SERIAL MONITOR)
// =================================================
void printStatus() {
  float p_out = bat_vin * bat_iout_f;
  
  // 1. SERIAL MONITOR
  // Format: PV Actual (Ref Target) agar mudah dibandingkan
  Serial.printf("S:%s | PV:%.2fV (Ref:%.2fV) %.2fA | Bus:%.2fV | Bat:%.2fV %.2fA | PWM:%d/%d\n",
    stateStr.c_str(), 
    pv_vin,       // PV Aktual
    v_ref_mppt,   // PV Target (MPPT)
    pv_iin_f, 
    dc_bus_vol, 
    bat_vin, 
    bat_iout_f, 
    bst_pwm, bck_pwm);

  // 2. LCD I2C 20x4
  // Baris 0: PV
  lcd.setCursor(0,0); 
  lcd.print("PV :"); lcd.print(pv_vin,1); lcd.print("V "); lcd.print(pv_iin_f,2); lcd.print("A");

  // Baris 1: BUS & REF (INI YANG PENTING UNTUK CEK SMC)
  lcd.setCursor(0,1); 
  lcd.print("BUS:"); lcd.print(dc_bus_vol,1); lcd.print("V "); 
  lcd.setCursor(11,1); 
  lcd.print("Ref:"); lcd.print(v_ref_mppt, 1); // Tampilkan Target

  // Baris 2: BAT
  lcd.setCursor(0,2); 
  lcd.print("BAT:"); lcd.print(bat_vin,1); lcd.print("V "); lcd.print(bat_iout_f,2); lcd.print("A");

  // Baris 3: PWR & STATE
  lcd.setCursor(0,3); 
  lcd.print("PWR:"); lcd.print(p_out,1); lcd.print("W "); 
  lcd.setCursor(12,3); lcd.print("ST:"); lcd.print(stateStr);
}