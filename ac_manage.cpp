// HVAC/Compressor Control with Hybrid Pressure Regulation
// PID version for faster response

// Pin definitions
const int acButtonPin    = A0;   // Analog input for AC start button
const int pressurePin    = A1;   // Analog input for high-side pressure sensor
const int oilPressurePin = A3;   // Analog input for oil pressure sensor
const int thermistorPin  = A2;   // Analog input for evaporator thermistor
const int pcmFanPin      = 2;    // Digital input for PCM low-speed request (active LOW)
const int pcmHighPin     = 3;    // Digital input for PCM high-speed request (active LOW)
const int compPin        = 7;    // Digital output: controls compressor (active LOW)
const int fanPin         = 8;    // Digital output: controls fan (active LOW)

// Pressure sensor calibration
const float PRES_MIN_V   = 0.1;
const float PRES_MAX_V   = 2.8;
const float PRES_MAX_PSI = 400.0;
const float STATIC_PSI   = 40.0;   // Minimum safe pressure
const float MAX_SAFE_PSI = 300.0; // Max safe high-side pressure
const float PRES_HYSTERESIS = 10.0; // 10 PSI hysteresis for pressure safety

// Oil pressure threshold
const float OIL_PRESS_ON = 1.0;

// Delays
const unsigned long ENG_START_DELAY = 5000;   // 5 seconds
const unsigned long PRES_COOLDOWN_TIME = 10000; // 10 seconds pressure cooldown

// PID Constants - AGGRESSIVE SETTINGS
const float Kp = 3.0;   // Proportional gain (increased from 1.5)
const float Ki = 0.15;  // Integral gain (increased from 0.05)
const float Kd = 0.5;   // Derivative gain (new)

// Pressure Setpoint
const float PRES_SETPOINT = 250.0; 

// PID Thresholds
const float PID_ON_TH  = 10.0;
const float PID_OFF_TH = 5.0;

// Hybrid control thresholds
const float PID_MIN_PSI = 0.4 * PRES_SETPOINT;   // 100 PSI
const float PID_MAX_PSI = 0.8 * MAX_SAFE_PSI;   // 240 PSI

// Thermistor parameters
const float THERM_BETA = 3950.0;      // Beta coefficient
const float THERM_NOM  = 10000.0;     // Resistance at 25°C
const float TEMP_NOM   = 25.0;        // Reference temperature
const float SERIES_RES = 10000.0;     // Voltage divider resistor
const float EVAP_OFF_T = 1.0;         // Compressor OFF below this temp
const float EVAP_ON_T  = 6.0;         // Compressor ON above this temp

// Cycle time limits
const unsigned long MIN_ON_TIME  = 120000;  // 2 min min on time
const unsigned long MIN_OFF_TIME = 10000;   // 10 sec min off time
const unsigned long MAX_RUN_TIME = 240000;  // 4 min max runtime

// Global variables
unsigned long engStartTime = 0;
unsigned long compOffTime = 0; 
unsigned long compOnTime = 0;       // Last compressor on time
float prevPres = STATIC_PSI;        // Previous pressure value
bool evapCooldown = false;          // Evaporator cooldown state
unsigned long presCooldownUntil = 0; // Pressure safety cooldown end time
float lastTriggerPressure = 0.0;    // Track pressure at trigger time

// Track PCM states
bool pcmLowReq = false;
bool pcmHighReq = false;

// PID variables
float pidInt = 0;                  // Integral sum
float prevErr = 0;                 // Previous error
unsigned long lastPidTime = 0;     // Last PID calc time
bool compOn = false; 
bool prevCompOn = false;           // Previous compressor state
bool pidActive = false;            // PID control active flag

float adcToPsi(int adcValue) {
  float volt = (adcValue / 1023.0) * 5.0;
  if (volt < PRES_MIN_V) return 0.0; 
  if (volt > PRES_MAX_V) return PRES_MAX_PSI; 
  return ((volt - PRES_MIN_V) / (PRES_MAX_V - PRES_MIN_V)) * PRES_MAX_PSI;
}

float readEvapTemp() {
  int raw = analogRead(thermistorPin);
  if (raw <= 0) return -100; // Safety
  
  float res = SERIES_RES / (1023.0 / raw - 1.0);
  float st = res / THERM_NOM;       // (R/Ro)
  st = log(st);                     // ln(R/Ro)
  st /= THERM_BETA;                 // 1/B * ln(R/Ro)
  st += 1.0 / (TEMP_NOM + 273.15); // + (1/To)
  st = 1.0 / st;                    // Invert
  // return st - 273.15;               // Convert to °C
  return 8; // Send 8C 
}

void setup() {
  pinMode(acButtonPin, INPUT);
  pinMode(pressurePin, INPUT);
  pinMode(oilPressurePin, INPUT);
  pinMode(thermistorPin, INPUT); 
  pinMode(pcmFanPin, INPUT_PULLUP); 
  pinMode(pcmHighPin, INPUT_PULLUP);
  pinMode(compPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  
  Serial.begin(9600);
  
  // Ensure outputs off (active LOW)
  digitalWrite(compPin, HIGH);
  digitalWrite(fanPin, HIGH);

  lastPidTime = millis();
  prevPres = adcToPsi(analogRead(pressurePin));
}

void loop() {
  // Read sensor values
  int btnADC = analogRead(acButtonPin);
  int presADC = analogRead(pressurePin);
  int oilADC = analogRead(oilPressurePin);
  
  float evapTemp = readEvapTemp();
  pcmLowReq = (digitalRead(pcmFanPin) == LOW);
  pcmHighReq = (digitalRead(pcmHighPin) == LOW);
  
  float oilVolt = (oilADC / 1023.0) * 5.0;
  float currPres = adcToPsi(presADC);
  unsigned long now = millis();

  // Engine state
  bool engOn = (oilVolt >= OIL_PRESS_ON);
  
  // Engine start transition
  static bool prevEngOn = false;
  if (engOn && !prevEngOn) {
    engStartTime = now;
    pidInt = 0;
    prevErr = 0;
    lastPidTime = now;
  }
  
  // SYSTEM RESET
  static bool lastAcReq = false;
  bool acReq = (btnADC < 90); 
  
  if ((!acReq && lastAcReq) ||  // AC turned off
      (!engOn && prevEngOn)) {  // Engine turned off
    // Full reset
    pidInt = 0;
    prevErr = 0;
    compOn = false;
    evapCooldown = false;
    compOnTime = 0;
    compOffTime = 0;
    presCooldownUntil = 0;
    pidActive = false;
    lastTriggerPressure = 0.0;
    
    if (!engOn) prevEngOn = false;
    Serial.println("SYSTEM RESET");
  }
  lastAcReq = acReq;
  prevEngOn = engOn;
  
  // Engine start delay
  bool engReady = engOn && ((now - engStartTime) >= ENG_START_DELAY);
  
  // Evaporator cooldown
  if (evapTemp <= EVAP_OFF_T) {
    evapCooldown = true;
  } else if (evapTemp >= EVAP_ON_T) {
    evapCooldown = false;
  }
  
  // PRESSURE SAFETY - FIXED COOLDOWN TIMER LOGIC
  if (currPres > MAX_SAFE_PSI) {
    if (now >= presCooldownUntil) {
      if (lastTriggerPressure == 0.0 || 
          currPres > (lastTriggerPressure + PRES_HYSTERESIS)) {
        presCooldownUntil = now + PRES_COOLDOWN_TIME;
        lastTriggerPressure = currPres;
        Serial.print("PRESSURE SAFETY TRIGGERED: ");
        Serial.print(currPres, 1);
        Serial.println(" PSI");
      }
    }
  } 
  // Clear trigger if pressure drops sufficiently
  else if (currPres < (MAX_SAFE_PSI - PRES_HYSTERESIS)) {
    lastTriggerPressure = 0.0;
  }

  // Pressure safety
  bool presSafe = (currPres >= STATIC_PSI && currPres <= MAX_SAFE_PSI);
  
  // Unsafe conditions
  bool unsafe = !presSafe || !engOn || !engReady || evapCooldown || (now < presCooldownUntil);

  // --- AGGRESSIVE HYBRID CONTROL SYSTEM ---
  float pidOut = 0;
  
  if (acReq && !unsafe && engOn && engReady && presSafe) {
    // Decide control mode based on pressure
    if (currPres > PID_MIN_PSI && currPres < PID_MAX_PSI) {
      // AGGRESSIVE PID MODE: Near setpoint (198-285 PSI)
      float dt = (now - lastPidTime) / 1000.0;
      if (dt > 0.001) {
        float err = PRES_SETPOINT - currPres;
        
        // More aggressive P term
        float pTerm = Kp * err;
        
        // I term with enhanced anti-windup
        pidInt += err * dt;
        
        // Enhanced anti-windup:
        // Reset 50% on zero-crossing and tighten limits
        if ((prevErr > 0 && err < 0) || (prevErr < 0 && err > 0)) {
          pidInt *= 0.5;  // Reset windup on zero-crossing
        }
        // State-based anti-windup
        if (compOn && err > 0) {
            pidInt = min(pidInt, 0.0f);
        } else if (!compOn && err < 0) {
            pidInt = max(pidInt, 0.0f);
        }
        pidInt = constrain(pidInt, -500.0, 500.0);  // Tighter limits
        
        float iTerm = Ki * pidInt;
        
        // D term - added for anticipatory control
        float dTerm = 0;
        if (dt > 0.1) {  // Only calculate if meaningful time passed
          dTerm = Kd * (err - prevErr) / dt;
        }
        
        pidOut = pTerm + iTerm + dTerm;
        prevErr = err;
        
        // Thresholds remain same but respond faster
        bool pidOn = (pidOut >= PID_ON_TH);
        bool pidOff = (pidOut < PID_OFF_TH);
        
        unsigned long onTime = now - compOnTime;
        unsigned long offTime = now - compOffTime;
        
        // More responsive cycling with safety checks
        if (pidOn) {
          if (!compOn && offTime >= MIN_OFF_TIME) {
            compOn = true;
            compOnTime = now;
          }
        } 
        else if (pidOff) {
          if (compOn && onTime >= MIN_ON_TIME) {
            compOn = false;
            compOffTime = now;
          }
        }
      }
      lastPidTime = now;
      pidActive = true;
    } else {
      // FIXED THRESHOLD MODE: Outside PID range
      pidActive = false;
      
      if (currPres < PID_MIN_PSI) {  // Below 198 PSI
        // Turn compressor ON with min cycle time
        if (!compOn && (now - compOffTime >= MIN_OFF_TIME)) {
          compOn = true;
          compOnTime = now;
        }
      } else if (currPres > PID_MAX_PSI) {  // Above 285 PSI
        // Turn compressor OFF with min cycle time
        if (compOn && (now - compOnTime >= MIN_ON_TIME)) {
          compOn = false;
          compOffTime = now;
        }
      }
    }
    
    // Max runtime protection
    if (compOn && (now - compOnTime > MAX_RUN_TIME)) {
      compOn = false;
      compOffTime = now;
      Serial.println("SAFETY: Max runtime");
    }
  } else {
    compOn = false;
    pidActive = false;
  }
  
  // Force compressor off during cooldown
  if (evapCooldown || (now < presCooldownUntil)) {
    compOn = false;
  }
  
  // Track compressor state change
  if (prevCompOn && !compOn) {
    compOffTime = now;
  }
  prevCompOn = compOn;

  // --- FAN CONTROL ---
  bool fanOn = false;
  
  if (pcmHighReq) {
    digitalWrite(fanPin, HIGH);
    fanOn = true;
  } 
  else {
    if (currPres > MAX_SAFE_PSI) {
      fanOn = true; 
    } 
    else if (pcmLowReq && engOn) { 
      fanOn = true;
    }
    else if (acReq && engOn && engReady) { 
      fanOn = true;
    }
    
    digitalWrite(fanPin, fanOn ? LOW : HIGH); 
  }
  
  // Update compressor
  digitalWrite(compPin, compOn ? LOW : HIGH);
  
  // --- Diagnostics ---
  static unsigned long lastDiag = 0;
  if (now - lastDiag >= 1000) {
    lastDiag = now;
    
    Serial.print("ENG: ");
    Serial.print(engOn ? "ON" : "OFF");
    if (engOn && !engReady) {
      long rem = (ENG_START_DELAY - (now - engStartTime)) / 1000;
      Serial.print(" (START: ");
      Serial.print(rem);
      Serial.print("s)");
    }
    
    Serial.print(" | BTN: ");
    Serial.print(acReq ? "ON" : "OFF"); 
    
    Serial.print(" | EVAP: ");
    Serial.print(evapTemp, 1);
    Serial.print("C");
    if (evapCooldown) {
      Serial.print(" (COOLDOWN)");
    }
    
    Serial.print(" | PRES: ");
    Serial.print(currPres, 1);
    Serial.print(" PSI (");
    Serial.print((presADC / 1023.0) * 5.0, 2);
    Serial.print("V)");

    if (!presSafe) {
      Serial.print(" (UNSAFE!)"); 
    } else if (currPres > PRES_SETPOINT) {
      Serial.print(" (HIGH)");
    } else if (currPres < PRES_SETPOINT) {
      Serial.print(" (LOW)");
    }
    
    // Cooldown status
    if (now < presCooldownUntil) {
      Serial.print(" | PRES_COOLDOWN: ");
      Serial.print((presCooldownUntil - now)/1000);
      Serial.print("s");
    }
    
    Serial.print(" | Safe: ");
    Serial.print(unsafe ? "NO" : "YES"); 
    
    Serial.print(" | Mode: ");
    Serial.print(pidActive ? "PID" : "FIXED");
    
    Serial.print(" | Cycle: ");
    if (compOn) {
      Serial.print("ON:");
      unsigned long ot = (now - compOnTime) / 1000;
      Serial.print(ot);
      Serial.print("s");
      
      if (ot > (MAX_RUN_TIME / 1000) - 30) {
        Serial.print(" (MAX SOON)");
      }
    } else {
      Serial.print("OFF:");
      if (compOffTime > 0) {
        Serial.print((now - compOffTime) / 1000);
        Serial.print("s");
      } else {
        Serial.print("N/A");
      }
    }
    
    Serial.print(" | PCM: ");
    Serial.print(pcmLowReq ? "LOW" : "OFF");
    Serial.print("/");
    Serial.print(pcmHighReq ? "HIGH" : "OFF");
    
    Serial.print(" | COMP: ");
    Serial.print(compOn ? "ON" : "OFF");

    Serial.print(" | FAN: ");
    Serial.print(fanOn ? "ON" : "OFF");
    
    Serial.print(" | PID: ");
    Serial.print(pidOut, 1);
    
    Serial.println();
  }
}
