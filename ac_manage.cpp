const int acButtonPin = A0;
const int pressurePin = A1;
const int oilPressurePin = A3;  // Oil pressure sensor input
const int compressorPin = 7;
const int fanPin = 8;

// Pressure calibration
const float PRESSURE_MIN_V = 0.1;
const float PRESSURE_MAX_V = 1.5;
const float PRESSURE_MAX_PSI = 380.0;

// Oil pressure thresholds (adjust based on your specific sensor)
const float OIL_PRESS_ENGINE_OFF = 1.0;  // Below this = engine off (0.0-1.0V)
const float OIL_PRESS_ENGINE_ON = 2.0;    // Above this = engine running (2.0-5.0V)

// Pressure thresholds for cycle timing
const float HIGH_CYCLE_MIN_PSI = 210.0;
const float HIGH_CYCLE_MAX_PSI = 340.0;
const float LOW_CYCLE_MIN_PSI = 150.0;
const float LOW_CYCLE_MAX_PSI = 210.0;

// Static pressure
const float STATIC_PSI = 25.0;

enum ShutdownReason {
  MANUAL,
  SAFETY_LIMIT,
  RUNTIME_LIMIT
};

// System state
unsigned long compressorStartTime = 0;
unsigned long compressorStopTime = 0;
bool compressorEnabled = false;
bool engineRunning = false;               // Added engine running flag
ShutdownReason lastShutdownReason = MANUAL;
unsigned long storedRunTimeLimit = 0;     
unsigned long storedRestartDelay = 0;     

float adcToPsi(int adcValue) {
  float voltage = (adcValue / 1023.0) * 5.0;
  if (voltage < PRESSURE_MIN_V) return 0.0;
  if (voltage > PRESSURE_MAX_V) return PRESSURE_MAX_PSI;
  return ((voltage - PRESSURE_MIN_V) / (PRESSURE_MAX_V - PRESSURE_MIN_V)) * PRESSURE_MAX_PSI;
}

void controlCompressor(bool enable, ShutdownReason reason = MANUAL) {
  digitalWrite(compressorPin, enable ? LOW : HIGH);
  
  if (enable) {
    compressorStartTime = millis();
    lastShutdownReason = MANUAL;
  } else {
    compressorStopTime = millis();
    lastShutdownReason = reason;
  }
  compressorEnabled = enable;
}

void setup() {
  pinMode(compressorPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(oilPressurePin, INPUT);  // Initialize oil pressure sensor
  controlCompressor(false);
  Serial.begin(9600);
}

void loop() {
  // Read sensors
  int buttonState = analogRead(acButtonPin);
  float pressurePSI = adcToPsi(analogRead(pressurePin));
  float oilVoltage = (analogRead(oilPressurePin) / 1023.0) * 5.0;  // Read oil pressure

  // Determine engine status
  engineRunning = (oilVoltage >= OIL_PRESS_ENGINE_ON);

  // System checks
  bool sensorFault = (pressurePSI <= 25 ) || (pressurePSI >= PRESSURE_MAX_PSI);
  bool staticPressureValid = (!compressorEnabled && (buttonState < 80)) ? (pressurePSI >= STATIC_PSI) : true;
  bool pressureSafe = (pressurePSI >= 25) && (pressurePSI < 300);
  bool systemFault = sensorFault || !staticPressureValid;
  
  // Fan control
  bool fanActive = (buttonState < 80) && !systemFault && engineRunning;
  digitalWrite(fanPin, fanActive ? LOW : HIGH);

  // Diagnostics header
  Serial.print("ENGINE: ");
  Serial.print(engineRunning ? "ON" : "OFF");
  Serial.print(" | OIL: ");
  Serial.print(oilVoltage, 1);
  Serial.print("V | BUTTON: ");
  Serial.print(buttonState);
  Serial.print(" (");
  Serial.print((buttonState / 1023.0) * 5.0, 2);
  Serial.print("V) | AC: ");
  Serial.print(compressorEnabled ? "ON" : "OFF");
  Serial.print(" | PRESSURE:");
  Serial.print(" (");
  Serial.print((analogRead(pressurePin) / 1023.0) * 5.0, 2);
  Serial.print(" V) ");
  Serial.print(pressurePSI, 1);
  
  // Pressure status annotation
  if (sensorFault) {
    Serial.print(" PSI (SENSOR FAULT)");
  } else if (!staticPressureValid) {
    Serial.print(" PSI (CHARGE FAULT)");
  } else if (!pressureSafe && compressorEnabled) {
    Serial.print(" PSI (UNSAFE)");
  } else {
    Serial.print(" PSI");
  }

  bool acRequested = (buttonState < 80) && !systemFault;
  bool runtimeLimit = compressorEnabled && 
                     (millis() - compressorStartTime >= storedRunTimeLimit);

  // Automatic shutdown
  if (compressorEnabled) {
    // Runtime limit check
    if (runtimeLimit) {
      controlCompressor(false, RUNTIME_LIMIT);
    } 
    // High-pressure cutoff
    else if (pressurePSI >= HIGH_CYCLE_MAX_PSI) {
      controlCompressor(false, SAFETY_LIMIT);
    }
    // Engine stopped while running
    else if (!engineRunning) {  // NEW: Engine status check
      controlCompressor(false, SAFETY_LIMIT);
    }
    // Existing safety checks
    else if (!pressureSafe || systemFault) {
      controlCompressor(false, SAFETY_LIMIT);
    }
  }

  // Compressor control logic
  if (acRequested && !compressorEnabled && !systemFault && engineRunning) {  // Added engine check
    bool pressureNowSafe = (pressurePSI >= 0) && (pressurePSI < 300);
    bool safetyCooldownActive = (lastShutdownReason != MANUAL) && 
                               (millis() - compressorStopTime < storedRestartDelay);
    
    if (pressureNowSafe && !safetyCooldownActive) {
      if (pressurePSI >= HIGH_CYCLE_MIN_PSI && pressurePSI <= HIGH_CYCLE_MAX_PSI) {
        storedRunTimeLimit = 600000;  // 10 minutes 600000
        storedRestartDelay = 60000;   // 60 seconds 60000
      } else if (pressurePSI >= LOW_CYCLE_MIN_PSI && pressurePSI < LOW_CYCLE_MAX_PSI) {
        storedRunTimeLimit = 480000;  // 8 minutes 480000
        storedRestartDelay = 90000;   // 90 seconds 90000
      } else {
        storedRunTimeLimit = 300000;  // 5 minutes 300000
        storedRestartDelay = 30000;   // 30 seconds 30000
      }
      controlCompressor(true);
    }
  } else if (!acRequested && compressorEnabled) {
    controlCompressor(false, MANUAL);
  }

  // Status display
  Serial.print(" | FAN: ");
  Serial.print(fanActive ? "ON" : "OFF");
  
  if (compressorEnabled) {
    Serial.print(" | RUNNING: ");
    Serial.print((millis() - compressorStartTime) / 1000);
    Serial.print("s/");
    Serial.print(storedRunTimeLimit / 1000);
    Serial.println("s");
  } else if (systemFault) {
    Serial.println(" | SYSTEM FAULT");    
  } else if (lastShutdownReason != MANUAL) {
    bool pressureNowSafe = (pressurePSI > 20) && (pressurePSI < 300);
    long remainingCooldown = storedRestartDelay - (millis() - compressorStopTime);
    remainingCooldown = remainingCooldown < 0 ? 0 : remainingCooldown;
    Serial.print(" | COOLDOWN: ");
    Serial.print(remainingCooldown / 1000);
    Serial.print("s (");
    Serial.print(pressureNowSafe ? "Safe" : "Unsafe");
    Serial.println(")");
  } else {
    Serial.println(" | SYSTEM READY");
  }
  
  delay(350);
}
